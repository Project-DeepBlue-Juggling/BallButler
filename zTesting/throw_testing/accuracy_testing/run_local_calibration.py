#!/usr/bin/env python3
"""Plan / run a randomized, nonuniform BallButler accuracy calibration.

Python 3.8 compatible. `plan` is offline and stdlib-only. `run` needs the
sourced Jugglebot ROS2 environment on the Jetson. Uses the production inverse
ballistics with an explicit positive hand offset, no affine, and the awaitable
bb/throw action. Does not edit the running node or production configuration.
See LOCAL_CALIBRATION.md for frames, capture, and resumption instructions.
"""
from __future__ import annotations

import argparse
import hashlib
import html
import json
import math
import random
import re
import queue
import signal
import subprocess
import sys
import time
import threading
from datetime import datetime, timezone
from pathlib import Path

SCHEMA = 'bb-local-calibration-v1'
DEFAULT_REFILL_EVERY = 9
DEFAULT_CORE = [-62.5, 62.5, 0.0, 0.0]  # Oct 4 sitting 4, schedule-frame mm
TOPICS = ['/mocap_data', '/rigid_body_poses', '/bb/calibration_result',
          '/bb/heartbeat', '/bb/axis_estimates', '/orchestrator_state',
          '/rosout', '/parameter_events', '/bb/local_calibration/event']
NUMBER = r'[-+]?\d+(?:\.\d+)?'
AIM_RE = re.compile(r'CATCH-AIM skill \d+:.*?landing=\(\s*('+NUMBER+r'),\s*('+NUMBER+r'),\s*('+NUMBER+r')\)')


def write_json(path, value):
    """Atomic checkpoint: interrupted capture keeps the last complete JSON."""
    path = Path(path)
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    tmp.replace(path)


def finite(values):
    return all(math.isfinite(float(v)) for v in values)


def goal_uuid_bytes(goal_id):
    """ROS2 fixed uint8 arrays can contain NumPy scalars, not Python ints."""
    return [int(value) for value in goal_id.uuid]


def refill_due(next_throw_index, batch_size=DEFAULT_REFILL_EVERY):
    """Initial ready prompt, then refill before each new batch of nine throws."""
    return next_throw_index % batch_size == 0


def flight_capture(release_position, release_velocity, catch_tof, ground_z, gravity=9806.0):
    """Observe through nominal ground arrival plus margin before permitting refill.

    Ground arrival is a timing estimate, not a sensor confirmation. The operator
    must also see the ball down before returning balls. Refills never occur in
    the analysis window when the prompted protocol is followed.
    """
    rz=release_position[2]; vz=release_velocity[2]
    if ground_z>=rz:
        raise ValueError('Ground plane must be below BB release height.')
    ground_tof=(vz+math.sqrt(vz*vz+2*gravity*(rz-ground_z)))/gravity
    return max(catch_tof,ground_tof)+0.5


def analysis_throw_at(session, wall_time_s):
    """Associate a raw sample by an explicitly completed BB capture window.

    Refill-period samples, failed/unknown commands, and overlapping windows
    have no assignment. Downstream trajectory extraction must use these
    windows, not projectile count/order. A window may still contain reflections
    or bounces: fit the outbound BB arc through the catch plane, never assume
    every raw marker in the interval is a ball.
    """
    if not math.isfinite(wall_time_s):
        return None
    for interval in session.get('refill_intervals',[]):
        if interval['start_wall_s']<=wall_time_s and (interval['end_wall_s'] is None or wall_time_s<interval['end_wall_s']):
            return None
    matches=[]
    for row in session.get('throws',[]):
        window=row.get('analysis_window_wall_s')
        if (row.get('status')=='released' and row.get('capture_complete') and window
                and window[0]<=wall_time_s<window[1]):
            matches.append(row['throw_idx'])
    return matches[0] if len(matches)==1 else None


def wait_for_operator(spin_once, read_line=input):
    """Keep servicing ROS while the operator refills; no refill timeout."""
    answers=queue.Queue()
    def read():
        try: answers.put(read_line())
        except Exception as error: answers.put(error)
    threading.Thread(target=read,daemon=True).start()
    while True:
        spin_once()
        try: answer=answers.get_nowait()
        except queue.Empty: continue
        if isinstance(answer,Exception):
            raise RuntimeError('Operator input closed; no further throws') from answer
        if answer.strip().lower() in ('q','quit','stop'):
            raise KeyboardInterrupt('Operator stopped at refill pause')
        if answer.strip():
            raise RuntimeError('Expected Enter to continue or q to stop; nothing dispatched')
        return


def streams_ready(cache, now):
    """Fresh receive timestamps; a latched/stale loaded heartbeat is insufficient."""
    hb=cache['heartbeat']
    return bool(hb and hb.connected and now-cache['hb_at']<2 and now-cache['mocap_at']<1
                and cache['state']=='IDLE' and now-cache['state_at']<3)


def check_recorded_topics(metadata):
    """Check finalized rosbag2 metadata, not just the existence of a bag file."""
    info=metadata.get('rosbag2_bagfile_information',{})
    counts={row['topic_metadata']['name']:int(row['message_count'])
            for row in info.get('topics_with_message_count',[])}
    required=('/mocap_data','/bb/local_calibration/event','/bb/heartbeat','/bb/calibration_result')
    missing=[topic for topic in required if counts.get(topic,0)<=0]
    if missing: raise ValueError('Recording missing messages on: '+', '.join(missing))
    return counts


def recorder_shutdown_ok(exit_code, requested_sigint, metadata_valid, close_error=False):
    """Foxy may return the SIGINT number (2) after orderly recorder shutdown.

    Accept it only for our requested stop and with finalized, populated metadata.
    This verifies capture bookkeeping, not the visibility of individual balls.
    """
    return bool(metadata_valid and not close_error and
                (exit_code == 0 or (requested_sigint and exit_code == 2)))


def core_from_logs(paths):
    """Use columns CATCH-AIM points only; ignore other skills and starts."""
    points = []
    for path in paths:
        in_columns = False
        for line in Path(path).read_text(encoding='utf-8', errors='replace').splitlines():
            if ' started' in line:
                in_columns = bool(re.search(r'\bcolumns(?:_\w+)?(?:\s|\()', line))
            match = AIM_RE.search(line)
            if in_columns and match:
                points.append(tuple(float(x) for x in match.groups()))
    if not points:
        raise ValueError('No columns CATCH-AIM points found; supply --core explicitly or use the documented default.')
    return [min(p[0] for p in points), max(p[0] for p in points),
            min(p[1] for p in points), max(p[1] for p in points)], points


def axis_points(lo, hi, padding, minimum, maximum, taper):
    """Symmetric dense core, then increasing gaps; never squeeze in an edge point.

    Core is rounded OUT to whole minimum-spacing intervals around its centre.
    Each outward gap is min + (max-min)*distance-from-core/taper, capped at max.
    Padding is an upper bound; the final point may fall inside that bound.
    """
    centre = (lo + hi) / 2
    half = max(minimum, math.ceil((hi-lo)/2/minimum)*minimum)
    count = int(round(half/minimum))
    positive = [i*minimum for i in range(count+1)]
    edge = half + padding
    while True:
        distance = positive[-1]-half
        gap = minimum + (maximum-minimum)*min(distance/taper, 1.0)
        nxt = positive[-1]+gap
        if nxt > edge + 1e-8:
            break
        positive.append(nxt)
    return [round(centre-v,6) for v in positive[:0:-1]] + [round(centre+v,6) for v in positive], half


def make_plan(args):
    values = [*args.core, args.spacing_min, args.spacing_max, args.taper, args.padding, args.z]
    if not finite(values) or not (0 < args.spacing_min <= args.spacing_max) or args.taper <= 0 or args.padding < 0:
        raise ValueError('Grid values must be finite, spacing positive and ordered, taper positive, padding nonnegative.')
    if args.repeats < 1 or args.core_repeats < args.repeats:
        raise ValueError('Require core-repeats >= repeats >= 1.')
    core, points = (core_from_logs(args.logs) if args.logs else (args.core, []))
    if core[0] > core[1] or core[2] > core[3]:
        raise ValueError('Core bounds must be xmin xmax ymin ymax.')
    xs,hx = axis_points(*core[:2], args.padding, args.spacing_min, args.spacing_max,args.taper)
    ys,hy = axis_points(*core[2:], args.padding, args.spacing_min, args.spacing_max,args.taper)
    cx,cy=(core[0]+core[1])/2,(core[2]+core[3])/2
    cells=[]
    for y in ys:
        for x in xs:
            dense = abs(x-cx)<=hx+1e-6 and abs(y-cy)<=hy+1e-6
            cells.append(dict(cell_idx=len(cells), target_mm=[x,y,args.z], dense=dense,
                              repeats=args.core_repeats if dense else args.repeats))
    rng=random.Random(args.seed)
    schedule=[]
    # Randomized blocks interleave repeats over time; every cell sampled once
    # per block before receiving its next repetition.
    for repetition in range(args.core_repeats):
        block=[dict(cell_idx=c['cell_idx'], repeat=repetition, target_mm=c['target_mm'])
               for c in cells if c['repeats']>repetition]
        rng.shuffle(block)
        schedule.extend(block)
    for i,entry in enumerate(schedule):
        entry['throw_idx']=i
    return dict(schema=SCHEMA, frame='schedule_xy_world_z', z_mm=args.z,
                core_bounds_mm=core, core_source=([str(p) for p in args.logs] if args.logs else 'Oct 4 sitting-4 logbook; override with --core / --logs'),
                observed_points=points, seed=args.seed,
                grid=dict(spacing_min_mm=args.spacing_min,spacing_max_mm=args.spacing_max,
                          taper_mm=args.taper,padding_mm=args.padding,xs_mm=xs,ys_mm=ys),
                cells=cells,schedule=schedule)


def preview(plan, path):
    xs,ys=plan['grid']['xs_mm'],plan['grid']['ys_mm']
    scale=min(720/max(xs[-1]-xs[0],1),650/max(ys[-1]-ys[0],1))
    shapes=[]
    for cell in plan['cells']:
        x,y,_=cell['target_mm']; px=75+(x-xs[0])*scale; py=740-(y-ys[0])*scale
        colour='#087ea4' if cell['dense'] else '#78909c'
        tip=html.escape('cell %d: (%g, %g) mm; %d throws'%(cell['cell_idx'],x,y,cell['repeats']))
        shapes.append(f'<circle cx="{px}" cy="{py}" r="5" fill="{colour}"><title>{tip}</title></circle>')
    info=f"{len(plan['cells'])} targets / {len(plan['schedule'])} throws; z = {plan['z_mm']:g} mm; seed {plan['seed']}"
    path.write_text('<!doctype html><meta charset="utf-8"><title>BB calibration grid</title>'
        '<style>body{font:17px system-ui;margin:30px;color:#19354b}svg{max-width:850px;width:100%}</style>'
        '<h1>BallButler local calibration</h1><p>'+info+'</p>'
        '<p>Blue: dense core. Grey: increasing spacing outward. Hover for coordinates and repeat counts.</p>'
        '<p>Schedule-frame X right / Y up; translated to mocap coordinates explicitly at execution.</p>'
        '<svg viewBox="0 0 850 800">'+''.join(shapes)+'</svg>',encoding='utf-8')


def validate_plan(plan):
    if plan.get('schema')!=SCHEMA or plan.get('frame')!='schedule_xy_world_z':
        raise ValueError('Unsupported plan schema/frame.')
    if not plan.get('schedule'):
        raise ValueError('Empty schedule.')
    ids=set()
    for entry in plan['schedule']:
        if entry['throw_idx'] in ids or len(entry['target_mm'])!=3 or not finite(entry['target_mm']):
            raise ValueError('Invalid or duplicate schedule entry.')
        ids.add(entry['throw_idx'])


def prepare_schedule(plan, position, yaw_offset, translation, solver, transform, signed_s):
    feasible, skipped = [], []
    for entry in plan['schedule']:
        x,y,z=entry['target_mm']
        world=[x+translation[0],y+translation[1],z]
        local=transform(*world, bb_position_mm=position, yaw_offset_rad=yaw_offset)
        try:
            sol=solver(*local,yaw_s_offset_mm=signed_s)
        except ValueError as error:
            skipped.append(dict(entry, reason=str(error)))
            continue
        feasible.append(dict(entry,target_global_mm=world,target_bb_local_mm=list(local),
                             solution=dict(yaw_rad=sol.yaw_rad,pitch_rad=sol.pitch_rad,
                                           speed_mps=sol.speed_mps,tof_s=sol.tof_s)))
    return feasible,skipped


def run(args):
    # Import only here: offline plan generation never needs ROS or numpy.
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
    from jugglebot_interfaces.action import BallButlerThrowCmd
    from jugglebot_interfaces.msg import BallButlerHeartbeat, BallButlerCalibrationResult, MocapDataMulti
    from std_msgs.msg import String
    from jugglebot.can.throw_ballistics import solve_throw_local, global_to_bb_local, bb_release_state
    from jugglebot import hardware_config as hw
    from jugglebot.protocol_config import BallButlerStates

    plan=json.loads(args.plan.read_text(encoding='utf-8')); validate_plan(plan)
    if not finite([*args.schedule_to_mocap,args.s,args.delay,args.pause,args.timeout,args.ground_z,args.refill_settle]) or args.s<=0 or not 1<=args.delay<=30 or args.pause<0 or args.timeout<10 or args.refill_settle<0:
        raise ValueError('Require finite arguments, positive s, delay 1..30 s, pause >=0, timeout >=10 s.')
    if args.limit is not None and args.limit<1:
        raise ValueError('--limit must be positive.')
    if args.refill_every<1:
        raise ValueError('--refill-every must be at least 1.')
    if not args.check_only and not sys.stdin.isatty():
        raise ValueError('Run in an interactive terminal: refill pauses require Enter (not redirected stdin).')
    stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
    out=args.out/stamp; out.mkdir(parents=True,exist_ok=False)
    rclpy.init()
    node=rclpy.create_node('bb_local_calibration')
    cache={'heartbeat':None,'hb_at':0.,'calibration':None,'mocap_at':0.,'mocap_count':0,'state':None,'state_at':0.,'mocap_max_gap':0.}
    def heartbeat(msg): cache.update(heartbeat=msg,hb_at=time.monotonic())
    def calibration(msg): cache['calibration']=msg
    def mocap(msg):
        now=time.monotonic()
        if cache['mocap_at']:
            cache['mocap_max_gap']=max(cache['mocap_max_gap'],now-cache['mocap_at'])
        cache.update(mocap_at=now,mocap_count=cache['mocap_count']+1)
    def state(msg): cache.update(state=msg.data,state_at=time.monotonic())
    latched=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(BallButlerHeartbeat,'/bb/heartbeat',heartbeat,20)
    node.create_subscription(BallButlerCalibrationResult,'/bb/calibration_result',calibration,latched)
    node.create_subscription(MocapDataMulti,'/mocap_data',mocap,qos_profile_sensor_data)
    node.create_subscription(String,'/orchestrator_state',state,10)
    events=node.create_publisher(String,'/bb/local_calibration/event',20)
    client=ActionClient(node,BallButlerThrowCmd,'/bb/throw')
    recorder=None; record_log=None; outstanding=False; last_flight_end=0.
    session=dict(schema=SCHEMA,created_utc=stamp,plan=plan,plan_sha256=hashlib.sha256(args.plan.read_bytes()).hexdigest(),
                 schedule_to_mocap_mm=args.schedule_to_mocap,signed_s_mm=args.s,affine_applied=False,
                 columns_feed_bias_applied=False,throws=[],refill_intervals=[],
                 refill_every=args.refill_every,ground_z_mm=args.ground_z,
                 status='preflight',bag=str(out/'bag'))
    def save(): write_json(out/'session.json',session)
    def emit(kind, **fields):
        event=dict(kind=kind,wall_time_ns=time.time_ns(),**fields)
        with (out/'events.jsonl').open('a',encoding='utf-8') as stream:
            stream.write(json.dumps(event,allow_nan=False)+'\n')
        events.publish(String(data=json.dumps(event,allow_nan=False)))
    def spin_until(predicate, timeout, description):
        end=time.monotonic()+timeout
        while rclpy.ok() and time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.05)
            if recorder is not None and recorder.poll() is not None:
                raise RuntimeError('Recorder exited; inspect recorder.log. No further throws dispatched.')
            value=predicate()
            if value: return value
        raise RuntimeError('Timed out: '+description)
    def live():
        return streams_ready(cache,time.monotonic())
    def ready():
        hb=cache['heartbeat']
        return live() and hb.ball_in_hand and hb.state in (int(BallButlerStates.IDLE),int(BallButlerStates.TRACKING))
    def operator_pause():
        interval=dict(start_wall_s=time.time(),end_wall_s=None)
        session['refill_intervals'].append(interval); save()
        emit('refill_start',interval=interval)
        print('\nREFILL: wait until the BB ball is on the ground, then return balls to the magazine.\n'
              'No new BB throw will be sent while paused. When ALL refill balls have landed and\n'
              'the area is clear, press Enter to continue (q + Enter to stop).',flush=True)
        def pump():
            if not rclpy.ok(): raise RuntimeError('ROS shut down during refill')
            rclpy.spin_once(node,timeout_sec=.05)
            if recorder.poll() is not None: raise RuntimeError('Recorder exited during refill')
        wait_for_operator(pump)
        # Exclusion interval includes a little settling time after acknowledgement.
        settle_until=time.monotonic()+args.refill_settle
        spin_until(lambda: time.monotonic()>=settle_until,args.refill_settle+2,'refill settling time')
        interval['end_wall_s']=time.time(); save(); emit('refill_end',interval=interval)
    try:
        save()
        spin_until(lambda: ready() and cache['calibration'] and cache['calibration'].success,
                   args.timeout,'fresh BB heartbeat, loaded ball, mocap, IDLE orchestrator, successful BB calibration')
        if not client.wait_for_server(timeout_sec=10): raise RuntimeError('/bb/throw action unavailable')
        cal=cache['calibration']
        position=[cal.position_mm.x,cal.position_mm.y,cal.position_mm.z]
        if not finite(position+[cal.yaw_offset_rad]): raise RuntimeError('Nonfinite BB pose')
        session['bb_pose']=dict(position_mm=position,yaw_offset_rad=cal.yaw_offset_rad,
                                yaw_offset_std_deg=cal.yaw_offset_std_deg,axis_tilt_deg=cal.axis_tilt_deg)
        session['solver_source']=sys.modules[solve_throw_local.__module__].__file__
        session['solver_sha256']=hashlib.sha256(Path(session['solver_source']).read_bytes()).hexdigest()
        session['hardware_constants']={k:v for k,v in vars(hw).items() if k.startswith('BB_') and isinstance(v,(str,int,float,bool))}
        feasible,skipped=prepare_schedule(plan,position,cal.yaw_offset_rad,args.schedule_to_mocap,
                                           solve_throw_local,global_to_bb_local,args.s)
        session.update(feasible_schedule=feasible,skipped_unreachable=skipped)
        print('%d feasible throws; %d skipped as unreachable. Output: %s'%(len(feasible),len(skipped),out),flush=True)
        if not feasible: raise RuntimeError('No reachable targets')
        # No resume of uncertain throws: pick a fresh session / independent block.
        selected=feasible[:args.limit] if args.limit else feasible
        for entry in selected:
            sol=entry['solution']
            r,v=bb_release_state(sol['yaw_rad'],sol['pitch_rad'],sol['speed_mps'],position,
                                 cal.yaw_offset_rad,yaw_s_offset_mm=args.s)
            entry['predicted_release_position_mm']=list(r)
            entry['predicted_release_velocity_mm_s']=list(v)
            entry['capture_duration_s']=flight_capture(r,v,sol['tof_s'],args.ground_z,
                                                      hw.GRAVITY_MPS2*1000.)
        session['selected_throw_indices']=[e['throw_idx'] for e in selected]
        save()
        if args.check_only:
            session['status']='checked_no_motion'; return out
        qos=out/'record_qos.yaml'
        qos.write_text('/bb/calibration_result:\n  reliability: reliable\n  durability: transient_local\n  history: keep_last\n  depth: 1\n/mocap_data:\n  reliability: best_effort\n  durability: volatile\n  history: keep_last\n  depth: 100\n',encoding='utf-8')
        cmd=['ros2','bag','record','-s','mcap','-o',str(out/'bag'),'--qos-profile-overrides-path',str(qos)]+TOPICS
        session['recorder_command']=cmd
        record_log=(out/'recorder.log').open('w',encoding='utf-8')
        recorder=subprocess.Popen(cmd,stdout=record_log,stderr=subprocess.STDOUT,start_new_session=True)
        started=time.monotonic()
        spin_until(lambda: time.monotonic()-started>3 and events.get_subscription_count()>0
                   and (out/'bag').exists(),15,'MCAP recorder and event subscription')
        session['status']='running'; save(); emit('session_start',bb_pose=session['bb_pose'],s_mm=args.s,affine=False)
        for selected_idx,entry in enumerate(selected):
            # An initial ready prompt and then explicit collection intervals.
            # No refill throws are allowed in the intervening automatic block.
            if refill_due(selected_idx,args.refill_every):
                operator_pause()
            spin_until(ready,args.timeout,'BB ready after reload, fresh mocap, and idle orchestrator')
            cache['mocap_max_gap']=0.
            latest=cache['calibration']
            if latest is None or not latest.success or [latest.position_mm.x,latest.position_mm.y,latest.position_mm.z]!=position or latest.yaw_offset_rad!=cal.yaw_offset_rad:
                raise RuntimeError('BB calibration changed during session; start a new session')
            sol=entry['solution']; goal=BallButlerThrowCmd.Goal()
            goal.yaw_angle_rad=sol['yaw_rad']; goal.pitch_angle_rad=sol['pitch_rad']; goal.throw_speed=sol['speed_mps']
            goal.throw_time=max(0.,args.delay-hw.BB_OP_THROW_RELEASE_LATENCY_MS/1000.)
            goal.suppress_announcement=True
            row=dict(entry,status='dispatching',dispatch_wall_time_ns=time.time_ns(),
                     commanded_delay_s=goal.throw_time,nominal_release_wall_s=time.time()+args.delay,
                     mocap_count_before=cache['mocap_count'])
            session['throws'].append(row); save()
            # A timeout is ambiguous, never retry it automatically.
            outstanding=True
            # Timestamp immediately before action dispatch, after checkpoint I/O.
            dispatched=time.time()
            row['dispatch_wall_time_ns']=int(dispatched*1e9)
            row['nominal_release_wall_s']=dispatched+args.delay
            row['analysis_window_wall_s']=[dispatched+max(0,args.delay-.25),
                                           dispatched+args.delay+entry['capture_duration_s']]
            last_flight_end=time.monotonic()+args.delay+entry['capture_duration_s']+args.pause
            future=client.send_goal_async(goal)
            save(); emit('dispatch',throw=row)
            print('BB THROW %d/%d: do not throw refill balls until REFILL is printed.'
                  %(selected_idx+1,len(selected)),flush=True)
            spin_until(future.done,10,'action goal response')
            handle=future.result()
            if not handle.accepted:
                row['status']='rejected'; outstanding=False; save()
                raise RuntimeError('Bridge rejected throw; stopped without retry')
            row['goal_uuid']=goal_uuid_bytes(handle.goal_id); row['status']='accepted'; save()
            result_future=handle.get_result_async()
            spin_until(result_future.done,args.delay+15,'firmware terminal throw outcome')
            response=result_future.result(); result=response.result
            outstanding=False
            row.update(status='released' if result.success else 'failed',action_status=int(response.status),
                       outcome=int(result.outcome),message=result.message,detail0=int(result.detail0),detail1=int(result.detail1),
                       result_wall_time_ns=time.time_ns())
            save(); emit('result',throw=row)
            if not result.success: raise RuntimeError('Firmware refused/aborted throw: '+result.message)
            spin_until(lambda: time.monotonic()>=last_flight_end,args.delay+entry['capture_duration_s']+args.pause+2,'flight observation window')
            if not live(): raise RuntimeError('Lost fresh heartbeat/mocap or orchestrator left IDLE; stopped')
            row['mocap_count_after']=cache['mocap_count']
            row['mocap_max_receive_gap_s']=cache['mocap_max_gap']; save()
            if cache['mocap_max_gap']>0.1:
                raise RuntimeError('Mocap receive gap exceeded 100 ms during throw; capture needs review')
            row['capture_complete']=True
            save(); emit('capture_complete',throw=row)
            print('Throw %d / %d: cell %d, firmware OK'%(len(session['throws']),len(selected),entry['cell_idx']),flush=True)
        session['status']='completed_subset' if len(selected)<len(feasible) else 'completed'
    except (Exception,KeyboardInterrupt) as error:
        session.update(status='interrupted' if isinstance(error,KeyboardInterrupt) else 'failed',error=str(error),
                       outstanding_action_uncertain=outstanding)
        if outstanding and session['throws']:
            session['throws'][-1]['status']='unknown_do_not_retry'
        print('Stopped: %s. Metadata retained in %s'%(error,out),file=sys.stderr)
    finally:
        # Already dispatched throws cannot be cancelled by the bridge. Keep
        # recording their full flight even on Ctrl-C, then close MCAP cleanly.
        if recorder is not None:
            print('Finishing flight capture and closing the bag...',flush=True)
            while time.monotonic()<last_flight_end+1 and recorder.poll() is None:
                try:
                    if rclpy.ok(): rclpy.spin_once(node,timeout_sec=.1)
                    else: time.sleep(.1)
                except KeyboardInterrupt: continue
                except Exception: time.sleep(.1)
            if rclpy.ok(): emit('session_end',status=session['status'])
            session['recorder_sigint_requested']=False
            if recorder.poll() is None:
                session['recorder_sigint_requested']=True
                recorder.send_signal(signal.SIGINT)
                try: recorder.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    session['recorder_close_error']='Recorder did not finalize within 20 seconds; inspect process and bag.'
            session['recorder_exit_code']=recorder.poll()
            metadata_valid=False
            try:
                import yaml
                metadata=yaml.safe_load((out/'bag'/'metadata.yaml').read_text(encoding='utf-8'))
                session['recorded_message_counts']=check_recorded_topics(metadata)
                metadata_valid=True
            except Exception as error:
                session.update(recording_needs_review=True,recording_validation_error=str(error))
            session['recording_needs_review']=not recorder_shutdown_ok(
                session['recorder_exit_code'],session['recorder_sigint_requested'],
                metadata_valid,'recorder_close_error' in session)
        save()
        if record_log: record_log.close()
        client.destroy()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
    if session['status'].startswith('completed') and not session.get('recording_needs_review'):
        return out
    raise RuntimeError('Session incomplete; read '+str(out/'session.json'))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    commands=parser.add_subparsers(dest='command',required=True)
    p=commands.add_parser('plan',help='offline: write randomized schedule and HTML grid preview')
    p.add_argument('--out',type=Path,default=Path('local_calibration_plan.json'))
    p.add_argument('--logs',type=Path,nargs='+',default=[])
    p.add_argument('--core',type=float,nargs=4,default=DEFAULT_CORE,metavar=('XMIN','XMAX','YMIN','YMAX'))
    p.add_argument('--z',type=float,default=830.)
    p.add_argument('--spacing-min',type=float,default=50.)
    p.add_argument('--spacing-max',type=float,default=200.)
    p.add_argument('--taper',type=float,default=200.,help='distance outside dense core at which spacing reaches maximum')
    p.add_argument('--padding',type=float,default=500.,help='maximum extension beyond rounded dense core, each direction')
    p.add_argument('--repeats',type=int,default=2)
    p.add_argument('--core-repeats',type=int,default=5)
    p.add_argument('--seed',type=int,default=42)
    p=commands.add_parser('run',help='Jetson: solve corrected geometry, await throws, record raw MCAP')
    p.add_argument('plan',type=Path)
    p.add_argument('--out',type=Path,default=Path.home()/'bb_calibration_sessions')
    p.add_argument('--schedule-to-mocap',type=float,nargs=2,required=True,metavar=('DX','DY'),help='explicit translation added to schedule XY, mm; use measured frame-check offset')
    p.add_argument('--s',type=float,default=105.65,help='corrected signed lateral offset in production solver convention')
    p.add_argument('--delay',type=float,default=3.)
    p.add_argument('--pause',type=float,default=1.,help='observation time after predicted landing before another command')
    p.add_argument('--refill-every',type=int,default=DEFAULT_REFILL_EVERY,help='pause for refill and Enter after N throws (default: 9); never refill during an automatic block')
    p.add_argument('--refill-settle',type=float,default=1.,help='additional settling seconds after Enter')
    p.add_argument('--ground-z',type=float,default=0.,help='ground height in QTM world mm, for minimum wait before REFILL prompt')
    p.add_argument('--timeout',type=float,default=60.,help='maximum wait for preflight/reload, seconds')
    p.add_argument('--limit',type=int,help='pilot batch: execute at most N feasible entries')
    p.add_argument('--check-only',action='store_true',help='live preflight and reachability only; no throws or recorder')
    args=parser.parse_args()
    try:
        if args.command=='plan':
            plan=make_plan(args); args.out.parent.mkdir(parents=True,exist_ok=True)
            write_json(args.out,plan); preview(plan,args.out.with_suffix('.html'))
            print('%d targets, %d throws. Plan: %s; preview: %s'%(len(plan['cells']),len(plan['schedule']),args.out,args.out.with_suffix('.html')))
        else:
            print('Saved session: '+str(run(args)))
    except (ValueError,RuntimeError) as error:
        parser.exit(1,str(error)+'\n')


if __name__=='__main__':
    main()

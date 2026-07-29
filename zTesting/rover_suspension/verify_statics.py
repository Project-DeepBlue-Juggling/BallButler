#!/usr/bin/env python3
"""Verify the parallelogram-suspension statics before porting to JS.

Model (side view, x right, y up):
  A_low=(0,0), A_up=(0,h) on the chassis wall; bars of length L at angle
  theta above horizontal; plate pivots B_low, B_up; spring pinned at P1
  (upper bar, x1 along bar) and P2 (lower bar, x2 along bar).
  Ground pushes up on the wheel with N at horizontal offset w_off from the
  plate pivots. Massless bars/plate, pin joints, quasi-static.

Unknowns u = [RAux,RAuy, RAlx,RAly, RBux,RBuy, RBlx,RBly, T]
  RAu/RAl: wall->bar forces at A_up/A_low
  RBu/RBl: plate->bar forces at B_up/B_low
  T: spring tension (T>0 pulls pins together; T<0 = spring in compression)

Checks:
  1. T from the 9x9 solve == T from virtual work: T = -N*L*s/(h*(x2-x1))
  2. T independent of w_off
  3. Wall reactions sum to -(0,N)  (system equilibrium)
  4. Moment balance of whole system about A_low
"""
import math, random

def cross(a, b):
    return a[0]*b[1] - a[1]*b[0]

def solve9(L, h, x1, x2, theta, N, w_off):
    c, s_ = math.cos(theta), math.sin(theta)
    P1 = (x1*c, h + x1*s_)
    P2 = (x2*c, s_*x2)
    ds = (P1[0]-P2[0], P1[1]-P2[1])
    slen = math.hypot(*ds)
    es = (ds[0]/slen, ds[1]/slen)          # unit vector P2 -> P1

    A = [[0.0]*9 for _ in range(9)]
    b = [0.0]*9
    # Upper bar: RAu + RBu - T*es = 0 ; M about A_up
    A[0][0] = 1; A[0][4] = 1; A[0][8] = -es[0]
    A[1][1] = 1; A[1][5] = 1; A[1][8] = -es[1]
    # cross((Lc,Ls), RBu) + cross((x1c,x1s), -T*es) = 0
    A[2][4] = -L*s_; A[2][5] = L*c
    A[2][8] = -(x1*c*es[1] - x1*s_*es[0])
    # Lower bar: RAl + RBl + T*es = 0 ; M about A_low
    A[3][2] = 1; A[3][6] = 1; A[3][8] = es[0]
    A[4][3] = 1; A[4][7] = 1; A[4][8] = es[1]
    A[5][6] = -L*s_; A[5][7] = L*c
    A[5][8] = (x2*c*es[1] - x2*s_*es[0])
    # Plate: -RBu - RBl + (0,N) = 0 ; M about B_low:
    # cross((0,h), -RBu) + w_off*N = 0  ->  h*RBux = -w_off*N
    A[6][4] = -1; A[6][6] = -1
    A[7][5] = -1; A[7][7] = -1; b[7] = -N
    A[8][4] = h;  b[8] = -w_off*N

    # Gaussian elimination with partial pivoting
    M = [row[:] + [b[i]] for i, row in enumerate(A)]
    n = 9
    for col in range(n):
        p = max(range(col, n), key=lambda r: abs(M[r][col]))
        if abs(M[p][col]) < 1e-12:
            raise ValueError("singular")
        M[col], M[p] = M[p], M[col]
        for r in range(n):
            if r != col and M[r][col] != 0:
                f = M[r][col] / M[col][col]
                for k in range(col, n+1):
                    M[r][k] -= f * M[col][k]
    u = [M[i][9] / M[i][i] for i in range(n)]
    return u, slen

def t_virtual_work(L, h, x1, x2, theta, N):
    c = math.cos(theta)
    s_ = math.sin(theta)
    d = x2 - x1
    slen = math.sqrt(d*d + h*h - 2*h*d*s_)
    return -N * L * slen / (h * d), slen

random.seed(1)
worst = 0.0
for trial in range(2000):
    L   = random.uniform(100, 300)
    h   = random.uniform(30, 200)
    x1  = random.uniform(5, L-5)
    x2  = random.uniform(5, L-5)
    if abs(x2-x1) < 2:
        continue
    theta = math.radians(random.uniform(-50, 50))
    N    = random.uniform(10, 1000)
    woff = random.uniform(-100, 100)

    u, slen = solve9(L, h, x1, x2, theta, N, woff)
    T_vw, slen2 = t_virtual_work(L, h, x1, x2, theta, N)
    assert abs(slen - slen2) < 1e-9 * max(1, slen)

    scale = max(1.0, abs(T_vw))
    err = abs(u[8] - T_vw) / scale
    worst = max(worst, err)
    assert err < 1e-8, (trial, u[8], T_vw)

    # T independent of w_off
    u0, _ = solve9(L, h, x1, x2, theta, N, 0.0)
    assert abs(u0[8] - u[8]) < 1e-8 * scale

    # wall reactions sum to -(0,N)
    assert abs(u[0] + u[2]) < 1e-8 * max(1, N)
    assert abs(u[1] + u[3] + N) < 1e-8 * max(1, N)

    # whole-system moment about A_low:
    # cross(A_up, RAu) + cross(contact, (0,N)) = 0, contact_x = L cos + w_off
    c = math.cos(theta)
    m = cross((0, h), (u[0], u[1])) + (L*c + woff) * N
    assert abs(m) < 1e-6 * max(1, N*L), (trial, m)

print("2000 randomized trials passed; worst T mismatch (rel):", worst)

# Print the default-case numbers for cross-checking against the JS port
L, h, x1, x2, m_kg, G = 175.0, 80.0, 60.0, 140.0, 35.0, 3.0
N = m_kg * 9.81 / 4 * G
for deg in (-13.2, 0.0, 34.8):
    th = math.radians(deg)
    u, slen = solve9(L, h, x1, x2, th, N, 0.0)
    mags = [math.hypot(u[i], u[i+1]) for i in (0, 2, 4, 6)]
    print("theta=%6.1f  s=%8.3f  T=%9.2f  |RAu|=%8.2f |RAl|=%8.2f |RBu|=%8.2f |RBl|=%8.2f"
          % (deg, slen, u[8], *mags))

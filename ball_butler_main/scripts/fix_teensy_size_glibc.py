"""
fix_teensy_size_glibc.py

PlatformIO extra-script that lets the Teensy 4.x build complete on hosts
whose glibc is older than 2.34 (e.g. Ubuntu 20.04 / Jetson Linux).

Symptom without this script:
    teensy_size: /lib/.../libc.so.6: version `GLIBC_2.34' not found
    *** [checkprogsize] Explicit exit, status 1

`target_firm` (the .hex used by `pio run -t upload`) depends on
`checkprogsize`, so the upload payload is never built and flashing fails
on these hosts.

Root cause: the `teensy_size` binary bundled with PIO's `tool-teensy`
package is linked against glibc 2.34+. PIO's `platform-teensy` invokes
it twice via `PATH`-resolved subprocess calls (see
`.platformio/platforms/teensy/builder/main.py`):

  * `exec_command(["teensy_size", ...])` from `CheckUploadSize`
    (i.e. the `checkprogsize` action)
  * the `size` target's `SIZEPRINTCMD = "teensy_size $SOURCES"`

Fix: prepend `scripts/bin/` to `env["ENV"]["PATH"]`. That directory
contains a drop-in `teensy_size` shell wrapper that just delegates to
`arm-none-eabi-size -A` from the bundled toolchain. PIO finds our
wrapper before the broken binary; the build proceeds; the .hex is
produced; `pio run -t upload` works.

We tried overriding `CheckUploadSize` with `env.AddMethod` first —
that turned out to *append* to the method dispatch list rather than
replace, so both the new and original methods run and the original
still `env.Exit(1)`s. The PATH approach sidesteps that entirely.

Wired into `platformio.ini` via:
    extra_scripts = post:scripts/fix_teensy_size_glibc.py
"""

from os.path import join

Import("env")  # noqa: F821  (provided by SCons at script-load time)

# SCons executes extra_scripts via `exec()`, so `__file__` is not
# defined. PIO does expose `PROJECT_DIR` (= the dir containing
# platformio.ini), which is what we need to locate our wrapper.
_WRAPPER_DIR = join(env["PROJECT_DIR"], "scripts", "bin")

# Prepend our wrapper dir so the platform's PATH-resolved
# `teensy_size` lookups find it first.
env.PrependENVPath("PATH", _WRAPPER_DIR)

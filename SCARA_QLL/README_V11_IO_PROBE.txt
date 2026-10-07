RHO4 V11 - I/O + FAST PROBE (stage 1)

Controller motion backend (HYRING) is unchanged.

New protocol
------------
CMD34  read-only I/O status
CMD35  ARM/DISARM physical output writes
CMD36  spindle digital output
CMD38  start fast-probe LINEAR MOVE UNTIL
CMD39  read probe result

Probe input
-----------
rho4.0 uses fast inputs 611..618 for rapid measurement.
This bundle uses INPUT BINARY 611 and trigger condition = 1.

Candidate spindle output
------------------------
OUTPUT BINARY channel 1.

The original robot EA.INC called output 1 "Greifer_zu".
Therefore treat it only as a candidate line to repurpose.
Do not ARM until that physical wire is understood/disconnected
from any old actuator.

Software lock
-------------
PERMPROG starts with IO_ARMED=0 and does not write output 1.
CMD36 cannot change it before explicit CMD35 ARM=1.
ARM=1 first drives the output LOW.

CMD15 turns spindle OFF when I/O is armed.
This is only a software action, not a safety-rated spindle stop.
A real spindle must also be interrupted by the physical E-stop/
contactor chain.

Probe
-----
PROBE_RESULT:
  0 pending/idle
  1 triggered, @MPOS captured and converted with WC(@MPOS)
  2 target reached without trigger

MOVE_ERROR 29010 = probe did not trigger before target.

Compile order
-------------
1  PERMPROG
2  PCMOVE
3  PCLINEAR
4  PCCIRC
5  PCSMOOTH
6  PCPATH
7  PCBUFA
8  PCBUFB
9  PCENDA
10 PCENDB
11 PCSTREAM
12 PCRING
13 PCHYB
14 HYBUFA
15 HYBUFB
16 HYENDA
17 HYENDB
18 HYRING
19 PCPROBE

FIRST TEST:
Do not ARM I/O and do not start probe motion.
Compile, then run the Python read-only link/status test.


V8a IMPORTANT FIX
-----------------
On rho4.0 the same physical fast input has different channel numbers
depending on the access mode:

  611..618  = fast measurement / MOVE ... UNTIL
  801..816  = normal BAPS interrogation of those fast inputs

Therefore:
  PERMPROG reads physical input #1 via channel 801.
  PCPROBE still uses channel 611 inside MOVE LINEAR ... UNTIL.

The previous V8 bundle incorrectly read 611 in CMD34/CMD38/CMD39,
which can cause the PERMPROG runtime error when CMD34 is called.


V8b PCPROBE FIX
---------------
PCPROBE now follows the Bosch rho4 rapid-measuring example literally:

  V=PROBE_SPEED
  MOVE LINEAR UNTIL PROBE_FAST=1
      ERROR JUMP NO_HIT
  TO TARGET

The previous local-speed form:
  MOVE LINEAR WITH V=... UNTIL ...
compiled, but is no longer used while diagnosing the old rho4 runtime.

Also removed HALT at the end of PCPROBE so the external process terminates
normally and CONDITION(PCPROBE) can return -1.

PUBLIC/EXTERNAL layout did not change.
Therefore only PCPROBE needs recompiling for this V8b change.

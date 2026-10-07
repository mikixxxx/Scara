RHO4 PCRING V5 - CONTINUOUS A/B RING STREAM

Goal
----
Remove the 32-buffer / 512-point PCSTREAM batch limit.

Old PCSTREAM is kept intact for regression/compatibility.
New motion process: PCRING

New protocol
------------
CMD26 upload ring slot:
  int32 26
  int32 slot       0=A / 1=B
  int32 sequence
  int32 last       0/1
  float32 speed
  64 x float32 XYZR

CMD27 start PCRING:
  no extra payload
  reply 10/-10/-11/-12/-24

CMD28 status, 11 int32:
  A_FREE, B_FREE, A_SEQ, B_SEQ,
  A_LAST, B_LAST, STR_EXPECT, STR_UNDER,
  MOVE_STATE, MOVE_ERROR, PCRING_CONDITION

CMD25 clears both old PCSTREAM and PCRING metadata when neither is active.
CMD15 stops PCRING as a controlled software stop.

Controller-side underrun protection
-----------------------------------
Before PCRING reuses A or B, it verifies:
  FREE == 0
  SEQ == STR_EXPECT

If Python misses the refill deadline:
  STR_UNDER=1
  MOVE_STATE=3
  MOVE_ERROR=29001
and stale slot data is NOT intentionally reused.

IMPORTANT EXPERIMENTAL POINT
----------------------------
PCRING contains IF/JUMP between external motion blocks while PROGR_SLOPE is
active. The current robot already proved assignments and external PCBUFA/B
handoffs can remain smooth, but this loop/branch form is NOT physically
validated yet. Compile first. Then use the small staged ring test before any
long G-code.

Safe compile order after PUBLIC layout change
---------------------------------------------
1 PERMPROG
2 PCMOVE
3 PCLINEAR
4 PCCIRC
5 PCSMOOTH
6 PCPATH
7 PCBUFA
8 PCBUFB
9 PCENDA
10 PCENDB
11 PCSTREAM
12 PCRING

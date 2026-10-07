RHO4 V9 - NATIVE MIXED-PRIMITIVE A/B RING

New process
-----------
HYRING

Primitive types
---------------
1 = native MOVE LINEAR
2 = native MOVE CIRCULAR

Each A/B slot contains up to 16 primitives.
Each primitive has its own speed.
CIRCULAR has MID + END.
LINEAR ignores MID.

Protocol
--------
CMD30 upload slot:
  int32 command=30
  int32 slot
  int32 sequence
  int32 last
  int32 count
  repeated 16x:
    int32 type
    float32 speed
    4x float32 END
    4x float32 MID

CMD31 start HYRING
CMD32 status (13 int32)
CMD33 clear HYRING

Controller underrun:
  MOVE_ERROR = 29002

Important
---------
PCHYB physical comparison already showed runtime IF/ELSE dispatch
was effectively identical to fixed PCSMOOTH on the real robot:
PCSMOOTH 12.040 s
PCHYB    12.002 s
ratio    0.997

This makes per-primitive LINEAR/CIRCULAR dispatch a reasonable V9
experiment, but HYRING itself still needs compile + staged physical test.

Compile order
-------------
PUBLIC COMMON changes again, so compile exporters before all dependents:

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

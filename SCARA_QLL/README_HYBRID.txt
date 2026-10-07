RHO4 V9 HYBRID MOTION EXPERIMENT

Purpose
-------
PCSMOOTH already proved a FIXED sequence:
  LINEAR -> CIRCULAR -> LINEAR -> CIRCULAR
can run smoothly under PROGR_SLOPE on this robot.

PCHYB tests the missing requirement for a future arbitrary hybrid ring:
runtime dispatch of each primitive using IF/ELSE.

PCHYB geometry is intentionally identical to PCSMOOTH:
  rounded rectangle
  width 20 mm
  height 12 mm
  radius 3 mm
  speed 5 mm/s
  END == START

CMD29 starts PCHYB.

Why this test matters
---------------------
The Bosch BAPS3 manual states that IF/THEN/ELSE can interrupt a coherent
PROGR_SLOPE movement. Current PCRING nevertheless looked physically smooth
with IF/JUMP around whole 16-MOVE buffers. PCHYB checks the stricter case:
one dynamic IF dispatch per actual motion primitive.

Interpretation
--------------
If PCHYB is as smooth as PCSMOOTH:
  -> a direct mixed-primitive ring becomes plausible.

If PCHYB visibly slows/stops at each primitive:
  -> do NOT build per-primitive IF dispatch.
  -> use homogeneous LINEAR/CIRCULAR chunks and external subroutine
     transitions, or another branch-minimizing architecture.

Compile order
-------------
Because PERMPROG changes its EXTERNAL/common dependency graph, compile:

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

No PUBLIC variables were added in this experiment.

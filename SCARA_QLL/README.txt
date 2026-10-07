PCSTREAM compile prototype
==========================

Purpose
-------
Compile-only first proof for a real RHO4 two-slot A/B motion architecture.

Important
---------
DO NOT run PCSTREAM on the real robot yet.
This version is intentionally for compiler validation only.

Key design:
- PERMPROG owns two PUBLIC ARRAY[1..16] POINT buffers A/B.
- PCBUFA / PCBUFB execute 16 VIA moves directly from those POINT arrays.
- PCENDA / PCENDB execute 15 VIA + final TO.
- PCSTREAM statically unrolls 1..8 buffer chains.
- Branching happens before motion starts.
- Between external buffer calls there is only one PUBLIC FREE assignment.

Compile order
-------------
1. PERMPROG.QLL
2. PCMOVE.QLL
3. PCLINEAR.QLL
4. PCCIRC.QLL
5. PCSMOOTH.QLL
6. PCPATH.QLL
7. PCBUFA.QLL
8. PCBUFB.QLL
9. PCENDA.QLL
10. PCENDB.QLL
11. PCSTREAM.QLL

The exporter PERMPROG must be compiled before every importer because the
PUBLIC COMMON layout changed.

What we want to learn from the PC compiler
------------------------------------------
1. Does PUBLIC ARRAY[1..16] POINT compile?
2. Can an importing program MOVE directly to EXTERNAL POINT-array elements?
3. Do the external main-program calls PCBUFA/PCBUFB/PCENDA/PCENDB compile
   inside PCSTREAM under PROGR_SLOPE?
4. Does ROPS accept the fixed static branch/unroll structure?

Protocol draft
--------------
CMD22 payload:
  int32 slot (0=A, 1=B)
  int32 sequence
  float32 speed_mm_s
  64 * float32 (16 * XYZR)

CMD23 payload:
  int32 STREAM_BLOCKS (1..8)

CMD24 reply:
  6 * int32:
  A_FREE, B_FREE, A_SEQ, B_SEQ, STREAM_BLOCKS, PROCSTATUS

No physical test until compiler is clean and the refill/underrun behavior
is reviewed again.


Compiler V3.14 fix
------------------
BAPS 3.14 on the user's PC reports a 12-character identifier limit.
All new stream identifiers were shortened accordingly:

STREAM_A_DATA   -> SA_DATA
STREAM_B_DATA   -> SB_DATA
STREAM_A_SPEED  -> SA_SPEED
STREAM_B_SPEED  -> SB_SPEED
STREAM_A_SEQ    -> SA_SEQ
STREAM_B_SEQ    -> SB_SEQ
STREAM_A_FREE   -> SA_FREE
STREAM_B_FREE   -> SB_FREE
STREAM_BLOCKS   -> STR_BLOCKS
STREAM_ACCEPT   -> STR_ACCEPT
STREAM_DUMMY    -> STR_DUMMY
STREAM_SLOT     -> STR_SLOT
STREAM_SEQ      -> STR_SEQ

STREAMSTATUS is exactly 12 characters and was left unchanged.


V3 safety/recovery addition
---------------------------
CMD25 resets the stream slot state without starting motion:
  SA_FREE=1
  SB_FREE=1
  SA_SEQ=-1
  SB_SEQ=-1
  STR_BLOCKS=0

CMD25 returns -11 while PCSTREAM is still active.
This makes an aborted/preloaded stream recoverable without restarting PERMPROG.

IMPORTANT: PERMPROG was changed. Compile PERMPROG first and then recompile
all programs that import its PUBLIC COMMON data, in the same order as before.


V4 LONG STREAM
--------------
PCSTREAM was statically extended from 8 to 32 buffers.
That is 512 LINEAR endpoint blocks in one PROGR_SLOPE chain.

No PUBLIC COMMON fields were added or removed, but compile in the safe order:
1. PERMPROG.QLL
2. PCMOVE.QLL
3. PCLINEAR.QLL
4. PCCIRC.QLL
5. PCSMOOTH.QLL
6. PCPATH.QLL
7. PCBUFA.QLL
8. PCBUFB.QLL
9. PCENDA.QLL
10. PCENDB.QLL
11. PCSTREAM.QLL

First physical long-stream proof should use 12 buffers / 192 points on the
already tested small circle envelope. Do not jump directly to a large path.

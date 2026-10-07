%
O9000
( RHO4 FINAL SAFE MOTION TEST )
( Work zero = robot START )
( No spindle, no probe, no negative Z )
( 10 x 6 mm rounded rectangle at Z +2 mm )

G21 G17 G90 G94 G54
G40 G49 G80
G91.1
G64

G0 Z2
G0 X1 Y0

G1 X9 Y0 F120
G3 X10 Y1 I0 J1
G1 X10 Y5
G3 X9 Y6 I-1 J0
G1 X1 Y6
G3 X0 Y5 I0 J-1
G1 X0 Y1
G3 X1 Y0 I1 J0

G0 X0 Y0
G0 Z0
M30
%

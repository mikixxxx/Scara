; PCPATH V3 demo - LARGE
; Rounded rectangle 60 x 40 mm, corner radius 5 mm
; Relative coordinates, constant Z/R
; Feed 300 mm/min = 5 mm/s
;
; With V3 defaults (arc tolerance 0.20 mm, max angle 30 deg)
; each R5 quarter arc becomes 3 LINEAR pieces.
; Total = 4 straight endpoints + 4*3 arc endpoints = 16 PCPATH points.
; So the complete contour still fits into ONE controller buffer.

G91
G17
F300

G1 X50 Y0
G3 X5 Y5 I0 J5
G1 X0 Y30
G3 X-5 Y5 I-5 J0
G1 X-50 Y0
G3 X-5 Y-5 I0 J-5
G1 X0 Y-30
G3 X5 Y-5 I5 J0

M30


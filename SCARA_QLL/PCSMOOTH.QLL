;;CONTROL = rho4
;;INT = LINEAR
;;KINEMATICS: (1=ROBI1)
;;ROBI1.JC_NAMES = A1,A2,A3,A4
;;ROBI1.WC_NAMES = K1,K2,K3,K4

PROGRAM PCSMOOTH

EXTERNAL PERMPROG: MOVE_STATE

POINT: STARTPOS
POINT: P1
POINT: M1
POINT: P2
POINT: P3
POINT: M2
POINT: P4
POINT: P5
POINT: M3
POINT: P6
POINT: P7
POINT: M4
POINT: P8

BEGIN

; ============================================================
; TEST PLYNULE DRAHY
;
; Za aktualni polohy vytvori zaobleny obdelnik:
; sirka 20 mm, vyska 12 mm, radius 3 mm.
; Rychlost 5 mm/s.
; Na konci se vrati presne do STARTPOS.
; ============================================================

STARTPOS=POS

; Dolni rovny usek
P1=STARTPOS
P1.K1=P1.K1+14.0

; Pravy dolni roh r=3
M1=STARTPOS
M1.K1=M1.K1+16.12132034
M1.K2=M1.K2+0.87867966

P2=STARTPOS
P2.K1=P2.K1+17.0
P2.K2=P2.K2+3.0

; Pravy svisly usek
P3=STARTPOS
P3.K1=P3.K1+17.0
P3.K2=P3.K2+9.0

; Pravy horni roh
M2=STARTPOS
M2.K1=M2.K1+16.12132034
M2.K2=M2.K2+11.12132034

P4=STARTPOS
P4.K1=P4.K1+14.0
P4.K2=P4.K2+12.0

; Horni rovny usek
P5=STARTPOS
P5.K2=P5.K2+12.0

; Levy horni roh
M3=STARTPOS
M3.K1=M3.K1-2.12132034
M3.K2=M3.K2+11.12132034

P6=STARTPOS
P6.K1=P6.K1-3.0
P6.K2=P6.K2+9.0

; Levy svisly usek
P7=STARTPOS
P7.K1=P7.K1-3.0
P7.K2=P7.K2+3.0

; Levy dolni roh
M4=STARTPOS
M4.K1=M4.K1-2.12132034
M4.K2=M4.K2+0.87867966

P8=STARTPOS

; ============================================================
; Souvisla pohybova sekvence.
; VIA = bez mezizastaveni.
; Posledni TO = rizene dobrzdeni na V=0.
; ============================================================

PROGR_SLOPE

MOVE LINEAR WITH V=5.0 VIA P1
MOVE CIRCULAR WITH V=5.0 VIA (M1,P2)
MOVE LINEAR WITH V=5.0 VIA P3
MOVE CIRCULAR WITH V=5.0 VIA (M2,P4)
MOVE LINEAR WITH V=5.0 VIA P5
MOVE CIRCULAR WITH V=5.0 VIA (M3,P6)
MOVE LINEAR WITH V=5.0 VIA P7
MOVE CIRCULAR WITH V=5.0 TO (M4,P8)

BLOCK_SLOPE

MOVE_STATE=2

PROGRAM_END

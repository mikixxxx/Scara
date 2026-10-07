;;CONTROL = rho4
;;SER_IO_STOP-
;;KINEMATICS: (1=ROBI1)
;;ROBI1.JC_NAMES = A1,A2,A3,A4
;;ROBI1.WC_NAMES = K1,K2,K3,K4
;;PROCESS_KIND=PERMANENT


PROGRAM PERMPROG


; ============================================================
; DATA SDILENA S PCMOVE
; ============================================================

PUBLIC REAL: TARGET_X
PUBLIC REAL: TARGET_Y
PUBLIC REAL: TARGET_Z
PUBLIC REAL: TARGET_R
PUBLIC REAL: MOVE_SPEED
PUBLIC REAL: LINEAR_SPEED

PUBLIC REAL: CIRC_MID_X
PUBLIC REAL: CIRC_MID_Y
PUBLIC REAL: CIRC_MID_Z
PUBLIC REAL: CIRC_MID_R

PUBLIC REAL: CIRC_END_X
PUBLIC REAL: CIRC_END_Y
PUBLIC REAL: CIRC_END_Z
PUBLIC REAL: CIRC_END_R

PUBLIC REAL: CIRC_SPEED

; MOVE_STATE:
; 0 = IDLE
; 1 = motion proces spusten / pohyb
; 2 = motion proces dosel za prikaz MOVE
; 3 = RC ERROR
; 4 = STOPPED
PUBLIC INTEGER: MOVE_STATE

; Kod RC chyby aktualniho motion procesu
; 0 = bez chyby
PUBLIC INTEGER: MOVE_ERROR

; ============================================================
; PCPATH BUFFER V1
; ============================================================
; PATH_COUNT = 1..16
; PATH_SPEED = mm/s
; PATHDATA = 16 x (X,Y,Z,R), absolutni souradnice
PUBLIC INTEGER: PATH_COUNT
PUBLIC REAL: PATH_SPEED
PUBLIC ARRAY [1..64] REAL: PATHDATA

; ============================================================
; PCSTREAM DOUBLE BUFFER - COMPILE PROTOTYPE
; ============================================================
; Fixed 16 POINTs per slot for the first real-RHO proof.
; A/B data are POINT arrays so PCBUFA/PCBUFB can MOVE directly
; without copying 64 REAL values immediately before motion.
PUBLIC ARRAY [1..16] POINT: SA_DATA
PUBLIC ARRAY [1..16] POINT: SB_DATA

PUBLIC REAL: SA_SPEED
PUBLIC REAL: SB_SPEED

PUBLIC INTEGER: SA_SEQ
PUBLIC INTEGER: SB_SEQ

; 1 = PC may write this slot, 0 = reserved/ready/in use
PUBLIC INTEGER: SA_FREE
PUBLIC INTEGER: SB_FREE

; Total number of 16-point buffers for the current stream.
; First prototype supports 1..8 buffers.
PUBLIC INTEGER: STR_BLOCKS

; ============================================================
; PCRING - CONTINUOUS / PRACTICALLY UNLIMITED A/B STREAM
; ============================================================
; LAST marks the final 16-point slot of the complete job.
; EXPECT is the sequence number PCRING expects next.
; UNDER=1 means controller detected missing/stale A/B refill.
PUBLIC INTEGER: SA_LAST
PUBLIC INTEGER: SB_LAST
PUBLIC INTEGER: STR_EXPECT
PUBLIC INTEGER: STR_UNDER

; ============================================================
; V9 HYBRID RING - NATIVE LINEAR + CIRCULAR
; ============================================================
; Primitive type:
;   1 = LINEAR
;   2 = CIRCULAR
;
; Each A/B slot contains up to 16 motion primitives.
; Non-final slots must contain exactly 16 primitives.
; Final slot may contain 1..16 primitives.
;
; END = endpoint for LINEAR/CIRCULAR
; MID = midpoint for CIRCULAR (ignored for LINEAR)
; V   = speed mm/s for each primitive
PUBLIC ARRAY [1..16] POINT: HA_END
PUBLIC ARRAY [1..16] POINT: HB_END
PUBLIC ARRAY [1..16] POINT: HA_MID
PUBLIC ARRAY [1..16] POINT: HB_MID

PUBLIC ARRAY [1..16] REAL: HA_V
PUBLIC ARRAY [1..16] REAL: HB_V

PUBLIC ARRAY [1..16] INTEGER: HA_TYPE
PUBLIC ARRAY [1..16] INTEGER: HB_TYPE

PUBLIC INTEGER: HA_SEQ
PUBLIC INTEGER: HB_SEQ
PUBLIC INTEGER: HA_FREE
PUBLIC INTEGER: HB_FREE
PUBLIC INTEGER: HA_LAST
PUBLIC INTEGER: HB_LAST
PUBLIC INTEGER: HA_COUNT
PUBLIC INTEGER: HB_COUNT
PUBLIC INTEGER: HY_EXPECT
PUBLIC INTEGER: HY_UNDER

; ============================================================
; V11 I/O + FAST PROBE
; ============================================================
PUBLIC REAL: PROBE_TGT_X
PUBLIC REAL: PROBE_TGT_Y
PUBLIC REAL: PROBE_TGT_Z
PUBLIC REAL: PROBE_TGT_R
PUBLIC REAL: PROBE_SPEED

; 0=pending/idle, 1=triggered, 2=target reached without trigger
PUBLIC INTEGER: PROBE_RESULT
PUBLIC REAL: PROBE_MEAS_X
PUBLIC REAL: PROBE_MEAS_Y
PUBLIC REAL: PROBE_MEAS_Z
PUBLIC REAL: PROBE_MEAS_R

; Output writes are locked after startup until explicit CMD35 ARM=1.
PUBLIC INTEGER: IO_ARMED
PUBLIC INTEGER: SPIND_STATE


; ============================================================
; EXTERNI PROCES
; ============================================================

EXTERNAL: PCMOVE
EXTERNAL: PCLINEAR
EXTERNAL: PCCIRC
EXTERNAL: PCSMOOTH
EXTERNAL: PCPATH
EXTERNAL: PCSTREAM
EXTERNAL: PCRING
EXTERNAL: PCHYB
EXTERNAL: HYRING
EXTERNAL: PCPROBE


; ============================================================
; RHO VSTUPY
; ============================================================

INPUT BINARY:189=ALARM_RC
INPUT BINARY:193=AUTO_RC

; Zatim ponechavame, ale jejich skutecny vyznam
; jeste nemame overeny.
INPUT BINARY:128=IN_POSITION
INPUT BINARY:129=REFERENCED

; rho4.0 physical fast input #1 for ordinary BAPS interrogation.
; Same physical input is addressed as 611 only inside PCPROBE MOVE UNTIL.
INPUT BINARY:801=PROBE_STATE

; Candidate physical line: X11 high-speed output channel 801
; as Greifer_zu. V11 does not write it before explicit CMD35 ARM=1.
OUTPUT BINARY:801=SPINDLE_HW


; ============================================================
; INTERNI PROMENNE
; ============================================================

INTEGER: INPOSSTATUS
INTEGER: REFSTATUS

INTEGER: CMD
INTEGER: RESPONSE
INTEGER: VERSION

INTEGER: ALARMSTATUS
INTEGER: AUTOSTATUS

INTEGER: WINSTATUS
INTEGER: PROCSTATUS

; Stav jednotlivych motion procesu pro spolecnou BUSY/ERROR kontrolu
INTEGER: PCMOVESTATUS
INTEGER: LINEARSTATUS
INTEGER: CIRCSTATUS
INTEGER: SMOOTHSTATUS
INTEGER: PATHSTATUS
INTEGER: STREAMSTATUS
INTEGER: RINGSTATUS
INTEGER: HYBSTATUS
INTEGER: HYRSTATUS
INTEGER: PROBESTATUS
INTEGER: MOTIONBUSY

INTEGER: MOVE_PROCESS
INTEGER: PATHIDX
INTEGER: STR_SLOT
INTEGER: STR_SEQ
INTEGER: STR_ACCEPT
INTEGER: STR_LAST
INTEGER: HY_COUNT
INTEGER: HY_TYPE
INTEGER: HY_DUMI
INTEGER: IO_ARG
INTEGER: PROBE_IN
REAL: STR_DUMMY
POINT: ACTPOS
JC_POINT: @ACTJOINT

REAL: XVAL
REAL: YVAL
REAL: ZVAL
REAL: RVAL


; ============================================================
; PROGRAM
; ============================================================

BEGIN


; ------------------------------------------------------------
; Inicializace
; ------------------------------------------------------------

MOVE_STATE=0
MOVE_ERROR=0
MOVE_SPEED=0.02
LINEAR_SPEED=10.0
CIRC_SPEED=10.0
PROCSTATUS=-1
MOTIONBUSY=0
MOVE_PROCESS=0
PATH_COUNT=0
PATH_SPEED=5.0
SA_SPEED=5.0
SB_SPEED=5.0
SA_SEQ=-1
SB_SEQ=-1
SA_FREE=1
SB_FREE=1
STR_BLOCKS=0
SA_LAST=0
SB_LAST=0
STR_EXPECT=0
STR_UNDER=0
HA_SEQ=-1
HB_SEQ=-1
HA_FREE=1
HB_FREE=1
HA_LAST=0
HB_LAST=0
HA_COUNT=0
HB_COUNT=0
HY_EXPECT=0
HY_UNDER=0
PROBE_SPEED=0.5
PROBE_RESULT=0
PROBE_MEAS_X=0.0
PROBE_MEAS_Y=0.0
PROBE_MEAS_Z=0.0
PROBE_MEAS_R=0.0
IO_ARMED=0
SPIND_STATE=0
Loop:


; ============================================================
; SLEDOVANI AKTUALNIHO MOTION PROCESU
; ============================================================
;
; MOVE_PROCESS:
; 0 = zadny proces
; 1 = PCMOVE
; 2 = PCLINEAR
; 3 = PCCIRC
; 4 = PCSMOOTH
; 5 = PCPATH
; 6 = PCSTREAM
; 7 = PCRING
; 8 = PCHYB
; 9 = HYRING
; 10 = PCPROBE
;
; Proces sledujeme i po MOVE_STATE=2, protoze pohybovy program
; muze nastavit MOVE_STATE=2 drive, nez CONDITION() prejde na -1
; nebo nez se pripadny RC error projevi.
;
; Po STOP nastavujeme MOVE_PROCESS=0, proto monitor nespoustime
; podle MOVE_STATE, ale podle skutecneho vybraneho procesu.
;

IF MOVE_PROCESS<>0 THEN
BEGIN

    PROCSTATUS=-1

    IF MOVE_PROCESS=1 THEN
    BEGIN
        PROCSTATUS=CONDITION(PCMOVE)
    END

    IF MOVE_PROCESS=2 THEN
    BEGIN
        PROCSTATUS=CONDITION(PCLINEAR)
    END

    IF MOVE_PROCESS=3 THEN
    BEGIN
        PROCSTATUS=CONDITION(PCCIRC)
    END

    IF MOVE_PROCESS=4 THEN
    BEGIN
        PROCSTATUS=CONDITION(PCSMOOTH)
    END

    IF MOVE_PROCESS=5 THEN
    BEGIN
        PROCSTATUS=CONDITION(PCPATH)
    END

    IF MOVE_PROCESS=6 THEN
    BEGIN
        PROCSTATUS=CONDITION(PCSTREAM)
    END

    IF MOVE_PROCESS=7 THEN
    BEGIN
        PROCSTATUS=CONDITION(PCRING)
    END

    IF MOVE_PROCESS=8 THEN
    BEGIN
        PROCSTATUS=CONDITION(PCHYB)
    END

    IF MOVE_PROCESS=9 THEN
    BEGIN
        PROCSTATUS=CONDITION(HYRING)
    END

    IF MOVE_PROCESS=10 THEN
    BEGIN
        PROCSTATUS=CONDITION(PCPROBE)
    END

    IF PROCSTATUS>100 THEN
    BEGIN
        MOVE_STATE=3
        MOVE_ERROR=PROCSTATUS

        IF IO_ARMED=1 THEN
        BEGIN
            SPINDLE_HW=0
            SPIND_STATE=0
        END
    END

END


; ============================================================
; KOMUNIKACE S PC
; ============================================================

READ WIN_1,CMD

WINSTATUS=CONDITION(WIN_1)


IF WINSTATUS=0 THEN
BEGIN


    ; --------------------------------------------------------
    ; CMD 1
    ; PING
    ; --------------------------------------------------------

    IF CMD=1 THEN
    BEGIN

        RESPONSE=123456

        WRITE WIN_1,RESPONSE

    END


    ; --------------------------------------------------------
    ; CMD 2
    ; AKTUALNI POZICE ROBOTA
    ;
    ; odpoved:
    ; REAL X
    ; REAL Y
    ; REAL Z
    ; REAL R
    ; --------------------------------------------------------

    IF CMD=2 THEN
    BEGIN

        ACTPOS=POS

        XVAL=ACTPOS.K1
        YVAL=ACTPOS.K2
        ZVAL=ACTPOS.K3
        RVAL=ACTPOS.K4

        WRITE WIN_1,XVAL
        WRITE WIN_1,YVAL
        WRITE WIN_1,ZVAL
        WRITE WIN_1,RVAL

    END


    ; --------------------------------------------------------
    ; CMD 3
    ; STATUS
    ;
    ; odpoved:
    ; VERSION
    ; WINSTATUS
    ; ALARMSTATUS
    ; AUTOSTATUS
    ; INPOSSTATUS
    ; REFSTATUS
    ; --------------------------------------------------------

    IF CMD=3 THEN
    BEGIN

        VERSION=1

        WINSTATUS=CONDITION(WIN_1)

        ALARMSTATUS=ALARM_RC
        AUTOSTATUS=AUTO_RC

        INPOSSTATUS=IN_POSITION
        REFSTATUS=REFERENCED

        WRITE WIN_1,VERSION
        WRITE WIN_1,WINSTATUS
        WRITE WIN_1,ALARMSTATUS
        WRITE WIN_1,AUTOSTATUS
        WRITE WIN_1,INPOSSTATUS
        WRITE WIN_1,REFSTATUS

    END


    ; --------------------------------------------------------
    ; CMD 10
    ; START PCMOVE
    ;
    ; odpoved:
    ;  10 = START prijat
    ; -10 = robot je MANUAL
    ; -11 = jiny/predchozi motion proces je aktivni
    ; -12 = nektery motion proces je v RC ERROR
    ; --------------------------------------------------------

    IF CMD=10 THEN
    BEGIN

        RESPONSE=0
        MOTIONBUSY=0

        ; Vzdy kontrolujeme vsechny tri motion procesy.
        ; Tohle chrani controller i proti chybe klienta, ktery by
        ; zkusil spustit PTP behem LINEAR/CIRCULAR pohybu.
        PCMOVESTATUS=CONDITION(PCMOVE)
        LINEARSTATUS=CONDITION(PCLINEAR)
        CIRCSTATUS=CONDITION(PCCIRC)
        SMOOTHSTATUS=CONDITION(PCSMOOTH)
        PATHSTATUS=CONDITION(PCPATH)
        STREAMSTATUS=CONDITION(PCSTREAM)
        RINGSTATUS=CONDITION(PCRING)
        HYBSTATUS=CONDITION(PCHYB)
        HYRSTATUS=CONDITION(HYRING)
        PROBESTATUS=CONDITION(PCPROBE)

        ; RC ERROR ma prioritu pred MANUAL/BUSY.
        IF PCMOVESTATUS>100 THEN
        BEGIN
            MOVE_STATE=3
            MOVE_ERROR=PCMOVESTATUS
            RESPONSE=-12
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF LINEARSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=LINEARSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF CIRCSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=CIRCSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF SMOOTHSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=SMOOTHSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF PATHSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=PATHSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF STREAMSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=STREAMSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF RINGSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=RINGSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF HYBSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=HYBSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF HYRSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=HYRSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF PROBESTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=PROBESTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF AUTO_RC=0 THEN
            BEGIN
                RESPONSE=-10
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF PCMOVESTATUS<>-1 THEN MOTIONBUSY=1
            IF LINEARSTATUS<>-1 THEN MOTIONBUSY=1
            IF CIRCSTATUS<>-1 THEN MOTIONBUSY=1
            IF SMOOTHSTATUS<>-1 THEN MOTIONBUSY=1
            IF PATHSTATUS<>-1 THEN MOTIONBUSY=1
            IF STREAMSTATUS<>-1 THEN MOTIONBUSY=1
            IF RINGSTATUS<>-1 THEN MOTIONBUSY=1
            IF HYBSTATUS<>-1 THEN MOTIONBUSY=1
            IF HYRSTATUS<>-1 THEN MOTIONBUSY=1
            IF PROBESTATUS<>-1 THEN MOTIONBUSY=1

            IF MOTIONBUSY=1 THEN
            BEGIN
                RESPONSE=-11
            END

            IF MOTIONBUSY=0 THEN
            BEGIN
                MOVE_ERROR=0
                MOVE_STATE=1
                MOVE_PROCESS=1
                START PCMOVE
                RESPONSE=10
            END
        END

        WRITE WIN_1,RESPONSE

    END


    ; --------------------------------------------------------
    ; CMD 11
    ; NASTAVENI TARGETU
    ;
    ; PC posle:
    ; REAL X
    ; REAL Y
    ; REAL Z
    ; REAL R
    ;
    ; RHO vrati stejne hodnoty jako potvrzeni.
    ; --------------------------------------------------------

    IF CMD=11 THEN
    BEGIN

        READ WIN_1,TARGET_X
        READ WIN_1,TARGET_Y
        READ WIN_1,TARGET_Z
        READ WIN_1,TARGET_R

        WRITE WIN_1,TARGET_X
        WRITE WIN_1,TARGET_Y
        WRITE WIN_1,TARGET_Z
        WRITE WIN_1,TARGET_R

    END

    ; --------------------------------------------------------
    ; CMD 14
    ; NASTAVENI PTP RYCHLOSTI
    ;
    ; PC POSLE:
    ; REAL MOVE_SPEED
    ;
    ; RHO VRATI:
    ; REAL SPEED
    ; --------------------------------------------------------

    IF CMD=14 THEN
    BEGIN
    
        READ WIN_1,MOVE_SPEED
    
        IF MOVE_SPEED<0.0001 THEN
        BEGIN
            MOVE_SPEED=0.0001
        END
    
        IF MOVE_SPEED>0.80 THEN
        BEGIN
           MOVE_SPEED=0.80
        END
    
        WRITE WIN_1,MOVE_SPEED
        
    END

    ; --------------------------------------------------------
    ; CMD 15
    ; STOP VSECH MOTION PROCESU
    ;
    ; odpoved:
    ; 15 = STOP proveden
    ; --------------------------------------------------------

    IF CMD=15 THEN
    BEGIN
        IF CONDITION(PCMOVE)<>-1 THEN STOP PCMOVE
        IF CONDITION(PCLINEAR)<>-1 THEN STOP PCLINEAR
        IF CONDITION(PCCIRC)<>-1 THEN STOP PCCIRC
        IF CONDITION(PCSMOOTH)<>-1 THEN STOP PCSMOOTH
        IF CONDITION(PCPATH)<>-1 THEN STOP PCPATH
        IF CONDITION(PCSTREAM)<>-1 THEN STOP PCSTREAM
        IF CONDITION(PCRING)<>-1 THEN STOP PCRING
        IF CONDITION(PCHYB)<>-1 THEN STOP PCHYB
        IF CONDITION(HYRING)<>-1 THEN STOP HYRING
        IF CONDITION(PCPROBE)<>-1 THEN STOP PCPROBE

        IF IO_ARMED=1 THEN
        BEGIN
            SPINDLE_HW=0
            SPIND_STATE=0
        END

        MOVE_STATE=4
        MOVE_ERROR=0
        MOVE_PROCESS=0
        PROCSTATUS=-1
        RESPONSE=15
        WRITE WIN_1,RESPONSE
    END

    IF CMD=16 THEN
    BEGIN
        ; Nejdrive vzdy precteme CELY payload, aby pri odmitnuti
        ; pohybu nezustala data ve WIN_1 bufferu.
        READ WIN_1,TARGET_X
        READ WIN_1,TARGET_Y
        READ WIN_1,TARGET_Z
        READ WIN_1,TARGET_R
        READ WIN_1,LINEAR_SPEED

        RESPONSE=0
        MOTIONBUSY=0

        PCMOVESTATUS=CONDITION(PCMOVE)
        LINEARSTATUS=CONDITION(PCLINEAR)
        CIRCSTATUS=CONDITION(PCCIRC)
        SMOOTHSTATUS=CONDITION(PCSMOOTH)
        PATHSTATUS=CONDITION(PCPATH)
        STREAMSTATUS=CONDITION(PCSTREAM)
        RINGSTATUS=CONDITION(PCRING)
        HYBSTATUS=CONDITION(PCHYB)
        HYRSTATUS=CONDITION(HYRING)
        PROBESTATUS=CONDITION(PCPROBE)

        IF PCMOVESTATUS>100 THEN
        BEGIN
            MOVE_STATE=3
            MOVE_ERROR=PCMOVESTATUS
            RESPONSE=-12
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF LINEARSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=LINEARSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF CIRCSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=CIRCSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF SMOOTHSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=SMOOTHSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF PATHSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=PATHSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF STREAMSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=STREAMSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF RINGSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=RINGSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF HYBSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=HYBSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF HYRSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=HYRSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF PROBESTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=PROBESTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF AUTO_RC=0 THEN
            BEGIN
                RESPONSE=-10
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF PCMOVESTATUS<>-1 THEN MOTIONBUSY=1
            IF LINEARSTATUS<>-1 THEN MOTIONBUSY=1
            IF CIRCSTATUS<>-1 THEN MOTIONBUSY=1
            IF SMOOTHSTATUS<>-1 THEN MOTIONBUSY=1
            IF PATHSTATUS<>-1 THEN MOTIONBUSY=1
            IF STREAMSTATUS<>-1 THEN MOTIONBUSY=1
            IF RINGSTATUS<>-1 THEN MOTIONBUSY=1
            IF HYBSTATUS<>-1 THEN MOTIONBUSY=1
            IF HYRSTATUS<>-1 THEN MOTIONBUSY=1
            IF PROBESTATUS<>-1 THEN MOTIONBUSY=1

            IF MOTIONBUSY=1 THEN
            BEGIN
                RESPONSE=-11
            END

            IF MOTIONBUSY=0 THEN
            BEGIN
                MOVE_ERROR=0
                MOVE_STATE=1
                MOVE_PROCESS=2
                START PCLINEAR
                RESPONSE=10
            END
        END

        WRITE WIN_1,RESPONSE
    END

    
    IF CMD=17 THEN
    BEGIN
        @ACTJOINT=@POS

        WRITE WIN_1,@ACTJOINT.A1
        WRITE WIN_1,@ACTJOINT.A2
        WRITE WIN_1,@ACTJOINT.A3
        WRITE WIN_1,@ACTJOINT.A4

    END

    IF CMD=18 THEN
    BEGIN
        ; Stejne jako u CMD16 nejdrive precteme cely payload.
        READ WIN_1,CIRC_MID_X
        READ WIN_1,CIRC_MID_Y
        READ WIN_1,CIRC_MID_Z
        READ WIN_1,CIRC_MID_R

        READ WIN_1,CIRC_END_X
        READ WIN_1,CIRC_END_Y
        READ WIN_1,CIRC_END_Z
        READ WIN_1,CIRC_END_R

        READ WIN_1,CIRC_SPEED

        RESPONSE=0
        MOTIONBUSY=0

        PCMOVESTATUS=CONDITION(PCMOVE)
        LINEARSTATUS=CONDITION(PCLINEAR)
        CIRCSTATUS=CONDITION(PCCIRC)
        SMOOTHSTATUS=CONDITION(PCSMOOTH)
        PATHSTATUS=CONDITION(PCPATH)
        STREAMSTATUS=CONDITION(PCSTREAM)
        RINGSTATUS=CONDITION(PCRING)
        HYBSTATUS=CONDITION(PCHYB)
        HYRSTATUS=CONDITION(HYRING)
        PROBESTATUS=CONDITION(PCPROBE)

        IF PCMOVESTATUS>100 THEN
        BEGIN
            MOVE_STATE=3
            MOVE_ERROR=PCMOVESTATUS
            RESPONSE=-12
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF LINEARSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=LINEARSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF CIRCSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=CIRCSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF SMOOTHSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=SMOOTHSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF PATHSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=PATHSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF STREAMSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=STREAMSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF RINGSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=RINGSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF HYBSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=HYBSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF HYRSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=HYRSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF PROBESTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=PROBESTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF AUTO_RC=0 THEN
            BEGIN
                RESPONSE=-10
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF PCMOVESTATUS<>-1 THEN MOTIONBUSY=1
            IF LINEARSTATUS<>-1 THEN MOTIONBUSY=1
            IF CIRCSTATUS<>-1 THEN MOTIONBUSY=1
            IF SMOOTHSTATUS<>-1 THEN MOTIONBUSY=1
            IF PATHSTATUS<>-1 THEN MOTIONBUSY=1
            IF STREAMSTATUS<>-1 THEN MOTIONBUSY=1
            IF RINGSTATUS<>-1 THEN MOTIONBUSY=1
            IF HYBSTATUS<>-1 THEN MOTIONBUSY=1
            IF HYRSTATUS<>-1 THEN MOTIONBUSY=1
            IF PROBESTATUS<>-1 THEN MOTIONBUSY=1

            IF MOTIONBUSY=1 THEN
            BEGIN
                RESPONSE=-11
            END

            IF MOTIONBUSY=0 THEN
            BEGIN
                MOVE_ERROR=0
                MOVE_STATE=1
                MOVE_PROCESS=3
                START PCCIRC
                RESPONSE=10
            END
        END

        WRITE WIN_1,RESPONSE
    END



    ; --------------------------------------------------------
    ; CMD 19
    ; START PCSMOOTH - test PROGR_SLOPE
    ;
    ; odpoved:
    ;  10 = START prijat
    ; -10 = robot je MANUAL
    ; -11 = jiny/predchozi motion proces je aktivni
    ; -12 = nektery motion proces je v RC ERROR
    ; --------------------------------------------------------

    IF CMD=19 THEN
    BEGIN

        RESPONSE=0
        MOTIONBUSY=0

        PCMOVESTATUS=CONDITION(PCMOVE)
        LINEARSTATUS=CONDITION(PCLINEAR)
        CIRCSTATUS=CONDITION(PCCIRC)
        SMOOTHSTATUS=CONDITION(PCSMOOTH)
        PATHSTATUS=CONDITION(PCPATH)
        STREAMSTATUS=CONDITION(PCSTREAM)
        RINGSTATUS=CONDITION(PCRING)
        HYBSTATUS=CONDITION(PCHYB)
        HYRSTATUS=CONDITION(HYRING)
        PROBESTATUS=CONDITION(PCPROBE)

        IF PCMOVESTATUS>100 THEN
        BEGIN
            MOVE_STATE=3
            MOVE_ERROR=PCMOVESTATUS
            RESPONSE=-12
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF LINEARSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=LINEARSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF CIRCSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=CIRCSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF SMOOTHSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=SMOOTHSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF PATHSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=PATHSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF STREAMSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=STREAMSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF RINGSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=RINGSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF HYBSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=HYBSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF HYRSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=HYRSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF PROBESTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=PROBESTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF AUTO_RC=0 THEN
            BEGIN
                RESPONSE=-10
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF PCMOVESTATUS<>-1 THEN MOTIONBUSY=1
            IF LINEARSTATUS<>-1 THEN MOTIONBUSY=1
            IF CIRCSTATUS<>-1 THEN MOTIONBUSY=1
            IF SMOOTHSTATUS<>-1 THEN MOTIONBUSY=1
            IF PATHSTATUS<>-1 THEN MOTIONBUSY=1
            IF STREAMSTATUS<>-1 THEN MOTIONBUSY=1
            IF RINGSTATUS<>-1 THEN MOTIONBUSY=1
            IF HYBSTATUS<>-1 THEN MOTIONBUSY=1
            IF HYRSTATUS<>-1 THEN MOTIONBUSY=1
            IF PROBESTATUS<>-1 THEN MOTIONBUSY=1

            IF MOTIONBUSY=1 THEN
            BEGIN
                RESPONSE=-11
            END

            IF MOTIONBUSY=0 THEN
            BEGIN
                MOVE_ERROR=0
                MOVE_STATE=1
                MOVE_PROCESS=4
                START PCSMOOTH
                RESPONSE=10
            END
        END

        WRITE WIN_1,RESPONSE

    END



    ; --------------------------------------------------------
    ; CMD 20 - upload PCPATH buffer
    ; PC posle:
    ; INTEGER PATH_COUNT
    ; REAL PATH_SPEED
    ; 64x REAL PATHDATA = 16 * (X,Y,Z,R)
    ; Payload ma vzdy pevnou delku.
    ; odpoved: 20 OK, -20 count, -21 speed
    ; --------------------------------------------------------

    IF CMD=20 THEN
    BEGIN

        READ WIN_1,PATH_COUNT
        READ WIN_1,PATH_SPEED

        PATHIDX=1

        REPEAT 64 TIMES
            READ WIN_1,PATHDATA[PATHIDX]
            PATHIDX=PATHIDX+1
        REPEAT_END

        RESPONSE=20
        IF PATH_COUNT<1 THEN RESPONSE=-20
        IF PATH_COUNT>16 THEN RESPONSE=-20
        IF PATH_SPEED<=0.0 THEN RESPONSE=-21

        WRITE WIN_1,RESPONSE

    END


    ; --------------------------------------------------------
    ; CMD 21 - START PCPATH
    ; odpoved: 10 OK, -10 MANUAL, -11 BUSY, -12 RC ERROR
    ;           -20 count, -21 speed
    ; --------------------------------------------------------

    IF CMD=21 THEN
    BEGIN

        RESPONSE=0
        MOTIONBUSY=0

        IF PATH_COUNT<1 THEN RESPONSE=-20
        IF PATH_COUNT>16 THEN RESPONSE=-20
        IF PATH_SPEED<=0.0 THEN RESPONSE=-21

        IF RESPONSE=0 THEN
        BEGIN
            PCMOVESTATUS=CONDITION(PCMOVE)
            LINEARSTATUS=CONDITION(PCLINEAR)
            CIRCSTATUS=CONDITION(PCCIRC)
            SMOOTHSTATUS=CONDITION(PCSMOOTH)
            PATHSTATUS=CONDITION(PCPATH)
        STREAMSTATUS=CONDITION(PCSTREAM)
        RINGSTATUS=CONDITION(PCRING)
        HYBSTATUS=CONDITION(PCHYB)
        HYRSTATUS=CONDITION(HYRING)
        PROBESTATUS=CONDITION(PCPROBE)
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF PCMOVESTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=PCMOVESTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF LINEARSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=LINEARSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF CIRCSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=CIRCSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF SMOOTHSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=SMOOTHSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF PATHSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=PATHSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF STREAMSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=STREAMSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF RINGSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=RINGSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF HYBSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=HYBSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF HYRSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=HYRSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF PROBESTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=PROBESTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF AUTO_RC=0 THEN RESPONSE=-10
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF PCMOVESTATUS<>-1 THEN MOTIONBUSY=1
            IF LINEARSTATUS<>-1 THEN MOTIONBUSY=1
            IF CIRCSTATUS<>-1 THEN MOTIONBUSY=1
            IF SMOOTHSTATUS<>-1 THEN MOTIONBUSY=1
            IF PATHSTATUS<>-1 THEN MOTIONBUSY=1
            IF STREAMSTATUS<>-1 THEN MOTIONBUSY=1
            IF RINGSTATUS<>-1 THEN MOTIONBUSY=1
            IF HYBSTATUS<>-1 THEN MOTIONBUSY=1
            IF HYRSTATUS<>-1 THEN MOTIONBUSY=1
            IF PROBESTATUS<>-1 THEN MOTIONBUSY=1

            IF MOTIONBUSY=1 THEN
            BEGIN
                RESPONSE=-11
            END

            IF MOTIONBUSY=0 THEN
            BEGIN
                MOVE_ERROR=0
                MOVE_STATE=1
                MOVE_PROCESS=5
                START PCPATH
                RESPONSE=10
            END
        END

        WRITE WIN_1,RESPONSE

    END



    ; --------------------------------------------------------
    ; CMD 22 - UPLOAD STREAM SLOT A/B
    ;
    ; PC posle:
    ; INTEGER slot       0=A, 1=B
    ; INTEGER sequence
    ; REAL    speed_mm_s
    ; 64x REAL = 16 * (X,Y,Z,R)
    ;
    ; V1 intentionally uses exactly 16 points per buffer.
    ;
    ; response:
    ;  22 = OK
    ; -21 = speed <= 0
    ; -22 = invalid slot
    ; -23 = slot not FREE
    ; --------------------------------------------------------

    IF CMD=22 THEN
    BEGIN

        READ WIN_1,STR_SLOT
        READ WIN_1,STR_SEQ

        RESPONSE=22
        STR_ACCEPT=0

        IF STR_SLOT=0 THEN
        BEGIN
            IF SA_FREE=1 THEN
            BEGIN
                STR_ACCEPT=1
                SA_FREE=0
                SA_SEQ=STR_SEQ
                READ WIN_1,SA_SPEED
            READ WIN_1,SA_DATA[1].K1
            READ WIN_1,SA_DATA[1].K2
            READ WIN_1,SA_DATA[1].K3
            READ WIN_1,SA_DATA[1].K4
            READ WIN_1,SA_DATA[2].K1
            READ WIN_1,SA_DATA[2].K2
            READ WIN_1,SA_DATA[2].K3
            READ WIN_1,SA_DATA[2].K4
            READ WIN_1,SA_DATA[3].K1
            READ WIN_1,SA_DATA[3].K2
            READ WIN_1,SA_DATA[3].K3
            READ WIN_1,SA_DATA[3].K4
            READ WIN_1,SA_DATA[4].K1
            READ WIN_1,SA_DATA[4].K2
            READ WIN_1,SA_DATA[4].K3
            READ WIN_1,SA_DATA[4].K4
            READ WIN_1,SA_DATA[5].K1
            READ WIN_1,SA_DATA[5].K2
            READ WIN_1,SA_DATA[5].K3
            READ WIN_1,SA_DATA[5].K4
            READ WIN_1,SA_DATA[6].K1
            READ WIN_1,SA_DATA[6].K2
            READ WIN_1,SA_DATA[6].K3
            READ WIN_1,SA_DATA[6].K4
            READ WIN_1,SA_DATA[7].K1
            READ WIN_1,SA_DATA[7].K2
            READ WIN_1,SA_DATA[7].K3
            READ WIN_1,SA_DATA[7].K4
            READ WIN_1,SA_DATA[8].K1
            READ WIN_1,SA_DATA[8].K2
            READ WIN_1,SA_DATA[8].K3
            READ WIN_1,SA_DATA[8].K4
            READ WIN_1,SA_DATA[9].K1
            READ WIN_1,SA_DATA[9].K2
            READ WIN_1,SA_DATA[9].K3
            READ WIN_1,SA_DATA[9].K4
            READ WIN_1,SA_DATA[10].K1
            READ WIN_1,SA_DATA[10].K2
            READ WIN_1,SA_DATA[10].K3
            READ WIN_1,SA_DATA[10].K4
            READ WIN_1,SA_DATA[11].K1
            READ WIN_1,SA_DATA[11].K2
            READ WIN_1,SA_DATA[11].K3
            READ WIN_1,SA_DATA[11].K4
            READ WIN_1,SA_DATA[12].K1
            READ WIN_1,SA_DATA[12].K2
            READ WIN_1,SA_DATA[12].K3
            READ WIN_1,SA_DATA[12].K4
            READ WIN_1,SA_DATA[13].K1
            READ WIN_1,SA_DATA[13].K2
            READ WIN_1,SA_DATA[13].K3
            READ WIN_1,SA_DATA[13].K4
            READ WIN_1,SA_DATA[14].K1
            READ WIN_1,SA_DATA[14].K2
            READ WIN_1,SA_DATA[14].K3
            READ WIN_1,SA_DATA[14].K4
            READ WIN_1,SA_DATA[15].K1
            READ WIN_1,SA_DATA[15].K2
            READ WIN_1,SA_DATA[15].K3
            READ WIN_1,SA_DATA[15].K4
            READ WIN_1,SA_DATA[16].K1
            READ WIN_1,SA_DATA[16].K2
            READ WIN_1,SA_DATA[16].K3
            READ WIN_1,SA_DATA[16].K4
                IF SA_SPEED<=0.0 THEN
                BEGIN
                    RESPONSE=-21
                    SA_FREE=1
                END
            END
        END

        IF STR_SLOT=1 THEN
        BEGIN
            IF SB_FREE=1 THEN
            BEGIN
                STR_ACCEPT=1
                SB_FREE=0
                SB_SEQ=STR_SEQ
                READ WIN_1,SB_SPEED
            READ WIN_1,SB_DATA[1].K1
            READ WIN_1,SB_DATA[1].K2
            READ WIN_1,SB_DATA[1].K3
            READ WIN_1,SB_DATA[1].K4
            READ WIN_1,SB_DATA[2].K1
            READ WIN_1,SB_DATA[2].K2
            READ WIN_1,SB_DATA[2].K3
            READ WIN_1,SB_DATA[2].K4
            READ WIN_1,SB_DATA[3].K1
            READ WIN_1,SB_DATA[3].K2
            READ WIN_1,SB_DATA[3].K3
            READ WIN_1,SB_DATA[3].K4
            READ WIN_1,SB_DATA[4].K1
            READ WIN_1,SB_DATA[4].K2
            READ WIN_1,SB_DATA[4].K3
            READ WIN_1,SB_DATA[4].K4
            READ WIN_1,SB_DATA[5].K1
            READ WIN_1,SB_DATA[5].K2
            READ WIN_1,SB_DATA[5].K3
            READ WIN_1,SB_DATA[5].K4
            READ WIN_1,SB_DATA[6].K1
            READ WIN_1,SB_DATA[6].K2
            READ WIN_1,SB_DATA[6].K3
            READ WIN_1,SB_DATA[6].K4
            READ WIN_1,SB_DATA[7].K1
            READ WIN_1,SB_DATA[7].K2
            READ WIN_1,SB_DATA[7].K3
            READ WIN_1,SB_DATA[7].K4
            READ WIN_1,SB_DATA[8].K1
            READ WIN_1,SB_DATA[8].K2
            READ WIN_1,SB_DATA[8].K3
            READ WIN_1,SB_DATA[8].K4
            READ WIN_1,SB_DATA[9].K1
            READ WIN_1,SB_DATA[9].K2
            READ WIN_1,SB_DATA[9].K3
            READ WIN_1,SB_DATA[9].K4
            READ WIN_1,SB_DATA[10].K1
            READ WIN_1,SB_DATA[10].K2
            READ WIN_1,SB_DATA[10].K3
            READ WIN_1,SB_DATA[10].K4
            READ WIN_1,SB_DATA[11].K1
            READ WIN_1,SB_DATA[11].K2
            READ WIN_1,SB_DATA[11].K3
            READ WIN_1,SB_DATA[11].K4
            READ WIN_1,SB_DATA[12].K1
            READ WIN_1,SB_DATA[12].K2
            READ WIN_1,SB_DATA[12].K3
            READ WIN_1,SB_DATA[12].K4
            READ WIN_1,SB_DATA[13].K1
            READ WIN_1,SB_DATA[13].K2
            READ WIN_1,SB_DATA[13].K3
            READ WIN_1,SB_DATA[13].K4
            READ WIN_1,SB_DATA[14].K1
            READ WIN_1,SB_DATA[14].K2
            READ WIN_1,SB_DATA[14].K3
            READ WIN_1,SB_DATA[14].K4
            READ WIN_1,SB_DATA[15].K1
            READ WIN_1,SB_DATA[15].K2
            READ WIN_1,SB_DATA[15].K3
            READ WIN_1,SB_DATA[15].K4
            READ WIN_1,SB_DATA[16].K1
            READ WIN_1,SB_DATA[16].K2
            READ WIN_1,SB_DATA[16].K3
            READ WIN_1,SB_DATA[16].K4
                IF SB_SPEED<=0.0 THEN
                BEGIN
                    RESPONSE=-21
                    SB_FREE=1
                END
            END
        END

        ; If the slot was invalid/busy, consume the full remaining
        ; payload so the TCP protocol stays aligned.
        IF STR_ACCEPT=0 THEN
        BEGIN
            READ WIN_1,STR_DUMMY

            REPEAT 64 TIMES
                READ WIN_1,STR_DUMMY
            REPEAT_END

            RESPONSE=-22
            IF STR_SLOT=0 THEN RESPONSE=-23
            IF STR_SLOT=1 THEN RESPONSE=-23
        END

        WRITE WIN_1,RESPONSE

    END


    ; --------------------------------------------------------
    ; CMD 23 - START PCSTREAM
    ;
    ; PC posle:
    ; INTEGER STR_BLOCKS (1..32)
    ;
    ; Before START:
    ;   seq0 must be in A.
    ;   if STR_BLOCKS>=2, seq1 must be in B.
    ;
    ; response:
    ;  10 = START accepted
    ; -10 = MANUAL
    ; -11 = another motion process active
    ; -12 = RC error
    ; -20 = STR_BLOCKS outside 1..32
    ; -24 = initial A/B buffers not ready
    ; --------------------------------------------------------

    IF CMD=23 THEN
    BEGIN

        READ WIN_1,STR_BLOCKS

        RESPONSE=0
        MOTIONBUSY=0

        IF STR_BLOCKS<1 THEN RESPONSE=-20
        IF STR_BLOCKS>32 THEN RESPONSE=-20

        IF RESPONSE=0 THEN
        BEGIN
            IF SA_FREE<>0 THEN RESPONSE=-24
            IF SA_SEQ<>0 THEN RESPONSE=-24
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF STR_BLOCKS>=2 THEN
            BEGIN
                IF SB_FREE<>0 THEN RESPONSE=-24
                IF SB_SEQ<>1 THEN RESPONSE=-24
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            PCMOVESTATUS=CONDITION(PCMOVE)
            LINEARSTATUS=CONDITION(PCLINEAR)
            CIRCSTATUS=CONDITION(PCCIRC)
            SMOOTHSTATUS=CONDITION(PCSMOOTH)
            PATHSTATUS=CONDITION(PCPATH)
            STREAMSTATUS=CONDITION(PCSTREAM)
        RINGSTATUS=CONDITION(PCRING)
        HYBSTATUS=CONDITION(PCHYB)
        HYRSTATUS=CONDITION(HYRING)
        PROBESTATUS=CONDITION(PCPROBE)
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF PCMOVESTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=PCMOVESTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF LINEARSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=LINEARSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF CIRCSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=CIRCSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF SMOOTHSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=SMOOTHSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF PATHSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=PATHSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF STREAMSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=STREAMSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF RINGSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=RINGSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF HYBSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=HYBSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF HYRSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=HYRSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF PROBESTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=PROBESTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF AUTO_RC=0 THEN RESPONSE=-10
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF PCMOVESTATUS<>-1 THEN MOTIONBUSY=1
            IF LINEARSTATUS<>-1 THEN MOTIONBUSY=1
            IF CIRCSTATUS<>-1 THEN MOTIONBUSY=1
            IF SMOOTHSTATUS<>-1 THEN MOTIONBUSY=1
            IF PATHSTATUS<>-1 THEN MOTIONBUSY=1
            IF STREAMSTATUS<>-1 THEN MOTIONBUSY=1
            IF RINGSTATUS<>-1 THEN MOTIONBUSY=1
            IF HYBSTATUS<>-1 THEN MOTIONBUSY=1
            IF HYRSTATUS<>-1 THEN MOTIONBUSY=1
            IF PROBESTATUS<>-1 THEN MOTIONBUSY=1

            IF MOTIONBUSY=1 THEN
            BEGIN
                RESPONSE=-11
            END

            IF MOTIONBUSY=0 THEN
            BEGIN
                MOVE_ERROR=0
                MOVE_STATE=1
                MOVE_PROCESS=6
                START PCSTREAM
                RESPONSE=10
            END
        END

        WRITE WIN_1,RESPONSE

    END


    ; --------------------------------------------------------
    ; CMD 24 - STREAM STATUS
    ;
    ; response: 6x INTEGER
    ; A_FREE, B_FREE, A_SEQ, B_SEQ, STR_BLOCKS, PROCSTATUS
    ; --------------------------------------------------------

    IF CMD=24 THEN
    BEGIN

        PROCSTATUS=CONDITION(PCSTREAM)

        WRITE WIN_1,SA_FREE
        WRITE WIN_1,SB_FREE
        WRITE WIN_1,SA_SEQ
        WRITE WIN_1,SB_SEQ
        WRITE WIN_1,STR_BLOCKS
        WRITE WIN_1,PROCSTATUS

    END





    ; --------------------------------------------------------
    ; CMD 26 - UPLOAD PCRING SLOT
    ;
    ; PC sends:
    ; INTEGER slot      0=A, 1=B
    ; INTEGER sequence
    ; INTEGER last      0=more buffers, 1=final buffer
    ; REAL speed_mm_s
    ; 64x REAL = 16 * (X,Y,Z,R)
    ;
    ; response:
    ;  26 = OK
    ; -21 = speed <= 0
    ; -22 = invalid slot
    ; -23 = slot not FREE
    ; -26 = LAST not 0/1
    ; --------------------------------------------------------

    IF CMD=26 THEN
    BEGIN

        READ WIN_1,STR_SLOT
        READ WIN_1,STR_SEQ
        READ WIN_1,STR_LAST

        RESPONSE=26
        STR_ACCEPT=0

        IF STR_SLOT=0 THEN
        BEGIN
            IF SA_FREE=1 THEN
            BEGIN
                STR_ACCEPT=1
                SA_FREE=0
                SA_SEQ=STR_SEQ
                SA_LAST=STR_LAST
                READ WIN_1,SA_SPEED
                READ WIN_1,SA_DATA[1].K1
                READ WIN_1,SA_DATA[1].K2
                READ WIN_1,SA_DATA[1].K3
                READ WIN_1,SA_DATA[1].K4
                READ WIN_1,SA_DATA[2].K1
                READ WIN_1,SA_DATA[2].K2
                READ WIN_1,SA_DATA[2].K3
                READ WIN_1,SA_DATA[2].K4
                READ WIN_1,SA_DATA[3].K1
                READ WIN_1,SA_DATA[3].K2
                READ WIN_1,SA_DATA[3].K3
                READ WIN_1,SA_DATA[3].K4
                READ WIN_1,SA_DATA[4].K1
                READ WIN_1,SA_DATA[4].K2
                READ WIN_1,SA_DATA[4].K3
                READ WIN_1,SA_DATA[4].K4
                READ WIN_1,SA_DATA[5].K1
                READ WIN_1,SA_DATA[5].K2
                READ WIN_1,SA_DATA[5].K3
                READ WIN_1,SA_DATA[5].K4
                READ WIN_1,SA_DATA[6].K1
                READ WIN_1,SA_DATA[6].K2
                READ WIN_1,SA_DATA[6].K3
                READ WIN_1,SA_DATA[6].K4
                READ WIN_1,SA_DATA[7].K1
                READ WIN_1,SA_DATA[7].K2
                READ WIN_1,SA_DATA[7].K3
                READ WIN_1,SA_DATA[7].K4
                READ WIN_1,SA_DATA[8].K1
                READ WIN_1,SA_DATA[8].K2
                READ WIN_1,SA_DATA[8].K3
                READ WIN_1,SA_DATA[8].K4
                READ WIN_1,SA_DATA[9].K1
                READ WIN_1,SA_DATA[9].K2
                READ WIN_1,SA_DATA[9].K3
                READ WIN_1,SA_DATA[9].K4
                READ WIN_1,SA_DATA[10].K1
                READ WIN_1,SA_DATA[10].K2
                READ WIN_1,SA_DATA[10].K3
                READ WIN_1,SA_DATA[10].K4
                READ WIN_1,SA_DATA[11].K1
                READ WIN_1,SA_DATA[11].K2
                READ WIN_1,SA_DATA[11].K3
                READ WIN_1,SA_DATA[11].K4
                READ WIN_1,SA_DATA[12].K1
                READ WIN_1,SA_DATA[12].K2
                READ WIN_1,SA_DATA[12].K3
                READ WIN_1,SA_DATA[12].K4
                READ WIN_1,SA_DATA[13].K1
                READ WIN_1,SA_DATA[13].K2
                READ WIN_1,SA_DATA[13].K3
                READ WIN_1,SA_DATA[13].K4
                READ WIN_1,SA_DATA[14].K1
                READ WIN_1,SA_DATA[14].K2
                READ WIN_1,SA_DATA[14].K3
                READ WIN_1,SA_DATA[14].K4
                READ WIN_1,SA_DATA[15].K1
                READ WIN_1,SA_DATA[15].K2
                READ WIN_1,SA_DATA[15].K3
                READ WIN_1,SA_DATA[15].K4
                READ WIN_1,SA_DATA[16].K1
                READ WIN_1,SA_DATA[16].K2
                READ WIN_1,SA_DATA[16].K3
                READ WIN_1,SA_DATA[16].K4

                IF SA_SPEED<=0.0 THEN
                BEGIN
                    RESPONSE=-21
                    SA_FREE=1
                END

                IF STR_LAST<0 THEN
                BEGIN
                    RESPONSE=-26
                    SA_FREE=1
                END

                IF STR_LAST>1 THEN
                BEGIN
                    RESPONSE=-26
                    SA_FREE=1
                END
            END
        END

        IF STR_SLOT=1 THEN
        BEGIN
            IF SB_FREE=1 THEN
            BEGIN
                STR_ACCEPT=1
                SB_FREE=0
                SB_SEQ=STR_SEQ
                SB_LAST=STR_LAST
                READ WIN_1,SB_SPEED
                READ WIN_1,SB_DATA[1].K1
                READ WIN_1,SB_DATA[1].K2
                READ WIN_1,SB_DATA[1].K3
                READ WIN_1,SB_DATA[1].K4
                READ WIN_1,SB_DATA[2].K1
                READ WIN_1,SB_DATA[2].K2
                READ WIN_1,SB_DATA[2].K3
                READ WIN_1,SB_DATA[2].K4
                READ WIN_1,SB_DATA[3].K1
                READ WIN_1,SB_DATA[3].K2
                READ WIN_1,SB_DATA[3].K3
                READ WIN_1,SB_DATA[3].K4
                READ WIN_1,SB_DATA[4].K1
                READ WIN_1,SB_DATA[4].K2
                READ WIN_1,SB_DATA[4].K3
                READ WIN_1,SB_DATA[4].K4
                READ WIN_1,SB_DATA[5].K1
                READ WIN_1,SB_DATA[5].K2
                READ WIN_1,SB_DATA[5].K3
                READ WIN_1,SB_DATA[5].K4
                READ WIN_1,SB_DATA[6].K1
                READ WIN_1,SB_DATA[6].K2
                READ WIN_1,SB_DATA[6].K3
                READ WIN_1,SB_DATA[6].K4
                READ WIN_1,SB_DATA[7].K1
                READ WIN_1,SB_DATA[7].K2
                READ WIN_1,SB_DATA[7].K3
                READ WIN_1,SB_DATA[7].K4
                READ WIN_1,SB_DATA[8].K1
                READ WIN_1,SB_DATA[8].K2
                READ WIN_1,SB_DATA[8].K3
                READ WIN_1,SB_DATA[8].K4
                READ WIN_1,SB_DATA[9].K1
                READ WIN_1,SB_DATA[9].K2
                READ WIN_1,SB_DATA[9].K3
                READ WIN_1,SB_DATA[9].K4
                READ WIN_1,SB_DATA[10].K1
                READ WIN_1,SB_DATA[10].K2
                READ WIN_1,SB_DATA[10].K3
                READ WIN_1,SB_DATA[10].K4
                READ WIN_1,SB_DATA[11].K1
                READ WIN_1,SB_DATA[11].K2
                READ WIN_1,SB_DATA[11].K3
                READ WIN_1,SB_DATA[11].K4
                READ WIN_1,SB_DATA[12].K1
                READ WIN_1,SB_DATA[12].K2
                READ WIN_1,SB_DATA[12].K3
                READ WIN_1,SB_DATA[12].K4
                READ WIN_1,SB_DATA[13].K1
                READ WIN_1,SB_DATA[13].K2
                READ WIN_1,SB_DATA[13].K3
                READ WIN_1,SB_DATA[13].K4
                READ WIN_1,SB_DATA[14].K1
                READ WIN_1,SB_DATA[14].K2
                READ WIN_1,SB_DATA[14].K3
                READ WIN_1,SB_DATA[14].K4
                READ WIN_1,SB_DATA[15].K1
                READ WIN_1,SB_DATA[15].K2
                READ WIN_1,SB_DATA[15].K3
                READ WIN_1,SB_DATA[15].K4
                READ WIN_1,SB_DATA[16].K1
                READ WIN_1,SB_DATA[16].K2
                READ WIN_1,SB_DATA[16].K3
                READ WIN_1,SB_DATA[16].K4

                IF SB_SPEED<=0.0 THEN
                BEGIN
                    RESPONSE=-21
                    SB_FREE=1
                END

                IF STR_LAST<0 THEN
                BEGIN
                    RESPONSE=-26
                    SB_FREE=1
                END

                IF STR_LAST>1 THEN
                BEGIN
                    RESPONSE=-26
                    SB_FREE=1
                END
            END
        END

        ; invalid/busy slot: consume speed + 64 REAL payload
        IF STR_ACCEPT=0 THEN
        BEGIN
            READ WIN_1,STR_DUMMY
            REPEAT 64 TIMES
                READ WIN_1,STR_DUMMY
            REPEAT_END

            RESPONSE=-22
            IF STR_SLOT=0 THEN RESPONSE=-23
            IF STR_SLOT=1 THEN RESPONSE=-23
        END

        WRITE WIN_1,RESPONSE

    END


    ; --------------------------------------------------------
    ; CMD 27 - START PCRING
    ;
    ; seq0 must be loaded in A.
    ; If seq0 is not final, seq1 must be loaded in B.
    ;
    ; response:
    ;  10 = START accepted
    ; -10 = MANUAL
    ; -11 = another motion process active
    ; -12 = RC error
    ; -24 = initial A/B slots not ready
    ; --------------------------------------------------------

    IF CMD=27 THEN
    BEGIN

        RESPONSE=0
        MOTIONBUSY=0
        STR_UNDER=0
        STR_EXPECT=0

        IF SA_FREE<>0 THEN RESPONSE=-24
        IF SA_SEQ<>0 THEN RESPONSE=-24

        IF RESPONSE=0 THEN
        BEGIN
            IF SA_LAST=0 THEN
            BEGIN
                IF SB_FREE<>0 THEN RESPONSE=-24
                IF SB_SEQ<>1 THEN RESPONSE=-24
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            PCMOVESTATUS=CONDITION(PCMOVE)
            LINEARSTATUS=CONDITION(PCLINEAR)
            CIRCSTATUS=CONDITION(PCCIRC)
            SMOOTHSTATUS=CONDITION(PCSMOOTH)
            PATHSTATUS=CONDITION(PCPATH)
            STREAMSTATUS=CONDITION(PCSTREAM)
            RINGSTATUS=CONDITION(PCRING)
        HYBSTATUS=CONDITION(PCHYB)
        HYRSTATUS=CONDITION(HYRING)
        PROBESTATUS=CONDITION(PCPROBE)
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF PCMOVESTATUS>100 THEN RESPONSE=-12
            IF LINEARSTATUS>100 THEN RESPONSE=-12
            IF CIRCSTATUS>100 THEN RESPONSE=-12
            IF SMOOTHSTATUS>100 THEN RESPONSE=-12
            IF PATHSTATUS>100 THEN RESPONSE=-12
            IF STREAMSTATUS>100 THEN RESPONSE=-12
            IF RINGSTATUS>100 THEN RESPONSE=-12
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF AUTO_RC=0 THEN RESPONSE=-10
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF PCMOVESTATUS<>-1 THEN MOTIONBUSY=1
            IF LINEARSTATUS<>-1 THEN MOTIONBUSY=1
            IF CIRCSTATUS<>-1 THEN MOTIONBUSY=1
            IF SMOOTHSTATUS<>-1 THEN MOTIONBUSY=1
            IF PATHSTATUS<>-1 THEN MOTIONBUSY=1
            IF STREAMSTATUS<>-1 THEN MOTIONBUSY=1
            IF RINGSTATUS<>-1 THEN MOTIONBUSY=1
            IF HYBSTATUS<>-1 THEN MOTIONBUSY=1
            IF HYRSTATUS<>-1 THEN MOTIONBUSY=1
            IF PROBESTATUS<>-1 THEN MOTIONBUSY=1

            IF MOTIONBUSY=1 THEN RESPONSE=-11

            IF MOTIONBUSY=0 THEN
            BEGIN
                MOVE_ERROR=0
                MOVE_STATE=1
                MOVE_PROCESS=7
                START PCRING
                RESPONSE=10
            END
        END

        WRITE WIN_1,RESPONSE

    END


    ; --------------------------------------------------------
    ; CMD 28 - PCRING STATUS
    ;
    ; response 11x INTEGER:
    ; A_FREE, B_FREE, A_SEQ, B_SEQ,
    ; A_LAST, B_LAST, EXPECT, UNDER,
    ; MOVE_STATE, MOVE_ERROR, PCRING CONDITION
    ; --------------------------------------------------------

    IF CMD=28 THEN
    BEGIN

        RINGSTATUS=CONDITION(PCRING)
        HYBSTATUS=CONDITION(PCHYB)
        HYRSTATUS=CONDITION(HYRING)
        PROBESTATUS=CONDITION(PCPROBE)

        WRITE WIN_1,SA_FREE
        WRITE WIN_1,SB_FREE
        WRITE WIN_1,SA_SEQ
        WRITE WIN_1,SB_SEQ
        WRITE WIN_1,SA_LAST
        WRITE WIN_1,SB_LAST
        WRITE WIN_1,STR_EXPECT
        WRITE WIN_1,STR_UNDER
        WRITE WIN_1,MOVE_STATE
        WRITE WIN_1,MOVE_ERROR
        WRITE WIN_1,RINGSTATUS

    END



    ; --------------------------------------------------------
    ; CMD 29 - START PCHYB
    ;
    ; V9 experiment:
    ; same rounded rectangle as PCSMOOTH, but LINEAR/CIRCULAR
    ; primitive selection happens through runtime IF/ELSE.
    ;
    ; response:
    ;  10 = START accepted
    ; -10 = MANUAL
    ; -11 = another motion process active
    ; -12 = RC error in a motion process
    ; --------------------------------------------------------

    IF CMD=29 THEN
    BEGIN

        RESPONSE=0
        MOTIONBUSY=0

        PCMOVESTATUS=CONDITION(PCMOVE)
        LINEARSTATUS=CONDITION(PCLINEAR)
        CIRCSTATUS=CONDITION(PCCIRC)
        SMOOTHSTATUS=CONDITION(PCSMOOTH)
        PATHSTATUS=CONDITION(PCPATH)
        STREAMSTATUS=CONDITION(PCSTREAM)
        RINGSTATUS=CONDITION(PCRING)
        HYBSTATUS=CONDITION(PCHYB)
        HYRSTATUS=CONDITION(HYRING)
        PROBESTATUS=CONDITION(PCPROBE)

        IF PCMOVESTATUS>100 THEN
        BEGIN
            MOVE_STATE=3
            MOVE_ERROR=PCMOVESTATUS
            RESPONSE=-12
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF LINEARSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=LINEARSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF CIRCSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=CIRCSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF SMOOTHSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=SMOOTHSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF PATHSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=PATHSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF STREAMSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=STREAMSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF RINGSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=RINGSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF HYBSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=HYBSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF HYRSTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=HYRSTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF PROBESTATUS>100 THEN
            BEGIN
                MOVE_STATE=3
                MOVE_ERROR=PROBESTATUS
                RESPONSE=-12
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF AUTO_RC=0 THEN RESPONSE=-10
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF PCMOVESTATUS<>-1 THEN MOTIONBUSY=1
            IF LINEARSTATUS<>-1 THEN MOTIONBUSY=1
            IF CIRCSTATUS<>-1 THEN MOTIONBUSY=1
            IF SMOOTHSTATUS<>-1 THEN MOTIONBUSY=1
            IF PATHSTATUS<>-1 THEN MOTIONBUSY=1
            IF STREAMSTATUS<>-1 THEN MOTIONBUSY=1
            IF RINGSTATUS<>-1 THEN MOTIONBUSY=1
            IF HYBSTATUS<>-1 THEN MOTIONBUSY=1
            IF HYRSTATUS<>-1 THEN MOTIONBUSY=1
            IF PROBESTATUS<>-1 THEN MOTIONBUSY=1

            IF MOTIONBUSY=1 THEN RESPONSE=-11

            IF MOTIONBUSY=0 THEN
            BEGIN
                MOVE_ERROR=0
                MOVE_STATE=1
                MOVE_PROCESS=8
                START PCHYB
                RESPONSE=10
            END
        END

        WRITE WIN_1,RESPONSE

    END



    ; --------------------------------------------------------
    ; CMD 30 - UPLOAD V9 HYBRID A/B SLOT
    ;
    ; PC sends:
    ; INTEGER slot       0=A, 1=B
    ; INTEGER sequence
    ; INTEGER last       0/1
    ; INTEGER count      1..16
    ;
    ; then ALWAYS 16 primitive records:
    ;   INTEGER type     1=LINEAR, 2=CIRCULAR
    ;   REAL speed_mm_s
    ;   4x REAL END  (X,Y,Z,R)
    ;   4x REAL MID  (X,Y,Z,R)
    ;
    ; Non-final slot must have count=16.
    ;
    ; response:
    ;  30 = OK
    ; -21 = speed <= 0
    ; -22 = invalid slot
    ; -23 = slot not FREE
    ; -26 = LAST invalid
    ; -27 = COUNT invalid
    ; -28 = TYPE invalid
    ; -29 = non-final COUNT must be 16
    ; --------------------------------------------------------

    IF CMD=30 THEN
    BEGIN

        READ WIN_1,STR_SLOT
        READ WIN_1,STR_SEQ
        READ WIN_1,STR_LAST
        READ WIN_1,HY_COUNT

        RESPONSE=30
        STR_ACCEPT=0

        IF STR_SLOT=0 THEN
        BEGIN
            IF HA_FREE=1 THEN
            BEGIN
                STR_ACCEPT=1
                HA_SEQ=STR_SEQ
                HA_LAST=STR_LAST
                HA_COUNT=HY_COUNT

                READ WIN_1,HA_TYPE[1]
                READ WIN_1,HA_V[1]
                READ WIN_1,HA_END[1].K1
                READ WIN_1,HA_END[1].K2
                READ WIN_1,HA_END[1].K3
                READ WIN_1,HA_END[1].K4
                READ WIN_1,HA_MID[1].K1
                READ WIN_1,HA_MID[1].K2
                READ WIN_1,HA_MID[1].K3
                READ WIN_1,HA_MID[1].K4
                READ WIN_1,HA_TYPE[2]
                READ WIN_1,HA_V[2]
                READ WIN_1,HA_END[2].K1
                READ WIN_1,HA_END[2].K2
                READ WIN_1,HA_END[2].K3
                READ WIN_1,HA_END[2].K4
                READ WIN_1,HA_MID[2].K1
                READ WIN_1,HA_MID[2].K2
                READ WIN_1,HA_MID[2].K3
                READ WIN_1,HA_MID[2].K4
                READ WIN_1,HA_TYPE[3]
                READ WIN_1,HA_V[3]
                READ WIN_1,HA_END[3].K1
                READ WIN_1,HA_END[3].K2
                READ WIN_1,HA_END[3].K3
                READ WIN_1,HA_END[3].K4
                READ WIN_1,HA_MID[3].K1
                READ WIN_1,HA_MID[3].K2
                READ WIN_1,HA_MID[3].K3
                READ WIN_1,HA_MID[3].K4
                READ WIN_1,HA_TYPE[4]
                READ WIN_1,HA_V[4]
                READ WIN_1,HA_END[4].K1
                READ WIN_1,HA_END[4].K2
                READ WIN_1,HA_END[4].K3
                READ WIN_1,HA_END[4].K4
                READ WIN_1,HA_MID[4].K1
                READ WIN_1,HA_MID[4].K2
                READ WIN_1,HA_MID[4].K3
                READ WIN_1,HA_MID[4].K4
                READ WIN_1,HA_TYPE[5]
                READ WIN_1,HA_V[5]
                READ WIN_1,HA_END[5].K1
                READ WIN_1,HA_END[5].K2
                READ WIN_1,HA_END[5].K3
                READ WIN_1,HA_END[5].K4
                READ WIN_1,HA_MID[5].K1
                READ WIN_1,HA_MID[5].K2
                READ WIN_1,HA_MID[5].K3
                READ WIN_1,HA_MID[5].K4
                READ WIN_1,HA_TYPE[6]
                READ WIN_1,HA_V[6]
                READ WIN_1,HA_END[6].K1
                READ WIN_1,HA_END[6].K2
                READ WIN_1,HA_END[6].K3
                READ WIN_1,HA_END[6].K4
                READ WIN_1,HA_MID[6].K1
                READ WIN_1,HA_MID[6].K2
                READ WIN_1,HA_MID[6].K3
                READ WIN_1,HA_MID[6].K4
                READ WIN_1,HA_TYPE[7]
                READ WIN_1,HA_V[7]
                READ WIN_1,HA_END[7].K1
                READ WIN_1,HA_END[7].K2
                READ WIN_1,HA_END[7].K3
                READ WIN_1,HA_END[7].K4
                READ WIN_1,HA_MID[7].K1
                READ WIN_1,HA_MID[7].K2
                READ WIN_1,HA_MID[7].K3
                READ WIN_1,HA_MID[7].K4
                READ WIN_1,HA_TYPE[8]
                READ WIN_1,HA_V[8]
                READ WIN_1,HA_END[8].K1
                READ WIN_1,HA_END[8].K2
                READ WIN_1,HA_END[8].K3
                READ WIN_1,HA_END[8].K4
                READ WIN_1,HA_MID[8].K1
                READ WIN_1,HA_MID[8].K2
                READ WIN_1,HA_MID[8].K3
                READ WIN_1,HA_MID[8].K4
                READ WIN_1,HA_TYPE[9]
                READ WIN_1,HA_V[9]
                READ WIN_1,HA_END[9].K1
                READ WIN_1,HA_END[9].K2
                READ WIN_1,HA_END[9].K3
                READ WIN_1,HA_END[9].K4
                READ WIN_1,HA_MID[9].K1
                READ WIN_1,HA_MID[9].K2
                READ WIN_1,HA_MID[9].K3
                READ WIN_1,HA_MID[9].K4
                READ WIN_1,HA_TYPE[10]
                READ WIN_1,HA_V[10]
                READ WIN_1,HA_END[10].K1
                READ WIN_1,HA_END[10].K2
                READ WIN_1,HA_END[10].K3
                READ WIN_1,HA_END[10].K4
                READ WIN_1,HA_MID[10].K1
                READ WIN_1,HA_MID[10].K2
                READ WIN_1,HA_MID[10].K3
                READ WIN_1,HA_MID[10].K4
                READ WIN_1,HA_TYPE[11]
                READ WIN_1,HA_V[11]
                READ WIN_1,HA_END[11].K1
                READ WIN_1,HA_END[11].K2
                READ WIN_1,HA_END[11].K3
                READ WIN_1,HA_END[11].K4
                READ WIN_1,HA_MID[11].K1
                READ WIN_1,HA_MID[11].K2
                READ WIN_1,HA_MID[11].K3
                READ WIN_1,HA_MID[11].K4
                READ WIN_1,HA_TYPE[12]
                READ WIN_1,HA_V[12]
                READ WIN_1,HA_END[12].K1
                READ WIN_1,HA_END[12].K2
                READ WIN_1,HA_END[12].K3
                READ WIN_1,HA_END[12].K4
                READ WIN_1,HA_MID[12].K1
                READ WIN_1,HA_MID[12].K2
                READ WIN_1,HA_MID[12].K3
                READ WIN_1,HA_MID[12].K4
                READ WIN_1,HA_TYPE[13]
                READ WIN_1,HA_V[13]
                READ WIN_1,HA_END[13].K1
                READ WIN_1,HA_END[13].K2
                READ WIN_1,HA_END[13].K3
                READ WIN_1,HA_END[13].K4
                READ WIN_1,HA_MID[13].K1
                READ WIN_1,HA_MID[13].K2
                READ WIN_1,HA_MID[13].K3
                READ WIN_1,HA_MID[13].K4
                READ WIN_1,HA_TYPE[14]
                READ WIN_1,HA_V[14]
                READ WIN_1,HA_END[14].K1
                READ WIN_1,HA_END[14].K2
                READ WIN_1,HA_END[14].K3
                READ WIN_1,HA_END[14].K4
                READ WIN_1,HA_MID[14].K1
                READ WIN_1,HA_MID[14].K2
                READ WIN_1,HA_MID[14].K3
                READ WIN_1,HA_MID[14].K4
                READ WIN_1,HA_TYPE[15]
                READ WIN_1,HA_V[15]
                READ WIN_1,HA_END[15].K1
                READ WIN_1,HA_END[15].K2
                READ WIN_1,HA_END[15].K3
                READ WIN_1,HA_END[15].K4
                READ WIN_1,HA_MID[15].K1
                READ WIN_1,HA_MID[15].K2
                READ WIN_1,HA_MID[15].K3
                READ WIN_1,HA_MID[15].K4
                READ WIN_1,HA_TYPE[16]
                READ WIN_1,HA_V[16]
                READ WIN_1,HA_END[16].K1
                READ WIN_1,HA_END[16].K2
                READ WIN_1,HA_END[16].K3
                READ WIN_1,HA_END[16].K4
                READ WIN_1,HA_MID[16].K1
                READ WIN_1,HA_MID[16].K2
                READ WIN_1,HA_MID[16].K3
                READ WIN_1,HA_MID[16].K4

                IF HA_LAST<0 THEN RESPONSE=-26
                IF HA_LAST>1 THEN RESPONSE=-26
                IF HA_COUNT<1 THEN RESPONSE=-27
                IF HA_COUNT>16 THEN RESPONSE=-27

                IF HA_LAST=0 THEN
                BEGIN
                    IF HA_COUNT<>16 THEN RESPONSE=-29
                END

                IF HA_COUNT>=1 THEN
                BEGIN
                    IF HA_TYPE[1]<1 THEN RESPONSE=-28
                    IF HA_TYPE[1]>2 THEN RESPONSE=-28
                    IF HA_V[1]<=0.0 THEN RESPONSE=-21
                END
                IF HA_COUNT>=2 THEN
                BEGIN
                    IF HA_TYPE[2]<1 THEN RESPONSE=-28
                    IF HA_TYPE[2]>2 THEN RESPONSE=-28
                    IF HA_V[2]<=0.0 THEN RESPONSE=-21
                END
                IF HA_COUNT>=3 THEN
                BEGIN
                    IF HA_TYPE[3]<1 THEN RESPONSE=-28
                    IF HA_TYPE[3]>2 THEN RESPONSE=-28
                    IF HA_V[3]<=0.0 THEN RESPONSE=-21
                END
                IF HA_COUNT>=4 THEN
                BEGIN
                    IF HA_TYPE[4]<1 THEN RESPONSE=-28
                    IF HA_TYPE[4]>2 THEN RESPONSE=-28
                    IF HA_V[4]<=0.0 THEN RESPONSE=-21
                END
                IF HA_COUNT>=5 THEN
                BEGIN
                    IF HA_TYPE[5]<1 THEN RESPONSE=-28
                    IF HA_TYPE[5]>2 THEN RESPONSE=-28
                    IF HA_V[5]<=0.0 THEN RESPONSE=-21
                END
                IF HA_COUNT>=6 THEN
                BEGIN
                    IF HA_TYPE[6]<1 THEN RESPONSE=-28
                    IF HA_TYPE[6]>2 THEN RESPONSE=-28
                    IF HA_V[6]<=0.0 THEN RESPONSE=-21
                END
                IF HA_COUNT>=7 THEN
                BEGIN
                    IF HA_TYPE[7]<1 THEN RESPONSE=-28
                    IF HA_TYPE[7]>2 THEN RESPONSE=-28
                    IF HA_V[7]<=0.0 THEN RESPONSE=-21
                END
                IF HA_COUNT>=8 THEN
                BEGIN
                    IF HA_TYPE[8]<1 THEN RESPONSE=-28
                    IF HA_TYPE[8]>2 THEN RESPONSE=-28
                    IF HA_V[8]<=0.0 THEN RESPONSE=-21
                END
                IF HA_COUNT>=9 THEN
                BEGIN
                    IF HA_TYPE[9]<1 THEN RESPONSE=-28
                    IF HA_TYPE[9]>2 THEN RESPONSE=-28
                    IF HA_V[9]<=0.0 THEN RESPONSE=-21
                END
                IF HA_COUNT>=10 THEN
                BEGIN
                    IF HA_TYPE[10]<1 THEN RESPONSE=-28
                    IF HA_TYPE[10]>2 THEN RESPONSE=-28
                    IF HA_V[10]<=0.0 THEN RESPONSE=-21
                END
                IF HA_COUNT>=11 THEN
                BEGIN
                    IF HA_TYPE[11]<1 THEN RESPONSE=-28
                    IF HA_TYPE[11]>2 THEN RESPONSE=-28
                    IF HA_V[11]<=0.0 THEN RESPONSE=-21
                END
                IF HA_COUNT>=12 THEN
                BEGIN
                    IF HA_TYPE[12]<1 THEN RESPONSE=-28
                    IF HA_TYPE[12]>2 THEN RESPONSE=-28
                    IF HA_V[12]<=0.0 THEN RESPONSE=-21
                END
                IF HA_COUNT>=13 THEN
                BEGIN
                    IF HA_TYPE[13]<1 THEN RESPONSE=-28
                    IF HA_TYPE[13]>2 THEN RESPONSE=-28
                    IF HA_V[13]<=0.0 THEN RESPONSE=-21
                END
                IF HA_COUNT>=14 THEN
                BEGIN
                    IF HA_TYPE[14]<1 THEN RESPONSE=-28
                    IF HA_TYPE[14]>2 THEN RESPONSE=-28
                    IF HA_V[14]<=0.0 THEN RESPONSE=-21
                END
                IF HA_COUNT>=15 THEN
                BEGIN
                    IF HA_TYPE[15]<1 THEN RESPONSE=-28
                    IF HA_TYPE[15]>2 THEN RESPONSE=-28
                    IF HA_V[15]<=0.0 THEN RESPONSE=-21
                END
                IF HA_COUNT>=16 THEN
                BEGIN
                    IF HA_TYPE[16]<1 THEN RESPONSE=-28
                    IF HA_TYPE[16]>2 THEN RESPONSE=-28
                    IF HA_V[16]<=0.0 THEN RESPONSE=-21
                END

                IF RESPONSE=30 THEN HA_FREE=0
            END
        END

        IF STR_SLOT=1 THEN
        BEGIN
            IF HB_FREE=1 THEN
            BEGIN
                STR_ACCEPT=1
                HB_SEQ=STR_SEQ
                HB_LAST=STR_LAST
                HB_COUNT=HY_COUNT

                READ WIN_1,HB_TYPE[1]
                READ WIN_1,HB_V[1]
                READ WIN_1,HB_END[1].K1
                READ WIN_1,HB_END[1].K2
                READ WIN_1,HB_END[1].K3
                READ WIN_1,HB_END[1].K4
                READ WIN_1,HB_MID[1].K1
                READ WIN_1,HB_MID[1].K2
                READ WIN_1,HB_MID[1].K3
                READ WIN_1,HB_MID[1].K4
                READ WIN_1,HB_TYPE[2]
                READ WIN_1,HB_V[2]
                READ WIN_1,HB_END[2].K1
                READ WIN_1,HB_END[2].K2
                READ WIN_1,HB_END[2].K3
                READ WIN_1,HB_END[2].K4
                READ WIN_1,HB_MID[2].K1
                READ WIN_1,HB_MID[2].K2
                READ WIN_1,HB_MID[2].K3
                READ WIN_1,HB_MID[2].K4
                READ WIN_1,HB_TYPE[3]
                READ WIN_1,HB_V[3]
                READ WIN_1,HB_END[3].K1
                READ WIN_1,HB_END[3].K2
                READ WIN_1,HB_END[3].K3
                READ WIN_1,HB_END[3].K4
                READ WIN_1,HB_MID[3].K1
                READ WIN_1,HB_MID[3].K2
                READ WIN_1,HB_MID[3].K3
                READ WIN_1,HB_MID[3].K4
                READ WIN_1,HB_TYPE[4]
                READ WIN_1,HB_V[4]
                READ WIN_1,HB_END[4].K1
                READ WIN_1,HB_END[4].K2
                READ WIN_1,HB_END[4].K3
                READ WIN_1,HB_END[4].K4
                READ WIN_1,HB_MID[4].K1
                READ WIN_1,HB_MID[4].K2
                READ WIN_1,HB_MID[4].K3
                READ WIN_1,HB_MID[4].K4
                READ WIN_1,HB_TYPE[5]
                READ WIN_1,HB_V[5]
                READ WIN_1,HB_END[5].K1
                READ WIN_1,HB_END[5].K2
                READ WIN_1,HB_END[5].K3
                READ WIN_1,HB_END[5].K4
                READ WIN_1,HB_MID[5].K1
                READ WIN_1,HB_MID[5].K2
                READ WIN_1,HB_MID[5].K3
                READ WIN_1,HB_MID[5].K4
                READ WIN_1,HB_TYPE[6]
                READ WIN_1,HB_V[6]
                READ WIN_1,HB_END[6].K1
                READ WIN_1,HB_END[6].K2
                READ WIN_1,HB_END[6].K3
                READ WIN_1,HB_END[6].K4
                READ WIN_1,HB_MID[6].K1
                READ WIN_1,HB_MID[6].K2
                READ WIN_1,HB_MID[6].K3
                READ WIN_1,HB_MID[6].K4
                READ WIN_1,HB_TYPE[7]
                READ WIN_1,HB_V[7]
                READ WIN_1,HB_END[7].K1
                READ WIN_1,HB_END[7].K2
                READ WIN_1,HB_END[7].K3
                READ WIN_1,HB_END[7].K4
                READ WIN_1,HB_MID[7].K1
                READ WIN_1,HB_MID[7].K2
                READ WIN_1,HB_MID[7].K3
                READ WIN_1,HB_MID[7].K4
                READ WIN_1,HB_TYPE[8]
                READ WIN_1,HB_V[8]
                READ WIN_1,HB_END[8].K1
                READ WIN_1,HB_END[8].K2
                READ WIN_1,HB_END[8].K3
                READ WIN_1,HB_END[8].K4
                READ WIN_1,HB_MID[8].K1
                READ WIN_1,HB_MID[8].K2
                READ WIN_1,HB_MID[8].K3
                READ WIN_1,HB_MID[8].K4
                READ WIN_1,HB_TYPE[9]
                READ WIN_1,HB_V[9]
                READ WIN_1,HB_END[9].K1
                READ WIN_1,HB_END[9].K2
                READ WIN_1,HB_END[9].K3
                READ WIN_1,HB_END[9].K4
                READ WIN_1,HB_MID[9].K1
                READ WIN_1,HB_MID[9].K2
                READ WIN_1,HB_MID[9].K3
                READ WIN_1,HB_MID[9].K4
                READ WIN_1,HB_TYPE[10]
                READ WIN_1,HB_V[10]
                READ WIN_1,HB_END[10].K1
                READ WIN_1,HB_END[10].K2
                READ WIN_1,HB_END[10].K3
                READ WIN_1,HB_END[10].K4
                READ WIN_1,HB_MID[10].K1
                READ WIN_1,HB_MID[10].K2
                READ WIN_1,HB_MID[10].K3
                READ WIN_1,HB_MID[10].K4
                READ WIN_1,HB_TYPE[11]
                READ WIN_1,HB_V[11]
                READ WIN_1,HB_END[11].K1
                READ WIN_1,HB_END[11].K2
                READ WIN_1,HB_END[11].K3
                READ WIN_1,HB_END[11].K4
                READ WIN_1,HB_MID[11].K1
                READ WIN_1,HB_MID[11].K2
                READ WIN_1,HB_MID[11].K3
                READ WIN_1,HB_MID[11].K4
                READ WIN_1,HB_TYPE[12]
                READ WIN_1,HB_V[12]
                READ WIN_1,HB_END[12].K1
                READ WIN_1,HB_END[12].K2
                READ WIN_1,HB_END[12].K3
                READ WIN_1,HB_END[12].K4
                READ WIN_1,HB_MID[12].K1
                READ WIN_1,HB_MID[12].K2
                READ WIN_1,HB_MID[12].K3
                READ WIN_1,HB_MID[12].K4
                READ WIN_1,HB_TYPE[13]
                READ WIN_1,HB_V[13]
                READ WIN_1,HB_END[13].K1
                READ WIN_1,HB_END[13].K2
                READ WIN_1,HB_END[13].K3
                READ WIN_1,HB_END[13].K4
                READ WIN_1,HB_MID[13].K1
                READ WIN_1,HB_MID[13].K2
                READ WIN_1,HB_MID[13].K3
                READ WIN_1,HB_MID[13].K4
                READ WIN_1,HB_TYPE[14]
                READ WIN_1,HB_V[14]
                READ WIN_1,HB_END[14].K1
                READ WIN_1,HB_END[14].K2
                READ WIN_1,HB_END[14].K3
                READ WIN_1,HB_END[14].K4
                READ WIN_1,HB_MID[14].K1
                READ WIN_1,HB_MID[14].K2
                READ WIN_1,HB_MID[14].K3
                READ WIN_1,HB_MID[14].K4
                READ WIN_1,HB_TYPE[15]
                READ WIN_1,HB_V[15]
                READ WIN_1,HB_END[15].K1
                READ WIN_1,HB_END[15].K2
                READ WIN_1,HB_END[15].K3
                READ WIN_1,HB_END[15].K4
                READ WIN_1,HB_MID[15].K1
                READ WIN_1,HB_MID[15].K2
                READ WIN_1,HB_MID[15].K3
                READ WIN_1,HB_MID[15].K4
                READ WIN_1,HB_TYPE[16]
                READ WIN_1,HB_V[16]
                READ WIN_1,HB_END[16].K1
                READ WIN_1,HB_END[16].K2
                READ WIN_1,HB_END[16].K3
                READ WIN_1,HB_END[16].K4
                READ WIN_1,HB_MID[16].K1
                READ WIN_1,HB_MID[16].K2
                READ WIN_1,HB_MID[16].K3
                READ WIN_1,HB_MID[16].K4

                IF HB_LAST<0 THEN RESPONSE=-26
                IF HB_LAST>1 THEN RESPONSE=-26
                IF HB_COUNT<1 THEN RESPONSE=-27
                IF HB_COUNT>16 THEN RESPONSE=-27

                IF HB_LAST=0 THEN
                BEGIN
                    IF HB_COUNT<>16 THEN RESPONSE=-29
                END

                IF HB_COUNT>=1 THEN
                BEGIN
                    IF HB_TYPE[1]<1 THEN RESPONSE=-28
                    IF HB_TYPE[1]>2 THEN RESPONSE=-28
                    IF HB_V[1]<=0.0 THEN RESPONSE=-21
                END
                IF HB_COUNT>=2 THEN
                BEGIN
                    IF HB_TYPE[2]<1 THEN RESPONSE=-28
                    IF HB_TYPE[2]>2 THEN RESPONSE=-28
                    IF HB_V[2]<=0.0 THEN RESPONSE=-21
                END
                IF HB_COUNT>=3 THEN
                BEGIN
                    IF HB_TYPE[3]<1 THEN RESPONSE=-28
                    IF HB_TYPE[3]>2 THEN RESPONSE=-28
                    IF HB_V[3]<=0.0 THEN RESPONSE=-21
                END
                IF HB_COUNT>=4 THEN
                BEGIN
                    IF HB_TYPE[4]<1 THEN RESPONSE=-28
                    IF HB_TYPE[4]>2 THEN RESPONSE=-28
                    IF HB_V[4]<=0.0 THEN RESPONSE=-21
                END
                IF HB_COUNT>=5 THEN
                BEGIN
                    IF HB_TYPE[5]<1 THEN RESPONSE=-28
                    IF HB_TYPE[5]>2 THEN RESPONSE=-28
                    IF HB_V[5]<=0.0 THEN RESPONSE=-21
                END
                IF HB_COUNT>=6 THEN
                BEGIN
                    IF HB_TYPE[6]<1 THEN RESPONSE=-28
                    IF HB_TYPE[6]>2 THEN RESPONSE=-28
                    IF HB_V[6]<=0.0 THEN RESPONSE=-21
                END
                IF HB_COUNT>=7 THEN
                BEGIN
                    IF HB_TYPE[7]<1 THEN RESPONSE=-28
                    IF HB_TYPE[7]>2 THEN RESPONSE=-28
                    IF HB_V[7]<=0.0 THEN RESPONSE=-21
                END
                IF HB_COUNT>=8 THEN
                BEGIN
                    IF HB_TYPE[8]<1 THEN RESPONSE=-28
                    IF HB_TYPE[8]>2 THEN RESPONSE=-28
                    IF HB_V[8]<=0.0 THEN RESPONSE=-21
                END
                IF HB_COUNT>=9 THEN
                BEGIN
                    IF HB_TYPE[9]<1 THEN RESPONSE=-28
                    IF HB_TYPE[9]>2 THEN RESPONSE=-28
                    IF HB_V[9]<=0.0 THEN RESPONSE=-21
                END
                IF HB_COUNT>=10 THEN
                BEGIN
                    IF HB_TYPE[10]<1 THEN RESPONSE=-28
                    IF HB_TYPE[10]>2 THEN RESPONSE=-28
                    IF HB_V[10]<=0.0 THEN RESPONSE=-21
                END
                IF HB_COUNT>=11 THEN
                BEGIN
                    IF HB_TYPE[11]<1 THEN RESPONSE=-28
                    IF HB_TYPE[11]>2 THEN RESPONSE=-28
                    IF HB_V[11]<=0.0 THEN RESPONSE=-21
                END
                IF HB_COUNT>=12 THEN
                BEGIN
                    IF HB_TYPE[12]<1 THEN RESPONSE=-28
                    IF HB_TYPE[12]>2 THEN RESPONSE=-28
                    IF HB_V[12]<=0.0 THEN RESPONSE=-21
                END
                IF HB_COUNT>=13 THEN
                BEGIN
                    IF HB_TYPE[13]<1 THEN RESPONSE=-28
                    IF HB_TYPE[13]>2 THEN RESPONSE=-28
                    IF HB_V[13]<=0.0 THEN RESPONSE=-21
                END
                IF HB_COUNT>=14 THEN
                BEGIN
                    IF HB_TYPE[14]<1 THEN RESPONSE=-28
                    IF HB_TYPE[14]>2 THEN RESPONSE=-28
                    IF HB_V[14]<=0.0 THEN RESPONSE=-21
                END
                IF HB_COUNT>=15 THEN
                BEGIN
                    IF HB_TYPE[15]<1 THEN RESPONSE=-28
                    IF HB_TYPE[15]>2 THEN RESPONSE=-28
                    IF HB_V[15]<=0.0 THEN RESPONSE=-21
                END
                IF HB_COUNT>=16 THEN
                BEGIN
                    IF HB_TYPE[16]<1 THEN RESPONSE=-28
                    IF HB_TYPE[16]>2 THEN RESPONSE=-28
                    IF HB_V[16]<=0.0 THEN RESPONSE=-21
                END

                IF RESPONSE=30 THEN HB_FREE=0
            END
        END

        ; invalid/busy slot: consume complete fixed payload
        IF STR_ACCEPT=0 THEN
        BEGIN
            READ WIN_1,HY_DUMI
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,HY_DUMI
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,HY_DUMI
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,HY_DUMI
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,HY_DUMI
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,HY_DUMI
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,HY_DUMI
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,HY_DUMI
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,HY_DUMI
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,HY_DUMI
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,HY_DUMI
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,HY_DUMI
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,HY_DUMI
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,HY_DUMI
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,HY_DUMI
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,HY_DUMI
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY
            READ WIN_1,STR_DUMMY

            RESPONSE=-22
            IF STR_SLOT=0 THEN RESPONSE=-23
            IF STR_SLOT=1 THEN RESPONSE=-23
        END

        WRITE WIN_1,RESPONSE

    END


    ; --------------------------------------------------------
    ; CMD 31 - START HYRING
    ;
    ; A must contain seq0.
    ; If A is not LAST, B must contain seq1.
    ; --------------------------------------------------------

    IF CMD=31 THEN
    BEGIN

        RESPONSE=0
        MOTIONBUSY=0
        HY_UNDER=0
        HY_EXPECT=0

        IF HA_FREE<>0 THEN RESPONSE=-24
        IF HA_SEQ<>0 THEN RESPONSE=-24

        IF RESPONSE=0 THEN
        BEGIN
            IF HA_LAST=0 THEN
            BEGIN
                IF HB_FREE<>0 THEN RESPONSE=-24
                IF HB_SEQ<>1 THEN RESPONSE=-24
            END
        END

        IF RESPONSE=0 THEN
        BEGIN
            PCMOVESTATUS=CONDITION(PCMOVE)
            LINEARSTATUS=CONDITION(PCLINEAR)
            CIRCSTATUS=CONDITION(PCCIRC)
            SMOOTHSTATUS=CONDITION(PCSMOOTH)
            PATHSTATUS=CONDITION(PCPATH)
            STREAMSTATUS=CONDITION(PCSTREAM)
            RINGSTATUS=CONDITION(PCRING)
            HYBSTATUS=CONDITION(PCHYB)
            HYRSTATUS=CONDITION(HYRING)
        PROBESTATUS=CONDITION(PCPROBE)
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF PCMOVESTATUS>100 THEN RESPONSE=-12
            IF LINEARSTATUS>100 THEN RESPONSE=-12
            IF CIRCSTATUS>100 THEN RESPONSE=-12
            IF SMOOTHSTATUS>100 THEN RESPONSE=-12
            IF PATHSTATUS>100 THEN RESPONSE=-12
            IF STREAMSTATUS>100 THEN RESPONSE=-12
            IF RINGSTATUS>100 THEN RESPONSE=-12
            IF HYBSTATUS>100 THEN RESPONSE=-12
            IF HYRSTATUS>100 THEN RESPONSE=-12
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF AUTO_RC=0 THEN RESPONSE=-10
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF PCMOVESTATUS<>-1 THEN MOTIONBUSY=1
            IF LINEARSTATUS<>-1 THEN MOTIONBUSY=1
            IF CIRCSTATUS<>-1 THEN MOTIONBUSY=1
            IF SMOOTHSTATUS<>-1 THEN MOTIONBUSY=1
            IF PATHSTATUS<>-1 THEN MOTIONBUSY=1
            IF STREAMSTATUS<>-1 THEN MOTIONBUSY=1
            IF RINGSTATUS<>-1 THEN MOTIONBUSY=1
            IF HYBSTATUS<>-1 THEN MOTIONBUSY=1
            IF HYRSTATUS<>-1 THEN MOTIONBUSY=1
            IF PROBESTATUS<>-1 THEN MOTIONBUSY=1

            IF MOTIONBUSY=1 THEN RESPONSE=-11

            IF MOTIONBUSY=0 THEN
            BEGIN
                MOVE_ERROR=0
                MOVE_STATE=1
                MOVE_PROCESS=9
                START HYRING
                RESPONSE=10
            END
        END

        WRITE WIN_1,RESPONSE

    END


    ; --------------------------------------------------------
    ; CMD 32 - HYRING STATUS
    ;
    ; reply 13x INTEGER:
    ; A_FREE,B_FREE,A_SEQ,B_SEQ,A_LAST,B_LAST,A_COUNT,B_COUNT,
    ; EXPECT,UNDER,MOVE_STATE,MOVE_ERROR,HYRING CONDITION
    ; --------------------------------------------------------

    IF CMD=32 THEN
    BEGIN

        HYRSTATUS=CONDITION(HYRING)
        PROBESTATUS=CONDITION(PCPROBE)

        WRITE WIN_1,HA_FREE
        WRITE WIN_1,HB_FREE
        WRITE WIN_1,HA_SEQ
        WRITE WIN_1,HB_SEQ
        WRITE WIN_1,HA_LAST
        WRITE WIN_1,HB_LAST
        WRITE WIN_1,HA_COUNT
        WRITE WIN_1,HB_COUNT
        WRITE WIN_1,HY_EXPECT
        WRITE WIN_1,HY_UNDER
        WRITE WIN_1,MOVE_STATE
        WRITE WIN_1,MOVE_ERROR
        WRITE WIN_1,HYRSTATUS

    END


    ; --------------------------------------------------------
    ; CMD 33 - CLEAR HYBRID RING
    ; --------------------------------------------------------

    IF CMD=33 THEN
    BEGIN

        HYRSTATUS=CONDITION(HYRING)
        PROBESTATUS=CONDITION(PCPROBE)
        RESPONSE=33

        IF HYRSTATUS<>-1 THEN RESPONSE=-11

        IF RESPONSE=33 THEN
        BEGIN
            HA_FREE=1
            HB_FREE=1
            HA_SEQ=-1
            HB_SEQ=-1
            HA_LAST=0
            HB_LAST=0
            HA_COUNT=0
            HB_COUNT=0
            HY_EXPECT=0
            HY_UNDER=0
        END

        WRITE WIN_1,RESPONSE

    END



    ; ========================================================
    ; V11 I/O + FAST PROBE
    ; ========================================================

    ; CMD34: read-only I/O status.
    ; reply 5x INTEGER:
    ;   IO_ARMED, SPIND_STATE, input611, spindle_channel, probe_channel
    IF CMD=34 THEN
    BEGIN
        PROBE_IN=0
        IF PROBE_STATE=1 THEN PROBE_IN=1

        WRITE WIN_1,IO_ARMED
        WRITE WIN_1,SPIND_STATE
        WRITE WIN_1,PROBE_IN

        RESPONSE=1
        WRITE WIN_1,RESPONSE

        RESPONSE=611
        WRITE WIN_1,RESPONSE
    END


    ; CMD35: explicit physical-output ARM/DISARM.
    ; payload int32 0/1, response 35 OK, -35 invalid.
    ; ARM=1 is the FIRST place where V11 writes output 1.
    IF CMD=35 THEN
    BEGIN
        READ WIN_1,IO_ARG
        RESPONSE=35

        IF IO_ARG<0 THEN RESPONSE=-35
        IF IO_ARG>1 THEN RESPONSE=-35

        IF RESPONSE=35 THEN
        BEGIN
            IF IO_ARG=1 THEN
            BEGIN
                SPINDLE_HW=0
                SPIND_STATE=0
                IO_ARMED=1
            END

            IF IO_ARG=0 THEN
            BEGIN
                IF IO_ARMED=1 THEN
                BEGIN
                    SPINDLE_HW=0
                    SPIND_STATE=0
                END
                IO_ARMED=0
            END
        END

        WRITE WIN_1,RESPONSE
    END


    ; CMD36: spindle digital output.
    ; payload int32 0/1.
    ; response 36 OK, -35 not armed, -36 invalid value.
    IF CMD=36 THEN
    BEGIN
        READ WIN_1,IO_ARG
        RESPONSE=36

        IF IO_ARG<0 THEN RESPONSE=-36
        IF IO_ARG>1 THEN RESPONSE=-36

        IF RESPONSE=36 THEN
        BEGIN
            IF IO_ARMED=0 THEN RESPONSE=-35
        END

        IF RESPONSE=36 THEN
        BEGIN
            IF IO_ARG=0 THEN SPINDLE_HW=0
            IF IO_ARG=1 THEN SPINDLE_HW=1
            SPIND_STATE=IO_ARG
        END

        WRITE WIN_1,RESPONSE
    END


    ; CMD38: start LINEAR MOVE UNTIL fast input 611 = 1.
    ; payload 5x REAL: X,Y,Z,R,speed_mm_s
    ; response 10/-10/-11/-12, -38=input already active, -39=speed
    IF CMD=38 THEN
    BEGIN
        READ WIN_1,PROBE_TGT_X
        READ WIN_1,PROBE_TGT_Y
        READ WIN_1,PROBE_TGT_Z
        READ WIN_1,PROBE_TGT_R
        READ WIN_1,PROBE_SPEED

        RESPONSE=0
        MOTIONBUSY=0

        IF PROBE_SPEED<0.05 THEN RESPONSE=-39
        IF PROBE_SPEED>5.0 THEN RESPONSE=-39

        PROBE_IN=0
        IF PROBE_STATE=1 THEN PROBE_IN=1

        IF RESPONSE=0 THEN
        BEGIN
            IF PROBE_IN=1 THEN RESPONSE=-38
        END

        IF RESPONSE=0 THEN
        BEGIN
            PCMOVESTATUS=CONDITION(PCMOVE)
            LINEARSTATUS=CONDITION(PCLINEAR)
            CIRCSTATUS=CONDITION(PCCIRC)
            SMOOTHSTATUS=CONDITION(PCSMOOTH)
            PATHSTATUS=CONDITION(PCPATH)
            STREAMSTATUS=CONDITION(PCSTREAM)
            RINGSTATUS=CONDITION(PCRING)
            HYBSTATUS=CONDITION(PCHYB)
            HYRSTATUS=CONDITION(HYRING)
            PROBESTATUS=CONDITION(PCPROBE)

            IF PCMOVESTATUS>100 THEN RESPONSE=-12
            IF LINEARSTATUS>100 THEN RESPONSE=-12
            IF CIRCSTATUS>100 THEN RESPONSE=-12
            IF SMOOTHSTATUS>100 THEN RESPONSE=-12
            IF PATHSTATUS>100 THEN RESPONSE=-12
            IF STREAMSTATUS>100 THEN RESPONSE=-12
            IF RINGSTATUS>100 THEN RESPONSE=-12
            IF HYBSTATUS>100 THEN RESPONSE=-12
            IF HYRSTATUS>100 THEN RESPONSE=-12
            IF PROBESTATUS>100 THEN RESPONSE=-12
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF AUTO_RC=0 THEN RESPONSE=-10
        END

        IF RESPONSE=0 THEN
        BEGIN
            IF PCMOVESTATUS<>-1 THEN MOTIONBUSY=1
            IF LINEARSTATUS<>-1 THEN MOTIONBUSY=1
            IF CIRCSTATUS<>-1 THEN MOTIONBUSY=1
            IF SMOOTHSTATUS<>-1 THEN MOTIONBUSY=1
            IF PATHSTATUS<>-1 THEN MOTIONBUSY=1
            IF STREAMSTATUS<>-1 THEN MOTIONBUSY=1
            IF RINGSTATUS<>-1 THEN MOTIONBUSY=1
            IF HYBSTATUS<>-1 THEN MOTIONBUSY=1
            IF HYRSTATUS<>-1 THEN MOTIONBUSY=1
            IF PROBESTATUS<>-1 THEN MOTIONBUSY=1

            IF MOTIONBUSY=1 THEN RESPONSE=-11
        END

        IF RESPONSE=0 THEN
        BEGIN
            PROBE_RESULT=0
            PROBE_MEAS_X=0.0
            PROBE_MEAS_Y=0.0
            PROBE_MEAS_Z=0.0
            PROBE_MEAS_R=0.0

            MOVE_ERROR=0
            MOVE_STATE=1
            MOVE_PROCESS=10

            START PCPROBE
            RESPONSE=10
        END

        WRITE WIN_1,RESPONSE
    END


    ; CMD39: probe result.
    ; reply: int result + 4 REAL measured XYZR + int current input611
    IF CMD=39 THEN
    BEGIN
        PROBE_IN=0
        IF PROBE_STATE=1 THEN PROBE_IN=1

        WRITE WIN_1,PROBE_RESULT
        WRITE WIN_1,PROBE_MEAS_X
        WRITE WIN_1,PROBE_MEAS_Y
        WRITE WIN_1,PROBE_MEAS_Z
        WRITE WIN_1,PROBE_MEAS_R
        WRITE WIN_1,PROBE_IN
    END


    ; --------------------------------------------------------
    ; CMD 25 - CLEAR / RESET STREAM BUFFERS
    ;
    ; Pouziti:
    ; - po prerusenem uploadu pred START
    ; - po STOP, az PCSTREAM uz neni aktivni
    ; - pred novym stream testem
    ;
    ; response:
    ;  25 = stream slots reset
    ; -11 = PCSTREAM je stale aktivni
    ; --------------------------------------------------------

    IF CMD=25 THEN
    BEGIN

        STREAMSTATUS=CONDITION(PCSTREAM)
        RINGSTATUS=CONDITION(PCRING)
        HYBSTATUS=CONDITION(PCHYB)
        HYRSTATUS=CONDITION(HYRING)
        PROBESTATUS=CONDITION(PCPROBE)
        RESPONSE=25

        IF STREAMSTATUS<>-1 THEN
        BEGIN
            RESPONSE=-11
        END

        IF RINGSTATUS<>-1 THEN
        BEGIN
            RESPONSE=-11
        END

        IF RESPONSE=25 THEN
        BEGIN
            SA_FREE=1
            SB_FREE=1
            SA_SEQ=-1
            SB_SEQ=-1
            STR_BLOCKS=0
            SA_LAST=0
            SB_LAST=0
            STR_EXPECT=0
            STR_UNDER=0
        END

        WRITE WIN_1,RESPONSE

    END


    ; --------------------------------------------------------
    ; CMD 12
    ; STAV POHYBU
    ;
    ; odpoved:
    ; MOVE_STATE
    ; MOVE_ERROR
    ; PROCSTATUS
    ; --------------------------------------------------------

    IF CMD=12 THEN
    BEGIN

        PROCSTATUS=-1
        IF MOVE_PROCESS=1 THEN
        BEGIN
            PROCSTATUS=CONDITION(PCMOVE)
        END

        IF MOVE_PROCESS=2 THEN
        BEGIN
            PROCSTATUS=CONDITION(PCLINEAR)
        END

        IF MOVE_PROCESS=3 THEN
        BEGIN
            PROCSTATUS=CONDITION(PCCIRC)
        END

        IF MOVE_PROCESS=4 THEN
        BEGIN
            PROCSTATUS=CONDITION(PCSMOOTH)
        END

        IF MOVE_PROCESS=5 THEN
        BEGIN
            PROCSTATUS=CONDITION(PCPATH)
        END

        IF MOVE_PROCESS=6 THEN
        BEGIN
            PROCSTATUS=CONDITION(PCSTREAM)
        END

        IF MOVE_PROCESS=7 THEN
        BEGIN
            PROCSTATUS=CONDITION(PCRING)
        END

        IF MOVE_PROCESS=8 THEN
        BEGIN
            PROCSTATUS=CONDITION(PCHYB)
        END

        IF MOVE_PROCESS=9 THEN
        BEGIN
            PROCSTATUS=CONDITION(HYRING)
        END

        IF MOVE_PROCESS=10 THEN
        BEGIN
            PROCSTATUS=CONDITION(PCPROBE)
        END

        ; jeste jednou zkontrolujeme RC error,
        ; aby CMD12 vratil rovnou MOVE_STATE=3

        IF PROCSTATUS>100 THEN
        BEGIN

            MOVE_STATE=3
            MOVE_ERROR=PROCSTATUS

        END


        WRITE WIN_1,MOVE_STATE
        WRITE WIN_1,MOVE_ERROR
        WRITE WIN_1,PROCSTATUS

    END


    ; --------------------------------------------------------
    ; CMD 13
    ; RESET NASEHO MOVE STATE
    ;
    ; POZOR:
    ; toto NERESETUJE skutecny RC error motion procesu.
    ; Resetuje pouze nase promenne.
    ; --------------------------------------------------------

    IF CMD=13 THEN
    BEGIN

        MOVE_STATE=0
        MOVE_ERROR=0
        MOVE_PROCESS=0
        PROCSTATUS=-1

        RESPONSE=13

        WRITE WIN_1,RESPONSE

    END


END


; ============================================================
; CYKLUS
; ============================================================

WAIT 0.05

JUMP Loop


PROGRAM_END
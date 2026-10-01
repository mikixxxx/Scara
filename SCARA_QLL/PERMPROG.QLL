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
; EXTERNI PROCES
; ============================================================

EXTERNAL: PCMOVE
EXTERNAL: PCLINEAR
EXTERNAL: PCCIRC
EXTERNAL: PCSMOOTH
EXTERNAL: PCPATH
EXTERNAL: PCSTREAM


; ============================================================
; RHO VSTUPY
; ============================================================

INPUT BINARY:189=ALARM_RC
INPUT BINARY:193=AUTO_RC

; Zatim ponechavame, ale jejich skutecny vyznam
; jeste nemame overeny.
INPUT BINARY:128=IN_POSITION
INPUT BINARY:129=REFERENCED


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
INTEGER: MOTIONBUSY

INTEGER: MOVE_PROCESS
INTEGER: PATHIDX
INTEGER: STR_SLOT
INTEGER: STR_SEQ
INTEGER: STR_ACCEPT
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

    IF PROCSTATUS>100 THEN
    BEGIN
        MOVE_STATE=3
        MOVE_ERROR=PROCSTATUS
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
    ; INTEGER STR_BLOCKS (1..8)
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
    ; -20 = STR_BLOCKS outside 1..8
    ; -24 = initial A/B buffers not ready
    ; --------------------------------------------------------

    IF CMD=23 THEN
    BEGIN

        READ WIN_1,STR_BLOCKS

        RESPONSE=0
        MOTIONBUSY=0

        IF STR_BLOCKS<1 THEN RESPONSE=-20
        IF STR_BLOCKS>8 THEN RESPONSE=-20

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
        RESPONSE=25

        IF STREAMSTATUS<>-1 THEN
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
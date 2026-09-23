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

; MOVE_STATE:
; 0 = IDLE
; 1 = PCMOVE spusten / pohyb
; 2 = PCMOVE dosel za prikaz MOVE
; 3 = RC ERROR
PUBLIC INTEGER: MOVE_STATE

; Kod RC chyby PCMOVE
; 0 = bez chyby
PUBLIC INTEGER: MOVE_ERROR


; ============================================================
; EXTERNI PROCES
; ============================================================

EXTERNAL: PCMOVE
EXTERNAL: PCLINEAR


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

INTEGER: MOVE_PROCESS
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
MOVE_PROCESS=0
Loop:


; ============================================================
; SLEDOVANI PCMOVE
; ============================================================
;
; PCMOVE sledujeme vzdy, pokud byl spusten.
;
; Pri nasem testu travel range limit jsme namerili:
;
; CONDITION(PCMOVE) = 22144
;
; a ROPS4 ukazoval:
;
; RC error!
; travel range limit
;
; Proto hodnotu >100 povazujeme za RC chybu procesu.
;
; Dulezite:
; sledujeme i MOVE_STATE=2, protoze PCMOVE muze stihnout
; nastavit MOVE_STATE=2 drive, nez se RC error projevi.
;

IF MOVE_STATE<>0 THEN
BEGIN

    IF MOVE_PROCESS=1 THEN
    BEGIN
        PROCSTATUS=CONDITION(PCMOVE)
    END
    IF MOVE_PROCESS=2 THEN
    
    BEGIN
        PROCSTATUS=CONDITION(PCLINEAR)
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
    ; -11 = predchozi pohyb jeste probiha
    ; -12 = PCMOVE je v RC ERROR
    ; --------------------------------------------------------

    IF CMD=10 THEN
    BEGIN

        RESPONSE=0
        PROCSTATUS=CONDITION(PCMOVE)

        ; PCMOVE ma RC chybu
        IF PROCSTATUS>100 THEN
        BEGIN

            MOVE_STATE=3
            MOVE_ERROR=PROCSTATUS
            RESPONSE=-12

        END

        ; nejsme v AUTO
        IF PROCSTATUS<=100 THEN
        BEGIN

            IF AUTO_RC=0 THEN
            BEGIN

                RESPONSE=-10

            END

        ; AUTO
        IF AUTO_RC=1 THEN
        BEGIN

            ; PCMOVE neni aktivni
            IF PROCSTATUS=-1 THEN
            BEGIN

                MOVE_ERROR=0
                MOVE_STATE=1
                MOVE_PROCESS=1
                START PCMOVE

                RESPONSE=10

            END

            ; PCMOVE uz existuje/bezi
            IF PROCSTATUS<>-1 THEN
            BEGIN

                RESPONSE=-11

            END

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
    ; STOP PCMOVE
    ;
    ; odpoved:
    ; 15 = STOP proveden
    ; --------------------------------------------------------

    IF CMD=15 THEN
    BEGIN
        IF CONDITION(PCMOVE)<>-1 THEN STOP PCMOVE
        IF CONDITION(PCLINEAR)<>-1 THEN STOP PCLINEAR

        MOVE_STATE=4
        MOVE_ERROR=0
        RESPONSE=15
        WRITE WIN_1,RESPONSE
    END

    IF CMD=16 THEN
    BEGIN
        READ WIN_1,TARGET_X
        READ WIN_1,TARGET_Y
        READ WIN_1,TARGET_Z
        READ WIN_1,TARGET_R
        READ WIN_1,LINEAR_SPEED

        RESPONSE=0
        PROCSTATUS=CONDITION(PCLINEAR)

        IF PROCSTATUS>100 THEN
        BEGIN
            MOVE_STATE=3
            MOVE_ERROR=PROCSTATUS
            RESPONSE=-12
        END

        IF PROCSTATUS<=100 THEN
        BEGIN
            IF AUTO_RC=0 THEN
            BEGIN
                RESPONSE=-10
            END

            IF AUTO_RC=1 THEN
            BEGIN
                IF PROCSTATUS=-1 THEN
                BEGIN
                    MOVE_ERROR=0
                    MOVE_STATE=1
                    MOVE_PROCESS=2
                    START PCLINEAR
                    RESPONSE=10
                END

                IF PROCSTATUS<>-1 THEN
                BEGIN
                    RESPONSE=-11
                END
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
    ; toto NERESETUJE skutecny RC error procesu PCMOVE.
    ; Resetuje pouze nase promenne.
    ; --------------------------------------------------------

    IF CMD=13 THEN
    BEGIN

        MOVE_STATE=0
        MOVE_ERROR=0

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
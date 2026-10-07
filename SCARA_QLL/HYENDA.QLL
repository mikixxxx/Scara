;;CONTROL = rho4
;;INT = LINEAR
;;KINEMATICS: (1=ROBI1)
;;ROBI1.JC_NAMES = A1,A2,A3,A4
;;ROBI1.WC_NAMES = K1,K2,K3,K4

PROGRAM HYENDA

EXTERNAL PERMPROG: HA_END
EXTERNAL PERMPROG: HA_MID
EXTERNAL PERMPROG: HA_V
EXTERNAL PERMPROG: HA_TYPE
EXTERNAL PERMPROG: HA_COUNT

BEGIN

PROGR_SLOPE

IF HA_COUNT>=1 THEN
BEGIN
    IF HA_COUNT=1 THEN
    BEGIN
        IF HA_TYPE[1]=1 THEN
        BEGIN
            MOVE LINEAR WITH V=HA_V[1] TO HA_END[1]
        END
        ELSE
        BEGIN
            MOVE CIRCULAR WITH V=HA_V[1] TO (HA_MID[1],HA_END[1])
        END
    END
    ELSE
    BEGIN
        IF HA_TYPE[1]=1 THEN
        BEGIN
            MOVE LINEAR WITH V=HA_V[1] VIA HA_END[1]
        END
        ELSE
        BEGIN
            MOVE CIRCULAR WITH V=HA_V[1] VIA (HA_MID[1],HA_END[1])
        END
    END
END

IF HA_COUNT>=2 THEN
BEGIN
    IF HA_COUNT=2 THEN
    BEGIN
        IF HA_TYPE[2]=1 THEN
        BEGIN
            MOVE LINEAR WITH V=HA_V[2] TO HA_END[2]
        END
        ELSE
        BEGIN
            MOVE CIRCULAR WITH V=HA_V[2] TO (HA_MID[2],HA_END[2])
        END
    END
    ELSE
    BEGIN
        IF HA_TYPE[2]=1 THEN
        BEGIN
            MOVE LINEAR WITH V=HA_V[2] VIA HA_END[2]
        END
        ELSE
        BEGIN
            MOVE CIRCULAR WITH V=HA_V[2] VIA (HA_MID[2],HA_END[2])
        END
    END
END

IF HA_COUNT>=3 THEN
BEGIN
    IF HA_COUNT=3 THEN
    BEGIN
        IF HA_TYPE[3]=1 THEN
        BEGIN
            MOVE LINEAR WITH V=HA_V[3] TO HA_END[3]
        END
        ELSE
        BEGIN
            MOVE CIRCULAR WITH V=HA_V[3] TO (HA_MID[3],HA_END[3])
        END
    END
    ELSE
    BEGIN
        IF HA_TYPE[3]=1 THEN
        BEGIN
            MOVE LINEAR WITH V=HA_V[3] VIA HA_END[3]
        END
        ELSE
        BEGIN
            MOVE CIRCULAR WITH V=HA_V[3] VIA (HA_MID[3],HA_END[3])
        END
    END
END

IF HA_COUNT>=4 THEN
BEGIN
    IF HA_COUNT=4 THEN
    BEGIN
        IF HA_TYPE[4]=1 THEN
        BEGIN
            MOVE LINEAR WITH V=HA_V[4] TO HA_END[4]
        END
        ELSE
        BEGIN
            MOVE CIRCULAR WITH V=HA_V[4] TO (HA_MID[4],HA_END[4])
        END
    END
    ELSE
    BEGIN
        IF HA_TYPE[4]=1 THEN
        BEGIN
            MOVE LINEAR WITH V=HA_V[4] VIA HA_END[4]
        END
        ELSE
        BEGIN
            MOVE CIRCULAR WITH V=HA_V[4] VIA (HA_MID[4],HA_END[4])
        END
    END
END

IF HA_COUNT>=5 THEN
BEGIN
    IF HA_COUNT=5 THEN
    BEGIN
        IF HA_TYPE[5]=1 THEN
        BEGIN
            MOVE LINEAR WITH V=HA_V[5] TO HA_END[5]
        END
        ELSE
        BEGIN
            MOVE CIRCULAR WITH V=HA_V[5] TO (HA_MID[5],HA_END[5])
        END
    END
    ELSE
    BEGIN
        IF HA_TYPE[5]=1 THEN
        BEGIN
            MOVE LINEAR WITH V=HA_V[5] VIA HA_END[5]
        END
        ELSE
        BEGIN
            MOVE CIRCULAR WITH V=HA_V[5] VIA (HA_MID[5],HA_END[5])
        END
    END
END

IF HA_COUNT>=6 THEN
BEGIN
    IF HA_COUNT=6 THEN
    BEGIN
        IF HA_TYPE[6]=1 THEN
        BEGIN
            MOVE LINEAR WITH V=HA_V[6] TO HA_END[6]
        END
        ELSE
        BEGIN
            MOVE CIRCULAR WITH V=HA_V[6] TO (HA_MID[6],HA_END[6])
        END
    END
    ELSE
    BEGIN
        IF HA_TYPE[6]=1 THEN
        BEGIN
            MOVE LINEAR WITH V=HA_V[6] VIA HA_END[6]
        END
        ELSE
        BEGIN
            MOVE CIRCULAR WITH V=HA_V[6] VIA (HA_MID[6],HA_END[6])
        END
    END
END

IF HA_COUNT>=7 THEN
BEGIN
    IF HA_COUNT=7 THEN
    BEGIN
        IF HA_TYPE[7]=1 THEN
        BEGIN
            MOVE LINEAR WITH V=HA_V[7] TO HA_END[7]
        END
        ELSE
        BEGIN
            MOVE CIRCULAR WITH V=HA_V[7] TO (HA_MID[7],HA_END[7])
        END
    END
    ELSE
    BEGIN
        IF HA_TYPE[7]=1 THEN
        BEGIN
            MOVE LINEAR WITH V=HA_V[7] VIA HA_END[7]
        END
        ELSE
        BEGIN
            MOVE CIRCULAR WITH V=HA_V[7] VIA (HA_MID[7],HA_END[7])
        END
    END
END

IF HA_COUNT>=8 THEN
BEGIN
    IF HA_COUNT=8 THEN
    BEGIN
        IF HA_TYPE[8]=1 THEN
        BEGIN
            MOVE LINEAR WITH V=HA_V[8] TO HA_END[8]
        END
        ELSE
        BEGIN
            MOVE CIRCULAR WITH V=HA_V[8] TO (HA_MID[8],HA_END[8])
        END
    END
    ELSE
    BEGIN
        IF HA_TYPE[8]=1 THEN
        BEGIN
            MOVE LINEAR WITH V=HA_V[8] VIA HA_END[8]
        END
        ELSE
        BEGIN
            MOVE CIRCULAR WITH V=HA_V[8] VIA (HA_MID[8],HA_END[8])
        END
    END
END

IF HA_COUNT>=9 THEN
BEGIN
    IF HA_COUNT=9 THEN
    BEGIN
        IF HA_TYPE[9]=1 THEN
        BEGIN
            MOVE LINEAR WITH V=HA_V[9] TO HA_END[9]
        END
        ELSE
        BEGIN
            MOVE CIRCULAR WITH V=HA_V[9] TO (HA_MID[9],HA_END[9])
        END
    END
    ELSE
    BEGIN
        IF HA_TYPE[9]=1 THEN
        BEGIN
            MOVE LINEAR WITH V=HA_V[9] VIA HA_END[9]
        END
        ELSE
        BEGIN
            MOVE CIRCULAR WITH V=HA_V[9] VIA (HA_MID[9],HA_END[9])
        END
    END
END

IF HA_COUNT>=10 THEN
BEGIN
    IF HA_COUNT=10 THEN
    BEGIN
        IF HA_TYPE[10]=1 THEN
        BEGIN
            MOVE LINEAR WITH V=HA_V[10] TO HA_END[10]
        END
        ELSE
        BEGIN
            MOVE CIRCULAR WITH V=HA_V[10] TO (HA_MID[10],HA_END[10])
        END
    END
    ELSE
    BEGIN
        IF HA_TYPE[10]=1 THEN
        BEGIN
            MOVE LINEAR WITH V=HA_V[10] VIA HA_END[10]
        END
        ELSE
        BEGIN
            MOVE CIRCULAR WITH V=HA_V[10] VIA (HA_MID[10],HA_END[10])
        END
    END
END

IF HA_COUNT>=11 THEN
BEGIN
    IF HA_COUNT=11 THEN
    BEGIN
        IF HA_TYPE[11]=1 THEN
        BEGIN
            MOVE LINEAR WITH V=HA_V[11] TO HA_END[11]
        END
        ELSE
        BEGIN
            MOVE CIRCULAR WITH V=HA_V[11] TO (HA_MID[11],HA_END[11])
        END
    END
    ELSE
    BEGIN
        IF HA_TYPE[11]=1 THEN
        BEGIN
            MOVE LINEAR WITH V=HA_V[11] VIA HA_END[11]
        END
        ELSE
        BEGIN
            MOVE CIRCULAR WITH V=HA_V[11] VIA (HA_MID[11],HA_END[11])
        END
    END
END

IF HA_COUNT>=12 THEN
BEGIN
    IF HA_COUNT=12 THEN
    BEGIN
        IF HA_TYPE[12]=1 THEN
        BEGIN
            MOVE LINEAR WITH V=HA_V[12] TO HA_END[12]
        END
        ELSE
        BEGIN
            MOVE CIRCULAR WITH V=HA_V[12] TO (HA_MID[12],HA_END[12])
        END
    END
    ELSE
    BEGIN
        IF HA_TYPE[12]=1 THEN
        BEGIN
            MOVE LINEAR WITH V=HA_V[12] VIA HA_END[12]
        END
        ELSE
        BEGIN
            MOVE CIRCULAR WITH V=HA_V[12] VIA (HA_MID[12],HA_END[12])
        END
    END
END

IF HA_COUNT>=13 THEN
BEGIN
    IF HA_COUNT=13 THEN
    BEGIN
        IF HA_TYPE[13]=1 THEN
        BEGIN
            MOVE LINEAR WITH V=HA_V[13] TO HA_END[13]
        END
        ELSE
        BEGIN
            MOVE CIRCULAR WITH V=HA_V[13] TO (HA_MID[13],HA_END[13])
        END
    END
    ELSE
    BEGIN
        IF HA_TYPE[13]=1 THEN
        BEGIN
            MOVE LINEAR WITH V=HA_V[13] VIA HA_END[13]
        END
        ELSE
        BEGIN
            MOVE CIRCULAR WITH V=HA_V[13] VIA (HA_MID[13],HA_END[13])
        END
    END
END

IF HA_COUNT>=14 THEN
BEGIN
    IF HA_COUNT=14 THEN
    BEGIN
        IF HA_TYPE[14]=1 THEN
        BEGIN
            MOVE LINEAR WITH V=HA_V[14] TO HA_END[14]
        END
        ELSE
        BEGIN
            MOVE CIRCULAR WITH V=HA_V[14] TO (HA_MID[14],HA_END[14])
        END
    END
    ELSE
    BEGIN
        IF HA_TYPE[14]=1 THEN
        BEGIN
            MOVE LINEAR WITH V=HA_V[14] VIA HA_END[14]
        END
        ELSE
        BEGIN
            MOVE CIRCULAR WITH V=HA_V[14] VIA (HA_MID[14],HA_END[14])
        END
    END
END

IF HA_COUNT>=15 THEN
BEGIN
    IF HA_COUNT=15 THEN
    BEGIN
        IF HA_TYPE[15]=1 THEN
        BEGIN
            MOVE LINEAR WITH V=HA_V[15] TO HA_END[15]
        END
        ELSE
        BEGIN
            MOVE CIRCULAR WITH V=HA_V[15] TO (HA_MID[15],HA_END[15])
        END
    END
    ELSE
    BEGIN
        IF HA_TYPE[15]=1 THEN
        BEGIN
            MOVE LINEAR WITH V=HA_V[15] VIA HA_END[15]
        END
        ELSE
        BEGIN
            MOVE CIRCULAR WITH V=HA_V[15] VIA (HA_MID[15],HA_END[15])
        END
    END
END

IF HA_COUNT>=16 THEN
BEGIN
    IF HA_COUNT=16 THEN
    BEGIN
        IF HA_TYPE[16]=1 THEN
        BEGIN
            MOVE LINEAR WITH V=HA_V[16] TO HA_END[16]
        END
        ELSE
        BEGIN
            MOVE CIRCULAR WITH V=HA_V[16] TO (HA_MID[16],HA_END[16])
        END
    END
    ELSE
    BEGIN
        IF HA_TYPE[16]=1 THEN
        BEGIN
            MOVE LINEAR WITH V=HA_V[16] VIA HA_END[16]
        END
        ELSE
        BEGIN
            MOVE CIRCULAR WITH V=HA_V[16] VIA (HA_MID[16],HA_END[16])
        END
    END
END

HALT

PROGRAM_END


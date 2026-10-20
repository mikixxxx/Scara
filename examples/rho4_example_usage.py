"""
RHO4 / SCARA - ukazkove pouziti finalniho Python driveru
=======================================================

Soubor predpoklada, ze ve stejne slozce je:
    rho4_driver.py

Ukazuje:
    - pripojeni / odpojeni od robota
    - PING a stav robota
    - cteni XYZR a kloubu A1..A4
    - PTP pohyb
    - LINEAR pohyb
    - CIRCULAR pohyb
    - JOG + STOP
    - cteni fast vstupu sondy
    - zapnuti/vypnuti vystupu X11 (spindle M3/M5 signal)
    - rychlou Z sondu MOVE ... UNTIL

DULEZITE:
    Pohybove a vystupni priklady jsou v main() standardne VYPNUTE.
    Zapinej je jednotlive az po kontrole souradnic a pracovního prostoru.

    rho.stop() / jog_stop() je rizeny softwarovy STOP.
    NENI to bezpecnostni E-STOP / TOTAL STOP.
"""

import time

from rho4_driver import Rho4


# ---------------------------------------------------------------------------
# NASTAVENI PRIPOJENI
# ---------------------------------------------------------------------------

ROBOT_IP = "192.168.4.1"
ROBOT_PORT = 6051
SOCKET_TIMEOUT = 3.0


def print_robot_state(rho: Rho4):
    """Vypise zakladni stav, TCP pozici a kloubove souradnice."""

    print("\n--- ROBOT STATE ---")

    # CMD1 - jednoduchy test komunikace.
    print("PING:", rho.ping())

    # CMD3 - stav controlleru.
    # automatic=True je potreba pro pohyb z PC.
    status = rho.get_status()
    print("STATUS:", status)

    # CMD2 - pracovni souradnice TCP:
    # X/Y/Z v mm, R ve stupnich.
    x, y, z, r = rho.get_position()
    print(
        f"XYZR: X={x:.3f} mm  Y={y:.3f} mm  "
        f"Z={z:.3f} mm  R={r:.3f} deg"
    )

    # CMD17 - skutecne kloubove souradnice robota.
    a1, a2, a3, a4 = rho.get_joint_position()
    print(
        f"JOINTS: A1={a1:.3f} deg  A2={a2:.3f} deg  "
        f"A3={a3:.3f} mm  A4={a4:.3f} deg"
    )

    # CMD12 - stav aktualniho pohyboveho procesu.
    state, error, proc = rho.get_move_state()
    print(
        f"MOVE STATE: state={state}, error={error}, "
        f"proc_status={proc}"
    )


# ---------------------------------------------------------------------------
# PTP
# ---------------------------------------------------------------------------

def example_ptp(rho: Rho4):
    """
    Bodovy PTP pohyb.

    PTP rychlost je v PROCENTECH, ne v mm/s.
    Driver povoluje 0.01 .. 80.0 %.

    V tomto prikladu vezmeme aktualni polohu a otocime R o +2 stupne.
    Pred spustenim vzdy zkontroluj, ze je cil bezpecny.
    """

    x, y, z, r = rho.get_position()

    target_r = r + 2.0

    print(
        f"\nPTP: R {r:.3f} -> {target_r:.3f} deg"
    )

    # Blokujici varianta:
    # funkce se vrati az po dokonceni pohybu.
    ok = rho.move_ptp(
        x,
        y,
        z,
        target_r,
        speed=2.0,       # 2 % PTP rychlosti
        timeout=30.0,
    )

    print("PTP DONE:", ok)


# ---------------------------------------------------------------------------
# LINEAR
# ---------------------------------------------------------------------------

def example_linear(rho: Rho4):
    """
    Primocary kartezsky pohyb.

    speed_mm_s je v mm/s.

    Priklad zvedne Z o 2 mm pri zachovani X/Y/R.
    """

    x, y, z, r = rho.get_position()

    target_z = z + 2.0

    print(
        f"\nLINEAR: Z {z:.3f} -> {target_z:.3f} mm"
    )

    ok = rho.move_linear(
        x,
        y,
        target_z,
        r,
        speed_mm_s=2.0,
        timeout=30.0,
    )

    print("LINEAR DONE:", ok)


# ---------------------------------------------------------------------------
# CIRCULAR
# ---------------------------------------------------------------------------

def example_circular(rho: Rho4):
    """
    Nativni RHO4 CIRCULAR pohyb.

    CIRCULAR potrebuje:
        - MID bod ležici na pozadovanem oblouku
        - END koncovy bod

    Aktualni TCP je automaticky START oblouku.

    Nize je pouze UKAZKA relativni geometrie:
        START = aktualni poloha
        MID   = X+5, Y+5
        END   = X+10, Y

    Pred skutecnym pouzitim zkontroluj, ze vsechny body lezi v pracovnim
    prostoru robota. R a Z zde nechavame stejne.
    """

    x, y, z, r = rho.get_position()

    mid_x = x + 5.0
    mid_y = y + 5.0

    end_x = x + 10.0
    end_y = y

    print(
        "\nCIRCULAR:"
        f"\n  START = ({x:.3f}, {y:.3f})"
        f"\n  MID   = ({mid_x:.3f}, {mid_y:.3f})"
        f"\n  END   = ({end_x:.3f}, {end_y:.3f})"
    )

    ok = rho.move_circular(
        mid_x,
        mid_y,
        z,
        r,
        end_x,
        end_y,
        z,
        r,
        speed_mm_s=2.0,
        timeout=30.0,
    )

    print("CIRCULAR DONE:", ok)


# ---------------------------------------------------------------------------
# JOG
# ---------------------------------------------------------------------------

def example_jog(rho: Rho4):
    """
    Jednoduchy JOG.

    Nas jog_start() neni nekonecny velocity jog.
    Driver vytvori PTP cil ve zvolenem smeru a pri jog_stop() posle
    softwarovy STOP.

    axis:
        "X", "Y", "Z", "R"

    direction:
        +1 nebo -1

    speed:
        PTP rychlost v procentech

    distance:
        X/Y/Z = mm
        R     = stupne
    """

    print("\nJOG R+...")

    rho.jog_start(
        axis="R",
        direction=+1,
        speed=0.5,
        distance=5.0,
    )

    # Jen demonstrace: nechame pohyb bezet 0.5 s.
    time.sleep(0.5)

    # Rizeny software STOP - neni to safety E-STOP.
    rho.jog_stop()

    print("JOG STOP")


# ---------------------------------------------------------------------------
# CTENI FAST VSTUPU
# ---------------------------------------------------------------------------

def example_read_input(rho: Rho4):
    """
    Precte stav aktualne implementovaneho fast vstupu sondy.

    Fyzicky jde o prvni high-speed input:
        bezne cteni v PERMPROG  -> kanal 801
        MOVE ... UNTIL mereni    -> kanal 611

    get_io_status() vraci i informace o spindle vystupu.
    """

    io = rho.get_io_status()

    print("\n--- I/O STATUS ---")
    print("I/O armed       :", io["io_armed"])
    print("Spindle state   :", io["spindle_state"])
    print("Probe input     :", io["probe_input"])
    print("Spindle channel :", io["spindle_channel"])
    print("Probe channel   :", io["probe_channel"])

    if io["probe_input"]:
        print("SONDA / FAST INPUT = 1")
    else:
        print("SONDA / FAST INPUT = 0")


# ---------------------------------------------------------------------------
# NASTAVENI VYSTUPU X11
# ---------------------------------------------------------------------------

def example_output(rho: Rho4):
    """
    Ovlada fyzicky vystup X11, ktery jsme overili jako BAPS OUTPUT 801.

    set_spindle() zamerne NEFUNGUJE, dokud neni I/O explicitne ARM.
    Je to ochrana proti nahodnemu zapnuti vystupu po startu programu.

    get_io_status()["spindle_state"] je SOFTWARE MIRROR prikazu,
    nikoliv elektricka zpetna vazba z fyzickeho pinu.
    """

    print("\nARM I/O...")

    # CMD35:
    # pri ARM controller nejprve nastavi vystup na LOW/OFF.
    print(rho.arm_io(True))

    try:
        # CMD36 -> fyzicky X11 output ON.
        print("OUTPUT ON")
        print(rho.set_spindle(True))

        time.sleep(1.0)

        # CMD36 -> fyzicky X11 output OFF.
        print("OUTPUT OFF")
        print(rho.set_spindle(False))

    finally:
        # Vzdy se pokusime odejit s vystupem vypnutym a I/O DISARM.
        try:
            rho.set_spindle(False)
        finally:
            print("DISARM I/O")
            print(rho.arm_io(False))


# ---------------------------------------------------------------------------
# FAST Z PROBE
# ---------------------------------------------------------------------------

def example_probe(rho: Rho4):
    """
    Ukazka hardwarove Z sondy.

    RHO4 provede nativni:
        MOVE LINEAR ... UNTIL fast_input

    a pri triggeru ulozi presnou meritelnou polohu do @MPOS.

    Aktualni implementace je zamerne pouze pro Z.
    Maximalni probe travel v driveru je omezen na 25 mm.

    TENTO PRIKLAD NEPOUSTEJ BEZ FYZICKE SONDY.
    """

    x, y, z, r = rho.get_position()

    # Hledame kontakt maximalne 3 mm smerem dolu.
    target_z = z - 3.0

    print(
        f"\nPROBE: Z {z:.3f} -> {target_z:.3f} mm"
    )

    result = rho.probe_linear(
        x,
        y,
        target_z,
        r,
        speed_mm_s=0.5,
        timeout=15.0,
    )

    print("PROBE RESULT:", result)

    if result and result["hit"]:
        mx, my, mz, mr = result["measured"]

        print(
            "TRIGGER @MPOS:"
            f" X={mx:.4f}"
            f" Y={my:.4f}"
            f" Z={mz:.4f}"
            f" R={mr:.4f}"
        )


# ---------------------------------------------------------------------------
# MAIN
# ---------------------------------------------------------------------------

def main():
    """
    Hlavni ukazka.

    Po spusteni se program pouze PRIPOJI A CTE STAV.
    Pohyby, vystup a probe jsou standardne vypnute pomoci flagu nize.
    """

    # Bezpecne vychozi nastaveni:
    RUN_PTP_DEMO = False
    RUN_LINEAR_DEMO = False
    RUN_CIRCULAR_DEMO = False
    RUN_JOG_DEMO = False
    RUN_OUTPUT_DEMO = False
    RUN_PROBE_DEMO = False

    rho = Rho4(
        ROBOT_IP,
        ROBOT_PORT,
        timeout=SOCKET_TIMEOUT,
    )

    try:
        # ---------------------------------------------------------------
        # PRIPOJENI
        # ---------------------------------------------------------------
        print(
            f"Pripojuji se k RHO4 "
            f"{ROBOT_IP}:{ROBOT_PORT}..."
        )

        rho.connect()
        print("CONNECTED")

        # ---------------------------------------------------------------
        # ZAKLADNI CTENI
        # ---------------------------------------------------------------
        print_robot_state(rho)
        example_read_input(rho)

        # ---------------------------------------------------------------
        # POHYBOVE PRIKLADY
        #
        # Zapinej po jednom.
        # ---------------------------------------------------------------
        if RUN_PTP_DEMO:
            example_ptp(rho)

        if RUN_LINEAR_DEMO:
            example_linear(rho)

        if RUN_CIRCULAR_DEMO:
            example_circular(rho)

        if RUN_JOG_DEMO:
            example_jog(rho)

        # ---------------------------------------------------------------
        # VYSTUP X11 / SPINDLE ENABLE
        #
        # Aktivuj az po overeni pripojeneho hardware.
        # ---------------------------------------------------------------
        if RUN_OUTPUT_DEMO:
            example_output(rho)

        # ---------------------------------------------------------------
        # FAST PROBE
        #
        # Aktivuj az s fyzickou sondou.
        # ---------------------------------------------------------------
        if RUN_PROBE_DEMO:
            example_probe(rho)

    except KeyboardInterrupt:
        print("\nPreruseno uzivatelem.")

        # Pokus o rizeny STOP pohybu.
        # Neni to safety-rated E-STOP.
        try:
            rho.stop()
        except Exception:
            pass

    except Exception as exc:
        print("\nCHYBA:", exc)

    finally:
        # Pokud by zustal I/O armovany, pokusime se jej pred odpojenim
        # dostat do bezpecneho stavu.
        try:
            if rho.is_connected():
                io = rho.get_io_status()

                if io["io_armed"]:
                    try:
                        rho.set_spindle(False)
                    finally:
                        rho.arm_io(False)

        except Exception as exc:
            print("I/O shutdown warning:", exc)

        # TCP odpojeni.
        rho.disconnect()
        print("DISCONNECTED")


if __name__ == "__main__":
    main()

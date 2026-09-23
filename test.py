from rho4 import Rho4

rho = Rho4("192.168.4.1", 6051)
rho.connect()

try:

    x, y, z, r = rho.get_position()

    try:
        rho.start_linear(
            650.0,   # mimo maximalni dosah 600 mm
            0.0,
            z,
            r,
            speed_mm_s=5.0
        )

        print("CHYBA: pohyb byl povolen!")

    except ValueError as e:
        print("SPRAVNE ZABLOKOVANO:")
        print(e)

finally:
    rho.disconnect()
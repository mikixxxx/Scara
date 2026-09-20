from rho4 import Rho4

rho = Rho4(
    host="127.0.0.1",
    port=6051
)

rho.connect()

try:
    speed = 30.0

    points = [
        (300, 300, 150, 0),
        (350, 300, 150, 0),
        (350, 350, 150, 0),
        (300, 350, 150, 0),
        (300, 300, 150, 0),
    ]

    for p in points:
        print("MOVE:", p)

        rho.move_linear(
            p[0],
            p[1],
            p[2],
            p[3],
            speed_mm_s=speed,
            timeout=10.0
        )

    print("HOTOVO")

finally:
    rho.disconnect()
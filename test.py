from rho4 import Rho4


rho = Rho4("127.0.0.1", 6051)
rho.connect()

x, y, z, r = rho.get_position()

path = rho.check_linear_path(
    x - 50,
    y + 50,
    z,
    r,
    step_mm=2.0
)

print("Vzorku:", len(path))

for p in path:
    print(
        f"{p['t']*100:6.1f}%  "
        f"XY=({p['x']:8.3f},{p['y']:8.3f})  "
        f"A1={p['a1']:8.3f} "
        f"A2={p['a2']:8.3f} "
        f"A4={p['a4']:8.3f}"
    )

rho.disconnect()
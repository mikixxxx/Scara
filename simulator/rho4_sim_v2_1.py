"""RHO4 protocol simulator V2.1 for rho4_driver.py / rho4_gcode_runner.py.
LOCALHOST ONLY. A test harness, NOT a real robot or safety/physics model.
Native G1/G2/G3 HYRING, standalone G0, PTP, I/O and Z probe.
"""
import argparse
import math
import socket
import struct
import threading
import time

class Simulator:
    def __init__(self, speedup=1.0):
        self.lock = threading.RLock()
        self.pos = [300., 300., 180., 0.]
        self.target = self.pos[:]
        self.joints = self._joint_for_pose(self.pos, None)
        self.speed = .02
        self.speedup = speedup
        self.auto = True
        self.alarm = False
        self.ref = True
        self.state, self.error, self.proc = 0, 0, -1
        self.generation = 0
        self.io_armed = False
        self.spindle = False
        self.probe_input = False
        self.probe_z = None
        self.probe_result = 0
        self.probe_measured = self.pos[:]
        self.rings = {26: [None,None], 30: [None,None]}
        self.expected = {26: 0, 30: 0}
        self.underrun = {26: False, 30: False}
        self.active_ring = None

    @staticmethod
    def _angle_delta(a, b):
        return (a - b + 180.0) % 360.0 - 180.0

    @classmethod
    def _joint_for_pose(cls, p, previous):
        """Choose a reachable SR6 elbow branch; follows driver geometry.

        The simulator does not implement actual servo motion or controller limits.
        This is sufficient for CMD17 and the Python driver's dry-run planner.
        """
        x, y, z, r = (float(v) for v in p)
        l1, l2 = 330.0, 270.0
        cos2 = (x*x + y*y - l1*l1 - l2*l2)/(2*l1*l2)
        if not -1.0 <= cos2 <= 1.0:
            # Do not invent valid joint readings for unreachable XY.
            return list(previous) if previous is not None else [0., 0., z, r]
        a2_abs = math.acos(max(-1.0, min(1.0, cos2)))
        candidates = []
        for a2 in (a2_abs, -a2_abs):
            a1 = math.atan2(y,x)-math.atan2(l2*math.sin(a2), l1+l2*math.cos(a2))
            a1 = math.atan2(math.sin(a1),math.cos(a1))
            d1, d2 = math.degrees(a1), math.degrees(a2)
            a4 = r-d1-d2
            if -140.0<=d1<=140.0 and -150.0<=d2<=150.0 and -180.0<=a4<=180.0:
                candidates.append([d1,d2,z,a4])
        if not candidates:
            return list(previous) if previous is not None else [0.,0.,z,r]
        if previous is None:
            return candidates[0]
        return min(candidates, key=lambda c: sum(cls._angle_delta(c[i],previous[i])**2 for i in (0,1,3)))

    def _update_position(self, p):
        previous = self.joints
        self.pos = list(p)
        self.joints = self._joint_for_pose(self.pos, previous)

    @staticmethod
    def _circular_points(start, mid, end):
        """XY circular interpolation for the G2/G3 native primitives.

        RHO4's real native primitive is 3D; this test simulator covers the
        constant-Z/constant-R XY arcs emitted by the current G-code runner.
        """
        ax, ay = start[:2]
        bx, by = mid[:2]
        cx, cy = end[:2]
        det = 2.0*(ax*(by-cy)+bx*(cy-ay)+cx*(ay-by))
        if abs(det) < 1e-9:
            return [list(mid),list(end)]
        a2 = ax*ax+ay*ay
        b2 = bx*bx+by*by
        c2 = cx*cx+cy*cy
        ux = (a2*(by-cy)+b2*(cy-ay)+c2*(ay-by))/det
        uy = (a2*(cx-bx)+b2*(ax-cx)+c2*(bx-ax))/det
        radius = math.hypot(ax-ux, ay-uy)
        first = math.atan2(ay-uy,ax-ux)
        middle = math.atan2(by-uy,bx-ux)
        last = math.atan2(cy-uy,cx-ux)
        ccw_total = (last-first)%(2*math.pi)
        ccw_mid = (middle-first)%(2*math.pi)
        sweep = ccw_total if 0.0 < ccw_mid < ccw_total else -((first-last)%(2*math.pi))
        segments = max(2,min(64,int(math.ceil(abs(sweep)*radius/8.0))))
        points = []
        for i in range(1,segments+1):
            u=i/segments
            a=first+sweep*u
            points.append([ux+radius*math.cos(a),uy+radius*math.sin(a),
                           start[2]+(end[2]-start[2])*u,
                           start[3]+(end[3]-start[3])*u])
        points[-1] = list(end)
        return points

    def move(self, points, speed, probe=False):
        with self.lock:
            if not self.auto: return -10
            if self.state == 1: return -11
            if self.state == 3 or self.alarm: return -12
            self.generation += 1
            gen = self.generation
            self.state, self.error, self.proc = 1, 0, 1
        threading.Thread(target=self._motion, args=(gen,points,speed,probe), daemon=True).start()
        return 10

    def _motion(self, gen, points, speed, probe=False):
        for endpoint in points:
            with self.lock: start=self.pos[:]
            distance = math.sqrt(sum((endpoint[i]-start[i])**2 for i in range(3)))
            duration = max(.025, distance/max(speed, .05)/self.speedup)
            t0 = time.monotonic()
            while True:
                u = min(1., (time.monotonic()-t0)/duration)
                with self.lock:
                    if gen != self.generation: return
                    self._update_position([start[i]+u*(endpoint[i]-start[i]) for i in range(4)])
                    if probe and self.probe_z is not None:
                        z0, z1 = start[2], endpoint[2]
                        if min(z0,z1) <= self.probe_z <= max(z0,z1):
                            if (z0 >= self.probe_z >= self.pos[2]) or (z0 <= self.probe_z <= self.pos[2]):
                                self.pos[2] = self.probe_z
                                self.joints[2] = self.probe_z
                                self.probe_input = True
                                self.probe_result = 1
                                self.probe_measured = self.pos[:]
                                self.state, self.proc = 2, -1
                                return
                if u >= 1: break
                time.sleep(.01)
        with self.lock:
            if gen == self.generation:
                if probe:
                    self.probe_result = 2
                    self.probe_measured = self.pos[:]
                    self.state, self.error, self.proc = 3, 29010, -1
                else:
                    self.state, self.error, self.proc = 2, 0, -1

    def ring_worker(self, kind, gen):
        while True:
            with self.lock:
                if gen != self.generation: return
                seq = self.expected[kind]
                slot = seq % 2
                block = self.rings[kind][slot]
                if block is not None and block['seq'] == seq:
                    self.rings[kind][slot] = None
                    self.expected[kind] += 1
                else:
                    block = None
            if block is None:
                with self.lock:
                    if gen != self.generation: return
                    self.underrun[kind] = True
                    self.state, self.error, self.proc = 3, (29002 if kind == 30 else 29001), -1
                return
            points = block['points']
            for primitive in points:
                if len(primitive)==4:
                    p,speed,ptype,mid=primitive
                else:
                    p,speed=primitive
                    ptype,mid=1,None
                with self.lock: start = self.pos[:]
                if ptype==2:
                    trajectory=self._circular_points(start, mid, p)
                else:
                    trajectory=[p]
                for target in trajectory:
                    self._motion_segment(gen, target, speed)
                    with self.lock:
                        if gen != self.generation: return
            if block['last']:
                with self.lock:
                    if gen == self.generation: self.state,self.proc=2,-1
                return

    def _motion_segment(self, gen, target, speed):
        with self.lock:
            start=self.pos[:]
        duration = max(.004, math.dist(start[:3],target[:3])/max(.05,speed)/self.speedup)
        t0 = time.monotonic()
        while True:
            u = min(1., (time.monotonic()-t0)/duration)
            with self.lock:
                if gen != self.generation: return
                self._update_position([start[i]+u*(target[i]-start[i]) for i in range(4)])
            if u >= 1.: break
            time.sleep(.003)

    def ring_status(self, kind):
        a,b = self.rings[kind]
        common = [int(a is None),int(b is None),a['seq'] if a else -1,b['seq'] if b else -1,int(a['last']) if a else 0,int(b['last']) if b else 0]
        if kind == 30: common += [len(a['points']) if a else 0,len(b['points']) if b else 0]
        return common+[self.expected[kind],int(self.underrun[kind]),self.state,self.error,self.proc]

    def command(self, sock, cmd):
        def read(fmt): return struct.unpack(fmt, recv(sock,struct.calcsize(fmt)))
        def reply(fmt,*values): sock.sendall(struct.pack(fmt,*values))
        with self.lock:
            if cmd == 1: reply('<i',123456)
            elif cmd == 2: reply('<4f',*self.pos)
            elif cmd == 3: reply('<6i',1,0,int(self.alarm),int(self.auto),int(self.state != 1),int(self.ref))
            elif cmd == 11:
                self.target=list(read('<4f'));reply('<4f',*self.target)
            elif cmd == 14:
                self.speed=max(.0001,min(.8,read('<f')[0]));reply('<f',self.speed)
            elif cmd == 10: reply('<i',self.move([self.target[:]],max(1.,self.speed*1000)))
            elif cmd == 12: reply('<3i',self.state,self.error,self.proc)
            elif cmd == 13:
                if self.state != 1:self.state,self.error,self.proc=0,0,-1
                reply('<i',13)
            elif cmd == 15:
                self.generation+=1;self.state,self.error,self.proc=4,0,-1
                self.active_ring=None
                reply('<i',15)
            elif cmd == 16:
                x,y,z,r,s=read('<5f');reply('<i',self.move([[x,y,z,r]],s))
            elif cmd == 17:
                reply('<4f',*self.joints)
            elif cmd == 18:
                v=read('<9f');reply('<i',self.move([list(v[:4]),list(v[4:8])],v[8]))
            elif cmd == 38:
                v=read('<5f');self.probe_result=0;self.probe_input=False
                reply('<i',self.move([list(v[:4])],v[4],probe=True))
            elif cmd == 39: reply('<i4fi',self.probe_result,*self.probe_measured,int(self.probe_input))
            elif cmd == 34: reply('<5i',int(self.io_armed),int(self.spindle),int(self.probe_input),0,611)
            elif cmd == 35:
                v=read('<i')[0]
                if v not in (0,1):reply('<i',-35)
                else:
                    self.io_armed=bool(v)
                    if not v:self.spindle=False
                    reply('<i',35)
            elif cmd == 36:
                v=read('<i')[0]
                if v not in (0,1):reply('<i',-36)
                elif not self.io_armed:reply('<i',-35)
                else:self.spindle=bool(v);reply('<i',36)
            elif cmd in (26,30):
                if cmd == 26:
                    slot,seq,last,speed=read('<3if');values=read('<64f')
                    points=[(list(values[i:i+4]),speed) for i in range(0,64,4)]
                else:
                    slot,seq,last,count=read('<4i')
                    records=[read('<i9f') for _ in range(16)]
                    # type, speed, END XYZR, MID XYZR
                    points=[(list(v[2:6]),v[1],int(v[0]),list(v[6:10]))
                            for v in records[:max(0,min(count,16))]]
                if slot not in (0,1):result=-22
                elif last not in (0,1):result=-26
                elif cmd==30 and not (1<=count<=16):result=-27
                elif cmd==30 and not last and count!=16:result=-29
                elif self.rings[cmd][slot] is not None:result=-23
                elif any(p[1]<=0 for p in points):result=-21
                elif cmd==30 and any(p[2] not in (1,2) for p in points):result=-28
                else:
                    self.rings[cmd][slot]={'seq':seq,'last':bool(last),'points':points};result=cmd
                reply('<i',result)
            elif cmd in (27,31):
                kind={27:26,31:30}[cmd]
                if not self.auto:result=-10
                elif self.state==1:result=-11
                elif self.state==3:result=-12
                elif self.rings[kind][0] is None or self.rings[kind][0]['seq']!=0:result=-24
                else:
                    self.generation+=1;gen=self.generation
                    self.state,self.error,self.proc=1,0,1
                    self.active_ring=kind
                    threading.Thread(target=self.ring_worker,args=(kind,gen),daemon=True).start()
                    result=10
                reply('<i',result)
            elif cmd in (28,32):
                kind={28:26,32:30}[cmd];values=self.ring_status(kind)
                reply('<%di'%len(values),*values)
            elif cmd in (25,33):
                # CMD25 clears legacy PCRING (26), CMD33 clears native HYRING (30).
                kind={25:26,33:30}[cmd]
                if self.state==1 and self.active_ring==kind:
                    reply('<i',-11)
                else:
                    self.rings[kind]=[None,None]
                    self.expected[kind]=0
                    self.underrun[kind]=False
                    reply('<i',cmd)
            else:
                raise ValueError(f'Unsupported CMD {cmd}; connection closed to prevent desync')

def recv(sock,n):
    buf=bytearray()
    while len(buf)<n:
        part=sock.recv(n-len(buf))
        if not part:raise ConnectionError('disconnected')
        buf.extend(part)
    return bytes(buf)

def client(sock,addr,sim):
    print('CONNECT',addr,flush=True)
    with sock:
        try:
            while True:
                cmd=struct.unpack('<i',recv(sock,4))[0]
                sim.command(sock,cmd)
        except (OSError,ConnectionError,ValueError) as e:
            print('DISCONNECT',addr,str(e),flush=True)

def console(sim):
    print('Console: auto 0|1, alarm 0|1, probe_z Z|off, pos, setpos X Y Z R, error N, clear, state')
    while True:
        try: parts=input('SIM2> ').strip().split()
        except EOFError:return
        if not parts:continue
        try:
            with sim.lock:
                k=parts[0]
                if k=='auto':sim.auto=bool(int(parts[1]))
                elif k=='alarm':sim.alarm=bool(int(parts[1]))
                elif k=='probe_z':sim.probe_z=None if parts[1]=='off' else float(parts[1])
                elif k=='pos':print(sim.pos)
                elif k=='setpos':sim._update_position(list(map(float,parts[1:5])))
                elif k=='error':sim.generation+=1;sim.state=3;sim.error=int(parts[1]);sim.proc=-1
                elif k=='clear':sim.state,sim.error,sim.proc=0,0,-1;sim.alarm=False
                elif k=='state':print(sim.state,sim.error,sim.proc,sim.rings,sim.expected)
                else:print('Unknown command')
        except (ValueError,IndexError) as e:print('Invalid argument:',e)

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--port',type=int,default=6051)
    parser.add_argument('--speedup',type=float,default=20.)
    parser.add_argument('--start',type=float,nargs=4,metavar=('X','Y','Z','R'),default=None,
                        help='simulated initial machine position; use to match CAD START')
    parser.add_argument('--probe-z',type=float,default=None,help='virtual Z probe trigger position')
    args=parser.parse_args()
    if args.speedup<=0:parser.error('speedup must be > 0')
    sim=Simulator(args.speedup)
    if args.start is not None:
        sim._update_position(args.start)
    sim.probe_z=args.probe_z
    threading.Thread(target=console,args=(sim,),daemon=True).start()
    with socket.socket() as server:
        server.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        server.bind(('127.0.0.1',args.port));server.listen()
        print(f'RHO4 Simulator V2.1 on 127.0.0.1:{args.port}',flush=True)
        while True:
            sock,addr=server.accept()
            threading.Thread(target=client,args=(sock,addr,sim),daemon=True).start()

if __name__=='__main__':main()

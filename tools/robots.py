"""Robot catalog for the vehicle mode: geometry, wheels, motors, defaults, INI sections and Gazebo SDF.

Sources (see docs/en/ROBOTS.md):
  robotino  Robotino 1.6 — calibrated on 576 recorded runs of the real robot (configs/calibrated_params.json, robotino)
  youbot    KUKA youBot base — youbot_driver config/youbot-base.cfg (wheel radius 0.0475 m, 0.47 x 0.30 m,
            gear 9405/364 = 25.84, kt 0.0335 N m/A, 4000 ticks/rev), youbot_description base.urdf.xacro (22 kg,
            0.570 x 0.360 x 0.100 m, wheel 0.4 kg) + folded arm 6 kg (KUKA 5.3-6.3 kg). Motor R, efficiency: ESTIMATES
            (maxon EC 45 flat with kt = 33.5 mNm/A); I0 = 0.20 A and roller friction mu_rb = 0.104: identified on recorded
            runs of the real robot (one pair for all floors).
  husky     Clearpath Husky A200 — husky_description (wheelbase 0.512 m, track 0.555 m, wheel r 0.1651 m, width
            0.1143 m, wheel 2.637 kg), Clearpath manual (50 kg, 1.0 m/s, 78 000 encoder ticks/m, 10.7 A cont.),
            two motors MMP S22-346F-24V GP52-079 (one per side, chain to both wheels). Motor R, kt: ESTIMATES from the
            catalogue (~60 rpm class gearmotor, ~20 N m continuous); no-load current I0, efficiency eta and the tyre
            lateral grip lat_grip: identified on recorded runs of the real robot (BorealTC, RoboNav).
  rover     MarsSim six-wheel rocker-bogie (geometry identified on Fig. 13, configs: rover). The paper has no motor
            data: generic 24 V gearmotor (ESTIMATE, documented).
  burger    TurtleBot3 Burger — ROBOTIS e-Manual (1 kg, 0.22 m/s, 2x DYNAMIXEL XL430-W250-T: gear 258.5, stall 1.4 N m at
            11.1 V / 1.3 A, no load 57 rpm, 4096 steps/rev), turtlebot3_description (wheels r 0.033 m, track 0.160 m,
            ball caster 0.081 m behind the axle). Motor R, Ce, kt, eta derived from the stall / no-load data.
  waffle    TurtleBot3 Waffle Pi — e-Manual (1.8 kg, 0.26 m/s, 2x XM430-W210-T: gear 212.6, stall 2.7 N m at 11.1 V /
            2.1 A, no load 70 rpm), description (r 0.033, track 0.288 m, two ball casters at x = -0.177, y = +-0.064).
  jackal    Clearpath Jackal — datasheet (17 kg, 2.0 m/s, 500 W drive, 78 000 encoder pulses/m), jackal_description
            (wheel r 0.098 m, wheelbase 0.262 m, track 0.376 m). Skid steer, one motor channel per side (as Husky).
            Gearmotor R, Ce, kt, gear: ESTIMATES (24 V, 250 W per side).
  pioneer   Pioneer 3-DX — Adept datasheet (9 kg, 1.2 m/s, 300 deg/s, foam-filled 195 mm tyres), ARIA (gear 38.3:1,
            2048 encoder counts/motor rev: TicksMM 128), p3dx description (track 0.30 m, swivel caster r 0.035 m at
            x = -0.185 m). 12 V motor R, Ce, kt: ESTIMATES.
Visuals: CAD meshes in models/meshes/<robot>/ (sources and licences: models/meshes/SOURCES.md).
Your own robots: configs/robots/<name>.json (a full definition or "base": <built-in> + changes), docs/en/ROBOTS.md.
"""
import math, os, sys, copy, json, re
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)


def _load():
    return json.load(open(os.path.join(ROOT, 'configs', 'calibrated_params.json')))


ROBOT_DIR = os.path.join(ROOT, 'configs', 'robots')
_TUPLES = ('com', 'I', 'drive', 'size', 'body_off')
_REQUIRED = ('mass', 'com', 'I', 'Jw', 'wheels', 'motor', 'drive', 'size')
_WHEEL_REQUIRED = ('label', 'x', 'y', 'r', 'type', 'group')
_DEFAULTS = dict(chassis='base_link', encoder_ticks=1024, odom_yaw_scale=1.0, speed=0.3, accel=0.5, v_max=1.0, w_max=2.0, map_size=5.0,
                 map_res=0.02, ctrl=dict(kp_xy=1.0, k_yaw=1.5, kx=1.0, ky=6.0, kth=3.0), base_h=0.0, wheel_mass=0.5)
_MOTOR_DEFAULTS = dict(w_eps=20.0, Vd=0.0, ff=1.0)


def user_robots():
    """names of the robot files configs/robots/<name>.json (docs/en/ROBOTS.md, "Your own robot")"""
    if not os.path.isdir(ROBOT_DIR): return []
    return sorted(f[:-5] for f in os.listdir(ROBOT_DIR) if f.endswith('.json') and not f.startswith('_'))


def robot_file(name):
    """path of a robot file: NAME -> configs/robots/NAME.json, or a path to a .json file; None if there is none"""
    if name.endswith('.json') and os.path.exists(name): return name
    p = os.path.join(ROBOT_DIR, name + '.json')
    return p if os.path.exists(p) else None


def _merge(dst, src):
    for k, v in src.items():
        if isinstance(v, dict) and isinstance(dst.get(k), dict): _merge(dst[k], v)
        else: dst[k] = copy.deepcopy(v)
    return dst


def _from_file(path):
    """A robot defined by a JSON file. Either a full definition (the same keys as the built-in robots below), or
    "base": <built-in robot> plus the keys to change; "wheel": {...} changes every wheel of the base robot."""
    D = {k: v for k, v in json.load(open(path)).items() if not k.startswith('_')}
    name = D.pop('name', os.path.splitext(os.path.basename(path))[0])
    if 'base' in D:
        base = D.pop('base')
        if base not in ROBOTS: raise SystemExit(f'{path}: base "{base}" is not a built-in robot ({", ".join(ROBOTS)})')
        if base in ('robotino', 'rover'):
            raise SystemExit(f'{path}: the Gazebo models of robotino and rover are special (omni layout / rocker-bogie); change them with '
                             f'--set name.param=value or in configs/calibrated_params.json, or write a full robot file')
        Rb = robot(base); Rb['look'] = Rb.get('look', base)
        W = D.pop('wheel', None)
        _merge(Rb, D)
        if W:
            for w in Rb['wheels']: w.update(W)
    else:
        miss = [k for k in _REQUIRED if k not in D]
        if miss: raise SystemExit(f'{path}: missing keys {", ".join(miss)} (see docs/en/ROBOTS.md)')
        Rb = _merge(copy.deepcopy(_DEFAULTS), D)
        Rb['motor'] = {**_MOTOR_DEFAULTS, **Rb['motor']}
    Rb['name'] = name
    for k in _TUPLES:
        if k in Rb: Rb[k] = tuple(Rb[k])
    for i, w in enumerate(Rb['wheels']):
        miss = [k for k in _WHEEL_REQUIRED if k not in w]
        if miss: raise SystemExit(f'{path}: wheel {i + 1} misses {", ".join(miss)}')
        w['label'] = str(w['label']); w.setdefault('z', w['r']); w.setdefault('heading_deg', 0)
        w.setdefault('link', f"wheel_{w['label'].lower()}"); w.setdefault('joint', w['link'] + '_joint')
        if w['type'] in ('omni', 'mecanum'): w.setdefault('gamma_deg', 90 if w['type'] == 'omni' else 45)
        if w['type'] == 'caster': w['group'] = -1
    bad = [w['label'] for w in Rb['wheels'] if abs(float(w['heading_deg'])) > 1e-9]
    if bad: print(f'note: {name}: wheels {", ".join(bad)} do not roll along x (heading_deg != 0); the generic Gazebo model puts every '
                  f'wheel axle along y, use --engine emulate for this layout (docs/en/ROBOTS.md)', file=sys.stderr)
    return Rb


def robot(name):
    P = _load()
    if name not in ROBOTS:
        f = robot_file(name)
        if f: return _from_file(f)
    if name == 'robotino':
        RP = P['robotino']; Rb = RP['robot']; R, r = Rb['R'], Rb['r']
        wheels = []
        for k, th in enumerate(Rb['theta_deg'], 1):
            a = math.radians(th)
            wheels.append(dict(label=str(k), link=f'wheel{k}', joint=f'wheel{k}_joint', x=R * math.cos(a), y=R * math.sin(a), z=r,
                               heading_deg=th + 90, r=r, type='omni', gamma_deg=90, group=k - 1))
        return dict(name='robotino', chassis='base_link', mass=Rb['mass'], com=(Rb['com_x'], Rb['com_y'], Rb['com_h']),
                    I=(0.09, 0.09, Rb['Iz']), Jw=Rb['Jw'], wheels=wheels, motor=dict(RP['motor']), drive=('holonomic', 'diff'),
                    encoder_ticks=1970, odom_yaw_scale=1.0, speed=0.2, accel=0.5, v_max=0.4, w_max=2.0, map_size=4.0, map_res=0.02,
                    ctrl=dict(kp_xy=1.5, k_yaw=2.0, kx=1.5, ky=12.0, kth=4.0), base_h=0.0, wheel_mass=0.25,
                    contact=dict(RP['omni_contact']))
    if name == 'youbot':
        lx, ly, r = 0.228, 0.158, 0.0475                     # youbot_description joints (lx + ly = 0.386, driver cfg 0.385)
        g = {'FL': -45, 'FR': 45, 'RL': 45, 'RR': -45}       # X-configuration: w_FL = (vx - vy - (lx+ly) wz)/r ...
        pos = {'FL': (lx, ly), 'FR': (lx, -ly), 'RL': (-lx, ly), 'RR': (-lx, -ly)}
        wheels = [dict(label=k, link=f'wheel_{k.lower()}', joint=f'wheel_{k.lower()}_joint', x=pos[k][0], y=pos[k][1], z=r, heading_deg=0,
                       r=r, type='mecanum', gamma_deg=g[k], group=i, mu_rb=0.104) for i, k in enumerate(('FL', 'FR', 'RL', 'RR'))]
        # with the 5-DOF arm (folded), as drawn: base 22 kg (youbot_description) + arm 6 kg
        # (KUKA: 5.3-6.3 kg) mounted at x = 0.143 -> COM 3.4 cm forward, 13 cm high; inertia estimated with the arm.
        # Losses identified on recorded runs of the real robot: I0 = 0.20 A (no-load current),
        # mu_rb = 0.104 (roller bearings; one pair for all floors, leave-one-floor-out 0.100-0.106 / 0.19-0.24 A)
        return dict(name='youbot', chassis='base_link', mass=28.0, com=(0.034, 0.0, 0.13), I=(0.37, 0.82, 0.95), Jw=0.0095, wheels=wheels,
                    motor=dict(R=0.9, Ce=0.0335, kt=0.0335, gear=25.84, eta=0.75, I0=0.20, w_eps=5.0, Vmax=24, Vd=0.0, kp=0.05, ff=1.0, ki=0.5),
                    drive=('holonomic', 'diff'), encoder_ticks=4000, odom_yaw_scale=1.0, speed=0.3, accel=0.8, v_max=0.8, w_max=2.0,
                    map_size=5.0, map_res=0.02, ctrl=dict(kp_xy=1.5, k_yaw=2.0, kx=1.5, ky=10.0, kth=4.0), base_h=0.0, wheel_mass=0.4,
                    size=(0.57, 0.36, 0.10))
    if name == 'husky':
        lx, ly, r = 0.256, 0.2854, 0.1651                    # husky_description: wheelbase 0.512, track 0.5708
        pos = {'FL': (lx, ly), 'FR': (lx, -ly), 'RL': (-lx, ly), 'RR': (-lx, -ly)}
        grp = {'FL': 0, 'RL': 0, 'FR': 1, 'RR': 1}
        wheels = [dict(label=k, link=f'wheel_{k.lower()}', joint=f'wheel_{k.lower()}_joint', x=pos[k][0], y=pos[k][1], z=r, heading_deg=0,
                       r=r, b=0.1143, patch=0.08, type='standard', group=grp[k], lat_grip=0.981) for k in ('FL', 'FR', 'RL', 'RR')]
        # I0, eta, lat_grip: identified on recorded runs of the real robot (asphalt, indoor floor, ice, loam, meadow,
        # gravel); before that I0 = 0.8 A, eta = 0.73, lat_grip = 1.0 (catalogue estimates)
        return dict(name='husky', chassis='base_link', mass=50.0, com=(0.0, 0.0, 0.25), I=(1.61, 4.32, 5.42), Jw=0.41, wheels=wheels,
                    motor=dict(R=0.37, Ce=0.048, kt=0.048, gear=79.0, eta=0.713, I0=1.77, w_eps=20.0, Vmax=24, Vd=0.0, kp=0.2, ff=1.0, ki=2.0),
                    drive=('diff',), encoder_ticks=1024, odom_yaw_scale=1 / 1.875, speed=0.5, accel=1.0, v_max=1.0, w_max=2.0,
                    map_size=8.0, map_res=0.04, ctrl=dict(kp_xy=1.0, k_yaw=1.5, kx=1.0, ky=4.0, kth=2.5), base_h=0.0, wheel_mass=2.637,
                    size=(0.9874, 0.5709, 0.2475))
    if name == 'rover':
        R = P['rover']; r = R['wheel_r']; yw = R['track'] / 2
        pF, pB, dM, dR = R['pF'], R['pB'], R['dM'], R['dR']
        xs = {'F': pF[0], 'M': pB[0] + dM[0], 'R': pB[0] + dR[0]}
        zrel = pF[1]                                   # wheel centres below the rocker pivot (all equal by design)
        wheels = []
        for i, (wl, lab) in enumerate((('f', 'F'), ('m', 'M'), ('r', 'R'))):
            for side, sy in (('l', 1), ('r', -1)):
                wheels.append(dict(label=f'{lab}{side.upper()}', link=f'wheel_{wl}{side}', joint=f'wheel_{wl}{side}_joint', x=xs[lab],
                                   y=sy * yw, z=zrel, heading_deg=0, r=r, b=R['wheel_b'], patch=0.04, type='standard', group=len(wheels)))
        return dict(name='rover', chassis='chassis', mass=R['mass_total'], com=(R['pc'][0], 0.0, R['pc'][1]), I=(35.0, 40.0, 60.0), Jw=1.4,
                    wheels=wheels, motor=dict(R=0.3, Ce=0.03, kt=0.03, gear=160.0, eta=0.7, I0=0.3, w_eps=20.0, Vmax=24, Vd=0.0, kp=0.03, ff=1.0, ki=0.3),
                    drive=('diff',), encoder_ticks=2048, odom_yaw_scale=1 / 1.5, speed=0.1, accel=0.2, v_max=0.5, w_max=0.5,
                    map_size=10.0, map_res=0.05, ctrl=dict(kp_xy=1.0, k_yaw=1.0, kx=0.8, ky=2.0, kth=2.0), base_h=r - zrel, wheel_mass=R['mw'],
                    differential=dict(left_joint='rocker_l_joint', right_joint='rocker_r_joint', k=2e4, c=500), rover=R)
    if name in ('burger', 'waffle'):
        B = name == 'burger'
        r, ly = 0.033, (0.080 if B else 0.144)
        # DYNAMIXEL at 11.1 V (LiPo): Ce = V / (w0 G), R = V / I_stall, eta = tau_stall / (Ce G I_stall), kt = Ce
        G, w0, ist, tst = (258.5, 57.0, 1.3, 1.4) if B else (212.6, 70.0, 2.1, 2.7)
        Ce = 11.1 / (w0 * 2 * math.pi / 60 * G); eta = tst / (Ce * G * ist)
        wheels = [dict(label='L', link='wheel_left', joint='wheel_left_joint', x=0.0, y=ly, z=r, heading_deg=0, r=r, b=0.018, patch=0.012,
                       type='standard', group=0),
                  dict(label='R', link='wheel_right', joint='wheel_right_joint', x=0.0, y=-ly, z=r, heading_deg=0, r=r, b=0.018, patch=0.012,
                       type='standard', group=1)]
        cas = [(-0.081, 0.0)] if B else [(-0.177, 0.064), (-0.177, -0.064)]
        for k, (cx, cy) in enumerate(cas):           # ROBOTIS ball caster: ball r ~ 6 mm (caster link 6 mm above the floor)
            # the ball rolls in any direction: modelled as a swivel (z) + spin (y) pair without trail; J ~ 1e-6 kg m^2
            # (a 6 mm ball is ~1e-7; kept 7x larger for the engine: equivalent mass J / r^2 = 28 g)
            wheels.append(dict(label=f'C{k + 1}' if not B else 'C', link=f'caster{k + 1}', joint=f'caster{k + 1}_joint', x=cx, y=cy, z=0.006,
                               heading_deg=0, r=0.006, b=0.006, patch=0.004, type='caster', group=-1, J=1e-6, swivel=1,
                               mass=0.01, trail=0.0, swivel_h=0.0))
        m = 1.0 if B else 1.8
        return dict(name=name, chassis='base_link', mass=m, com=((-0.028, 0.0, 0.055) if B else (-0.058, 0.0, 0.045)),
                    I=((0.0046, 0.0046, 0.0033) if B else (0.017, 0.0148, 0.0259)), Jw=(0.003 if B else 0.0025), wheels=wheels,
                    motor=dict(R=round(11.1 / ist, 4), Ce=round(Ce, 6), kt=round(Ce, 6), gear=G, eta=round(eta, 3), I0=0.05, w_eps=20.0, Vmax=11.1,
                               Vd=0.0, kp=0.03, ff=1.0, ki=0.3),
                    drive=('diff',), encoder_ticks=round(4096 / G, 3), odom_yaw_scale=1.0, speed=(0.15 if B else 0.18), accel=0.3,
                    v_max=(0.22 if B else 0.26), w_max=(2.84 if B else 1.82), map_size=3.0, map_res=0.01,
                    ctrl=dict(kp_xy=1.5, k_yaw=2.0, kx=1.5, ky=12.0, kth=4.0), base_h=0.0, wheel_mass=0.0285,
                    size=((0.14, 0.14, 0.143) if B else (0.266, 0.266, 0.094)),
                    body_off=((-0.032, 0.0, 0.080) if B else (-0.064, 0.0, 0.057)))
    if name == 'jackal':
        lx, ly, r = 0.131, 0.187795, 0.098
        pos = {'FL': (lx, ly), 'FR': (lx, -ly), 'RL': (-lx, ly), 'RR': (-lx, -ly)}
        grp = {'FL': 0, 'RL': 0, 'FR': 1, 'RR': 1}
        wheels = [dict(label=k, link=f'wheel_{k.lower()}', joint=f'wheel_{k.lower()}_joint', x=pos[k][0], y=pos[k][1], z=r, heading_deg=0,
                       r=r, b=0.04, patch=0.04, type='standard', group=grp[k]) for k in ('FL', 'FR', 'RL', 'RR')]
        G = 23.0; Ce = 25.6 / (24.0 * G)               # ESTIMATE: no-load wheel speed 24 rad/s at 25.6 V
        return dict(name='jackal', chassis='base_link', mass=17.0, com=(0.012, 0.002, 0.13), I=(0.3136, 0.3922, 0.4485), Jw=0.005,
                    wheels=wheels, motor=dict(R=0.655, Ce=round(Ce, 5), kt=round(Ce, 5), gear=G, eta=0.8, I0=0.5, w_eps=20.0, Vmax=25.6, Vd=0.0,
                                              kp=0.2, ff=1.0, ki=2.0),
                    drive=('diff',), encoder_ticks=round(78000 * 2 * math.pi * r / G), odom_yaw_scale=1 / 1.49, speed=1.0, accel=1.0,
                    v_max=2.0, w_max=4.0, map_size=8.0, map_res=0.03, ctrl=dict(kp_xy=1.0, k_yaw=1.5, kx=1.0, ky=4.0, kth=2.5), base_h=0.0,
                    wheel_mass=0.477, size=(0.42, 0.31, 0.184), body_off=(0.0, 0.0, 0.0635 + 0.092))
    if name == 'pioneer':
        r, ly = 0.0975, 0.15
        wheels = [dict(label='L', link='wheel_left', joint='wheel_left_joint', x=0.0, y=ly, z=r, heading_deg=0, r=r, b=0.047, patch=0.04,
                       type='standard', group=0),
                  dict(label='R', link='wheel_right', joint='wheel_right_joint', x=0.0, y=-ly, z=r, heading_deg=0, r=r, b=0.047, patch=0.04,
                       type='standard', group=1),
                  dict(label='C', link='caster1', joint='caster1_joint', x=-0.185, y=0.0, z=0.035, heading_deg=0, r=0.035, b=0.0175, patch=0.02,
                       type='caster', group=-1, J=1.1e-4, swivel=1, mass=0.18, trail=0.01575, swivel_h=0.042)]   # p3dx description
        return dict(name='pioneer', chassis='base_link', mass=9.0, com=(-0.03, 0.0, 0.15), I=(0.151, 0.197, 0.264), Jw=0.018, wheels=wheels,
                    motor=dict(R=0.8, Ce=0.0229, kt=0.0229, gear=38.3, eta=0.7, I0=0.3, w_eps=20.0, Vmax=12.0, Vd=0.0, kp=0.1, ff=1.0, ki=1.0),
                    drive=('diff',), encoder_ticks=2048, odom_yaw_scale=1.0, speed=0.5, accel=0.6, v_max=1.2, w_max=5.2, map_size=6.0,
                    map_res=0.03, ctrl=dict(kp_xy=1.0, k_yaw=1.5, kx=1.0, ky=6.0, kth=3.0), base_h=0.0, wheel_mass=0.5,
                    size=(0.42, 0.34, 0.22), body_off=(-0.045, 0.0, 0.0175 + 0.148))
    raise SystemExit(f'unknown robot "{name}" (built-in: {", ".join(ROBOTS)}; files in configs/robots: {", ".join(user_robots()) or "none"})')


ROBOTS = ('robotino', 'youbot', 'husky', 'rover', 'burger', 'waffle', 'jackal', 'pioneer')
MESH = os.path.join(ROOT, 'models', 'meshes')


def ini_sections(Rb, links=True):
    """[robot], [wheel k], [motor] (+ [differential]) for the vehicle mode"""
    L = ['[robot]', 'type = vehicle', f"name = {Rb['name']}", f"mass = {Rb['mass']}", f"com_x = {Rb['com'][0]}", f"com_y = {Rb['com'][1]}",
         f"com_h = {Rb['com'][2]}", f"Ixx = {Rb['I'][0]}", f"Iyy = {Rb['I'][1]}", f"Izz = {Rb['I'][2]}", f"Jw = {Rb['Jw']}",
         f"encoder_ticks = {Rb['encoder_ticks']}", f"odom_yaw_scale = {Rb['odom_yaw_scale']:.6g}"]
    for w in Rb['wheels']:
        L += [f"[wheel {w['label']}]"] + [f'{k} = {v}' for k, v in w.items() if k != 'label' and (links or k not in ('link', 'joint'))]
    L += ['[motor]'] + [f'{k} = {v}' for k, v in Rb['motor'].items()]
    if 'differential' in Rb: L += ['[differential]'] + [f'{k} = {v}' for k, v in Rb['differential'].items()]
    return L


# ------------------------------------------------------------------------------------------- body contact
# The body touches the ground at points of its collision shapes (chassis-link frame): every corner, edge midpoint and
# face centre of a box, two rings of a cylinder, plus the outer face of every wheel (front, back, top, centre: a wheel
# lying on its side rests on its face). Robot files may give their own "body": [{"box": [sx, sy, sz], "at": [x, y, z]},
# {"cyl": [r, h], "at": [x, y, z]}, ...].
def body_shapes(Rb):
    if 'body' in Rb: return Rb['body']
    n = Rb.get('look', Rb['name'])
    if Rb['name'] == 'robotino': return [dict(cyl=[0.18, 0.12], at=[0.0, 0.0, 0.11])]
    if Rb['name'] == 'rover':
        pc = _load()['rover']['pc']; return [dict(box=[1.15, 0.85, 0.45], at=[pc[0], 0.0, pc[1] + 0.12])]
    if n == 'youbot':                                   # base (2.5 cm ground clearance) + the folded arm
        return [dict(box=[0.57, 0.36, 0.10], at=[0.0, 0.0, 0.0815]), dict(box=[0.16, 0.16, 0.40], at=[0.175, 0.0, 0.33])]
    return [dict(box=list(Rb['size']), at=list(Rb.get('body_off', (0.0, 0.0, Rb['com'][2]))))]


def body_points(Rb):
    P = []
    for sh in body_shapes(Rb):
        cx, cy, cz = sh['at']
        if 'box' in sh:
            sx, sy, sz = sh['box']
            P += [(cx + i * sx / 2, cy + j * sy / 2, cz + k * sz / 2) for i in (-1, 0, 1) for j in (-1, 0, 1) for k in (-1, 0, 1) if (i, j, k) != (0, 0, 0)]
        else:
            r, h = sh['cyl']
            P += [(cx, cy, cz - h / 2), (cx, cy, cz + h / 2)]
            P += [(cx + r * math.cos(2 * math.pi * k / 12), cy + r * math.sin(2 * math.pi * k / 12), z) for k in range(12) for z in (cz - h / 2, cz + h / 2)]
    if Rb['name'] not in ('robotino', 'rover'):         # wheels on fixed axles of the chassis (rover: on the suspension; Robotino: inside the body)
        for w in Rb['wheels']:
            if w['type'] == 'caster': continue
            hd = math.radians(w.get('heading_deg', 0)); ex, ey = math.cos(hd), math.sin(hd); axx, axy = -ey, ex
            sg = 1.0 if axx * w['x'] + axy * w['y'] >= 0 else -1.0
            b, r, z = w.get('b', 0.03), w['r'], w.get('z', w['r'])
            c = (w['x'] + sg * axx * b / 2, w['y'] + sg * axy * b / 2, z)
            P += [c, (c[0] + 0.7 * r * ex, c[1] + 0.7 * r * ey, z), (c[0] - 0.7 * r * ex, c[1] - 0.7 * r * ey, z), (c[0], c[1], z + 0.7 * r)]
    return P


def body_section(Rb, on=True, mu=0.6, on_tip='stop', tip_deg=50.0, extra=None):
    """[body] of the plugin INI: per-point stiffness so that the robot resting on 4 points sinks by 1 mm, damping 0.4
    of critical for a quarter of the mass"""
    m = Rb['mass']; k = m * 9.81 / (4 * 0.001); c = 2 * 0.4 * math.sqrt(k * m / 4)
    B = dict(contact=int(bool(on)), k=round(k, 1), c=round(c, 2), mu=mu, on_tip=on_tip, tip_deg=tip_deg)
    B.update(extra or {})
    pts = ' / '.join(f'{x:.4f},{y:.4f},{z:.4f}' for x, y, z in body_points(Rb))
    return ['[body]'] + [f'{k} = {v}' for k, v in B.items()] + [f'points = {pts}']


# ------------------------------------------------------------------------------------------- visuals
# A visual item: dict(geom=('mesh', relpath, scale) | ('box', sx, sy, sz) | ('cyl', r, L) | ('sphere', r),
# pose=(x, y, z, roll, pitch, yaw) in the link frame, color=(r, g, b) or None (DAE: own materials)).
# Links: 'base' = model frame (origin on the ground under the robot reference point, x forward),
# wheel label = wheel link frame (centre of the wheel, spin axis +y; Robotino: rotated by heading).
def _T(x=0, y=0, z=0, R=0, P=0, Y=0):
    cr, sr, cp, sp, cy, sy = math.cos(R), math.sin(R), math.cos(P), math.sin(P), math.cos(Y), math.sin(Y)
    M = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, x], [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, y],
         [-sp, cp * sr, cp * cr, z], [0, 0, 0, 1]]
    return M


def _mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def _pose(M):
    P = math.asin(max(-1.0, min(1.0, -M[2][0])))
    R = math.atan2(M[2][1], M[2][2]); Y = math.atan2(M[1][0], M[0][0])
    return (M[0][3], M[1][3], M[2][3], R, P, Y)


def _mesh(rel, pose, scale=1.0, color=None):
    return dict(geom=('mesh', rel, scale), pose=tuple(pose), color=color)


def _have(rel):
    return os.path.exists(os.path.join(MESH, rel))


def _youbot_arm(h, cad=True):
    """KUKA youBot arm in the folded rest pose (all joint values 0 of youbot_description); cad=False: boxes instead of
    the meshes (used only if models/meshes/youbot is missing)"""
    chain = [((0.143, 0, h + 0.046, 0, 0, 0), 'arm0.dae', (0, 0, 0)),
             ((0.024, 0, 0.096, 0, 0, 2.9270597283903603), 'arm1.dae', (0, 0, 0)),
             ((0.033, 0, 0.019, 0, -1.0944640137963142, 0), 'arm2.dae', (0, -0.032, 0.078)),
             ((0.0, 0, 0.155, 0, 2.508180707911721, 0), 'arm3.dae', (0, 0.028, 0.079)),
             ((0.0, 0, 0.135, 0, -1.7489624832941877, 0), 'arm4.dae', (0, -0.010, 0.029)),
             ((-0.002, 0, 0.130, 0, 0, 2.9634264970905018), 'arm5.dae', (0.003, 0, -0.034))]
    M = _T(); out = []
    orange, grey = (0.93, 0.45, 0.10), (0.25, 0.25, 0.27)
    if not cad:
        for k, (jp, mesh, vo) in enumerate(chain):
            M = _mul(M, _T(*jp))
            if k == 0: out.append(dict(geom=('cyl', 0.075, 0.07), pose=_pose(_mul(M, _T(0, 0, 0.035))), color=grey)); continue
            L = max(0.06, 2 * abs(vo[2]))
            out.append(dict(geom=('box', 0.065, 0.065, L), pose=_pose(_mul(M, _T(vo[0], vo[1], vo[2]))), color=orange))
        out.append(dict(geom=('box', 0.04, 0.06, 0.04), pose=_pose(_mul(M, _T(0, 0, 0.02))), color=grey))
        for sy in (0.012, -0.012):
            out.append(dict(geom=('box', 0.01, 0.008, 0.045), pose=_pose(_mul(M, _T(0, sy, 0.06))), color=grey))
        return out
    for jp, mesh, vo in chain:
        M = _mul(M, _T(*jp)); out.append(_mesh(f'youbot/{mesh}', _pose(_mul(M, _T(*vo)))))
    out.append(_mesh('youbot/palm.dae', _pose(M)))
    out.append(_mesh('youbot/finger.dae', _pose(_mul(M, _T(0.004, 0.008, 0.046, 0, 0, math.pi)))))
    out.append(_mesh('youbot/finger.dae', _pose(_mul(M, _T(0.004, -0.008, 0.046)))))
    return out


def _robotino_visuals(Rb):
    r = Rb['wheels'][0]['r']
    base = [dict(geom=('cyl', 0.185, 0.10), pose=(0, 0, 0.115, 0, 0, 0), color=(0.80, 0.81, 0.83)),          # aluminium body
            dict(geom=('cyl', 0.190, 0.035), pose=(0, 0, 0.055, 0, 0, 0), color=(0.12, 0.12, 0.13)),         # rubber bumper ring
            dict(geom=('cyl', 0.170, 0.006), pose=(0, 0, 0.168, 0, 0, 0), color=(0.25, 0.27, 0.30)),         # top deck
            dict(geom=('box', 0.05, 0.11, 0.035), pose=(-0.09, 0, 0.19, 0, 0, 0), color=(0.15, 0.15, 0.16)),   # control unit
            dict(geom=('box', 0.03, 0.05, 0.03), pose=(0.15, 0, 0.19, 0, 0, 0), color=(0.10, 0.10, 0.12)),     # camera
            dict(geom=('cyl', 0.008, 0.012), pose=(0.165, 0, 0.19, 0, math.pi / 2, 0), color=(0.2, 0.4, 0.9))]
    for k in range(9):                                                                                     # 9 IR distance sensors
        a = 2 * math.pi * k / 9
        base.append(dict(geom=('box', 0.012, 0.03, 0.02), pose=(0.191 * math.cos(a), 0.191 * math.sin(a), 0.10, 0, 0, a), color=(0.05, 0.05, 0.05)))
    wheel = [dict(geom=('cyl', 0.6 * r, 0.032), pose=(0, 0, 0, math.pi / 2, 0, 0), color=(0.75, 0.76, 0.78)),     # hub
             dict(geom=('cyl', 0.012, 0.036), pose=(0, 0, 0, math.pi / 2, 0, 0), color=(0.2, 0.2, 0.22))]
    n = 12
    for row, dy in ((0, -0.008), (1, 0.008)):                                                              # two rows of rollers
        for k in range(n):
            ph = 2 * math.pi * (k + 0.5 * row) / n
            wheel.append(dict(geom=('cyl', 0.0085, 0.019), pose=((r - 0.0085) * math.cos(ph), dy, (r - 0.0085) * math.sin(ph), 0, -ph, 0),
                              color=(0.15, 0.15, 0.16)))
    return base, wheel


def visuals(Rb):
    """{'base': [items], wheel label: [items]} for Gazebo and the previews"""
    n = Rb.get('look', Rb['name']); W = {w['label']: w for w in Rb['wheels']}
    V = {'base': []}
    dark, black, grey = (0.22, 0.22, 0.24), (0.06, 0.06, 0.07), (0.6, 0.6, 0.62)
    if n == 'robotino':
        b, wv = _robotino_visuals(Rb); V['base'] = b
        for k in W: V[k] = wv
    elif n == 'youbot':
        h = 0.0815
        if _have('youbot/base.dae') and _have('youbot/arm0.dae'):        # CAD meshes of youbot_description
            V['base'] = [_mesh('youbot/base.dae', (0, 0, h, 0, 0, 0)), _mesh('youbot/plate.dae', (-0.159, 0, h + 0.046, 0, 0, 0))] + _youbot_arm(h)
        else:                                                              # meshes missing: procedural look, same geometry
            V['base'] = [dict(geom=('box', 0.53, 0.30, 0.085), pose=(0, 0, h, 0, 0, 0), color=(0.20, 0.20, 0.22)),
                         dict(geom=('box', 0.24, 0.30, 0.01), pose=(-0.159, 0, h + 0.048, 0, 0, 0), color=(0.75, 0.75, 0.77))] + _youbot_arm(h, cad=False)
        for k, side in (('FL', 'left'), ('FR', 'right'), ('RL', 'right'), ('RR', 'left')):
            if _have(f'youbot/mecanum_{side}.STL'):
                V[k] = [_mesh(f'youbot/mecanum_{side}.STL', (0, 0, 0, 0, 0, 0), 0.001, dark)]
            else:                                                          # hub + 9 rollers at +-45 deg
                g = math.radians(W[k].get('gamma_deg', 45)); r = W[k]['r']
                V[k] = [dict(geom=('cyl', 0.6 * r, 0.05), pose=(0, 0, 0, math.pi / 2, 0, 0), color=(0.7, 0.7, 0.72))]
                for j in range(9):
                    ph = 2 * math.pi * j / 9
                    V[k].append(dict(geom=('cyl', 0.012, 0.045), pose=((r - 0.012) * math.cos(ph), 0, (r - 0.012) * math.sin(ph), g, -ph, 0),
                                     color=dark))
    elif n == 'husky':
        h = W['FL']['r'] - 0.03282
        V['base'] = [_mesh('husky/base_link.dae', (0, 0, h, 0, 0, 0)), _mesh('husky/top_chassis.dae', (0, 0, h, 0, 0, 0)),
                     _mesh('husky/user_rail.dae', (0.272, 0, h + 0.245, 0, 0, 0)), _mesh('husky/bumper.dae', (0.48, 0, h + 0.091, 0, 0, 0)),
                     _mesh('husky/bumper.dae', (-0.48, 0, h + 0.091, 0, 0, math.pi))]
        for k in W: V[k] = [_mesh('husky/wheel.dae', (0, 0, 0, 0, 0, 0), round(W[k]['r'] / 0.1778, 5))]
    elif n in ('burger', 'waffle'):
        h = 0.010; d = 'turtlebot3_burger' if n == 'burger' else 'turtlebot3_waffle_pi'; xo = -0.032 if n == 'burger' else -0.064
        V['base'] = [_mesh(f'{d}/{"burger" if n == "burger" else "waffle"}_base.stl', (xo, 0, h, 0, 0, 0), 0.001, (0.33, 0.33, 0.35)),
                     _mesh('turtlebot3_burger/lds.stl', (xo, 0, h + (0.172 if n == 'burger' else 0.122), 0, 0, 0), 0.001, (0.12, 0.12, 0.13))]
        if n == 'waffle': V['base'].append(dict(geom=('box', 0.02, 0.03, 0.02), pose=(0.07, 0, h + 0.10, 0, 0, 0), color=(0.1, 0.5, 0.2)))  # Pi camera
        V['L'] = [_mesh(f'{d}/left_tire.stl', (0, 0, 0, 0, 0, 0), 0.001, black)]
        V['R'] = [_mesh(f'{d}/right_tire.stl', (0, 0, 0, 0, 0, 0), 0.001, black)]
        for k, w in W.items():
            if w['type'] == 'caster':
                V[k] = [dict(geom=('sphere', w['r']), pose=(0, 0, 0, 0, 0, 0), color=(0.85, 0.85, 0.87))]
                V['base'].append(dict(geom=('box', 0.020, 0.030, 0.009), pose=(w['x'], w['y'], 2 * w['r'] + 0.004, 0, 0, 0), color=(0.2, 0.2, 0.22)))
    elif n == 'jackal':
        h = 0.0635; yel = (0.83, 0.72, 0.12)
        V['base'] = [_mesh('jackal/jackal-base.stl', (0, 0, h - 0.0655, math.pi / 2, 0, math.pi / 2), 1.0, (0.2, 0.2, 0.2)),
                     _mesh('jackal/jackal-fender.stl', (0, 0, h, 0, 0, 0), 1.0, yel), _mesh('jackal/jackal-fender.stl', (0, 0, h, 0, 0, math.pi), 1.0, yel)]
        for k in W: V[k] = [_mesh('jackal/jackal-wheel.stl', (0, 0, 0, math.pi / 2, 0, 0), round(W[k]['r'] / 0.1, 4), (0.12, 0.12, 0.12))]
    elif n == 'pioneer':
        h = W['L']['r'] - 0.08; red = (0.80, 0.02, 0.02)
        V['base'] = [_mesh('pioneer3dx/chassis.stl', (0, 0, h + 0.148, 0, 0, 0), 1.0, red), _mesh('pioneer3dx/top.stl', (0, 0, h + 0.24, 0, 0, 0), 1.0, black)]
        c = W['C']; V['C_swivel'] = [_mesh('pioneer3dx/swivel.stl', (0.026 - c['trail'], 0, 0.016 - c['swivel_h'], 0, 0, 0), 1.0, grey)]
        sc = round(W['L']['r'] / 0.0912, 4)
        V['L'] = [_mesh('pioneer3dx/left_wheel.stl', (0, 0, 0, 0, 0, 0), sc, (0.1, 0.1, 0.1))]
        V['R'] = [_mesh('pioneer3dx/right_wheel.stl', (0, 0, 0, 0, 0, 0), sc, (0.1, 0.1, 0.1))]
        V['C'] = [_mesh('pioneer3dx/center_wheel.stl', (0, 0, 0, 0, 0, 0), round(c['r'] / 0.0382, 4), (0.15, 0.15, 0.16))]
    if not V['base'] and 'size' in Rb:          # robot files without CAD visuals: the body is drawn as its collision box
        sx, sy, sz = Rb['size']; bx, by, bz = Rb.get('body_off', (0.0, 0.0, Rb['com'][2]))
        V['base'] = [dict(geom=('box', sx, sy, sz), pose=(bx, by, bz, 0, 0, 0), color=(0.55, 0.57, 0.60))]
    return V


def _visual_xml(name, it):
    g = it['geom']; x, y, z, R, P, Y = it['pose']
    if g[0] == 'mesh':
        sc = g[2] if isinstance(g[2], (tuple, list)) else (g[2],) * 3
        geo = f'<mesh><uri>file://{os.path.join(MESH, g[1])}</uri><scale>{sc[0]} {sc[1]} {sc[2]}</scale></mesh>'
    elif g[0] == 'box': geo = f'<box><size>{g[1]} {g[2]} {g[3]}</size></box>'
    elif g[0] == 'cyl': geo = f'<cylinder><radius>{g[1]}</radius><length>{g[2]}</length></cylinder>'
    else: geo = f'<sphere><radius>{g[1]}</radius></sphere>'
    mat = ''
    if it.get('color') is not None:
        c = ' '.join(f'{v:.3f}' for v in it['color']) + ' 1'
        mat = f'<material><ambient>{c}</ambient><diffuse>{c}</diffuse><specular>0.2 0.2 0.2 1</specular></material>'
    return f'<visual name="{name}"><pose>{x:.6f} {y:.6f} {z:.6f} {R:.6f} {P:.6f} {Y:.6f}</pose><geometry>{geo}</geometry>{mat}</visual>'


def visuals_xml(Rb, link):
    return ''.join(_visual_xml(f'v{k}', it) for k, it in enumerate(visuals(Rb).get(link, [])))


# ------------------------------------------------------------------------------------------- SDF
def sdf_model(flavor, Rb, ini, pose):
    import gz_sdf as GE
    x, y, z, yaw = pose[:4]
    roll, pitch = (pose[4], pose[5]) if len(pose) >= 6 else (0.0, 0.0)      # tilted spawn on sloped terrain
    rpy = f'{roll:.6f} {pitch:.6f} {yaw:.6f}'
    if Rb['name'] == 'rover':
        R = dict(Rb['rover']); R['Iw'] = Rb['Jw']
        s = GE.rover_model(flavor, R, ini, x, y, z_ground=z - Rb['base_h'])
        return re.sub(r'(<model name="rover"><pose>\S+ \S+ \S+) 0 0 0</pose>', rf'\1 {rpy}</pose>', s, count=1)
    if Rb['name'] == 'robotino':
        s = GE.robotino_model_sdf(flavor, _load()['robotino'], ini)
        return s.replace('<model name="robotino"><pose>0 0 0.0005 0 0 0</pose>', f'<model name="robotino"><pose>{x} {y} {z + 0.002} {rpy}</pose>', 1)
    # chassis (collision box + CAD visuals) + wheels / casters on revolute joints (axis +y = left); the floor contact
    # of every wheel and caster is computed by the plugin
    mw = Rb['wheel_mass']; cas = [w for w in Rb['wheels'] if w['type'] == 'caster']
    m_c = Rb['mass'] - mw * (len(Rb['wheels']) - len(cas)) - sum(w.get('mass', 0.05) * (1.1 if w.get('swivel') else 1.0) for w in cas)
    sx, sy, sz = Rb['size']; r0 = Rb['wheels'][0]['r']
    if 'body_off' in Rb: bx, by, bz = Rb['body_off']
    else: bx, by, bz = 0.0, 0.0, (r0 + 0.02 + sz / 2 if Rb.get('look', Rb['name']) == 'youbot' else Rb['com'][2])
    com_pose = f"{Rb['com'][0]} {Rb['com'][1]} {Rb['com'][2]} 0 0 0"
    links = [f'<link name="base_link"><pose>0 0 0 0 0 0</pose>{GE.inertial(m_c, *Rb["I"], com_pose)}'
             f'<collision name="col"><pose>{bx} {by} {bz} 0 0 0</pose><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>'
             + visuals_xml(Rb, 'base') + '</link>']
    joints = []
    for w in Rb['wheels']:
        b = w.get('b', 0.05); r = w['r']
        caster = w['type'] == 'caster'
        if caster:            # ball / swivel caster: swivel link (free about z) + spinning wheel link (free about its axle)
            m = w.get('mass', 0.05); J = max(w.get('J', 2.0 / 5.0 * m * r * r), 1e-7)
            ixx = J if b <= r else max(m * (3 * r * r + b * b) / 12, 0.55 * J)
        else:
            m = mw; J = Rb['Jw']; ixx = max(m * (3 * r * r + b * b) / 12, 0.55 * J)
        geo = f'<sphere><radius>{r}</radius></sphere>' if caster else f'<cylinder><radius>{r}</radius><length>{b}</length></cylinder>'
        cpose = '0 0 0 0 0 0' if caster else '0 0 0 1.5707963 0 0'
        vis = visuals_xml(Rb, w['label']) or (f'<visual name="vis"><pose>{cpose}</pose><geometry>{geo}</geometry>'
                                              f'<material><ambient>0.15 0.15 0.15 1</ambient><diffuse>0.15 0.15 0.15 1</diffuse></material></visual>')
        parent = 'base_link'
        if caster and w.get('swivel'):
            sl = w['link'] + '_swivel'; sxp = w['x'] + w.get('trail', 0.0); szp = r + w.get('swivel_h', 0.0)
            ms = max(0.1 * m, 0.005); Is = max(ms * 0.01 ** 2, 1e-7)
            links.append(f'<link name="{sl}"><pose>{sxp} {w["y"]} {szp} 0 0 0</pose>{GE.inertial(ms, Is, Is, Is)}{visuals_xml(Rb, w["label"] + "_swivel")}</link>')
            joints.append(GE.joint(w['joint'].replace('_joint', '') + '_swivel_joint', 'revolute', 'base_link', sl, '0 0 1'))
            parent = sl
        links.append(f'<link name="{w["link"]}"><pose>{w["x"]} {w["y"]} {r} 0 0 0</pose>{GE.inertial(m, ixx, J, ixx)}'
                     f'<collision name="col"><pose>{cpose}</pose><geometry>{geo}</geometry></collision>{vis}</link>')
        joints.append(GE.joint(w['joint'], 'revolute', parent, w['link'], '0 1 0'))
    return (f'<model name="{Rb["name"]}"><pose>{x} {y} {z + 0.002} {rpy}</pose>' + ''.join(links) + ''.join(joints)
            + GE.plugin(flavor, ini) + '</model>')


def gz_sensors(Rb):
    """optional native gz-sim sensors (for ros_gz_bridge): IMU on the chassis + odometry publisher"""
    imu = ('<sensor name="imu" type="imu"><always_on>1</always_on><update_rate>100</update_rate><topic>imu</topic>'
           '<imu><angular_velocity><z><noise type="gaussian"><mean>0</mean><stddev>0.009</stddev></noise></z></angular_velocity>'
           '<linear_acceleration><x><noise type="gaussian"><mean>0</mean><stddev>0.05</stddev></noise></x>'
           '<y><noise type="gaussian"><mean>0</mean><stddev>0.05</stddev></noise></y></linear_acceleration></imu></sensor>')
    odom = (f'<plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">'
            f'<odom_frame>odom</odom_frame><robot_base_frame>{Rb["chassis"]}</robot_base_frame><odom_publish_frequency>50</odom_publish_frequency>'
            f'<dimensions>3</dimensions></plugin>'
            '<plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher"><topic>joint_states</topic></plugin>')
    return imu, odom

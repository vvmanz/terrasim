"""Gazebo SDF helpers shared by the runners: inertials, joints, wheels, static boxes, visual-only terrain meshes,
the plugin element, the world file, the MarsSim rover and the Robotino models, and a world checker.

Nothing collides with the terrain in DART: terrain is visual only (mesh_visual_model, static_box(collide=False)). The
wheel-ground and body-ground forces come from the terrasim plugin, computed on the terrain map (the same in the
emulator). Only a safety floor below the terrain has engine collisions.
"""
import os, sys, json, math
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

PARAMS_FILE = os.path.join(ROOT, 'configs', 'calibrated_params.json')


def load_params():
    with open(PARAMS_FILE) as f:
        return json.load(f)


# ------------------------------------------------------------------------------------------ SDF helpers
def inertial(m, ixx, iyy, izz, pose='0 0 0 0 0 0'):
    return (f'<inertial><pose>{pose}</pose><mass>{m}</mass><inertia><ixx>{ixx}</ixx><ixy>0</ixy><ixz>0</ixz>'
            f'<iyy>{iyy}</iyy><iyz>0</iyz><izz>{izz}</izz></inertia></inertial>')


def box_visual(name, size, pose, color):
    return (f'<visual name="{name}"><pose>{pose}</pose><geometry><box><size>{size}</size></box></geometry>'
            f'<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>')


def wheel_link(flavor, name, pos, r_vis, r_col, b, mass, iyy, mu=1.0, kp=2e6, kd=1.5e4):
    # Iyy (spin) includes reflected motor/gearbox inertia; sdformat>=14 rejects tensors violating the
    # triangle inequality (Ixx + Izz >= Iyy), so the transverse moments are raised accordingly.
    ixx = max(mass * (3 * r_col ** 2 + b ** 2) / 12, 0.55 * iyy)
    ode_contact = ''                          # contact stiffness is DART's (Gazebo Harmonic); friction below is read by DART too
    return (f'<link name="{name}"><pose>{pos[0]} {pos[1]} {pos[2]} 0 0 0</pose>{inertial(mass, ixx, iyy, ixx)}'
            f'<collision name="col"><pose>0 0 0 1.5707963 0 0</pose><geometry><cylinder><radius>{r_col}</radius><length>{b}</length></cylinder></geometry>'
            f'<surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction>{ode_contact}</surface></collision>'
            f'<visual name="vis"><pose>0 0 0 1.5707963 0 0</pose><geometry><cylinder><radius>{r_vis}</radius><length>{b}</length></cylinder></geometry>'
            f'<material><ambient>0.75 0.75 0.78 1</ambient><diffuse>0.75 0.75 0.78 1</diffuse></material></visual>'
            f'<visual name="mark"><pose>0 {0.5 * b + 0.002} {0.6 * r_vis} 0 0 0</pose><geometry><box><size>0.03 0.004 0.06</size></box></geometry>'
            f'<material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material></visual></link>')


def joint(name, jtype, parent, child, axis, lower=-1e16, upper=1e16, damping=0.0, feedback=False):
    fb = '<physics><provide_feedback>true</provide_feedback></physics>' if feedback else ''
    return (f'<joint name="{name}" type="{jtype}"><parent>{parent}</parent><child>{child}</child>'
            f'<axis><xyz>{axis}</xyz><limit><lower>{lower}</lower><upper>{upper}</upper></limit>'
            f'<dynamics><damping>{damping}</damping></dynamics></axis>{fb}</joint>')


def plugin(flavor, ini):
    return f'<plugin filename="terramech_gz_system" name="terramech::TerramechSystem"><config>{ini}</config></plugin>'


def world(flavor, name, models, dt=0.001, rtf=None):
    """Gazebo Harmonic (gz-sim 8, DART) world. rtf: target real-time factor (<real_time_factor>); None or 0 = as
    fast as possible (headless default)."""
    fast = not rtf or rtf <= 0
    return (f'<?xml version="1.0"?>\n<sdf version="1.9"><world name="{name}">'
            f'<physics name="1ms" type="ignored"><max_step_size>{dt}</max_step_size><real_time_factor>{1000 if fast else rtf:g}</real_time_factor></physics>'
            f'<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>'
            f'<plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>'
            f'<plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>'
            f'<gravity>0 0 -9.81</gravity>'
            f'<scene><ambient>0.6 0.55 0.5 1</ambient><background>0.85 0.62 0.45 1</background></scene>'
            f'<light name="sun" type="directional"><cast_shadows>false</cast_shadows><pose>0 0 10 0 0 0</pose>'
            f'<diffuse>0.9 0.9 0.9 1</diffuse><specular>0.1 0.1 0.1 1</specular><direction>-0.4 0.2 -0.9</direction></light>'
            + ''.join(models) + '</world></sdf>\n')


def static_box(name, center, size, color, mu=0.6, collide=True):
    col = (f'<collision name="col"><geometry><box><size>{size[0]} {size[1]} {size[2]}</size></box></geometry>'
           f'<surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction></surface></collision>') if collide else ''
    return (f'<model name="{name}"><static>true</static><pose>{center[0]} {center[1]} {center[2]} 0 0 0</pose><link name="link">{col}'
            f'<visual name="vis"><geometry><box><size>{size[0]} {size[1]} {size[2]}</size></box></geometry>'
            f'<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual></link></model>')


def mesh_visual_model(name, obj, color):
    return (f'<model name="{name}"><static>true</static><link name="link"><visual name="vis"><geometry><mesh><uri>file://{obj}</uri></mesh></geometry>'
            f'<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual></link></model>')


# ------------------------------------------------------------------------------------------ rover
def rover_model(flavor, R, ini, x0, y0=0.0, z_ground=0.0, terramech=True):
    """6-wheel rocker-bogie rover. Frame: model origin = rocker pivot axis (centre)."""
    track = R['track']; yw = track / 2; yr = yw - 0.10
    pF, pB, dM, dR, pc = R['pF'], R['pB'], R['dM'], R['dR'], R['pc']
    piv_h = -pF[1]
    z0 = z_ground + R['wheel_r'] + piv_h + 0.004  # wheels slightly above ground; settle under gravity
    m_w, m_r, m_b = R['mw'], 3.0, 2.0
    m_c = R['mass_total'] - 6 * m_w - 2 * m_r - 2 * m_b
    links, joints = [], []
    links.append(f'<link name="chassis"><pose>0 0 0 0 0 0</pose>{inertial(m_c, m_c * (0.8**2 + 0.5**2) / 12, 2 * R["Ic"], m_c * (1.2**2 + 0.8**2) / 12, f"{pc[0]} 0 {pc[1]} 0 0 0")}'
                 + box_visual('body', '1.15 0.85 0.45', f'{pc[0]} 0 {pc[1] + 0.12} 0 0 0', '0.80 0.62 0.20 1')
                 + box_visual('mast', '0.03 0.03 0.6', f'{pc[0] + 0.35} 0.25 {pc[1] + 0.65} 0 0 0', '0.8 0.7 0.3 1') + '</link>')
    for side, sy in (('l', 1), ('r', -1)):
        y = sy * yr
        # rocker: from front wheel to bogie pivot
        fx, fz = pF; bx, bz = pB
        L = math.hypot(fx - bx, fz - bz); ang = math.atan2(fz - bz, fx - bx)
        links.append(f'<link name="rocker_{side}"><pose>0 {y} 0 0 0 0</pose>{inertial(m_r, 0.02, 0.3, 0.3, f"{(fx + bx) / 2} 0 {(fz + bz) / 2} 0 0 0")}'
                     + box_visual('arm', f'{L:.3f} 0.05 0.04', f'{(fx + bx) / 2} 0 {(fz + bz) / 2} 0 {-ang:.4f} 0', '0.95 0.95 0.95 1') + '</link>')
        joints.append(joint(f'rocker_{side}_joint', 'revolute', 'chassis', f'rocker_{side}', '0 1 0', -0.6, 0.6, damping=2.0))
        mx, mz = bx + dM[0], bz + dM[1]; rx, rz = bx + dR[0], bz + dR[1]
        Lb = math.hypot(mx - rx, mz - rz)
        links.append(f'<link name="bogie_{side}"><pose>{bx} {y} {bz} 0 0 0</pose>{inertial(m_b, 0.01, 0.1, 0.1)}'
                     + box_visual('arm', f'{Lb:.3f} 0.05 0.04', f'{(dM[0] + dR[0]) / 2} 0 {(dM[1] + dR[1]) / 2 * 0.5} 0 0 0', '0.95 0.95 0.95 1') + '</link>')
        joints.append(joint(f'bogie_{side}_joint', 'revolute', f'rocker_{side}', f'bogie_{side}', '0 1 0', -0.6, 0.6, damping=2.0))
        for wl, (wx, wz), parent in (('f', pF, f'rocker_{side}'), ('m', (mx, mz), f'bogie_{side}'), ('r', (rx, rz), f'bogie_{side}')):
            links.append(wheel_link(flavor, f'wheel_{wl}{side}', (wx, sy * yw, wz), R['wheel_r'], R['r_collision'], R['wheel_b'], m_w, R['Iw']))
            joints.append(joint(f'wheel_{wl}{side}_joint', 'revolute', parent, f'wheel_{wl}{side}', '0 1 0', feedback=True))
    return (f'<model name="rover"><pose>{x0} {y0} {z0} 0 0 0</pose>' + ''.join(links) + ''.join(joints) + plugin(flavor, ini) + '</model>')


# ------------------------------------------------------------------------------------------ Robotino
SURF_COLOR = {'gray': '0.55 0.55 0.57 1', 'green': '0.25 0.55 0.25 1', 'table': '0.80 0.68 0.50 1', 'red': '0.75 0.18 0.15 1',
              'table_wet': '0.62 0.55 0.45 1', 'carpet': '0.35 0.35 0.55 1', 'carpet_shag': '0.55 0.45 0.65 1', 'foam': '0.85 0.80 0.35 1',
              'sand_lete': '0.85 0.75 0.55 1', 'sandy_loam': '0.55 0.42 0.30 1', 'clay_wet': '0.45 0.33 0.25 1', 'mars_soil': '0.75 0.45 0.30 1',
          'tile': '0.88 0.90 0.92 1', 'linoleum': '0.70 0.62 0.48 1', 'concrete': '0.62 0.62 0.60 1', 'gray_wet': '0.38 0.40 0.44 1', 'green_wet': '0.15 0.40 0.18 1', 'ice': '0.80 0.90 0.97 1'}


def robotino_model_sdf(flavor, RP, ini):
    """Robotino 1.6: base_link + 3 omni wheels on revolute joints (axle towards the centre). Contact with
    the floor is computed by the plugin (omni contact law); engine collisions only with a safety floor."""
    Rb = RP['robot']; R, r = Rb['R'], Rb['r']
    m_tot, mw, Jw = Rb['mass'], 0.25, Rb['Jw']
    mc = m_tot - 3 * mw
    cx, cy, ch = Rb['com_x'] * m_tot / mc, Rb['com_y'] * m_tot / mc, Rb['com_h']
    Izc = max(Rb['Iz'] - 3 * mw * R * R, 0.02)
    import robots as RB                      # CAD-like procedural visuals (body, bumper, deck, sensors, omni rollers)
    Rv = RB.robot('robotino')
    links = [f'<link name="base_link"><pose>0 0 0 0 0 0</pose>{inertial(mc, 0.6 * Izc, 0.6 * Izc, Izc, f"{cx} {cy} {ch} 0 0 0")}'
             f'<collision name="col"><pose>0 0 0.11 0 0 0</pose><geometry><cylinder><radius>0.18</radius><length>0.12</length></cylinder></geometry></collision>'
             + RB.visuals_xml(Rv, 'base') + '</link>']
    joints = []
    for k, th in enumerate(Rb['theta_deg'], 1):
        a = math.radians(th)
        x, y = R * math.cos(a), R * math.sin(a)
        ixx = max(mw * (3 * r * r + 0.03 ** 2) / 12, 0.55 * Jw)
        links.append(f'<link name="wheel{k}"><pose>{x:.5f} {y:.5f} {r} 0 0 {a + math.pi / 2:.6f}</pose>{inertial(mw, ixx, Jw, ixx)}'
                     f'<collision name="col"><pose>0 0 0 1.5707963 0 0</pose><geometry><cylinder><radius>{r}</radius><length>0.03</length></cylinder></geometry></collision>'
                     + RB.visuals_xml(Rv, str(k))
                     + f'<visual name="mark"><pose>0 0.019 0.02 0 0 0</pose><geometry><box><size>0.010 0.002 0.016</size></box></geometry>'
                     f'<material><ambient>0.9 0.9 0.1 1</ambient><diffuse>0.9 0.9 0.1 1</diffuse></material></visual></link>')
        joints.append(joint(f'wheel{k}_joint', 'revolute', 'base_link', f'wheel{k}', '0 1 0'))
    return '<model name="robotino"><pose>0 0 0.0005 0 0 0</pose>' + ''.join(links) + ''.join(joints) + plugin(flavor, ini) + '</model>'


def validate_world(path):
    """Checks that sdformat 14 / Gazebo would reject: inertia validity, duplicate names, dangling joints."""
    import xml.etree.ElementTree as ET
    errs = []
    root = ET.parse(path).getroot()
    for model in root.iter('model'):
        links = [l.get('name') for l in model.findall('link')]
        if len(set(links)) != len(links): errs.append(f"{model.get('name')}: duplicate link names")
        for l in model.findall('link'):
            ine = l.find('inertial')
            if ine is None: continue
            m = float(ine.find('mass').text); I = ine.find('inertia')
            a, b, c = (float(I.find(k).text) for k in ('ixx', 'iyy', 'izz'))
            if m <= 0 or min(a, b, c) <= 0 or a + b < c * (1 - 1e-9) or b + c < a * (1 - 1e-9) or a + c < b * (1 - 1e-9):
                errs.append(f"{model.get('name')}/{l.get('name')}: invalid inertia m={m} I=({a},{b},{c})")
        for j in model.findall('joint'):
            for tag in ('parent', 'child'):
                n = j.find(tag).text
                if n != 'world' and n not in links: errs.append(f"{model.get('name')}/{j.get('name')}: unknown {tag} '{n}'")
    return errs

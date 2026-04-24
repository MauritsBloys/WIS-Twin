from flask import Flask, render_template, jsonify, request
import subprocess
import threading
import time
import re
import serial

app = Flask(__name__, static_url_path='', static_folder='static')

# ── WTBU via wtbu.py CLI ────────────────────────────────────────────────────
WTBU_PY = '/home/wtbu/driver/wtbu.py'
RELAYS = ['main', 'fireflies', 'well-pump', 'rain-pump', 'pc', 'free1', 'free2', 'free3']


def _parse_wtbu_output(stdout, stderr):
    if stderr:
        last_line = stderr.strip().splitlines()[-1]
        return {'error': last_line}
    result = {'monitor': '', 'relays': '', 'valves': ''}
    for line in stdout.splitlines():
        if line.startswith('Monitor: '):
            result['monitor'] = line[9:]
        elif line.startswith('Relays: '):
            result['relays'] = line[8:]
        elif line.startswith('Valves: '):
            result['valves'] = line[8:]
    return result


def run_wtbu(*args):
    try:
        proc = subprocess.run(
            [WTBU_PY] + list(args),
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            universal_newlines=True, timeout=10
        )
        return _parse_wtbu_output(proc.stdout, proc.stderr)
    except subprocess.TimeoutExpired:
        return {'error': 'Timeout'}
    except Exception as e:
        return {'error': str(e)}


# ── Firefly serial ──────────────────────────────────────────────────────────
FIREFLY_PORT = "/dev/serial/by-id/usb-Silicon_Labs_Zolertia_Firefly_platform_ZOL-B001-A200002574-if00-port0"
FIREFLY_BAUDRATE = 460800

CALIBRATION = {
    1: (0.0188, -188.0213),
    2: (0.0054, -65.1942),
    3: (0.0181, -181.4417),
    4: (0.0184, -182.8977),
    5: (0.0187, -184.0834),
    6: (0.0054, -59.3130),
    7: (0.0054, -59.6097),
}

NODE_SENSOR_MAP = {
    201: (1, 2),
    202: (3, 4),
    203: (5, 6),
    204: (7, None),
}

_sensor_re   = re.compile(r"Node:\s*(20[1-4])\sSensor1:\s(\d+)\sSensor2:\s(\d+)")
_actuator_re = re.compile(r"Node:\s*(20[1-4])\sActuator:\s(\d+)")

_firefly_data_lock  = threading.Lock()
_firefly_write_lock = threading.Lock()
_firefly_serial = None
_sensor_raw   = {}
_sensor_cm    = {}
_actuator_state = {}


def _raw_to_cm(sensor_id, raw):
    a, b = CALIBRATION[sensor_id]
    return round(a * raw + b, 2)


def _firefly_reader():
    global _firefly_serial
    while True:
        try:
            if _firefly_serial is None or not _firefly_serial.is_open:
                _firefly_serial = serial.Serial(
                    FIREFLY_PORT, FIREFLY_BAUDRATE, timeout=0.2, write_timeout=0.5
                )
                time.sleep(0.2)

            raw = _firefly_serial.readline()
            if not raw:
                continue
            line = raw.decode('utf-8', errors='replace').strip()
            if not line:
                continue

            m = _sensor_re.search(line)
            if m:
                node = int(m.group(1))
                s1, s2 = int(m.group(2)), int(m.group(3))
                sid1, sid2 = NODE_SENSOR_MAP[node]
                with _firefly_data_lock:
                    _sensor_raw[sid1] = s1
                    _sensor_cm[sid1]  = _raw_to_cm(sid1, s1)
                    if sid2 is not None:
                        _sensor_raw[sid2] = s2
                        _sensor_cm[sid2]  = _raw_to_cm(sid2, s2)
                continue

            m = _actuator_re.search(line)
            if m:
                node = int(m.group(1))
                with _firefly_data_lock:
                    _actuator_state[node] = int(m.group(2))

        except Exception:
            _firefly_serial = None
            time.sleep(1)


threading.Thread(target=_firefly_reader, daemon=True).start()


def _send_firefly(cmd: str):
    global _firefly_serial
    with _firefly_write_lock:
        try:
            if _firefly_serial is None or not _firefly_serial.is_open:
                return {'error': 'Firefly niet verbonden'}
            _firefly_serial.write((cmd + '\n').encode('utf-8'))
            _firefly_serial.flush()
            return {'ok': True}
        except Exception as e:
            return {'error': str(e)}


# ── Routes: WTBU ────────────────────────────────────────────────────────────
@app.route('/')
def index():
    return render_template('Water Testbed SCADA.html')


@app.route('/api/status')
def status():
    return jsonify(run_wtbu('--status'))


@app.route('/api/relay/<relay>/<action>', methods=['POST'])
def relay(relay, action):
    if relay not in RELAYS or action not in ['on', 'off']:
        return jsonify({'error': 'Ongeldige relay of actie'}), 400
    return jsonify(run_wtbu(f'--{action}', relay))


@app.route('/api/valve', methods=['POST'])
def valve():
    data = request.get_json()
    try:
        valve_num = int(data['valve'])
        position  = float(data['position'])
    except (KeyError, ValueError, TypeError):
        return jsonify({'error': 'Ongeldige invoer'}), 400
    if not (1 <= valve_num <= 9):
        return jsonify({'error': 'Valve moet tussen 1 en 9 zijn'}), 400
    if not (0 <= position <= 90):
        return jsonify({'error': 'Positie moet tussen 0 en 90 graden zijn'}), 400
    return jsonify(run_wtbu('--valves', f'{valve_num}={position}'))


@app.route('/api/close-all', methods=['POST'])
def close_all():
    return jsonify(run_wtbu('--close-all'))


@app.route('/api/reset-relays', methods=['POST'])
def reset_relays():
    return jsonify(run_wtbu('--reset-relays'))


@app.route('/api/shutdown', methods=['POST'])
def shutdown():
    return jsonify(run_wtbu('--shutdown'))


# ── Routes: Firefly ─────────────────────────────────────────────────────────
@app.route('/api/firefly/status')
def firefly_status():
    with _firefly_data_lock:
        sensors = {
            str(sid): {'raw': _sensor_raw.get(sid), 'cm': _sensor_cm.get(sid)}
            for sid in range(1, 8)
        }
        actuators = {str(node): _actuator_state.get(node) for node in NODE_SENSOR_MAP}
    connected = _firefly_serial is not None and _firefly_serial.is_open
    return jsonify({'sensors': sensors, 'actuators': actuators, 'connected': connected})


@app.route('/api/firefly/gate/<int:node>/<int:value>', methods=['POST'])
def firefly_gate(node, value):
    if node not in NODE_SENSOR_MAP:
        return jsonify({'error': f'Ongeldig node {node}'}), 400
    if not (0 <= value <= 255):
        return jsonify({'error': 'Waarde moet tussen 0 en 255 zijn'}), 400
    return jsonify(_send_firefly(f"{node} {value}"))


@app.route('/api/firefly/mode/<mode>', methods=['POST'])
def firefly_mode(mode):
    if mode == 'manual':
        return jsonify(_send_firefly('205 0'))
    if mode == 'auto':
        return jsonify(_send_firefly('205 1'))
    return jsonify({'error': 'Ongeldig mode'}), 400


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=False)

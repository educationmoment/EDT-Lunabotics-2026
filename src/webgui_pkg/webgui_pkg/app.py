"""
webgui_pkg/app.py
EDT Lunabotics UIC 2026 – Flask web GUI server

Routes:
  /                   → index.html (hub page)
  /pilot              → pilot.html (lunabot control panel)
  /internal_systems   → internal_systems.html (engineer view)
  /networking         → networking.html
  /get_active_nodes   → JSON: active nodes / topics / services
  /launch_node        → POST: launch a ROS2 node by name
  /kill_node          → POST: kill a ROS2 node by name
  /gui_cmd            → POST: receive GUI commands (e.g. launch_rviz)
"""

import os
import subprocess
import threading
from flask import Flask, render_template, jsonify, request

app = Flask(__name__)

# ── background process registry ────────────────────────────
_launched_procs: dict[str, subprocess.Popen] = {}
_lock = threading.Lock()


# ── ROUTES ─────────────────────────────────────────────────

@app.route('/', methods=['GET'])
def get_main():
    return render_template('index.html')


@app.route('/pilot', methods=['GET'])
def get_pilot():
    """
    Pass the rosbridge WS URL into the template so it can be
    pre-filled from a server-side env var or config.
    ROSBRIDGE_URL defaults to ws://localhost:9090 but can be
    overridden: export ROSBRIDGE_URL=ws://192.168.1.100:9090
    """
    rosbridge_url = os.environ.get('ROSBRIDGE_URL', 'ws://localhost:9090')
    return render_template('pilot.html', rosbridge_url=rosbridge_url)


@app.route('/pilot_old', methods=['GET'])
def get_engineer():
    return render_template('pilot_old.html')


@app.route('/networking', methods=['GET'])
def get_networking():
    return render_template('networking.html')


# ── ROS2 INTROSPECTION ─────────────────────────────────────

@app.route('/get_active_nodes', methods=['GET'])
def get_active_nodes():
    """Return currently active nodes, topics, and services as JSON."""

    def _run(cmd):
        try:
            result = subprocess.run(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=5
            )
            out = result.stdout.decode('utf-8').strip()
            return [l for l in out.split('\n') if l] if out else []
        except subprocess.TimeoutExpired:
            return ['(timeout)']
        except FileNotFoundError:
            return ['(ros2 not found)']

    active_nodes    = _run(['ros2', 'node', 'list'])
    active_topics   = _run(['ros2', 'topic', 'list'])
    active_services = _run(['ros2', 'service', 'list'])

    return jsonify({
        'number_nodes':    len(active_nodes),
        'number_topics':   len(active_topics),
        'number_services': len(active_services),
        'active_nodes':    active_nodes,
        'active_topics':   active_topics,
        'active_services': active_services,
    })


# ── NODE LIFECYCLE ─────────────────────────────────────────

# Map of friendly names → launch commands
# Add more entries as needed.
NODE_COMMANDS: dict[str, list[str]] = {
    'controller':    ['ros2', 'run', 'controller_pkg', 'controller_node'],
    'health':        ['ros2', 'run', 'controller_pkg', 'health_node'],
    'excavation':    ['ros2', 'run', 'controller_pkg', 'excavation_node'],
    'depositing':    ['ros2', 'run', 'controller_pkg', 'depositing_node'],
    'rosbridge':     ['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml'],
    'rviz2':         ['rviz2', '-d',
                      os.path.expanduser('~') + '/robot_WS/install/config_pkg/share/config_pkg/rviz/robot_view.rviz'],
    'slam_manual':   ['ros2', 'launch', 'config_pkg', 'neptune_sim.launch.py', 'robot_mode:=manual'],
    'slam_auto':     ['ros2', 'launch', 'config_pkg', 'neptune_sim.launch.py', 'robot_mode:=auto'],
    'navigation_client': ['ros2', 'run', 'navigation_pkg', 'navigation_client'],
}


@app.route('/launch_node', methods=['POST'])
def launch_node():
    """
    POST JSON: {"node": "rosbridge"}
    Launches the corresponding command in a background process.
    """
    data = request.get_json(silent=True) or {}
    node_key = data.get('node', '')
    cmd = NODE_COMMANDS.get(node_key)
    if not cmd:
        return jsonify({'ok': False, 'error': f'Unknown node key: {node_key}'}), 400

    with _lock:
        # Kill existing instance if running
        if node_key in _launched_procs:
            try:
                _launched_procs[node_key].terminate()
            except Exception:
                pass

        try:
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            _launched_procs[node_key] = proc
            return jsonify({'ok': True, 'pid': proc.pid, 'node': node_key})
        except Exception as e:
            return jsonify({'ok': False, 'error': str(e)}), 500


@app.route('/kill_node', methods=['POST'])
def kill_node():
    """
    POST JSON: {"node": "rosbridge"}
    Terminates a previously launched node.
    """
    data = request.get_json(silent=True) or {}
    node_key = data.get('node', '')

    with _lock:
        proc = _launched_procs.get(node_key)
        if proc:
            try:
                proc.terminate()
                del _launched_procs[node_key]
                return jsonify({'ok': True, 'node': node_key})
            except Exception as e:
                return jsonify({'ok': False, 'error': str(e)}), 500
        else:
            return jsonify({'ok': False, 'error': 'Node not tracked'}), 404


@app.route('/node_status', methods=['GET'])
def node_status():
    """Return which tracked nodes are currently running."""
    with _lock:
        statuses = {}
        for key, proc in list(_launched_procs.items()):
            alive = proc.poll() is None
            statuses[key] = 'running' if alive else 'stopped'
            if not alive:
                del _launched_procs[key]
    return jsonify(statuses)


# ── GUI COMMAND ENDPOINT ───────────────────────────────────

@app.route('/gui_cmd', methods=['POST'])
def gui_cmd():
    """
    Receive GUI commands from the pilot page (AJAX) or rosbridge.
    POST JSON: {"cmd": "launch_rviz"}
    """
    data = request.get_json(silent=True) or {}
    cmd = data.get('cmd', '')

    if cmd == 'launch_rviz':
        rviz_cmd = NODE_COMMANDS.get('rviz2', ['rviz2'])
        try:
            proc = subprocess.Popen(rviz_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            _launched_procs['rviz2'] = proc
            return jsonify({'ok': True, 'cmd': cmd, 'pid': proc.pid})
        except Exception as e:
            return jsonify({'ok': False, 'error': str(e)}), 500

    return jsonify({'ok': False, 'error': f'Unknown cmd: {cmd}'}), 400


# ── ENTRY POINT ───────────────────────────────────────────

if __name__ == '__main__' or __name__ == 'webgui_pkg.app':
    app.run(host='0.0.0.0', port=59440, debug=True)



def main():
    """Entry point for: ros2 run webgui_pkg webgui_server"""
    app.run(host='0.0.0.0', port=59440, debug=False)
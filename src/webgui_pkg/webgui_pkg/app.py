import os
from flask import Flask, render_template, jsonify
import subprocess

# Get Current Working Directory
# -- Set Paths for Templates and Static Files
##################################################
# current_dir = "/ssd/home/edt/Desktop/robot_WS/src/webgui_pkg/webgui_pkg"
# templates_dir = f"{current_dir}/templates"
# static_dir = f"{current_dir}/static"
##################################################


# Create Flask App
##################################################
# app = Flask(
        # __name__,
        # template_folder=templates_dir,
        # static_folder=static_dir
    # )
##################################################
app = Flask(
    __name__
)

### Define Routes
##################################################
## Route to HOME
@app.route('/', methods=['GET'])
def get_main():
    return render_template('index.html')
    # return """<h1>Main Route</h1><ul><li>/pilot</li><li>/engineer</li><li>/networking</li></ul>"""

## Route to PILOT GUI
@app.route('/pilot', methods=['GET'])
def get_pilot():
    return render_template('pilot.html')

## Route to ENGINEER GUI
@app.route('/internal_systems', methods=['GET'])
def get_engineer():
    return render_template('internal_systems.html')
    # return """<h1>Engineer</h1><hr><p>Not Yet Started</p>"""

## Route to NETWORKING GUI
@app.route('/networking', methods=['GET'])
def get_networking():
    return render_template('networking.html')
##################################################

@app.route('/get_active_nodes', methods=['GET'])
def get_active_nodes():

    # Get Active Nodes
    active_nodes = subprocess.run(
        ['ros2', 'node', 'list'],
        stdout=subprocess.PIPE
    )

    # Get Active Topics
    active_topics = subprocess.run(
        ['ros2', 'topic', 'list'],
        stdout=subprocess.PIPE
    )

    # Get Active Services
    active_services = subprocess.run(
        ['ros2', 'service', 'list'],
        stdout=subprocess.PIPE
    )

    # Decode and Split Output
    active_nodes = active_nodes.stdout.decode('utf-8').strip('\n').split('\n')
    active_topics = active_topics.stdout.decode('utf-8').strip('\n').split('\n')
    active_services = active_services.stdout.decode('utf-8').strip('\n').split('\n')

    # Return JSON
    return jsonify({
        "number_nodes": len(active_nodes),
        "number_topics": len(active_topics),
        "number_services": len(active_services),
        "active_nodes": active_nodes,
        "active_topics": active_topics,
        "active_services": active_services
    })


## Start Application:
#       Any IP can connect to server on port 59440
##################################################
if __name__ == "__main__" or __name__ == "webgui_pkg.app":
    app.run(host='0.0.0.0', port=59440, debug=True)  # Run the app
##################################################

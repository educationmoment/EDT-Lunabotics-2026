import os
from flask import Flask, render_template

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
    return """<h1>Main Route</h1><ul><li>/pilot</li><li>/engineer</li><li>/networking</li></ul>"""

## Route to PILOT GUI
@app.route('/pilot', methods=['GET'])
def get_pilot():
    return render_template('pilot.html')

## Route to ENGINEER GUI
@app.route('/engineer', methods=['GET'])
def get_engineer():
    return """<h1>Engineer</h1><hr><p>Not Yet Started</p>"""

## Route to NETWORKING GUI
@app.route('/networking', methods=['GET'])
def get_networking():
    return """<h1>Network</h1><hr><p>Not Yet Started</p>"""
##################################################


### Debugging: Issue - Flask server served to default ip. 127.0.0.1 on 5000
#   Issue appears to be resolved as of February 7
##################################################
# print('----------')
# print('\033[104m' + f"{__name__}" + '\033[0m')
# print('----------')
##################################################


## Start Application:
#       Any IP can connect to server on port 59440
##################################################
if __name__ == "__main__" or __name__ == "webgui_pkg.app":
    app.run(host='0.0.0.0', port=59440, debug=True)  # Run the app
##################################################

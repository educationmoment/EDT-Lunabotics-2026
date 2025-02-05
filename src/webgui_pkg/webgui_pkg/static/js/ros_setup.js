//////////////////////////////////////////////////
// Setup ROS Environment 
// 
//  Description:
//    This script is used to setup the ROS environment 
//    for the interface. All non-specific client-side
//    ROS code is contained here.
//////////////////////////////////////

// Get Server URL and IP Address
const serverURL = window.location.origin;
const serverIP = window.location.hostname;

// Create a connection to the ROS server
const ROS = new ROSLIB.Ros({
    url: `ws://${serverIP}:9090`
});

// Create a connection to the rosbridge WebSocket server.
ROS.on('connection', function() {
    console.log('Connected to websocket server.');
});

// Log an error to the console in case of a connection error.
ROS.on('error', function(error) {  
    console.log('Error connecting to websocket server: ', error);
});

// Log a message once the connection is closed.
ROS.on('close', function() {
    console.log('Connection to websocket server closed.');
});
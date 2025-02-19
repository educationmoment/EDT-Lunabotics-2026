// Gather Inner Overlayed Objects
////////////////////////////////////////////////////////////////////////////////////////
// const dgtTwin_canvas = document.getElementById("digital-twin");
// const fctIndicators_div = document.getElementById("function-indicators");
// const outLogs_div = document.getElementById("output-logs");
const img = document.getElementById("camera-output");
////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////


// DEBUG: Prove Objects Source Correctly
// console.log("[ INIT ] Demonstrating Connectivity to DOM");
// fctIndicators_div.textContent += "   Hello World";
// outLogs_div.textContent += "   Hello World";



// Create a Listener on Compressed Image Topic Published by a Camera Node
////////////////////////////////////////////////////////////////////////////////////////
var listener = new ROSLIB.Topic({
    ros : ROS,
    name : '/rs_node/camera/compressed_video',
    messageType : 'sensor_msgs/CompressedImage'
});

listener.subscribe(function(message) {
    console.log('Received message on ' + listener.name);

    var imageElement = document.getElementById('camera-output');
    imageElement.src = 'data:image/jpeg;base64, ' + message.data
});
////////////////////////////////////////////////////////////////////////////////////////


// Create a Gamepad Publisher Topic from Client page over ROSBRIDGE
////////////////////////////////////////////////////////////////////////////////////////
var gamepadPublisher = new ROSLIB.Topic({
    ros : ROS,
    name : '/joy',
    messageType : 'sensor_msgs/Joy'
});

function publishGamepadMessage() {
    var gamepadState = updateGamepadState( GAMEPAD, GAMEPAD_INDEX );

    // Gamepade Valid State is False, is the Gamepad Connected?
    if ( gamepadState === null ) {
        return;
    }

    FRAME_ID += 1;

    // Create Message
    var message = new ROSLIB.Message({
        header : {
            frame_id : `${FRAME_ID}`
        },
        axes : gamepadState.axes,
        buttons : gamepadState.buttons
    });

    // Publish Message
    gamepadPublisher.publish(message);
    return;
}
////////////////////////////////////////////////////////////////////////////////////////

// Animation Loop
////////////////////////////////////////////////////////////////////////////////////////
var rot_y = 0.0;
function animationLoop() {
    console.log("[ ANIMATION-LOOP ] Running");


    rot_y += 0.01;

    // Update Renders
    render_quest_arrow(rot_y);
    render_twin(rot_y);
    requestAnimationFrame(animationLoop);
    return;
}

////////////////////////////////////////////////////////////////////////////////////////

// Main Code
////////////////////////////////////////////////////////////////////////////////////////
console.log("[ Main ] Running");
function main() {
    console.log("[ Main ] Running");
    try {
        setInterval(publishGamepadMessage, 50);
    } catch (error) {
        console.error('An error occurred while attempting to publish the gamepad message: ', error);
    }
    
    requestAnimationFrame(animationLoop);
    return;
}
////////////////////////////////////////////////////////////////////////////////////////
main();

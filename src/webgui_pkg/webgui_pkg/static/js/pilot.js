// Gather Inner Overlayed Objects
////////////////////////////////////////////////////////////////////////////////////////
// const dgtTwin_canvas = document.getElementById("digital-twin");
// const fctIndicators_div = document.getElementById("function-indicators");
// const outLogs_div = document.getElementById("output-logs");
const img = document.getElementById("camera-output");
const cameraWrapper = document.getElementById("camera-wrapper");
////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////


// DEBUG: Prove Objects Source Correctly
// console.log("[ INIT ] Demonstrating Connectivity to DOM");
// fctIndicators_div.textContent += "   Hello World";
// outLogs_div.textContent += "   Hello World";

/**
 * @function toggleFullscreen
 * @brief Toggle fullscreen mode
 */
function toggleFullscreen() {
    const elem = document.getElementById("camera-wrapper");

    if (!document.fullscreenElement) {
        elem.requestFullscreen().catch(err => {
            alert(`Error attempting to enable fullscreen mode: ${err.message}`);
        });
    } else {
        document.exitFullscreen();
    }
}
////////////////////////////////////////////////////////////////////////////////////////


// Create a Gamepad Publisher Topic from Client page over ROSBRIDGE
////////////////////////////////////////////////////////////////////////////////////////
var gamepadPublisher = new ROSLIB.Topic({
    ros: ROS,
    name: '/joy',
    messageType: 'sensor_msgs/Joy'
});

function publishGamepadMessage() {
    var gamepadState = updateGamepadState(GAMEPAD, GAMEPAD_INDEX);

    // Gamepade Valid State is False, is the Gamepad Connected?
    if (gamepadState === null) {
        return;
    }
    FRAME_ID += 1;

    // Create Message
    var message = new ROSLIB.Message({
        header: {
            frame_id: `${FRAME_ID}`
        },
        axes: gamepadState.axes,
        buttons: gamepadState.buttons
    });

    // Publish Message
    gamepadPublisher.publish(message);
    return;
}
////////////////////////////////////////////////////////////////////////////////////////


const listener = new ROSLIB.Topic({
    ros: ROS,
    name: '/rs_node/camera1/compressed_video',
    messageType: 'sensor_msgs/CompressedImage'
});

listener.subscribe((message)=>{
    const imgEl = document.getElementById('main-camera-frame');
    console.log("Updating Image");
    if(imgEl) {
        imgEl.src = "data:image/jpeg;base64, " + message.data;
    }
});

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
    return;
}
////////////////////////////////////////////////////////////////////////////////////////
main();

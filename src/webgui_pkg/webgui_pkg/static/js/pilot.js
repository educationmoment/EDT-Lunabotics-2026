// Gather Inner Overlayed Objects
////////////////////////////////////////////////////////////////////////////////////////
// const dgtTwin_canvas = document.getElementById("digital-twin");
// const fctIndicators_div = document.getElementById("function-indicators");
// const outLogs_div = document.getElementById("output-logs");
const img = document.getElementById("camera-output");
const cameraWrapper = document.getElementById("camera-wrapper");
////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////


/*
// DEBUG: Prove Objects Source Correctly
// console.log("[ INIT ] Demonstrating Connectivity to DOM");
// fctIndicators_div.textContent += "   Hello World";
// outLogs_div.textContent += "   Hello World";
*/

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
    // FIX: ROS 2 message path
    messageType: 'sensor_msgs/msg/Joy'
});

// Helpers added (non-breaking)
function deadzone(v, dz = 0.05) {
    return Math.abs(v) < dz ? 0 : v;
}
function toIntButtons(btnArray) {
    // Convert boolean/analog buttons to 0/1 integers as expected by Joy.msg
    return btnArray.map(b => {
        // b may be true/false or a GamepadButton with .pressed/.value
        if (typeof b === 'boolean') return b ? 1 : 0;
        if (typeof b === 'number') return b > 0.5 ? 1 : 0;
        return (b && (b.pressed || (b.value && b.value > 0.5))) ? 1 : 0;
    });
}

function publishGamepadMessage() {
    // Refresh the current gamepad snapshot each tick (prevents stale object)
    try {
        if (typeof navigator !== 'undefined' && navigator.getGamepads && typeof GAMEPAD_INDEX !== 'undefined') {
            const pads = navigator.getGamepads();
            if (pads && GAMEPAD_INDEX >= 0 && pads[GAMEPAD_INDEX]) {
                // Update global reference without removing original variables
                GAMEPAD = pads[GAMEPAD_INDEX];
            }
        }
    } catch (e) {
        // Swallow to avoid breaking publishing loop
        // console.debug('getGamepads not available yet:', e);
    }

    var gamepadState = updateGamepadState(GAMEPAD, GAMEPAD_INDEX);

    // Gamepade Valid State is False, is the Gamepad Connected?
    if (gamepadState === null) {
        return;
    }
    FRAME_ID += 1;

    // Normalize data: apply deadzone to axes; convert buttons to int[]
    var axesOut = Array.isArray(gamepadState.axes)
        ? gamepadState.axes.map(a => deadzone(a))
        : [];
    var buttonsOut = Array.isArray(gamepadState.buttons)
        ? toIntButtons(gamepadState.buttons)
        : [];

    // Header time (ROS 2)
    const now = Date.now();
    const sec = Math.floor(now / 1000);
    const nanosec = Math.floor((now % 1000) * 1e6);

    // Create Message
    var message = new ROSLIB.Message({
        header: {
            stamp: { sec: sec, nanosec: nanosec },
            frame_id: `${FRAME_ID}`
        },
        axes: axesOut,
        buttons: buttonsOut
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

    // Ensure at least one user gesture so browsers deliver gamepad data reliably
    try {
        ['click','keydown','pointerdown','touchstart'].forEach(evt => {
            window.addEventListener(evt, function once() {
                window.removeEventListener(evt, once, { capture:false });
                // No-op; presence of a gesture often unlocks Gamepad API sampling
            }, { once:true });
        });
    } catch (e) {
        // Safe to ignore
    }

    try {
        setInterval(publishGamepadMessage, 50);
    } catch (error) {
        console.error('An error occurred while attempting to publish the gamepad message: ', error);
    }
    return;
}
////////////////////////////////////////////////////////////////////////////////////////
main();


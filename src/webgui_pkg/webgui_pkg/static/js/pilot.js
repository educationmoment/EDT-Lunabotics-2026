console.log("[ INIT ] Gathering objects from DOM");

// Gather Inner Overlayed Objects
////////////////////////////////////////////////////////////////////////////////////////
const quest_canvas = document.getElementById("quest-arrow");
const dgtTwin_canvas = document.getElementById("digital-twin");
const fctIndicators_div = document.getElementById("function-indicators");
const outLogs_div = document.getElementById("output-logs");
const img = document.getElementById("camera-output");
////////////////////////////////////////////////////////////////////////////////////////

// Create Quest Arrow Object
////////////////////////////////////////////////////////////////////////////////////////
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(30, quest_canvas.clientWidth / quest_canvas.clientHeight, 0.1, 1000);
const renderer = new THREE.WebGLRenderer({
    antialias: true,
    canvas: quest_canvas,
    alpha: true
});

renderer.setSize(quest_canvas.clientWidth, quest_canvas.clientHeight);
renderer.domElement.className = "viewport";

// Add Light
const light = new THREE.DirectionalLight(0x00FF00, 1);
light.position.set(0, 0, 5);

// Add Cube
const geometry = new THREE.BoxGeometry();
const material = new THREE.MeshBasicMaterial({ color: 0xFFFFFF});
const cube = new THREE.Mesh(geometry, material);
cube.position.x = 0;
cube.position.y = 0;
cube.position.z = 0;



// Add Grid Helper
const gridHelper = new THREE.GridHelper(100, 100);

// Add Axes Helper
const axesHelper = new THREE.AxesHelper(5);

// Add Objects to Scene
scene.add(light);
scene.add(cube);
scene.add(gridHelper);
scene.add(axesHelper);

// Set Camera Position
camera.position.x = 2.5;
camera.position.y = 1.25;
camera.position.z = 2.5;
camera.lookAt( new THREE.Vector3(0,0,0) );

renderer.render(scene, camera);
////////////////////////////////////////////////////////////////////////////////////////


// DEBUG: Prove Objects Source Correctly
console.log("[ INIT ] Demonstrating Connectivity to DOM");
fctIndicators_div.textContent += "   Hello World";
outLogs_div.textContent += "   Hello World";



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
function animationLoop() {
    console.log("[ ANIMATION-LOOP ] Running");
    requestAnimationFrame(animationLoop);
    return;
}

////////////////////////////////////////////////////////////////////////////////////////

// Main Code
////////////////////////////////////////////////////////////////////////////////////////
function main() {
    setInterval(publishGamepadMessage, 50);
    animationLoop();
    return;
}
////////////////////////////////////////////////////////////////////////////////////////

main()
// Gather Inner Overlayed Objects
////////////////////////////////////////////////////////////////////////////////////////
const quest_canvas = document.getElementById("quest-arrow");
// const dgtTwin_canvas = document.getElementById("digital-twin");
// const fctIndicators_div = document.getElementById("function-indicators");
// const outLogs_div = document.getElementById("output-logs");
const img = document.getElementById("camera-output");
////////////////////////////////////////////////////////////////////////////////////////

// Create Quest Arrow Object
////////////////////////////////////////////////////////////////////////////////////////
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(4, quest_canvas.clientWidth / quest_canvas.clientHeight, 0.1, 1000);
const renderer = new THREE.WebGLRenderer({
    antialias: true,
    canvas: quest_canvas,
    alpha: true
});

renderer.setSize(quest_canvas.clientWidth, quest_canvas.clientHeight);
renderer.domElement.className = "viewport";

// Add Light
const light_x = new THREE.DirectionalLight(0xFF0000, 1);
const light_y = new THREE.DirectionalLight(0x00FF00, 1);
const light_z = new THREE.DirectionalLight(0x0000FF, 1);
light_x.position.set(0, 5, 0);
light_y.position.set(0, 5, 0);
light_z.position.set(0, 0, 5);

// Add Draco Loader
const dracoLoader = new THREE.DRACOLoader();
dracoLoader.setDecoderPath('https://cdn.jsdelivr.net/npm/three@0.146.0/examples/js/libs/draco/');

// Add GLTF Loader
const gltfLoader = new THREE.GLTFLoader();
gltfLoader.setDRACOLoader(dracoLoader);
gltfLoader.load(
    '../static/objects/pointer_arrow.glb',
    (gltf) => {
        console.log('Loaded Model: ', gltf);
        gltf.scene.position.x = 0;
        gltf.scene.position.y = 0;
        gltf.scene.position.z = 0;
        scene.add(gltf.scene);
    },
    undefined,
    (error) => {
        console.error('An error occurred while loading the model: ', error);
    }
);

// Add Cube
// const geometry = new THREE.BoxGeometry();
// const material = new THREE.MeshBasicMaterial({ color: 0xFFFFFF});
// const cube = new THREE.Mesh(geometry, material);
// cube.position.x = -5;
// cube.position.y = 0;
// cube.position.z = -5;



// Add Grid Helper
const gridHelper = new THREE.GridHelper(100, 100);

// Add Axes Helper
const axesHelper = new THREE.AxesHelper(5);

// Add Objects to Scene
scene.add(light_x);
scene.add(light_y);
scene.add(light_z);

// scene.add(cube);
scene.add(gridHelper);
scene.add(axesHelper);

// Set Camera Position
camera.position.x = 5.0;
camera.position.y = 1.25;
camera.position.z = 5.0;
camera.lookAt( new THREE.Vector3(0,0,0) );

// renderer.render(scene, camera);
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
    camera.position.x = 5.0 * Math.sin(rot_y);
    camera.position.z = 5.0 * Math.cos(rot_y);
    camera.lookAt( new THREE.Vector3(0,0,0) );

    rot_y += 0.01;
    renderer.render( scene, camera );
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

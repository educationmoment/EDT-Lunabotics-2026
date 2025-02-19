// Create Objects
const twin_canvas = document.getElementById('digital-twin');


// Create Scene, Camera, and Renderer
const twin_scene    = new THREE.Scene();
const twin_renderer = new THREE.WebGLRenderer({
    antialias: true,
    canvas: twin_canvas,
    alpha: true
});

// Set Renderer Size
twin_renderer.setSize(twin_canvas.clientWidth, twin_canvas.clientHeight);
twin_renderer.domElement.className = "viewport";

const twin_camera = new THREE.PerspectiveCamera(30, twin_canvas.clientWidth / twin_canvas.clientHeight, 0.1, 100);

// Set Camera Position
twin_camera.position.x = 5.0;
twin_camera.position.y = 2.0;
twin_camera.position.z = 0.0;

// Create Draco Loader
var dracoLoader = new THREE.DRACOLoader();
const twin_gltfLoader = new THREE.GLTFLoader();
dracoLoader.setDecoderPath('https://cdn.jsdelivr.net/npm/three@0.146.0/examples/js/libs/draco/');

// Add Grid/Axis Helpers
const twin_gridHelper = new THREE.GridHelper(100, 100);
const twin_axesHelper = new THREE.AxesHelper(10);

// Load GLTF/GLB Model
twin_gltfLoader.setDRACOLoader(dracoLoader);
twin_gltfLoader.load(
    '../static/objects/quad_big_drone.glb',
    (gltf) => {
        console.log('Loaded Model: ', gltf);
        gltf.scene.position.x = 0.5;
        gltf.scene.position.y = 0.5;
        gltf.scene.position.z = -0.5;
        twin_scene.add(gltf.scene);
    },
    undefined,
    (error) => {
        console.error('An error occurred while loading the model: ', error);
    }
);

// Add Objects to Scene
twin_scene.add(twin_gridHelper);
twin_scene.add(twin_axesHelper);

// Final Renderer
function render_twin( time ) {
    twin_camera.lookAt( new THREE.Vector3( 0, 0, 0 ) );
    twin_renderer.render(twin_scene, twin_camera);
    return;
}   
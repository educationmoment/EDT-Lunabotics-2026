// Create Objects
const quest_canvas   = document.getElementById('quest-arrow');

// Create Scene, Camera, and Renderer
const quest_scene    = new THREE.Scene();
const quest_renderer = new THREE.WebGLRenderer({
    antialias: true,
    canvas: quest_canvas,
    alpha: true
});

// Set Renderer Size
quest_renderer.setSize(quest_canvas.clientWidth, quest_canvas.clientHeight);
quest_renderer.domElement.className = "viewport";

const quest_camera   = new THREE.PerspectiveCamera(4, quest_canvas.clientWidth / quest_canvas.clientHeight, 0.1, 100);

// Set Camera Position
quest_camera.position.x = 5.0;
quest_camera.position.y = 1.25;
quest_camera.position.z = 5.0;

// Create Lights
const light_x = new THREE.DirectionalLight(0xFF0000, 1);
const light_y = new THREE.DirectionalLight(0x00FF00, 1);
const light_z = new THREE.DirectionalLight(0x0000FF, 1);
light_x.position.set(5, 0, 0);
light_y.position.set(0, 5, 0);
light_z.position.set(0, 0, 5);

// Create Draco Loader
var dracoLoader = new THREE.DRACOLoader();
const quest_gltfLoader = new THREE.GLTFLoader();
dracoLoader.setDecoderPath('https://cdn.jsdelivr.net/npm/three@0.146.0/examples/js/libs/draco/');


// Add Grid/Axis Helpers
const quest_gridHelper = new THREE.GridHelper(100, 100);
const quest_axesHelper = new THREE.AxesHelper(10);

// Load GLTF/GLB Model
quest_gltfLoader.setDRACOLoader(dracoLoader);
quest_gltfLoader.load(
    '../static/objects/pointer_arrow.glb',
    (gltf) => {
        console.log('Loaded Model: ', gltf);
        gltf.scene.position.x = 0;
        gltf.scene.position.y = 0;
        gltf.scene.position.z = 0;
        quest_scene.add(gltf.scene);
    },
    undefined,
    (error) => {
        console.error('An error occurred while loading the model: ', error);
    }
);

// Add Objects to Scene
quest_scene.add(light_x);
quest_scene.add(light_y);
quest_scene.add(light_z);

quest_scene.add(quest_gridHelper);
quest_scene.add(quest_axesHelper);

// Final Renderer
// quest_renderer.render(quest_scene, quest_camera);
function render_quest_arrow( time ) {
    quest_camera.position.x = 5.0 * Math.cos( time * 0.05 );
    quest_camera.position.z = 5.0 * Math.sin( time * 0.05 );
    quest_camera.lookAt( new THREE.Vector3( 0, 0, 0) );
    quest_renderer.render( quest_scene, quest_camera );
    return;
}
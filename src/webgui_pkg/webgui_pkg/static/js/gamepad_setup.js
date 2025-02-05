////////////////////////////////////////////////////////////////////////////////////////
// Gamepad Setup
// 
// Description:
//    This script is used to setup the Gamepad API in JavaScript.
////////////////////////////////////////////////////////////////////////////////////////

// Variables
var GAMEPAD = null;
var GAMEPAD_INDEX = -1;
var FRAME_ID = 0;

// Gamepad Connection/Disconnection Event Listeners
////////////////////////////////////////////////////////////////////////////////////////
window.addEventListener('gamepadconnected', function(event) {
    console.log('A gamepad connected:');
    console.log(event.gamepad);
    
    GAMEPAD = this.navigator.getGamepads()[event.gamepad.index];
    GAMEPAD_INDEX = event.gamepad.index;
    FRAME_ID = 0;
});

window.addEventListener('gamepaddisconnected', function(event) {
    console.log('A gamepad disconnected:');
    console.log(event.gamepad);

    // Only Connected Gamepad Disconnected
    if (gamepadIndex === 0 ) {
        GAMEPAD = null;
        GAMEPAD_INDEX = -1;

    // One of Multiple Gamepads Disconnected
    } else {
        GAMEPAD = this.navigator.getGamepads()[GAMEPAD_INDEX - 1];
        GAMEPAD_INDEX = GAMEPAD_INDEX - 1;
    }
    FRAME_ID = 0;
});
////////////////////////////////////////////////////////////////////////////////////////

// Update Gamepad State
////////////////////////////////////////////////////////////////////////////////////////
function updateGamepadState( gamepad, gamepadIndex ) {
    var state = {
        buttons : new Array(),
        axes : new Array()
    };
    
    // No Gamepad Connected
    if (  gamepad === null || gamepadIndex === -1  ) {
        console.log(`[ updateGamepadState.error ] No Gamepad Connected (${gamepad}:${gamepadIndex})`);
        return null;
    }
    
    // Gamepad Connected, Continue Processing
    gamepad.buttons.forEach( (button) => {
        state.buttons.push( button.pressed ); 
    });

    state.axes = gamepad.axes;
    return state;
}
////////////////////////////////////////////////////////////////////////////////////////

// Get Document Objects for Displaying Active ROS Nodes
const activeNodesDiv = document.getElementById("active-nodes-div");

// Display Server URL and IP Address
console.log(`Server URL: ${serverURL}`);
console.log(`Server IP: ${serverIP}`);

// Get List of Active ROS Nodes
var active_data;
fetch(`${serverURL}/get_active_nodes`)
    .then(response => response.json())
    .then(data => {
        // active_data = data;
        // console.log(data);

        // Add Active Nodes to the Webpage
        for( var i = 0; i < data.active_nodes.length; i++ ) {
            var newElement = document.createElement("div");
            // newElement.textContent = active_nodes[i];
            newElement.textContent = data.active_nodes[i];
            newElement.className = "node-item";
            activeNodesDiv.appendChild(newElement);
            console.log("Adding Item to Active Nodes");
        }
});

console.log(active_data);



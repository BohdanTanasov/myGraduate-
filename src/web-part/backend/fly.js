const express = require('express');
const {json} = require("express");
const app = express();
const server = require('http').Server(app);
const io = require('socket.io')(server);

// Listen for connections
io.on('connection', (socket) => {
    console.log(`Client ${socket.id} connected`);



    // Listen for drone state updates from Python program
    socket.on('drone-state', (data) => {
        console.log(`Received drone state update from drone ${JSON.stringify(data)}`);
        io.sockets.emit('drone-state-web', JSON.stringify(data))
        // Handle the state update
        // ...
    });

    socket.on('test-from-web-app', (data) => {
        console.log(`test-from-web-app ${JSON.stringify(data)}`);
        // io.sockets.emit('drone-state-web', 'Test message')

        // io.sockets.emit('send', 'test')
        // Handle the state update
        // ...
    });
    //
    // // Send commands to Python program
    // socket.on('send-command', (data) => {
    //     console.log(`Sending command to drone ${data.droneId}: ${data.command}`);
    //     // Send the command to the appropriate drone
    //     // ...
    // });
});

// Start the server
const PORT = process.env.PORT || 3000;
server.listen(PORT, () => {
    console.log(`Server listening on port ${PORT}`);
});
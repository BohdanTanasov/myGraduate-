const express = require('express');
const { json } = require('express');
const app = express();
const server = require('http').Server(app);
const io = require('socket.io')(server);

const countOfDrones = 1;
const dronesState = {};

// for (let i = 0; i < countOfDrones; i++) {
//     dronesState[i] = {};
// }
// Listen for connections
io.on('connection', (socket) => {
  console.log(`Client ${socket.id} connected`);

  // Listen for drone state updates from Python program
  socket.on('drone-state', (data) => {
    console.log(`Received drone state update from drone ${data['state']}`);
    dronesState[data['drone_id']] = { state: data.state, coordinates: data.coordinates };
    io.sockets.emit('drone-state-web', dronesState);
    // Handle the state update
    // ...
  });

  socket.on('test-from-web-app', (data) => {
    console.log(`test-from-web-app ${JSON.stringify(data)}`);
    // io.sockets.emit('drone-state-web', 'Test message')

    io.sockets.emit('send', data);
    // Handle the state update
    // ...
  });
});

// Start the server
const PORT = process.env.PORT || 3000;
server.listen(PORT, () => {
  console.log(`Server listening on port ${PORT}`);
});

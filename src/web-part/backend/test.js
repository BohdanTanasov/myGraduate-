const { io } = require("socket.io-client");

const socket = io('http://localhost:3000');
socket.on('connect', ()=> {
    console.log('sdfdsfdsfds')
    socket.emit('drone-state', 'test')
})


socket.on('send', (data) => {
    console.log(data)
})


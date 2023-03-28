import React, { useEffect, useState } from 'react';
import io from 'socket.io-client';

export const SocketContext = React.createContext();

export const SocketProvider = ({ children }) => {
  const [socket, setSocket] = useState(null);

  useEffect(() => {
    // const socket = io('https://6033-147-232-175-132.eu.ngrok.io');
    const socket = io('http://192.168.31.102:3000');
    setSocket(socket);
  }, []);

  console.log('socket:', socket); // Add this line

  return <SocketContext.Provider value={socket}>{children}</SocketContext.Provider>;
};

export default SocketContext;

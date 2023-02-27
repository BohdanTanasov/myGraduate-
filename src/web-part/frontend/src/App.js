import {useEffect, useState} from "react";
import logo from './logo.svg';
import './App.css';
import io from 'socket.io-client';

const socket = io('http://localhost:3000');

function App() {
  const [message, setMessage] = useState('');

  useEffect(() => {
    socket.on('drone-state-web', (data) => {
      console.log(data)
      setMessage(data);
    });

    // return () => {
    //   socket.disconnect();
    // };
  }, []);


  return (
    <div className="App">
      <header className="App-header">
        <img src={logo} className="App-logo" alt="logo" />
        <p>
          Edit <code>src/App.js</code> and save to reload.
        </p>
        {/*{message}*/}
        <button onClick={() => {
          socket.emit('test-from-web-app', 'Hello from the client!');
        }
        }>Test</button>
      </header>
    </div>
  );
}

export default App;

import { Container } from '@mui/material';
import './App.css';
import Component from './components/component';
import { SocketProvider } from './socket-context';

// const socket = io('http://localhost:3000');

function App() {
  return (
    <SocketProvider>
      <div className="App">
        <Container sx={{ py: 3 }}>
          <Component />
        </Container>
      </div>
    </SocketProvider>
  );
}

export default App;

import NavigationIcon from '@mui/icons-material/Navigation';
import VisibilityIcon from '@mui/icons-material/Visibility';
import VisibilityOffIcon from '@mui/icons-material/VisibilityOff';
import {
  Box,
  Button,
  Dialog,
  Fade,
  Grid,
  Icon,
  IconButton,
  Tab,
  Tabs,
  ToggleButton,
  ToggleButtonGroup,
  Typography,
} from '@mui/material';
import { useContext, useEffect, useState } from 'react';
import BatteryGauge from 'react-battery-gauge';
import Plot from 'react-plotly.js';
import { SocketContext } from '../socket-context';
import Drone from './../assets/drone.svg';
import DroneDisabled from './../assets/drone-disabled.svg';
import FlightTakeoffIcon from '@mui/icons-material/FlightTakeoff';
import FlightLandIcon from '@mui/icons-material/FlightLand';
const NavigationButton = ({ direction, icon, onClick }) => {
  return (
    <IconButton sx={{ background: '#73B1FF' }} onClick={onClick}>
      <Icon
        fontSize={'large'}
        sx={{
          color: 'white',
          '& > svg': {
            transform: `rotate(${
              direction === 'left' ? 270 : direction === 'right' ? 90 : direction === 'bottom' ? 180 : 0
            }deg)`,
          },
        }}
      >
        {icon}
      </Icon>
    </IconButton>
  );
};

const ControlBlock = ({ firstElement, secondElement, thirdElement, fourElement }) => {
  return (
    <Grid container width={150} height={150}>
      <Grid item xs={4}></Grid>
      <Grid item xs={4}>
        {firstElement}
      </Grid>
      <Grid item xs={4}></Grid>
      <Grid item xs={4}>
        {secondElement}
      </Grid>
      <Grid item xs={4}></Grid>
      <Grid item xs={4}>
        {thirdElement}
      </Grid>
      <Grid item xs={4}></Grid>
      <Grid item xs={4}>
        {fourElement}
      </Grid>
      <Grid item xs={4}></Grid>
    </Grid>
  );
};

export const Component = () => {
  const socket = useContext(SocketContext);

  const [mode, setMode] = useState('individual');
  const [drone, setDrone] = useState(0);
  const [state, setState] = useState({
    0: { bat: 34, pitch: 23, roll: 0.55, yaw: 2.78, h: 60 },
    4: { bat: 31, pitch: 23, roll: 0.55, yaw: 2.78, h: 60 },
  });

  const [checked, setChecked] = useState(false);

  const handleChange = () => {
    setChecked((prev) => !prev);
  };

  const handleModeChange = (event, newAlignment) => {
    setMode(newAlignment);
  };

  const handleDroneChange = (event, newValue) => {
    newValue in state && setDrone(newValue);
  };

  useEffect(() => {
    socket?.on('drone-state-web', (data) => {
      // console.log(data);
      // console.log(data);
      console.log('sdfdsf');
      setState(data);
    });
  }, []);

  console.log(state);

  const result = [];

  for (let i = 0; i < state[0].coordinates?.length; i++) {
    const arr = [];
    for (let j = 0; j < Object.keys(state)?.length; j++) {
      arr.push(state[j].coordinates[i]);
    }
    result.push(arr);
  }

  return (
    <Grid container spacing={3}>
      <Grid item xs={12} sx={{ display: 'flex', justifyContent: 'center' }}>
        <ToggleButtonGroup
          color="primary"
          sx={{
            background: '#393939',
            fontWeight: 'bold',
            '& .Mui-selected': { background: '#73B1FF', color: '#E7E7E7 !important' },
            '& .MuiToggleButton-root': { color: '#B4B4B4', fontWeight: 'bold' },
          }}
          value={mode}
          exclusive
          onChange={handleModeChange}
          aria-label="Platform"
        >
          <ToggleButton value="individual">Individual mode</ToggleButton>
          <ToggleButton value="group">Group mode</ToggleButton>
        </ToggleButtonGroup>
      </Grid>

      <Grid item xs={12} sx={{ display: 'flex', justifyContent: 'center' }}>
        <Tabs
          value={drone}
          onChange={handleDroneChange}
          aria-label="disabled tabs example"
          variant="scrollable"
          scrollButtons="auto"
        >
          {Array.from({ length: 6 }, (_, index) => (
            <Tab
              label={<img src={index in state ? Drone : DroneDisabled} width={'100px'} />}
              value={index}
              sx={{ p: 3, cursor: index in state ? 'pointer' : 'not-allowed;' }}
            />
          ))}
        </Tabs>
      </Grid>

      <Grid item container xs={12} justifyContent={'center'} mb={3}>
        {/*<FormControlLabel*/}
        {/*  sx={{ display: 'flex', alignItems: 'flex-start' }}*/}
        {/*  control={*/}
        <Button variant={'text'} onClick={handleChange}>
          <Typography pr={1}>Status</Typography> {checked ? <VisibilityIcon /> : <VisibilityOffIcon />}
        </Button>
        {/*}*/}
        {/*/>*/}

        <Fade in={checked}>
          <Box sx={{ display: 'flex' }}>
            <Grid item>
              <BatteryGauge value={state[drone].state?.bat} orientation={'vertical'} size={150} />
            </Grid>
            <Grid item>
              <Typography variant={'h6'} sx={{ background: '#73B1FF', py: 1, px: 3 }}>
                Pitch: {state[drone].state?.pitch}
              </Typography>
            </Grid>
            <Grid item>
              <Typography variant={'h6'} sx={{ background: '#9AADD2', p: 1, px: 3 }}>
                Roll: {state[drone].state?.roll}
              </Typography>
            </Grid>
            <Grid item>
              <Typography variant={'h6'} sx={{ background: '#7394A6', p: 1, px: 3 }}>
                Yaw: {state[drone].state?.yaw}
              </Typography>
            </Grid>
          </Box>
        </Fade>
      </Grid>
      <Grid container>
        <Grid item container xs={12} md={4} px={15} justifyContent={'center'} alignContent={'center'}>
          <ControlBlock
            firstElement={<NavigationButton direction={'top'} icon={<NavigationIcon />} onClick={() => {}} />}
            secondElement={<NavigationButton direction={'left'} icon={<NavigationIcon />} onClick={() => {}} />}
            thirdElement={<NavigationButton direction={'right'} icon={<NavigationIcon />} onClick={() => {}} />}
            fourElement={<NavigationButton direction={'bottom'} icon={<NavigationIcon />} onClick={() => {}} />}
          />
        </Grid>
        <Grid item container xs={12} md={4}>
          {/*<img src={Space3D} width={'100%'} height={'100%'} />*/}
        </Grid>
        <Grid item container xs={12} md={4} px={15} justifyContent={'center'} alignContent={'center'}>
          <ControlBlock
            firstElement={
              <NavigationButton
                direction={'top'}
                icon={<FlightTakeoffIcon />}
                onClick={() => {
                  socket?.emit('test-from-web-app', { drone: drone, command: 'take-off' });
                }}
              />
            }
            secondElement={<NavigationButton direction={'bottom'} onClick={() => {}} />}
            thirdElement={<NavigationButton direction={'right'} onClick={() => {}} />}
            fourElement={
              <NavigationButton
                direction={'top'}
                icon={<FlightLandIcon />}
                onClick={() => {
                  socket?.emit('test-from-web-app', { drone: drone, command: 'land' });
                }}
              />
            }
          />
        </Grid>
      </Grid>

      <Button
        variant={'contained'}
        onClick={() => {
          socket?.emit('test-from-web-app', 'Hello from the client!');
        }}
      >
        Tale Off
      </Button>

      <Dialog open={false} maxWidth>
        <Plot
          data={[
            {
              x: result[0],
              y: result[1],
              z: result[2],
              mode: 'markers',
              type: 'scatter3d',
            },
          ]}
          layout={{
            height: 1000,
            width: 1000,
          }}
        />
      </Dialog>

      {/*<p className="status">Status: </p>*/}
      {/*<span>Battery</span>*/}
      {/*{message[0]?.bat}*/}
      {/*<span>pitch</span>*/}
      {/*{message[0]?.pitch}*/}
      {/*<span>roll</span>*/}
      {/*{message[0]?.roll}*/}
      {/*<span>yaw</span>*/}
      {/*{message[0]?.yaw}*/}
      {/*<span>h</span>*/}
      {/*{message[0]?.h}*/}

      {/*{message}*/}
    </Grid>
  );
};

export default Component;

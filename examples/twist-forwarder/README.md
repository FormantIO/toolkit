## Twist Forwarder

This module connects to local Twists and Forwards them across the Formant Realtime data channel to the device

This depends on

```
https://github.com/ltiseni94/rosreact
```

Please install:

```
sudo apt-get install ros-DISTRO-rosbridge-suite
```
Then run, in separate terminals
```
#terminal 1:
roslaunch rosbridge_server rosbridge_websocket.launch
#terminal 2:
yarn
yarn dev
```

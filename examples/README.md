
# Formant Toolkit Examples

A series of examples about how to extend Formants capabilities; including views, adapters and functionalities.

## Index

### Views/Moules

- Examples of what can be built that can be embeded on Formant or individually hosted.

### [Represent altitude and orientation on a map](https://github.com/FormantIO/toolkit/tree/master/examples/altitude-on-map) (React)

The Formant agent is able to collect data points that represent the altitude and orientation of a device. This data is collected from the 'Location' stream.

![App Screenshot](https://github.com/FormantIO/toolkit/blob/master/examples/altitude-on-map/images/orientation.png)
The altitude is measured in Meters (m), the orientation is measured in Radians (rotation). To use orientation in the map it is necessary to convert Radian to Degrees.

### [Issue a command](https://github.com/FormantIO/toolkit/tree/master/examples/command-issuer) (React)

This example encapsulates the proccess of sending a command to a device using the data-sdk inside a React component. It is a button which takes as a parametere the name of the command, and when you click on it it issue the command to your device.
 

```javascript
import { CommandIssuer } from "./CommandIssuer/index";

function App() {
    <CommandIssuer command="Start" />
}
```

![App Screenshot](https://github.com/FormantIO/toolkit/blob/master/examples/command-issuer/images/command-issuer.png)



### [Commands](https://github.com/FormantIO/toolkit/tree/master/examples/commands) (TypeScript)

This custom view is a demonstration of how the data sdk can be used for sending commands to a robot. When you're device is ready, find a command, type its name in the input, and type in a string into the command data input.

![App Screenshot](https://github.com/FormantIO/toolkit/blob/master/examples/commands/images/commands.png)



### [Download video with a button](https://github.com/FormantIO/toolkit/tree/master/examples/commands) (React)

This example shows how to download a video from a stream.

![Download button widget](https://github.com/FormantIO/toolkit/blob/master/examples/download-video-stream/images/video.gif)


[ROS Topics Diagnostics](https://github.com/FormantIO/toolkit/tree/master/examples/ros-diagnostics) (React)

[Login Form](https://github.com/FormantIO/toolkit/tree/master/examples/vr) (React)

[Customizable welcome page](https://github.com/FormantIO/toolkit/tree/master/examples/simple-welcome) (HTML/Formant CDN)

[Weather visualization](https://github.com/FormantIO/toolkit/tree/master/examples/simple-weather-visualization) (HTML/ Formant CDN)

[URDF](https://github.com/FormantIO/toolkit/tree/master/examples/simple-urdf) (TypeScript / Three.JS / Data-SDK)

[Visualize different streams in the same graph](https://github.com/FormantIO/toolkit/tree/master/examples/scatter-plot) (TypeScript / D3)

### Teleop

- Examples of how to perform a teleoperation using the Data SDK.

[Teleoperation using Data-SDK](https://github.com/FormantIO/toolkit/tree/master/examples/teleop-cra) (React)

[Teleoperation using Data-SDK](https://github.com/FormantIO/toolkit/tree/master/examples/teleop) (TypeScript)

### [Binary request response](https://github.com/FormantIO/toolkit/tree/master/examples/binary-request-response) (TypeScript + Python Adapter)
This custom view demonstrates how RequestDataChannel can be used to write applications with a request-response pattern. This particular example is for binary request data channels, which accept Uint8Array requests in javascript and can be used for protocols like protobuf. Press spacebar to make a request. If the adapter is running, a response will come in. If no adapter is running, the request will time out.
#### Styling

- Example of components using Formant's CDN, providing the ability to focus on developing while we will handle the design.

[Style components](https://github.com/FormantIO/toolkit/tree/master/examples/simple-style) (HTML / Formant CDN)

#### Desktop

- Example of how to create a Formant desktop application.

[Login and list devices](https://github.com/FormantIO/toolkit/tree/master/examples/simple-electron) (Electron)

#### Adapter

- Examples of how to extend Formant's agent caplabilities to perform communication with our APIs

[ROS Topics Diagnostics](https://github.com/FormantIO/toolkit/tree/master/examples/ros-diagnostics) (Python)

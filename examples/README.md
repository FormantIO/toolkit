# Formant Toolkit Examples

A series of examples about how to extend Formants capabilities; including views, adapters and functionalities.

# Index

- #### [Views/Modules](https://github.com/FormantIO/toolkit/tree/master/examples#viewsmoules)
  - [Represent altitude and orientation on a map](https://github.com/FormantIO/toolkit/tree/master/examples#represent-altitude-and-orientation-on-a-map-react)(React)
  - [Issue a command](https://github.com/FormantIO/toolkit/tree/master/examples#issue-a-command-react) (React)
  - [Commands](https://github.com/FormantIO/toolkit/tree/master/examples#commands-typescript) (TypeScript)
  - [How to download video stream](https://github.com/FormantIO/toolkit/tree/master/examples#download-video-with-a-button-react) (React)
  - [ROS Topic diagnostics](https://github.com/FormantIO/toolkit/tree/master/examples#ros-topics-diagnostics-react--python-adapter) (React)
  - [Login form](https://github.com/FormantIO/toolkit/tree/master/examples#login-form-react) (React)
  - [Customizable welcome page](https://github.com/FormantIO/toolkit/tree/master/examples#customizable-welcome-page-htmlformant-cdn) (HTML / CDN)
  - [Weather visualization](https://github.com/FormantIO/toolkit/tree/master/examples#weather-visualization-html-formant-cdn) (HTML/CDN)
  - [URDF](https://github.com/FormantIO/toolkit/tree/master/examples#urdf-typescript--threejs--data-sdk) (TypeScript / Three.JS / Data-SDK)
  - [Scatter plot](https://github.com/FormantIO/toolkit/tree/master/examples#visualize-different-streams-in-the-same-graph-typescript--d3) (Typescript / D3)
- #### [Teleoperation](https://github.com/FormantIO/toolkit/tree/master/examples#teleop)

  - [Teleoperation using Data-SDK](https://github.com/FormantIO/toolkit/tree/master/examples#teleoperation-using-data-sdk-react) (React)
  - [Teleoperation using Data-SDK](https://github.com/FormantIO/toolkit/tree/master/examples#teleoperation-using-data-sdk-typescript) (TypeScript)
  - [Binary request response](https://github.com/FormantIO/toolkit/tree/master/examples#teleoperation-using-data-sdk-typescript) (TypeScript + Python Adapter)
  - [Custom data channel comminucation](https://github.com/FormantIO/toolkit/tree/master/examples/custom-data-channel-communication) (TypeScript + Python Adapter)

  - [Protobuf communication](https://github.com/FormantIO/toolkit/tree/master/examples/protobufjs-communication) (TypeScript + Python Adapter)

  - [Request response protocol](https://github.com/FormantIO/toolkit/tree/master/examples#request-response-protocol) (TypeScript + Python Adapter)

- #### [Styling](https://github.com/FormantIO/toolkit/tree/master/examples#styling)
  - [Style components](https://github.com/FormantIO/toolkit/tree/master/examples#style-components-html--formant-cdn) (HTML /CDN)
- #### [Desktop](https://github.com/FormantIO/toolkit/tree/master/examples#desktop)
  - [Login and list devices](https://github.com/FormantIO/toolkit/tree/master/examples#login-and-list-devices-electron) (Electron)
- #### [Adapters](https://github.com/FormantIO/toolkit/tree/master/examples#login-and-list-devices-electron)
  - [ROS Topics Monitor](https://github.com/FormantIO/toolkit/tree/master/examples#adapter) (Python)
  - [ROS Services](https://github.com/FormantIO/toolkit/tree/master/examples#adapter) (Python)


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
  <CommandIssuer command="Start" />;
}
```

![App Screenshot](https://github.com/FormantIO/toolkit/blob/master/examples/command-issuer/images/command-issuer.png)

### [Commands](https://github.com/FormantIO/toolkit/tree/master/examples/commands) (TypeScript)

This custom view is a demonstration of how the data sdk can be used for sending commands to a robot. When you're device is ready, find a command, type its name in the input, and type in a string into the command data input.

![App Screenshot](https://github.com/FormantIO/toolkit/blob/master/examples/commands/images/commands.png)

### [Download video with a button](https://github.com/FormantIO/toolkit/tree/master/examples/download-video-stream) (React)

This example shows how to download a video from a stream.

![Download button widget](https://github.com/FormantIO/toolkit/blob/master/examples/download-video-stream/images/video.gif)

### [ROS Topics Diagnostics](https://github.com/FormantIO/toolkit/tree/master/examples/ros-diagnostics) (React + Python Adapter)

This module has a configuration that shows which ROS topics are expected. It reads the data that was sent from the adapter and then it compares the data from the configuration with the data that is being received.

![App Screenshot](https://github.com/FormantIO/toolkit/blob/master/examples/ros-diagnostics/images/table.png)

### [Login Form](https://github.com/FormantIO/toolkit/tree/master/examples/vr) (React)

We provide a simple component ui-sdk-login to handle authorization for you if you just want to focus on user experience.
[Try it](https://formantio.github.io/toolkit/examples/simple-login/dist/index.html)

![App Screenshot](https://user-images.githubusercontent.com/66638393/134783659-5491c1d2-ea63-4b85-b3ea-22c3de4c5df8.png)

### [Customizable welcome page](https://github.com/FormantIO/toolkit/tree/master/examples/simple-welcome) (HTML/Formant CDN)

This is a sample welcome page that can be completely customized to show off your robot's value at your customer's farm.

Formant CDN:

```javascript
<head>
  <link
    rel="stylesheet"
    href="https://cdn.jsdelivr.net/gh/formantio/toolkit/css/formant.css"
  />
</head>
```

![App Screenshot](https://github.com/FormantIO/toolkit/blob/master/examples/simple-welcome/images/farm.png)

### [Weather visualization](https://github.com/FormantIO/toolkit/tree/master/examples/simple-weather-visualization) (HTML/ Formant CDN)

Example of a custom module that can be embeded inside Formant

![App Screenshot](https://github.com/FormantIO/toolkit/blob/master/examples/simple-weather-visualization/images/weather.png)

### [URDF](https://github.com/FormantIO/toolkit/tree/master/examples/simple-urdf) (TypeScript / Three.JS / Data-SDK)

This exaple shows how to use tranformation tree stream to create visual representation of the robot.

![App Screenshot](https://github.com/FormantIO/toolkit/blob/master/examples/simple-urdf/images/urdf.png)

### [Visualize different streams in the same graph](https://github.com/FormantIO/toolkit/tree/master/examples/scatter-plot) (TypeScript / D3)

This example how you can display different streams in the same graph.

![App Screenshot](https://github.com/FormantIO/toolkit/blob/master/examples/scatter-plot/images/Screen%20Shot%202022-04-20%20at%202.06.20%20PM.png)

## Teleop

- Examples of how to perform a teleoperation using the Data SDK.

### [Teleoperation using Data-SDK](https://github.com/FormantIO/toolkit/tree/master/examples/teleop-cra) (React)

This custom view is a demonstration of how the data sdk can be used with the ui sdk for realtime teleop views. In particular we use ui-sdk-joystick and ui-sdk-realtime-player. Checkout https://github.com/FormantIO/toolkit for more information!

### [Teleoperation using Data-SDK](https://github.com/FormantIO/toolkit/tree/master/examples/teleop) (TypeScript)

This custom view is a demonstration of how the data sdk can be used with the ui sdk for realtime teleop views. In particular we use ui-sdk-joystick and ui-sdk-realtime-player. Checkout https://github.com/FormantIO/toolkit for more information!

### [Binary request response](https://github.com/FormantIO/toolkit/tree/master/examples/binary-request-response) (TypeScript + Python Adapter)

This custom view demonstrates how RequestDataChannel can be used to write applications with a request-response pattern. This particular example is for binary request data channels, which accept Uint8Array requests in javascript and can be used for protocols like protobuf. Press spacebar to make a request. If the adapter is running, a response will come in. If no adapter is running, the request will time out.

### [Custom data channel communication](https://github.com/FormantIO/toolkit/tree/master/examples/custom-data-channel-communication) (TypeScript + Python Adapter)

This custom view demonstrates how arbitrary bi-directional communication
over real-time custom data channels works.

### [Protobuf communication](https://github.com/FormantIO/toolkit/tree/master/examples/protobufjs-communication) (TypeScript + Python Adapter)

This custom view demonstrates how to send binary data, such as protobuf,
over real-time custom data channels.

### [Request response protocol](https://github.com/FormantIO/toolkit/tree/master/examples/request-response)

This custom view demonstrates how RequestDataChannel can be used to write applications
with a request-response pattern.

### Styling

- Example of components using Formant's CDN, providing the ability to focus on developing while we will handle the design.

### [Style components](https://github.com/FormantIO/toolkit/tree/master/examples/simple-style) (HTML / Formant CDN)

You can easily set the default styling of your HTML to something Formant-y with a single line:

```javascript
<link
  rel="stylesheet"
  href="https://cdn.jsdelivr.net/gh/formantio/toolkit/css/formant.css"
/>
```

With this in your html head you will get default colors, appropriate fonts, and very basic elements will be styled.

![App Screenshot](https://github.com/FormantIO/toolkit/blob/master/examples/simple-style/images/style.png)

### Desktop

- Example of how to create a Formant desktop application.

### [Login and list devices](https://github.com/FormantIO/toolkit/tree/master/examples/simple-electron) (Electron)

![App Screenshot](https://github.com/FormantIO/toolkit/blob/master/images/preview_electron.png)

### Adapter

- Examples of how to extend Formant's agent caplabilities to perform communication with our APIs

### [ROS Topics Monitor](https://github.com/FormantIO/toolkit/tree/master/examples/ros-diagnostics) (Python)

This adapter collects all the online ROS topics and publish them to our API

### [ROS Services](https://github.com/FormantIO/toolkit/tree/master/examples/ros-service-call-adapter) (Python)

This adapter collects all the available ROS services and publish them to our API
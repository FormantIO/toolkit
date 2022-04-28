# Altitude and orientation on a map

The Formant agent is able to collect data points that represent the altitude and orientation of a device. This data is collected from the 'Location' stream.

## Run Locally

go to the project directory

```bash
  cd toolkit/examples/altitude-on-map
```

Install dependencies

```bash
  npm i
```

Start the server

```bash
  npm run dev
```

![App Screenshot](https://github.com/FormantIO/toolkit/blob/master/examples/altitude-on-map/images/orientation.png)

The altitude is measured in Meters (m), the orientation is measured in Radians (rotation). To use orientation in the map it is necessary to convert Radian to Degrees.

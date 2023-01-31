# ROS Diagnostic Message Table
This module displays the structure of a JSON object used to represent the status of individual components of a robot. The JSON object contains information about the level of operation, the name and description of the test/component, a hardware ID, and an array of values associated with the status.


### JSON Object Structure

The JSON object has the following structure:
```javascript
{
  "level": byte,
  "name": string,
  "message": string,
  "hardware_id": string,
  "values": KeyValue[]
}
```

#### Field Descriptions

- **level**: An enumerated byte representing the level of operation of the component. The possible values are:

  - **OK** (0): indicates normal operation.
  - **WARN** (1): indicates a warning condition.
  -  **ERROR** (2): indicates an error condition.
  - **STALE** (3): indicates a stale or outdated status.
-**name**: A string that provides a description of the test/component reporting the status.

-**message**: A string that provides a description of the status of the component.

-**hardware_id**: A string that is a hardware unique identifier for the component.

-**values**: An array of KeyValue objects associated with the status of the component. The structure of a KeyValue object is:

```javascript
{
  "key": string,
  "value": string
}
```

The **`key`** field is a string representing the name of the value, and the **`value`** field is a string representing the value associated with the key.

## Usage/Examples
Expected JSON object
```javascript
{
  "level": 0,
  "name": "Arm component",
  "message": "Normal operation",
  "hardware_id": "ABC123",
  "values": [
    {
      "key": "Angle",
      "value": "45 degrees"
    },
    {
      "key": "Speed",
      "value": "10 RPM"
    }
  ]
}

```


###  URL

https://formantio.github.io/toolkit/examples/ros-diagnostics-msgs/dist/index.html?auth={auth}&device={device_id}
# Heatmap

This module allows you to generate a heatmap from datapoints ingested in Formant. In the confiuration you must set a start, and end point in time to query the datapoints.

## Configuration

| Parameter                     | Type     | Description                                                                              |
| :---------------------------- | :------- | :--------------------------------------------------------------------------------------- |
| `locationStream`              | `string` | **Required**. Formant's location stream name                                             |
| `start`                       | `object` | **Required**. Time to start the query.                                                   |
| `end`                         | `object` | **Required**. Time to end the query                                                      |
| `numericStream`               | `string` | **Required**. Formant's numeric stream name                                              |
| `latitude`                    | `string` | Will center the map at this location in the first render                                 |
| `longitude`                   | `string` | Will center the map at this location in the first render                                 |
| `zoom`                        | `number` | Will zoom into the the center of tha map if longitude and latidude are set               |
| `maxSecondsBetweenDatapoints` | `string` | Max time range to coorelate datapoints if numeric and location streams are set (seconds) |
| `distinctZoomLevel`           | `number` | When zoom level gets at this level single points will replace heatmap layer              |
| `circleRadius`                | `number` | Circle radius of distinc points                                                          |

#### Start query Parameter

The start object contains information about the starting time stamp of the query. It has the following properties:
| Properties | Value(s) |Type | Description |  
| :--------- | :---- |:----|:-------------------- |
| `type`| `timeRange` | `integer` | Represent how many hours to query back before the end time|
|-|`Event` | `string` | will query the last time this event was trigger and use the timestimap as starting point|
|`value`| -| -| If the `type` is `timeRange` the value must be an integer, otherwise the if the `type` is `event` the value will be a string with the name of the event |

#### End query Parameter

The end object contains information about the final time stamp of the query. It has the following properties:

| Properties | Value(s)     | Type      | Description                                                                             |
| :--------- | :----------- | :-------- | :-------------------------------------------------------------------------------------- |
| `type`     | `timeRange`  | `integer` | Represent how many hours to query after the start time                                  |
| -          | `Event`      | `string`  | will query the last time this event was trigger and use the timestimap as end point     |
| -          | `Annotation` | `string`  | will query the last time this event was trigger and use the end timestimap as end point |
| -          | `scrubber`   | `string`  | will sync with the time line to query data                                              |
| `value`    | -            | -         | It can be a interger or a string depending on the type selected                         |

## Usage/Examples

The minimum configuration requires a start and end point, as well as a location stream. If a numeric stream is not provided, the configuration will plot a point for each location data point ingested within the specified time frame and set its numeric value to 1.

```javascript
{
  "locationStream": "location_stream_name",
  "start": {
    "type": "timeRange",
    "value": 1
  },
  "end": {
    "type": "scrubber",
    "value": ""
  }
}
```

In the following example, we set a numeric data stream and correlate it with the nearest location data point in time. The `maxSecondsBetweenDatapoints` property determines the validity of a data point. If the time difference between the location and numeric data points is greater than 5 seconds, it will be considered invalid and the next data point will be evaluated.

```javascript
{
  "locationStream": "location_stream_name",
  "start": {
    "type": "timeRange",
    "value": 1
  },
  "end": {
    "type": "scrubber",
    "value": ""
  },
  "longitude": -77.47419738769531,
  "latitude": 39.043701171875,
  "zoom": 15,
  "numericStream": "heatmap_point_weight",
  "maxSecondsBetweenDatapoints": 5,
  "distinctZoomLevel": 19
}
```

### URL

https://formantio.github.io/toolkit/examples/generic-heatmap/dist/index.html?auth={auth}&device={device_id}&configuration={configuration}&module={module}

![App Screenshot](https://github.com/FormantIO/toolkit/blob/master/examples/generic-heatmap/src/images/heatmap.png)

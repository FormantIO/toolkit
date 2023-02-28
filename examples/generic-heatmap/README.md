# Heatmap

Display location data over time as a heat map.

## Required parameters

| Parameter        | Type     | Description                                                                                                                             |
| :--------------- | :------- | :-------------------------------------------------------------------------------------------------------------------------------------- |
| `locationStream` | `string` | Name of location stream in Formant to view as heatmap name                                                                              |
| `queryType`      | `string` | Determines how the data is queried. It is an enumeration of three possible values: `Events`, `Annotation`, and `Active Timeline point`. |

### Active Timeline point (Scrubber)

The `scrubber` property is an object that is visible only when queryType is set to `Active Timeline point`
| Parameter | Type | Description |
| :---------------------------- | :------- | :--------------------------------------------------------------------------------------- |
| `startFrom` | `string` | Determines where to start the query. It is an enumeration of two possible values: `Time Delta` or `Event`. |
| `minutes` | `integer`| Available if `startFrom` is set to `Time Delta` |
| `hours` | `integer`| Available if `startFrom` is set to `Time Delta` |
| `eventName` | `string` | Will query and use the time stamp of the last instance of this event, Available if `startFrom` is set to `Event` |

```javascript
//Time Delta configuration                                  //Event name configuration
{                                                            {
  "locationStream": "location_stream_name",                    "locationStream": "location_stream_name",
  "queryType": "Active Timeline point"                         "queryType": "Active Timeline point"
   "scrubber": {                                               "scrubber": {
    "startFrom": "Time Delta",                                  "startFrom": "Event",
    "Hours": 1,                                                 "eventName": "event_name"
    "minutes": 1                                                 }
  }                                                           }
}
```

### Events Query

The `eventsQuery` property is an object that is visible only when queryType is set to `Events`.

| Parameter    | Type      | Description                                                                                                        |
| :----------- | :-------- | :----------------------------------------------------------------------------------------------------------------- |
| `endEvent`   | `string`  | **Required**. Will query and use the time stamp of the last instance of this event as end point for the data query |
| `startFrom`  | `string`  | Determines where to start the query. It is an enumeration of two possible values: `Time Delta` or `Event`.         |
| `minutes`    | `integer` | Available if `startFrom` is set to `Time Delta`                                                                    |
| `hours`      | `integer` | Available if `startFrom` is set to `Time Delta`                                                                    |
| `startEvent` | `string`  | Will query and use the time stamp of the last instance of this event, Available if `startFrom` is set to `Event`   |

```javascript
//Time Delta configuration                                  //Event name configuration
{                                                            {
  "locationStream": "location_stream_name",                    "locationStream": "location_stream_name",
  "queryType": "Events"                                        "queryType": "Events"
   "eventsQuery": {                                            "eventsQuery": {
    "startFrom": "Time Delta",                                  "startFrom": "Event",
    "Hours": 1,                                                 "endEvent": "event_name"
    "minutes": 1                                                "startEvent" : "event_name"
    "endEvent": "event_name"                                    }
  }                                                           }
}
```

### Annotation Query

The `annotation` property is a string that is visible only when `queryType` is set to `Annotation`.
| Parameter | Type | Description |
| :---------------------------- | :------- | :--------------------------------------------------------------------------------------- |
|`annotation` | `string` | Query will use annotation start, and end time to request data. |

## Advanced Settings

They control the level of detail in the visualization, including the radius of the heat circles, the intensity of the heatmap, and the tooltip text that is displayed when hovering over a heat circle.

| Parameter                | Type     | Description                                                                                                                                                                       |
| :----------------------- | :------- | :-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `mapboxKey`              | `string` | It specifies a Mapbox API key to use for map visualization. https://docs.mapbox.com/help/glossary/access-token/                                                                   |
| `weightStream`           | `string` | Name of a numeric stream in Formant. If timestamps are within the search window, the datapoint’s value will scale the size of the heat circle. If not, a value of 1 will be used. |
| `weightSearchWindow`     | `string` | Max window around location timestamp to search for a corresponding weight value in the numeric stream                                                                             |
| `latitude`               | `string` | Will center the map at this location in the first render                                                                                                                          |
| `longitude`              | `string` | Will center the map at this location in the first render                                                                                                                          |
| `defaultZoomLevel`       | `number` | from 1 (farthest) to 20 (closest)                                                                                                                                                 |
| `distinctPointZoomLevel` | `string` | At what zoom level to fade the heatmap and show individual locations.                                                                                                             |
| `heatmapIntensity`       | `number` | Increase intensity as zoom level increases.                                                                                                                                       |
| `circleRadius`           | `number` | Circle heat radius (px)                                                                                                                                                           |
| `tooltipLabel`           | `string` | Hover Tooltip Label                                                                                                                                                               |

## Usage/Examples

The minimum configuration requires a start and end point, as well as a location stream. If a numeric stream is not provided, the configuration will plot a point for each location data point ingested within the specified time frame and set its numeric value to 1.

```javascript
{
  "locationStream": "location_stream_name",
  "queryType": "Active Timeline point"
   "scrubber": {
    "startFrom": "Time Delta",
    "Hours": 1,
    "minutes": 1
  }
}
```

In the following example, we set a numeric data stream and correlate it with the nearest location data point in time. The `weightSearchWindow` property determines the validity of a data point. If the time difference between the location and numeric data points is greater than 5 seconds, it will be considered invalid and the next data point will be evaluated.

```javascript
{
  "locationStream": "location_stream_name",
  "queryType": "Active Timeline point"
   "scrubber": {
    "startFrom": "Time Delta",
    "Hours": 1,
    "minutes": 1
  },
  "weightStream": "numeric_stream_name",
  "weightSearchWindow": 2
  "longitude": -77.47419738769531,
  "latitude": 39.043701171875,
  "defaultZoomLevel": 15,
  "heatmapIntensity": 5,
  "distinctPointZoomLevel": 19,
  "circleRadius": 3,
  "tooltipLabel": "Robot"
}
```

### URL

https://formantio.github.io/toolkit/examples/generic-heatmap/dist/index.html?auth={auth}&device={device_id}&configuration={configuration}&module={module}

![App Screenshot](https://github.com/FormantIO/toolkit/blob/master/examples/generic-heatmap/src/images/heatmap.png)

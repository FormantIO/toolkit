
# Heatmap 

This module allows you to generate a heatmap feeded by a location stream

## Configuration
| Parameter | Type     | Key |Options|  Description         |       
| :-------- | :------- | :---| :-----|:----------------- |
| `locationStream` | `string` |-| -|**Required**. Formant's location stream name |
| `start` | `Object` |type|-| **Required**. Desire time to start the query |
|-|-|-|`timeRange`| **Required**. Time range will be an integer that represents how many hours to query|
|-|-|-|`Event`| Event will be a string which is the name of the event and the time stamp of when it was last triggered will be use as an starting point for the query|
|        -      |-| value | -| Actual Value to make the query. Event name for time range which will be an integer
| `end` | `Object` | - | -|**Required**. Desire end time for the query. |
| -|-|type| `timeRange`  |Time range will be an integer that represents how many hours to query|
| -|-|-|  `Event`  |  Event will be a string which is the name of the event and the time stamp of when it was last triggered will be use as the end point for the query|
| -|-|-|   `Annotation`  |Annotations can have a end time stamp. If you choose Event as your started point you can use the end time stamp in anootations to represent the end of the query |
| -|-|-|  `scrubber`  |Scrubber will sync with the time line and the scrubber position. This could be use to have generate a map with live incomming data|
| -|-|value|||-|
| `center` | `object` | -|-| Center on map when it first render, if not zoom is selected, center won't have any effect |
| - | -| longitude|-|coordinates|
| - | -| latitude|-|coordinates|
| `zoom` | `integer` | -|-| Integer between 1 - 20 |
| `numericStream` | `string` |--| | Formant's numeric stream, which will give a weight to the location data point |



### URL


https://formantio.github.io/toolkit/examples/generic-heatmap/dist/index.html?auth={auth}&device={device_id}&configuration={configuration}


![App Screenshot](https://github.com/FormantIO/toolkit/blob/master/examples/generic-heatmap/src/images/heatmap.png)

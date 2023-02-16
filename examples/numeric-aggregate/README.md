
# Numeric Aggregate

This module enables you to create a chart based on aggregated data from a numeric, or numeric set stream. You can select the aggregation type, such as sum or average, and the stream name. If you opt for a numeric set, you can specify the key to perform the aggregation on. You can also choose the aggregation time frame (day, week, or month) and the number of aggregations to display in the chart. Additionally, you can send data from multiple devices to generate a chart that combines information from several sources.

## Configuration
| Parameter | Type     |  Description         |    
| :-------- | :------- | :----------------- |
|`aggregateType`| `string` | **Required**. one of: `min`, `max`, `standard deviation`, `average`, `sum` |
|`streamName`| `string` | **Required**. Formant's data stream name. Either a numeric or numeric set stream name |
|`aggregateBy`| `string` |**Required** One of: `day`, `week`, `month` |
|`numericSetKey`| `string` | Key to aggregate from a numeric set |
|`numAggregates`| `integer` | Number of bars to show in the chart |
|`deviceIds`| `array` | List of devices ids|



## Usage/Examples
simple configuration with only list of names

```javascript
{
  "aggregateType": "sum",
  "streamName": "$.host.disk",
  "numericSetKey": "utilization",
  "aggregateBy": "day",
  "numAggregates": 7,
  "deviceIds": ["f5ca4b36-413d-43c5-b03c-91bd4b92a492"]
}


```




### URL


https://formantio.github.io/toolkit/examples/numeric-aggregate/dist/index.html?auth={auth}&device={device_id}&configuration={configuration}&module={module}



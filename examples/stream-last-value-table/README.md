## Stream Last Value Table

The Table module is used to monitor the status of a fleet of robots by tracking a list of data streams and comparing their latest values with expected values.

1. Set the list of data streams to monitor: To add a data stream to the Table, specify the stream name and its expected value. You can add multiple streams in this way.

2. Monitor data stream values: The Table will regularly retrieve the latest value for each data stream and compare it with the expected value. Based on this comparison, the status of each data stream will be updated. 
3. View the status of the robot fleet: The Table will display the status of each data stream, allowing you to quickly see the overall status of your robot fleet at a glance.


### Parameters

|Property | Type | Description | 
|:--------|:-----|:------------|
|`streams`|`array`|Array of formant streams. Allowed types are: Numeric, bitset, text|

## Stream Object

|Property | Type | Description | 
|:--------|:-----|:------------|
|`name`|`string`|Formant's stream name|
|`streamType` | `numeric`, `text`, `bitset`| Stream data type |
|`expectedString` | `string` | Expected value for stream of `text`  type |
|`expectedNumber` | `object` | Expected range where the number is considered healthy | 
|`expectedBool` | `object` | Set of keys and values |

### JSON Object Structure

The JSON object has the following structure:
```javascript
{
  "streams": [
    {
      "name": "blade.mode",
      "expectedString": "manual",
      "fullWidth": true,
      "streamType": "text"
    },
    {
      "name": "blade.range",
      "streamType": "numeric",
      "fullWidth": true,
      "expectedNumber": {
        "greaterThan": 10,
        "lesserThan": 60
      }
    },
    {
      "name": "blade.set",
      "fullWidth": true,
      "streamType": "bitset",
      "expectedBool": {
        "keys": ["isOn", "isManual", "isAutonomous"],
        "values": [true, true, true]
      }
    }
  ]
}

```




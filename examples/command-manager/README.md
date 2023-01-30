
# Command Buttons 

This module enables you to issue commands with a simple click of a button. You have the option to send a parameter with the command by providing a text input, or select a stream which will generate a drop-down list based on the stream's values.

## Configuration
| Parameter | Type     |  Description         |    
| :-------- | :------- | :----------------- |
|`commands`| `array` | **Required**. List of Formant's commands |

### Command object
 | Properties | Type     |  Description         |   
| :-------- | :------- | :----------------- |
|`name`| `string`|**Required** Name of the command in Formant|
|`enabledParameters`| `boolean`| Enables textfield to send parameters when the command is issued|
|`streamName` | `string` | This feature converts a CSV format text stream into a drop-down list based on its values to use as a parameter.|
|`needsConfirmation`| `boolean` | This feature activates a modal that prompts for confirmation before sending a command.|


## Usage/Examples
simple configuration with only list of names

```javascript
{
  "commands": [
    {
      "name": "download file test",
    },
    {
      "name": "publish.string.test",
    },
    {
      "name": "rosservice",
    }
  ]
}

````
Deeper configuration
```javascript
{
  "commands": [
    {
      "name": "download file test",
      "enabledParameters": false,
      "streamName": "",
      "needsConfirmation": false
    },
    {
      "name": "publish.string.test",
      "enabledParameters": true,
      "streamName": "",
      "needsConfirmation": false
    },
    {
      "name": "rosservice",
      "enabledParameters": true,
      "streamName": "blade.mode",
      "needsConfirmation": false
    },
    {
      "name": "test",
      "enabledParameters": false,
      "streamName": "",
      "needsConfirmation": true
    },
    {
      "name": "update_topic_list",
      "enabledParameters": false,
      "streamName": "",
      "needsConfirmation": false
    }
  ]
}

```



### URL


https://formantio.github.io/toolkit/examples/command-manager/dist/index.html?auth={auth}&device={device_id}&configuration={configuration}&module={module_id}



# Data Manager

A library for easily accessing the devices in your Formant fleet.

```
import { DataManager } from "@formant/data-manager";

await DataManager.waitUntilAuthenticated();
console.log(await DataManager.getDevices());
```

# Data Manager

A library for easily accessing the devices in your Formant fleet.

```
import { Fleet } from "@formant/data-sdk";

await Fleet.waitUntilAuthenticated();
console.log(await Fleet.getDevices());
```

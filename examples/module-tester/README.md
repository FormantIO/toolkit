# Module Tester

A comprehensive example demonstrating how to build custom modules using `@formant/data-sdk`. This example shows both single-device and **multi-device (group)** module patterns.

## Features Demonstrated

- ✅ Module detection and initialization
- ✅ Message bus communication with host
- ✅ Single-device context
- ✅ **Multi-device (group) context** - NEW!
- ✅ Stream data processing
- ✅ Channel data communication
- ✅ Module configuration
- ✅ Authentication token refresh

## Group Device Support

This example demonstrates how modules can work in **coherence group views** (multi-device contexts):

### Automatic Group Detection

The SDK automatically detects when you're in a group context:

```typescript
import { App, Fleet } from "@formant/data-sdk";

// Automatically receives group devices when host sends them
App.addOverviewDeviceListener((devices) => {
  console.log("Received devices:", devices);
  // Fleet.getGroupDevices() now returns all devices
});
```

### Accessing Group Devices

```typescript
// Get device info (id, name, tags) - works without API calls
const groupDevices = Fleet.getGroupDevices();
if (groupDevices && groupDevices.length > 1) {
  console.log(`Group context: ${groupDevices.length} devices`);
  groupDevices.forEach(device => {
    console.log(`${device.name} (${device.id})`);
  });
}

// Get full Device instances (requires API authentication)
const deviceInstances = await Fleet.getGroupDevicesAsDeviceInstances();
```

### Processing Multi-Device Stream Data

```typescript
import { App, ModuleData } from "@formant/data-sdk";

App.addModuleDataListener((data: ModuleData) => {
  // Each stream contains data for ALL devices
  Object.entries(data.streams).forEach(([streamName, stream]) => {
    // stream.data is an array - one entry per device
    stream.data.forEach((deviceData) => {
      console.log(`${deviceData.name}: ${deviceData.points.length} points`);
      // Process data for this device
    });
  });
});
```

## Quick Start

1. **Install dependencies:**
   ```bash
   npm install
   ```

2. **Start dev server:**
   ```bash
   npm run dev
   ```
   Note the URL (usually `http://localhost:4000`)

3. **Test with group-module-host:**
   - See `examples/group-module-host/README.md` for host setup and testing instructions

## Testing Group Features

1. Use the **"Test Group Devices"** button to:
   - Check if group devices are available
   - Display all devices in the group
   - Test fetching full Device instances

2. Use the **"Request module data"** button to:
   - Request stream data for all devices
   - See multi-device stream processing in console

3. Check browser console for:
   - `"Overview devices received: [...]"` - Group devices detected
   - `"Stream: X, Devices: N"` - Multi-device stream data
   - Device-specific data points

## Key APIs Used

- `App.isModule()` - Check if running as module
- `App.addOverviewDeviceListener()` - Listen for group devices
- `Fleet.getGroupDevices()` - Get group device info
- `Fleet.getGroupDevicesAsDeviceInstances()` - Get full Device objects
- `App.addModuleDataListener()` - Process multi-device stream data
- `App.requestModuleData()` - Request data for all devices

## Notes

- Works in both single-device and group contexts
- Group device detection is automatic (no code changes needed)
- Stream data already supports multiple devices per stream
- Full Device instances require API authentication

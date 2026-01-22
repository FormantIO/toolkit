# Building Group-Aware Custom Modules

This guide explains how to build custom modules that work in coherence group views (multi-device contexts).

## Quick Start

### For Module Developers

**Your module automatically receives group devices** - no code changes needed for basic support!

```typescript
import { Fleet } from "@formant/data-sdk";

// Check if in group context
const devices = Fleet.getGroupDevices();
if (devices && devices.length > 1) {
  // You're in a group context!
  console.log(`${devices.length} devices available`);
}
```

### For Host Containers

**Just return all devices** when the module requests them:

```javascript
case "request_devices": {
  // Return ALL devices in the group (not just one)
  const allDevices = getGroupDevices().map(d => ({
    id: d.id,
    name: d.name,
    tags: d.tags
  }));
  postMessage({ type: "overview_devices", data: allDevices });
}
```

## Examples

- **[module-tester](module-tester/)** - Complete example showing group device APIs
- **[group-module-host](group-module-host/)** - Host container implementation example

## Key Concepts

### Single-Device vs Group Context

- **Single-device context:** Module receives one device (traditional behavior)
- **Group context:** Module receives multiple devices (coherence group views)

### Automatic Detection

The SDK automatically detects group context when:
1. Module URL has `?module=name` parameter
2. Host sends multiple devices in `overview_devices` response
3. SDK stores them automatically - no code needed!

### Stream Data Structure

Stream data **already supports multiple devices**:

```typescript
interface Stream {
  data: StreamData[];  // Array - one entry per device
}

interface StreamData {
  deviceId: string;    // Which device this data is from
  name: string;        // Device name
  tags: { ... };       // Device tags
  points: DataPoint[]; // Data points for this device
}
```

## API Reference

### Fleet.getGroupDevices()

Returns device information (id, name, tags) for all devices in the group.

```typescript
const devices = Fleet.getGroupDevices();
// Returns: IDevice[] | undefined
// - Array of devices if in group context
// - undefined if single-device context
```

**No API calls needed** - uses data from `overview_devices` message.

### Fleet.getGroupDevicesAsDeviceInstances()

Returns full `Device` objects for all devices in the group.

```typescript
const devices = await Fleet.getGroupDevicesAsDeviceInstances();
// Returns: Device[] - full Device instances
// Requires: API authentication
```

**Requires authentication** - makes API calls to fetch full device details.

### useGroupDevices() Hook

React hook for group-aware modules.

```tsx
import { useGroupDevices } from "@formant/ui-sdk";

const devices = useGroupDevices();
// Returns: Device[] | undefined
```

## Common Patterns

### Pattern 1: Conditional Rendering

```tsx
const devices = useGroupDevices();

if (!devices) {
  return <SingleDeviceView />;
}

return <GroupView devices={devices} />;
```

### Pattern 2: Process All Devices

```typescript
App.addModuleDataListener((data) => {
  Object.values(data.streams).forEach(stream => {
    stream.data.forEach(deviceData => {
      // Process data for each device
      processDevice(deviceData.deviceId, deviceData.points);
    });
  });
});
```

### Pattern 3: Aggregate Metrics

```typescript
function calculateFleetMetrics(data: ModuleData) {
  const metrics = { total: 0, average: 0 };
  
  const stream = data.streams["battery.level"];
  if (stream) {
    metrics.total = stream.data.length;
    const sum = stream.data.reduce((acc, deviceData) => {
      const latest = deviceData.points[deviceData.points.length - 1];
      return acc + latest[1];
    }, 0);
    metrics.average = sum / metrics.total;
  }
  
  return metrics;
}
```

## Backward Compatibility

✅ **All existing modules continue to work:**
- Single-device modules work unchanged
- `Fleet.getCurrentDevice()` still works
- `useDevice()` hook still works
- No breaking changes

## Migration Guide

### Existing Modules

**No changes required!** Your modules will automatically receive group devices if the host sends them.

### Making Modules Group-Aware (Optional)

1. Add group detection:
   ```typescript
   const devices = Fleet.getGroupDevices();
   ```

2. Process multi-device streams:
   ```typescript
   stream.data.forEach(deviceData => { /* ... */ });
   ```

3. Update UI if needed:
   ```tsx
   const devices = useGroupDevices();
   if (devices) {
     // Show group-specific UI
   }
   ```

## Host Container Requirements

For coherence group views to support modules:

1. **URL Parameter:** Use `?group=groupId` instead of `?device=deviceId`
2. **Device List:** Return all group devices in `overview_devices` response
3. **Stream Data:** Include data for all devices in `module_data` streams

See [group-module-host](group-module-host/) for a complete example.

## Troubleshooting

**Q: Module only sees one device**
- Check that host returns all devices in `overview_devices`
- Verify `Fleet.getGroupDevices()` is called after devices are received

**Q: Stream data only has one device**
- Ensure `module_data` includes data for all devices in the `data` array
- Check that host aggregates data for all group devices

**Q: `Fleet.getGroupDevices()` returns undefined**
- Verify module URL has `?module=name` parameter
- Check that host responds to `request_devices` with multiple devices
- Check browser console for errors

## More Information

- [Module Tester Example](module-tester/) - Complete working example
- [Group Module Host](group-module-host/) - Host implementation example
- [Data SDK Documentation](https://formantio.github.io/toolkit/docs/data-sdk/)

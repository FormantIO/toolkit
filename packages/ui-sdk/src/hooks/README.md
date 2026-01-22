## Formant Hooks

Simplify the hard stuff, help performance, and developer tooling

### useDevice

It uses current device as default

```html
const device = useDevice()
```

| Parameter               | Type     | Description      |
| :---------------------- | :------- | :--------------- |
| `deviceId` **Optional** | `string` | Target device ID |

### useLatestTelemetry

It uses current device as default

```html
const device = useLatestTelemetry()
```

| Parameter                          | Type                 | Description                   |
| :--------------------------------- | :------------------- | :---------------------------- |
| `deviceIdOrDeviceIds` **Optional** | `string or string[]` | Id or ids of target device(s) |

### useGroupDevices

Returns all devices when in a group context (coherence group views). Returns `undefined` for single-device contexts.

**Works without authentication** - uses `Fleet.getGroupDevices()` which returns device info from the host.

```tsx
import { useGroupDevices } from "@formant/ui-sdk";

function MyModule() {
  const devices = useGroupDevices();
  
  if (!devices) {
    return <div>Single device context</div>;
  }
  
  return (
    <div>
      <h2>Group: {devices.length} devices</h2>
      {devices.map(device => (
        <DeviceCard key={device.id} device={device} />
      ))}
    </div>
  );
}
```

**Returns:** `IDevice[] | undefined`
- `IDevice[]` when in group context (contains `id`, `name`, `tags`)
- `undefined` when in single-device context or loading

**Note:** Returns `IDevice[]` (device info), not full `Device` instances. For full `Device` instances with API access, use `Fleet.getGroupDevicesAsDeviceInstances()` directly.

**Use Cases:**
- Building modules for coherence group views
- Displaying fleet-wide information
- Aggregating data across multiple devices
- Creating group-aware dashboards
- **When you don't need authentication or full Device API access**

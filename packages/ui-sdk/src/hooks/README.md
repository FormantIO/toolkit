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

## [1.0.0] - 2023-06-05

### Added
- Add time information to `request_date` message. (#93)

### Fixed
- Possible to removeListener too early when waiting for `request_date` to respond. (#93)

## [1.0.0-rc.3] - 2023-05-25 

`Device.startRealtimeConnection()` now accepts an optional `options` argument, to more fine-grained control over
the connection handshake behavior.

```ts
const device = Fleet.getCurrentDevice();
await device.startRealtimeConnection({
  sessionType: SessionType.Observe,
  // wait an entire minute for a connection to establish.
  deadlineMs: 60_000,
});
```

The default value for `deadlineMs` is `10000`, or 10 seconds. If `device.startRealtimeConnection()` is unsuccessful 
in establishing a connection within the deadline, it will throw an error.

```ts
try {
  await device.startRealtimeConnection({
    sessionType: SessionType.Observe,
  });
} catch (err) {
  console.error("Unable to estbalish connection!");
  return;
}
```

If you would like to wait _forever_ for the connection, you can use:

```ts
await device.startRealtimeConnection({
  sessionType: SessionType.Observe,
  deadlineMs: Infinity,
});
```

### Added
- Improvements to `Device.startRealtimeConnection()` implementation (#91)

### Changed
- Pool `RtcClients` by `sessionType` to reduce network overhead for multiple conections (#89)

## [1.0.0-rc.2] - 2023-05-18

### Added
- Added App.disableAnalyticsBottomBar() to hide date picker in analytics view (#84)

### Fixed
- Increase number of tries to create a connection (#88)
- Re-enforce type constraints on aggregateByDateFunctions (#85)

## [1.0.0-rc.1] - 2023-05-17

### Added
- Adding WAN connectivity helpers (#83)
  - `App.isOnline` returns the last-known connectivity status to formant apis.
  - `App.checkConnection(ms?: number)` returns a promise for the _current_ connectivity status to formant apis.
  - `App.waitForConnection(ms?: number)` returns a promise that will resolve when `App.isOnline` is `true`. 

### Fixed
- Prefer cached remoteDevicePeerId when set.  (#81)
- Adding logging to RealtimeVideo and RealtimeDataStream methods (#82)
- Try to reconnect when rtc.connect returns undefined (#78)
- Update function to use remotePeerId (#80)

## 1.0.0-rc.0 - 2023-05-12

### Added
- Improve typescript definitions `ConfigurationDocument` (#77)
  - Added `adapters`
  - Added `tags` 

### Fixed
- correct support for `getLatestTelemetry(oneUuid)` (#76)

[1.0.0]: https://github.com/FormantIO/toolkit/compare/release/data-sdk/1.0.0-rc.3...release/data-sdk/1.0.0~
[1.0.0-rc.3]: https://github.com/FormantIO/toolkit/compare/release/data-sdk/1.0.0-rc.2...release/data-sdk/1.0.0-rc.3~
[1.0.0-rc.2]: https://github.com/FormantIO/toolkit/compare/release/data-sdk/1.0.0-rc.1...release/data-sdk/1.0.0-rc.2~
[1.0.0-rc.1]: https://github.com/FormantIO/toolkit/compare/release/data-sdk/1.0.0-rc.0...release/data-sdk/1.0.0-rc.1~

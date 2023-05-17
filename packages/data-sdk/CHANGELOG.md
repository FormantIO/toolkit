## [1.0.0-rc.1] - 2023-05-17

### Added

- Adding WAN connectivity helpers (#83)
  - `App.isOnline` returns the last-known connectivity status to formant apis.
  - `App.checkConnection(ms?: number)` returns a promise for the _current_ connectivity status to formant apis.
  - `App.waitForConnection(ms?: number)` returns a promise that will resolve when `App.isOnline` is `true`. 

### Fixed
- Prefer cached remoteDevicePeerId when set.  (#81)
- Adding logging to RealtimeVideo and RealtimeDataStream methods (#82)
- Try to reconect when rtc.connect returns undefined (#78)
- Update function to use remotePeerId (#80)

## 1.0.0-rc.0 - 2023-05-12

### Added
- Improve typescript definitions `ConfigurationDocument` (#77)
  - Added `adapters`
  - Added `tags` 

### Fixed
- correct support for `getLatestTelemetry(oneUuid)` (#76)


[1.0.0-rc.1]: https://github.com/FormantIO/toolkit/compare/release/data-sdk/1.0.0-rc.0..1.0.0-rc.1
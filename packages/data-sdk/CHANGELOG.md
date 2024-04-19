## [1.36.0] - 2024-04-19

- adding agent getTelemetry

## [1.34.0] - 2024-04-03

- New auth mechanism

## [1.33.1] - 2024-03-19

- Remove JSON subscription as an option for pointclouds in universe-connector

## [1.33.0] - 2024-03-11

- Caching universe-connector network calls
- Using workers to fetch external assets

## [1.32.0] - 2024-03-06

- Adding support for multiple pointclouds subscriptions

## [1.31.0] - 2024-02-27

- adding physical events

## [1.30.0] - 2024-02-27

- adding more events

## [1.29.0] - 2024-02-27

- fixing interface of IAnnotation

## [1.28.0] - 2024-02-27

- fixing interface of ITriggeredEvent

## [1.26.0] - 2024-01-24

- targeting es2019

## [1.26.0] - 2024-01-24

- Merging in data connector

## [1.25.0] - 2024-01-24

### Added

- PeerDevice function to get commands

## [1.24.0] - 2024-01-24

### Added

- Exposing function on interface

## [1.23.0] - 2024-01-22

### Added

- Authentication method for peer

## [1.22.0] - 2024-01-17

### Added

- bundling deps to prevent typing issues

## [1.21.0] - 2024-01-17

### Added

- exposing realtime message creator

## [1.20.0] - 2024-01-17

### Added

- exposed new interface IMarker3DArray

## [1.19.0] - 2024-01-17

### Added

- exposed new interface IMarker3D

## [1.18.0] - 2023-12-29

### Added

- improvements to storeCache

## [1.17.0] - 2023-10-09

### Added

- New functions for looking up SSO login information and logging in with SSO tokens

## [1.16.0] - 2023-09-21

### Fixed

- Device entity should have correct organization property set. (#138)

## [1.15.0] - 2023-09-21

### Added

- Add support for agent version querying (#136)

## [1.14.1] - 2023-09-06

### Added

- Add parameter to query desire configuration vs reported configuration (#132)

## [1.14.0] - 2023-09-06

### Fixed

- Fixed delete account error handling (#131)

## [1.13.2] - 2023-08-27

### Fixed

- Fix IAuthenticationStore types for login() method.

## [1.13.1] - 2023-08-25

### Fixed

- Only resolve `Authentication.respondToNewPasswordRequiredChallenge` on `response.ok`.

## [1.13.0] - 2023-08-25

### Added

- Enhance Authentication.login() method for _advanced_ login flows.

## [1.12.0] - 2023-08-07

### Upgrade

- Added support for `HEADLESS` session types in RtcClientPool.
- Upgrade `@formant/realtime-sdk` to 1.2.0.

## [1.11.1] - 2023-08-07

### Added

- Added right files

## [1.11.0] - 2023-08-07

### Added

- Add getAllEventTriggerGroup, getEventTriggerGroup, and patchEventTriggerGroup

## [1.10.0] - 2023-07-25

### Upgrade

- Upgrade grpc-web to 1.4.2

## [1.9.0] - 2023-07-24

### Upgrade

- Upgrade data-sdk realtime-sdk to 1.0.0 (#117)

## [1.8.0] - 2023-07-17

### Added

- query to KeyValue (#116)

## [1.7.0] - 2023-07-14

### Added

- disableDevice, and timeout (#115)

## [1.6.0] - 2023-07-09

### Added

- createFleet

## [1.5.4] - 2023-07-03

### Added

- createDevice, patchDevice, getDeviceData, and queryDevicesData (#113)

## [1.5.3] - 2023-06-30

### Fixed

- DataChannel check ready fix (#112)

## [1.5.2] - 2023-06-29

### Fixed

- Resolve leaky reference to `NodeJS.Timeout`.

## [1.5.1] - 2023-06-29

### Fixed

- Resolve dependency cycles. (#111)

## [1.5.0] - 2023-06-28

### Changed

- Rework data-sdk bundle/package (#92)
  - packages `cjs`/`es` for package.json/bundlers _exclude_ all of their dependencies.
  - pre-bundled assets `umd`/`es6` bundles all required dependencies for stand-alone use.

### Update Notes

The dependency tree has changed slightly to support better performance/security updates moving forward. As such the
`@formant/realtime-sdk` dependency has been changed from a `"dependency"` to a `"peerDependency"`. You will need to add
this dependency as well:

```sh
$ npm install --save @formant/realtime-sdk@latest

# or with

$ yarn add @formant/realtime-sdk@latest
```

## [1.4.0] - 2023-06-28

### Added

- Support for react native env (#108)
- Implement `sendCommand` for `PeerDevice` (#106)

### Fixed

- Ensure `Realtime*` types get exported at the top-level from `BaseDevice` (#110)
- Remove debug logs (#109)

## [1.3.1] - 2023-06-27

### Fixed

- Ensure `Realtime*` types get exported at the top-level from `BaseDevice` (#110)

## [1.3.0] - 2023-06-22

### Added

- Improving API support for `/users`, `/accounts`, and `/fleets` (#107)
- Support for offline direct RTC (#99)

## [1.2.0] - 2023-06-15

### Added

- Added `tags` support to `Device` and `Command` entity. (#104)

## [1.1.0] - 2023-06-15

_Released…_

## [1.1.0-rc.0] - 2023-06-15

### Added

- Add support for custom API paths with `formant_url` query parameter. (#97) (MC-90)

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

- Prefer cached remoteDevicePeerId when set. (#81)
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

[1.18.0]: https://github.com/FormantIO/toolkit/compare/release/data-sdk/1.17.0...release/data-sdk/1.18.0~
[1.17.0]: https://github.com/FormantIO/toolkit/compare/release/data-sdk/1.16.0...release/data-sdk/1.17.0~
[1.16.0]: https://github.com/FormantIO/toolkit/compare/release/data-sdk/1.15.0...release/data-sdk/1.16.0~
[1.15.0]: https://github.com/FormantIO/toolkit/compare/release/data-sdk/1.14.1...release/data-sdk/1.15.0~
[1.14.1]: https://github.com/FormantIO/toolkit/compare/release/data-sdk/1.14.0...release/data-sdk/1.14.1~
[1.14.0]: https://github.com/FormantIO/toolkit/compare/release/data-sdk/1.13.2...release/data-sdk/1.14.0~
[1.13.2]: https://github.com/FormantIO/toolkit/compare/release/data-sdk/1.13.1...release/data-sdk/1.13.2~
[1.13.1]: https://github.com/FormantIO/toolkit/compare/release/data-sdk/1.13.0...release/data-sdk/1.13.1~
[1.13.0]: https://github.com/FormantIO/toolkit/compare/release/data-sdk/1.12.0...release/data-sdk/1.13.0~
[1.12.0]: https://github.com/FormantIO/toolkit/compare/release/data-sdk/1.11.1...release/data-sdk/1.12.0~
[1.11.1]: https://github.com/FormantIO/toolkit/compare/release/data-sdk/1.11.0...release/data-sdk/1.11.1~
[1.11.0]: https://github.com/FormantIO/toolkit/compare/release/data-sdk/1.10.0...release/data-sdk/1.11.0~
[1.10.0]: https://github.com/FormantIO/toolkit/compare/release/data-sdk/1.9.0...release/data-sdk/1.10.0~
[1.9.0]: https://github.com/FormantIO/toolkit/compare/release/data-sdk/1.8.0...release/data-sdk/1.9.0~
[1.8.0]: https://github.com/FormantIO/toolkit/compare/release/data-sdk/1.7.0...release/data-sdk/1.8.0~
[1.7.0]: https://github.com/FormantIO/toolkit/compare/release/data-sdk/1.6.0...release/data-sdk/1.7.0~
[1.6.0]: https://github.com/FormantIO/toolkit/compare/release/data-sdk/1.5.4...release/data-sdk/1.6.0~
[1.5.4]: https://github.com/FormantIO/toolkit/compare/release/data-sdk/1.5.3...release/data-sdk/1.5.4~
[1.5.3]: https://github.com/FormantIO/toolkit/compare/release/data-sdk/1.5.2...release/data-sdk/1.5.3~
[1.5.2]: https://github.com/FormantIO/toolkit/compare/release/data-sdk/1.5.1...release/data-sdk/1.5.2~
[1.5.1]: https://github.com/FormantIO/toolkit/compare/release/data-sdk/1.5.0...release/data-sdk/1.5.1~
[1.5.0]: https://github.com/FormantIO/toolkit/compare/release/data-sdk/1.4.0...release/data-sdk/1.5.0~
[1.4.0]: https://github.com/FormantIO/toolkit/compare/release/data-sdk/1.3.0...release/data-sdk/1.4.0~
[1.3.1]: https://github.com/FormantIO/toolkit/compare/release/data-sdk/1.3.0...release/data-sdk/1.3.1~
[1.3.0]: https://github.com/FormantIO/toolkit/compare/release/data-sdk/1.2.0...release/data-sdk/1.3.0~
[1.2.0]: https://github.com/FormantIO/toolkit/compare/release/data-sdk/1.1.0...release/data-sdk/1.2.0~
[1.1.0]: https://github.com/FormantIO/toolkit/compare/release/data-sdk/1.1.0-rc.0...release/data-sdk/1.1.0~
[1.1.0-rc.0]: https://github.com/FormantIO/toolkit/compare/release/data-sdk/1.0.0...release/data-sdk/1.1.0-rc.0~
[1.0.0]: https://github.com/FormantIO/toolkit/compare/release/data-sdk/1.0.0-rc.3...release/data-sdk/1.0.0~
[1.0.0-rc.3]: https://github.com/FormantIO/toolkit/compare/release/data-sdk/1.0.0-rc.2...release/data-sdk/1.0.0-rc.3~
[1.0.0-rc.2]: https://github.com/FormantIO/toolkit/compare/release/data-sdk/1.0.0-rc.1...release/data-sdk/1.0.0-rc.2~
[1.0.0-rc.1]: https://github.com/FormantIO/toolkit/compare/release/data-sdk/1.0.0-rc.0...release/data-sdk/1.0.0-rc.1~

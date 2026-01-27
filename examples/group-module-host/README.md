## Group-aware embedded module host (Path A template)

This example is **the host/container side** of the embedded module protocol used by `@formant/data-sdk` modules.

It demonstrates (in plain browser JS):
- Creating an iframe for a module URL and injecting context via query params (`module`, `auth`, optional `group`).
- Listening for `postMessage` requests from the module (`request_devices`, `request_module_data`, etc.).
- Responding with `overview_devices` and `module_data` in a **multi-device (group)** shape.

### Why this exists

In Formant, the product UI is normally the "host/container". If you want coherence group views to support embedded modules, the host must be group-aware:
- `request_devices` should return **only group devices**
- `module_data` should include stream payloads for **all devices in the group**

This repo (`toolkit`) contains the **module-side** SDK helpers, but not the product host implementation. This example is meant to be copy/pasted into your real host.

### Quick Test (5 minutes)

**Prerequisites:** Node.js, Python 3

1. **Start module-tester** (Terminal 1):
   ```bash
   cd examples/module-tester
   npm install
   npm run dev
   ```
   Note the URL (usually `http://localhost:4000`)

2. **Start host** (Terminal 2):
   ```bash
   cd examples/group-module-host
   python3 -m http.server 8001
   ```

3. **Open browser:**
   - Go to `http://localhost:8001`
   - Enter Module URL: `http://localhost:4000/?module=module-tester&auth=TEST_TOKEN`
   - Click **"Load iframe"**

4. **Test:**
   - Click **"Test Group Devices"** in the module
   - Should show 3 devices detected
   - Check browser console for multi-device stream logs


### Notes

- This example intentionally stubs `module_data` (demo streams) so it can run without hitting real Formant APIs.
- In your real coherence host, replace the demo data generation with your actual aggregation logic.

### SDK Support for Group Views

The `@formant/data-sdk` now automatically initializes group devices when `overview_devices` is received:

- `Fleet.setGroupDevices(devices)` - Called automatically by SDK
- `Fleet.getGroupDevices()` - Returns IDevice[] from group context
- `Fleet.getGroupDevicesAsDeviceInstances()` - Returns full Device[] instances
- `useGroupDevices()` - React hook for group-aware modules (from `@formant/ui-sdk`)

Modules can now:
1. Use `App.addOverviewDeviceListener()` to detect group context
2. Use `Fleet.getGroupDevices()` to access all group devices
3. Process `module_data` streams which already contain per-device data
4. Use `useGroupDevices()` hook in React modules for group awareness

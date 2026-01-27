const moduleUrlEl = document.querySelector("#moduleUrl");
const groupIdEl = document.querySelector("#groupId");
const devicesJsonEl = document.querySelector("#devicesJson");
const authTokenEl = document.querySelector("#authToken");
const loadBtn = document.querySelector("#load");
const sendDemoDataBtn = document.querySelector("#sendDemoData");
const iframe = document.querySelector("#moduleFrame");
const logsEl = document.querySelector("#logs");

function log(...args) {
  const line = args
    .map((a) => (typeof a === "string" ? a : JSON.stringify(a, null, 2)))
    .join(" ");
  logsEl.textContent = `${new Date().toISOString()} ${line}\n${logsEl.textContent}`;
}

function safeParseJson(text, fallback) {
  try {
    return JSON.parse(text);
  } catch {
    return fallback;
  }
}

function getGroupDevices() {
  const fallback = [
    { id: "device-1", name: "Device 1", tags: { fleet: "demo", unit: "1" } },
    { id: "device-2", name: "Device 2", tags: { fleet: "demo", unit: "2" } },
    { id: "device-3", name: "Device 3", tags: { fleet: "demo", unit: "3" } },
  ];

  const parsed = safeParseJson(devicesJsonEl.value, fallback);
  return Array.isArray(parsed) ? parsed : fallback;
}

function postToModule(message) {
  if (!iframe.contentWindow) return;
  iframe.contentWindow.postMessage(message, "*");
}

let moduleDataRangeMs = { before: 60_000, after: 0 };

function buildDemoModuleData() {
  const devices = getGroupDevices();
  const now = Date.now();
  const start = now - moduleDataRangeMs.before;
  const end = now + moduleDataRangeMs.after;

  const mkPoint = () => [now, Math.round(1000 * Math.random()) / 10];

  const streams = {
    "demo.temperature": {
      type: "numeric",
      loading: false,
      tooMuchData: false,
      data: devices.map((d) => ({
        deviceId: d.id,
        agentId: "agent-" + d.id,
        name: d.name,
        tags: d.tags || {},
        type: "numeric",
        points: [mkPoint()],
      })),
    },
    "demo.status": {
      type: "string",
      loading: false,
      tooMuchData: false,
      data: devices.map((d) => ({
        deviceId: d.id,
        agentId: "agent-" + d.id,
        name: d.name,
        tags: d.tags || {},
        type: "string",
        points: [[now, `ok:${d.id}`]],
      })),
    },
  };

  return {
    type: "module_data",
    time: now,
    queryRange: { start, end },
    streams,
  };
}

function loadIframe() {
  const raw = moduleUrlEl.value.trim();
  if (!raw) return;

  const groupId = groupIdEl.value.trim();
  const url = new URL(raw, window.location.href);

  // If the user set a group, ensure `group=` exists for module-side helpers like getCurrentGroup()
  if (groupId) {
    url.searchParams.set("group", groupId);
  }

  iframe.src = url.toString();
  log("iframe.src =", iframe.src);
}

// Basic host-side implementation of the embedded-module protocol.
window.addEventListener("message", (event) => {
  // Only accept messages from the current iframe
  if (!iframe.contentWindow || event.source !== iframe.contentWindow) return;

  const msg = event.data;
  if (!msg || typeof msg !== "object" || typeof msg.type !== "string") return;

  log("from module:", msg.type);

  switch (msg.type) {
    case "request_devices": {
      // IMPORTANT: Return ALL devices in the group, not just one
      // The SDK will automatically initialize Fleet.groupDevices from this
      const groupDevices = getGroupDevices();
      postToModule({ type: "overview_devices", data: groupDevices });
      log(`sent ${groupDevices.length} devices to module`);
      return;
    }

    case "request_module_data": {
      postToModule(buildDemoModuleData());
      return;
    }

    case "set_module_data_time_range": {
      moduleDataRangeMs = {
        before: Number(msg.before) || 0,
        after: Number(msg.after) || 0,
      };
      postToModule(buildDemoModuleData());
      return;
    }

    // Connectivity ping from App.checkConnection()
    case "formant_online": {
      // If `online` is present, it’s already a response; ignore it.
      if ("online" in msg) return;
      postToModule({ type: "formant_online", online: true });
      return;
    }

    // Module may request a token refresh; real host should mint a fresh token.
    case "refresh_auth_token": {
      // Echoing back a placeholder here; your real host should generate a new token.
      postToModule({ type: "auth_token", token: authTokenEl.value || "" });
      return;
    }

    // Demonstrate channel echo for testing sendChannelData/addChannelDataListener
    case "send_channel_data": {
      postToModule({
        type: "channel_data",
        channel: msg.channel,
        source: msg.source,
        data: msg.data,
      });
      return;
    }

    // These are host-handled side effects in the real app; we just log them here.
    case "show_message":
    case "go_to_time":
    case "go_to_device":
    case "setup_module_menus":
    case "prompt":
    case "request_date":
    case "hide_analytics_date_picker":
      log("host would handle:", msg);
      return;

    default:
      log("unhandled message:", msg);
  }
});

loadBtn.addEventListener("click", loadIframe);

sendDemoDataBtn.addEventListener("click", () => {
  postToModule(buildDemoModuleData());
  log("to module: module_data (manual push)");
});

// Prefill device JSON for quick experimentation
devicesJsonEl.value = JSON.stringify(getGroupDevices(), null, 2);

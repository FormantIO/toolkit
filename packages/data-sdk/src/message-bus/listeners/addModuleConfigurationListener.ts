import {
  EmbeddedAppMessage,
  ModuleConfigurationMessage,
} from "./EmbeddedAppMessage";

export function addModuleConfigurationListener(
  handler: (event: ModuleConfigurationMessage) => void
) {
  const listener = (event: MessageEvent<EmbeddedAppMessage>) => {
    const msg = event.data;
    if (msg.type === "module_configuration") {
      handler(msg as ModuleConfigurationMessage);
    }
  };

  window.addEventListener("message", listener);
  return () => window.removeEventListener("message", listener);
}

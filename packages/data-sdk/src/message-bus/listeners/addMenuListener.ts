import { EmbeddedAppMessage } from "./EmbeddedAppMessage";

export function addMenuListener(handler: (label: string) => void) {
  const listener = (event: MessageEvent<EmbeddedAppMessage>) => {
    const msg = event.data;
    if (msg.type === "module_menu_item_clicked") {
      handler(msg.menu);
    }
  };

  window.addEventListener("message", listener);
  return () => window.removeEventListener("message", listener);
}

import { JsonSchema } from "../../model/JsonSchema";
import { sendAppMessage } from "../senders/sendAppMessage";
import { EmbeddedAppMessage } from "../listeners/EmbeddedAppMessage";

export async function prompt(
  schema: JsonSchema,
  options?: { okText?: string; cancelText?: string }
): Promise<any> {
  return new Promise((resolve) => {
    const promptId = Math.random().toString();
    sendAppMessage({
      type: "prompt",
      promptId,
      schema,
      okText: options?.okText,
      cancelText: options?.cancelText,
    });
    const handler = (event: MessageEvent<EmbeddedAppMessage>) => {
      const msg = event.data;
      if (msg.type === "prompt_response" && msg.promptId === promptId) {
        resolve(msg.data);
      }
      window.removeEventListener("message", handler);
    };
    window.addEventListener("message", handler);
  });
}

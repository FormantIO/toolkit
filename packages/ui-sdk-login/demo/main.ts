import "../src/main";
import "../../../css/formant.css";

const j = document.querySelector("formant-login") as HTMLElement;
const l = document.querySelector("#log") as HTMLElement;
j.addEventListener("authenticated", () => {
  l.innerHTML = "Authenticated";
});

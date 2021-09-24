import "../src/main";
import "../../../css/formant.css";

const l = document.querySelector("formant-login") as HTMLElement;

l.addEventListener("login", (e) => {
  console.log(e);
  l.setAttribute("message", "clicked");
});

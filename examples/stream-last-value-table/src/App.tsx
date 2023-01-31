import { useEffect } from "react";
import "./App.css";
import { Main } from "./components/Main";

function App() {
  useEffect(() => {
    localStorage.setItem("show", "false");
  }, []);
  return (
    <div className="App">
      <Main />
    </div>
  );
}

export default App;

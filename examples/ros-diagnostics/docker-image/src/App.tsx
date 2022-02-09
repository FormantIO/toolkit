import "./App.css";
import { DockerImage } from "./components/DockerImage";

function App() {
  return (
    <div className="App">
      <DockerImage command="Fake update docker" />
    </div>
  );
}

export default App;

import { NumericAggregateBar } from "./NumericAggregateBar";
import "./App.css";
import { LoadingIndicator } from "./LoadingIndicator";
import { useFormant } from "@formant/ui-sdk";

function App() {
  const config = useFormant();

  return (
    <div className="App">
      {!config.configuration ? <LoadingIndicator /> : <NumericAggregateBar />}
    </div>
  );
}

export default App;

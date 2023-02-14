import { NumericAggregateBar } from "./NumericAggregateBar";
import "./App.css";
import { useFormant, LoadingIndicator } from "@formant/ui-sdk";

function App() {
  const config = useFormant();

  return (
    <div className="App">
      {!config.configuration ? <LoadingIndicator /> : <NumericAggregateBar />}
    </div>
  );
}

export default App;

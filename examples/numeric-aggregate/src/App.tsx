import { NumericAggregateBar } from "./NumericAggregateBar";
import "./App.css";
import { useFormant, LoadingIndicator, useScrubberTime } from "@formant/ui-sdk";

function App() {
  const config = useFormant();
  const time = useScrubberTime();

  return (
    <div className="App">
      {!config.configuration ? (
        <LoadingIndicator />
      ) : (
        <NumericAggregateBar time={time!} />
      )}
    </div>
  );
}

export default App;

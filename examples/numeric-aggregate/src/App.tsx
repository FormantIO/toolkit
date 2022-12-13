import { NumericAggregateBar } from "./NumericAggregateBar";
import "./App.css";
import { LoadingIndicator } from "./LoadingIndicator";
import { useConfiguration } from "./hooks/useConfiguration";

function App() {
  const [config, loading] = useConfiguration();

  return (
    <div className="App">
      {loading ? (
        <LoadingIndicator />
      ) : (
        <NumericAggregateBar
          aggregateType={config!.aggregateType}
          streamName={config!.streamName}
          numericSetKey={config!.numericSetKey}
          aggregateBy={config!.aggregateBy}
          numAggregates={config!.numAggregates}
        />
      )}
    </div>
  );
}

export default App;

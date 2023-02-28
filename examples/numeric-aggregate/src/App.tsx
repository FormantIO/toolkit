import { NumericAggregateBar } from "./NumericAggregateBar";
import "./App.css";
import { useFormant, LoadingIndicator, useScrubberTime } from "@formant/ui-sdk";
import { dummyData } from "./utils/dummyData";
import { getTypedConfiguration } from "./utils/getTypedConfiguration";
import { ConfigurationTypes } from "./types";

function App() {
  const context = useFormant();
  const configuration = getTypedConfiguration(
    context.configuration as ConfigurationTypes
  );
  const time = useScrubberTime();

  const handleConfiguration = () =>
    !!configuration && Object.keys(configuration).length > 3
      ? configuration
      : dummyData;

  return (
    <div className="App">
      <NumericAggregateBar config={handleConfiguration()} time={time!} />
    </div>
  );
}

export default App;

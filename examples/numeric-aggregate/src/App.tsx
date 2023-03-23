import { NumericAggregateBar } from "./NumericAggregateBar";
import "./App.css";
import { LoadingIndicator, useFormant, useScrubberTime } from "@formant/ui-sdk";
import { dummyData } from "./utils/dummyData";
import { getTypedConfiguration } from "./utils/getTypedConfiguration";
import { ConfigurationTypes } from "./types";
import { useEffect, useMemo, useState } from "react";

function App() {
  const context = useFormant();
  const [loading, setLoading] = useState(true);
  const configuration = getTypedConfiguration(
    context.configuration as ConfigurationTypes
  );
  const time = useScrubberTime();

  useEffect(() => {
    setTimeout(() => {
      setLoading(false);
    }, 2000);
  }, []);

  const config = useMemo(() => {
    return !!configuration && Object.keys(configuration).length > 3
      ? configuration
      : dummyData;
  }, [configuration]);

  return (
    <div className="App">
      {loading ? (
        <LoadingIndicator />
      ) : (
        <NumericAggregateBar config={config} time={time!} />
      )}
    </div>
  );
}

export default App;

import { NumericAggregateBar } from "./NumericAggregateBar";
import "./App.css";
import { LoadingIndicator, useFormant, useScrubberTime } from "@formant/ui-sdk";
import { dummyData } from "./utils/dummyData";
import { getTypedConfiguration } from "./utils/getTypedConfiguration";
import { ConfigurationTypes } from "./types";
import { useCallback, useEffect, useMemo, useState } from "react";

function timeout(ms: number) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

function App() {
  const context = useFormant();
  const [loading, setLoading] = useState(true);
  const configuration = getTypedConfiguration(
    context.configuration as ConfigurationTypes
  );
  const time = useScrubberTime();
  const config = useMemo(() => {
    return !!configuration && Object.keys(configuration).length > 3
      ? configuration
      : dummyData;
  }, [configuration, loading]);

  return (
    <div className="App">
      {!configuration ? (
        <LoadingIndicator />
      ) : (
        <NumericAggregateBar config={configuration} time={time!} />
      )}
    </div>
  );
}

export default App;

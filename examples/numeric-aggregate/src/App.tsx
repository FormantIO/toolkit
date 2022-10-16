import { FormantProvider } from "@formant/ui-sdk";
import { NumericAggregateBar } from "./NumericAggregateBar";
function App() {
  return (
    <FormantProvider>
      <NumericAggregateBar
        aggregateType="average"
        streamName="$.host.cpu"
        numericSetKey="utilization"
        aggregateBy="day"
        numAggregates={10}
      />
    </FormantProvider>
  );
}

export default App;

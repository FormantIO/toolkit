import "./App.css";
import useModuleDataListener from "./hooks/useDataModuleListener";
import { Text, Numeric, NumericSet, NullComponent, Bitset } from "./components";

function App() {
  const streams = useModuleDataListener();

  return (
    <div className="App">
      {Object.entries(streams).map((stream) => {
        if (stream[1] === null) {
          return (
            <NullComponent
              key={stream[0]}
              name={stream[0]}
              value="No datapoint is being ingested"
            />
          );
        }
        const name = stream[0];
        const { value, type } = stream[1];
        if (type === "bitset") {
          return <Bitset key={name} name={name} value={value} />;
        }
        if (type === "text") {
          return <Text key={name} name={name} value={value} />;
        }
        if (type === "numeric") {
          return <Numeric key={name} name={name} value={value} />;
        }
        if (type === "numeric set") {
          return <NumericSet key={name} name={name} value={value} />;
        }

        return (
          <NullComponent
            key={name}
            name={name}
            value="Stream type not supported"
          />
        );
      })}
    </div>
  );
}

export default App;

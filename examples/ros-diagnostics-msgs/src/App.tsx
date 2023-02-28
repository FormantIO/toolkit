import "./App.css";
import { Table } from "./components/table";
import useModuleDataListener from "./hooks/useDataModuleListener";
import { useFetchDiagnostics } from "./hooks/useFetchDiagnostics";
import { LoadingIndicator, useFormant } from "@formant/ui-sdk";
import { dummyData } from "./util";
import { IConfiguration } from "./types/types";
import { useEffect, useMemo } from "react";

function App() {
  const context = useFormant();
  const config = context.configuration as IConfiguration;
  const diagnostics = useModuleDataListener();
  const messages = useFetchDiagnostics(diagnostics);
  const loading = useMemo(() => {
    if (!config) return true;

    return false;
  }, [config]);

  const handleConfiguration =
    config.stream?.length > 1 ? messages : [dummyData];

  return (
    <div className="App">
      {loading ? (
        <div className="container">
          <LoadingIndicator />
        </div>
      ) : (
        <Table messages={handleConfiguration} config={config} />
      )}
    </div>
  );
}

export default App;

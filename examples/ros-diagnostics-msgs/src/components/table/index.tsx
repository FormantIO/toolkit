import styles from "./index.module.scss";
import { StatusHeader } from "../StatusHeader";
import { useCallback, useState, useMemo } from "react";
import { useFetchDiagnostics } from "../../hooks/useFetchDiagnostics";
import useModuleDataListener from "../../hooks/useDataModuleListener";
import { StatusFilter, KeyValue, SeverityLevel } from "../../types/types";
import { DiagnosticsTable } from "../DiagnosticsTable";
import { MessageTable } from "../MessageTable";
import { LoadingIndicator } from "../LoadingIndicator";
import { Typography } from "@formant/ui-sdk";

export const Table = () => {
  const diagnostics = useModuleDataListener();
  const messages = useFetchDiagnostics(diagnostics);

  const [active, setActive] = useState<string | null>(null);
  const [filters, setFilters] = useState<StatusFilter>({
    ok: false,
    warning: false,
    critical: false,
    stale: false,
  });

  const currentDiagnostics = useMemo(() => {
    if (!!!messages) return [];
    const activeFilters: (number | string)[] = [];

    Object.values(filters).forEach((_filter, idx) => {
      if (_filter) {
        activeFilters.push(idx);
        activeFilters.push(`b'\\x0${idx}'`);
        //Fix to include python byte value for severity level in filter
      }
    });

    if (activeFilters.length === 0) return messages;
    return messages.filter((_) => activeFilters.includes(_.status[0].level));
  }, [messages, filters]);

  const activeValues: KeyValue[] | null = useMemo(() => {
    if (!!!messages) return null;
    if (active === null || currentDiagnostics.length === 0) return [];
    const _activeMessage = messages.filter((_) => _.status[0].name === active);
    if (_activeMessage.length === 0) return [];
    return _activeMessage[0].status[0].values;
  }, [active, currentDiagnostics]);

  const handleFilter = (filter: SeverityLevel) => {
    let currentFilters = filters;
    currentFilters[filter] = currentFilters[filter] ? false : true;
    setFilters({ ...currentFilters });
  };

  const handleSetActive = useCallback((_: string) => {
    const table = document.getElementById("diagnostics-table")!;
    table.classList.remove("slide-out-right");
    table.classList.add("slide-out-left");
    setActive(_);
  }, []);

  return !!diagnostics && !!messages ? (
    <>
      <StatusHeader filters={filters} handleFilter={handleFilter} />
      <div className={styles.table}>
        <MessageTable
          handleSetActive={handleSetActive}
          messages={currentDiagnostics}
          active={active}
        />
        <DiagnosticsTable active={active} diagnosticsDetails={activeValues} />
        {messages.length === 0 && <LoadingIndicator />}
      </div>
    </>
  ) : (
    <div className={styles.offline}>
      <Typography>No current data</Typography>
    </div>
  );
};

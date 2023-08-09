import styles from "./index.module.scss";
import { StatusHeader } from "../StatusHeader";
import { useCallback, useState, useMemo, FC, useEffect } from "react";
import {
  StatusFilter,
  KeyValue,
  SeverityLevel,
  DiagnosticStatusMessage,
  IConfiguration,
} from "../../types/types";
import { DiagnosticsTable } from "../DiagnosticsTable";
import { MessageTable } from "../MessageTable";
import { LoadingIndicator } from "../LoadingIndicator";
import { Typography } from "@formant/ui-sdk";
import styled from "@emotion/styled";

interface ITableProps {
  messages: DiagnosticStatusMessage | null;
}
export const Table: FC<ITableProps> = ({ messages }) => {
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

    if (activeFilters.length === 0) return messages.status;
    return messages.status.filter((_) => activeFilters.includes(_.level));
  }, [messages, filters]);

  const activeValues: KeyValue[] | null = useMemo(() => {
    if (!!!messages) return null;
    if (active === null || currentDiagnostics.length === 0) return null;
    const activeMsg = messages.status.find((_) => _.name === active);
    if (!activeMsg) return null;
    return activeMsg.values;
  }, [active, currentDiagnostics]);

  const handleFilter = (filter: SeverityLevel) => {
    let currentFilters = filters;
    currentFilters[filter] = currentFilters[filter] ? false : true;
    setFilters({ ...currentFilters });
  };

  const handleSetActive = useCallback((_: string) => {
    const table = document.getElementById("diagnostics-table")!;
    table.style.visibility = "visible";
    table.classList.remove("slide-out-right");
    table.classList.add("slide-out-left");
    setActive(_);
  }, []);

  const filteredMessages = useMemo(() => {
    if (!messages) return [];

    const filter: number[] = [];
    Object.values(filters).forEach((_, idx) => {
      if (_) filter.push(idx);
    });

    if (filter.length === 0 || filter.length === 4) return messages.status;
    return messages.status.filter((_) => filter.includes(_.level));
  }, [messages, filters]);

  if (messages === null || messages === undefined) {
    return (
      <div className={styles.offline}>
        <Typography>No current data</Typography>
      </div>
    );
  }

  return (
    <Main>
      <StatusHeader
        messages={messages}
        filters={filters}
        handleFilter={handleFilter}
      />
      <Container>
        <MessageTable
          handleSetActive={handleSetActive}
          messages={filteredMessages}
          active={active}
        />
        <DiagnosticsTable active={active} diagnosticsDetails={activeValues} />
        {messages.status.length === 0 && <LoadingIndicator />}
      </Container>
    </Main>
  );
};

const Main = styled.div`
  height: 100vh;
`;

const Container = styled.div`
  display: flex;
  max-height: calc(100vh - 60px);
`;

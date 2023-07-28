import { useState, useEffect } from "react";
import { DiagnosticStatusMessage, Streams } from "../types/types";

const fetchDiagnostics = async (
  diagnostics: Streams
): Promise<DiagnosticStatusMessage | null> => {
  const isInvalid = Object.values(diagnostics).includes(null);
  if (isInvalid) return null;

  const promises = Object.values(diagnostics).map((_: any) => fetch(_));

  const results = await Promise.all(promises);
  const parsedResults = results.map((_) => _.json());
  const jsonReponse = await Promise.all(parsedResults);
  return jsonReponse[0];
};

export const useFetchDiagnostics = (_diagnostics: Streams | null) => {
  const [diagnosticMessages, setDiagnosticMessages] =
    useState<DiagnosticStatusMessage | null>(null);

  useEffect(() => {
    if (_diagnostics !== null)
      fetchDiagnostics(_diagnostics).then((_) => setDiagnosticMessages(_));
  }, [_diagnostics]);
  return diagnosticMessages;
};

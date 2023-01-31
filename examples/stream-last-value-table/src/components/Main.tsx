import { useConfiguration } from "../hooks/useConfiguration";
import { Table } from "./Table";

export const Main = () => {
  const currentConfiguration = useConfiguration();

  return <Table currentConfiguration={currentConfiguration} />;
};

import { Icon, Box, Typography } from "@formant/ui-sdk";
import { FC, useMemo } from "react";
import { StatusFilter, SeverityLevel } from "../types/types";
import styled from "@emotion/styled";
import { DiagnosticStatusMessage } from "../types/types";

interface IStatusHeaderProps {
  filters: StatusFilter;
  handleFilter: (_: SeverityLevel) => void;
  messages: DiagnosticStatusMessage | null;
}
export const StatusHeader: FC<IStatusHeaderProps> = ({
  filters,
  handleFilter,
  messages,
}) => {
  const messagesCount = useMemo(() => {
    if (messages === null)
      return {
        ok: 0,
        warning: 0,
        stale: 0,
        critical: 0,
      };

    return {
      ok: messages.status.filter((_) => _.level === 0).length,
      warning: messages.status.filter((_) => _.level === 1).length,
      stale: messages.status.filter((_) => _.level === 3).length,
      critical: messages.status.filter((_) => _.level === 2).length,
    };
  }, [messages]);

  return (
    <Container>
      <FilterButton
        active={filters.ok}
        onClick={() => {
          handleFilter("ok");
        }}
      >
        <Icon name="check" />
        <Typography
          sx={{ color: "white" }}
        >{`OK ${messagesCount.ok}`}</Typography>
      </FilterButton>
      <FilterButton
        active={filters.warning}
        onClick={() => handleFilter("warning")}
      >
        <Icon name="warning" />
        <Typography sx={{ color: "white", marginLeft: 0.5 }}>
          {`Warning ${messagesCount.warning}`}
        </Typography>
      </FilterButton>
      <FilterButton
        onClick={() => handleFilter("critical")}
        active={filters.critical}
      >
        <Icon name="critical" />
        <Typography
          sx={{ color: "white", marginLeft: 0.5 }}
        >{`Error ${messagesCount.critical}`}</Typography>
      </FilterButton>
      <FilterButton
        onClick={() => handleFilter("stale")}
        active={filters.stale}
      >
        {/* <Icon name="check" /> */}
        <Typography
          sx={{ color: "white" }}
        >{`Stale ${messagesCount.stale}`}</Typography>
      </FilterButton>
    </Container>
  );
};

const Container = styled.div`
  display: flex;
  width: 100%;
  height: 60px;
  align-items: center;
  padding: 0 20px 0;
  gap: 20px;
`;

interface IFilterButtonProps {
  active: boolean;
}

const FilterButton = styled.button<IFilterButtonProps>`
  all: unset;
  min-width: 142px;
  align-items: center;
  justify-content: center;
  padding: 4px 8px;
  border-radius: 25px;
  background-color: ${(props) => (props.active ? "#677194" : "transparent")};
  display: flex;
  border: 1px solid #677194;
  &:hover {
    transition: 0.2s ease-in-out;
    cursor: pointer;
    svg {
      fill: white;
    }
  }
`;

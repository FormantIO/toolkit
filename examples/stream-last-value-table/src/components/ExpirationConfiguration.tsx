import { Box, Typography, TextField, Icon } from "@formant/ui-sdk";
import { useRecoilState, useRecoilValue } from "recoil";
import { KeyValue, Authentication } from "@formant/data-sdk";
import {
  expirationMinutesState,
  expirationHoursState,
  moduleNameState,
} from "../atoms/configuration";
import { useCallback } from "react";


export const ExpirationConfiguation = () => {
  const moduleName = useRecoilValue(moduleNameState);
  const [expirationHours, setExpirationHours] =
    useRecoilState(expirationHoursState);
  const [expirationMinutes, setExpirationMinutes] = useRecoilState(
    expirationMinutesState
  );

  const handleOnChange = useCallback(async () => {
    if (await Authentication.waitTilAuthenticated()) {
      const expirationTime = {
        hours: expirationHours,
        minutes: expirationMinutes,
      };
      await KeyValue.set(
        `${moduleName}-expiration`,
        JSON.stringify(expirationTime)
      );
    }
  }, [expirationHours, expirationMinutes]);

  return (
    <Box
      sx={{
        display: "flex",
        alignItems: "left",
        justifyContent: "space-between",
        width: 224,
        flexDirection: "column",
        marginBottom: 1,
      }}
    >
      <Typography
        sx={{
          marginRight: 1,
          fontSize: 14,
        }}
      >
        Clean data in:{" "}
      </Typography>
      <Box
        sx={{
          width: "100%",
          display: "flex",
          alighitems: "center",
          justifyContent: "center",
        }}
      >
        <TextField
          type="number"
          name="hours"
          onBlur={handleOnChange}
          onChange={(e) => {
            if (e.target.value.length === 0) {
              setExpirationHours(e.target.value);
              return;
            }
            setExpirationHours(e.target.value);
          }}
          value={expirationHours}
          variant="filled"
          label={"Hour(s)"}
        />
        <TextField
          type="number"
          name="minutes"
          onBlur={handleOnChange}
          onChange={(e) => {
            if (e.target.value.length === 0) {
              setExpirationMinutes(e.target.value);
              return;
            }
            setExpirationMinutes(e.target.value);
          }}
          value={expirationMinutes}
          variant="filled"
          label={"Minute(s)"}
        />
      </Box>
    </Box>
  );
};

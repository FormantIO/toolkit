import { Authentication, Fleet } from "../src/main";
import "./style.css";

(async function () {
  const el = document.querySelector("#app");
  if (el) {
    if (await Authentication.waitTilAuthenticated()) {
      const tables = await Fleet.getTaskReportTables();

      const sum = tables.map((tab) => {
        return Fleet.getTaskReportRows({
          taskColumns: [
            {
              columns: tab.columns,
              name: "DURATION_SECONDS",
              tableName: tab.tableName,
            },
          ],
        });
      });

      const aggregates = await Promise.all(sum);
      console.log(aggregates);
    }
  }
})();

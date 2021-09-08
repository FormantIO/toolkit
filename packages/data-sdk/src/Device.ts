export class Device {
  constructor(private token: string, public id: string, public name: string) {}
  async getLatestTelemetry() {
    const data = await fetch(
      `https://api.formant.io/v1/queries/stream-current-value`,
      {
        method: "POST",
        body: JSON.stringify({
          deviceIds: [this.id],
        }),
        headers: {
          "Content-Type": "application/json",
          Authorization: "Bearer " + this.token,
        },
      }
    );
    const telemetry = await data.json();
    return telemetry.items;
  }
}

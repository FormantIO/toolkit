export const config = {
  streams: [
    {
      name: "blade.mode",
      expectedValue: "autonomy",
    },
    {
      name: "blade.range",
      expectedValue: {
        greaterThan: 10,
        lesserThan: 20,
      },
    },
    {
      name: "blade.set",
      expectedValue: false,
    },
  ],
};

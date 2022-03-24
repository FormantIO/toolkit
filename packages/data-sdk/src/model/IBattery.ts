export interface IBattery {
  percentage: number; // 0 to 1
  voltage?: number; // Volts
  current?: number; // Amps
  charge?: number; // Amp-hours
}

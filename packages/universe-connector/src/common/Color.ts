export class Color {
  public h: number;
  public s: number;
  public l: number;

  public static fromString(color: string): Color | undefined {
    const match = /#([\da-fA-F]{2})([\da-fA-F]{2})([\da-fA-F]{2})/.exec(color);
    if (match) {
      return new Color(
        parseInt(match[1], 16),
        parseInt(match[2], 16),
        parseInt(match[3], 16)
      );
    }
    return undefined;
  }

  constructor(
    public r: number,
    public g: number,
    public b: number,
    public a: number = 1.0
  ) {
    const _r = this.r / 255;
    const _g = this.g / 255;
    const _b = this.b / 255;

    const max = Math.max(_r, _g, _b);
    const min = Math.min(_r, _g, _b);
    let h = 0;
    let s = 0;
    const l = (max + min) / 2;

    if (max !== min) {
      const d = max - min;
      s = l > 0.5 ? d / (2 - max - min) : d / (max + min);

      switch (max) {
        case _r:
          h = (_g - _b) / d + (_g < _b ? 6 : 0);
          break;
        case _g:
          h = (_b - _r) / d + 2;
          break;
        case _b:
          h = (_r - _g) / d + 4;
          break;
      }

      h /= 6;
    }

    this.h = h;
    this.s = s;
    this.l = l;
  }

  public toString() {
    const hex = (x: number) => x.toString(16).padStart(2, "0");
    return `#${hex(this.r)}${hex(this.g)}${hex(this.b)}${
      this.a === 1.0 ? "" : hex(Math.round(this.a * 255.0))
    }`;
  }

  public withAlpha(a: number) {
    return new Color(this.r, this.g, this.b, a);
  }
}

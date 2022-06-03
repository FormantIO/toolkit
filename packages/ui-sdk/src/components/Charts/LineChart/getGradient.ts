export const getGradient = (
  color: string,
  ctx: CanvasRenderingContext2D,
  height?: number
) => {
  const hexToRgbColor = () => {
    let rgbColor: any = color;
    if (rgbColor.length === 7) rgbColor = rgbColor.replace("#", "");
    rgbColor = rgbColor.match(/.{1,2}/g);
    return `rgba(
          ${parseInt(rgbColor![0], 16)}, 
          ${parseInt(rgbColor![1], 16)}, 
          ${parseInt(rgbColor![2], 16)},
        `;
  };

  const createGradient = () => {
    const gradient = ctx.createLinearGradient(
      0,
      0,
      0,
      !!height ? height - height * 0.25 : 150
    );
    let gradientColor = color.replace(")", ",");

    if (color.length <= 7) gradientColor = hexToRgbColor();
    gradient.addColorStop(1, gradientColor + " 0)");
    gradient.addColorStop(0.25, gradientColor + " 0.33)");

    return gradient;
  };
  return createGradient();
};

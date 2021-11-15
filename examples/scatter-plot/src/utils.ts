const utils = {
  getMaxandMinValuesInOurData: (data: object) => {
    let maxValueInOurdata: number;
    let minValueInOurData: number;

    Object.keys(data).forEach((stream: string) =>
      data[stream].forEach((value: Array<number>) => {
        maxValueInOurdata === undefined
          ? (maxValueInOurdata = value[1])
          : (maxValueInOurdata = Math.max(maxValueInOurdata, value[1]));
        minValueInOurData === undefined
          ? (minValueInOurData = value[1])
          : (minValueInOurData = Math.min(minValueInOurData, value[1]));
      })
    );

    return {
      minValueInOurData: minValueInOurData,
      maxValueInOurdata: maxValueInOurdata,
    };
  },
  normalizeSingleValue: (
    range: Array<number>,
    minValueOfOurData: number,
    maxValueOfOurData: number,
    valueToBeNormalize: number
  ) => {
    let minValueInNormalizationRange = range[0];
    let maxValueInNormalizationRange = range[1];

    let topLeftValue = valueToBeNormalize - minValueOfOurData;
    let topRightValue =
      maxValueInNormalizationRange - minValueInNormalizationRange;
    let bottomValue = maxValueOfOurData - minValueOfOurData;

    let r = (topLeftValue * topRightValue) / bottomValue;
    r = r + minValueInNormalizationRange;
    return r;
  },
  normalizeArrayData: (range: Array<number>, data: Array<number>) => {
    let minValueInNormalizationRange = range[0];
    let maxValueInNormalizationRange = range[1];
    let minValueOfOurData = Math.min(...data);
    let maxValueOfOurData = Math.max(...data);

    const normalizeEquation = (valueToBeNormalize: number) => {
      let topLeftValue = valueToBeNormalize - minValueOfOurData;
      let topRightValue =
        maxValueInNormalizationRange - minValueInNormalizationRange;
      let bottomValue = maxValueOfOurData - minValueOfOurData;

      let r = (topLeftValue * topRightValue) / bottomValue;
      r = r + minValueInNormalizationRange;
      return r;
    };

    let normalizeValues = data.map((value) => normalizeEquation(value));

    return normalizeValues;
  },

  //Normalize second value in a two dimensinal array
  normalizeTwoDimensionalArray: (
    range: Array<number>,
    minValueOfOurData: number,
    maxValueOfOurData: number,
    arrayToBeNormalize: Array<Array<number>>
  ) => {
    return arrayToBeNormalize.map((value: Array<number>) => {
      let normalizeValue = utils.normalizeSingleValue(
        range,
        minValueOfOurData,
        maxValueOfOurData,
        value[1]
      );

      return [value[0], normalizeValue];
    });
  },
};

export default utils;

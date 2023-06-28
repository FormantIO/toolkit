import { describe, it, expect } from "vitest";
import { deserializeHash, serializeHash } from "./serializeHash";

describe("serializeHash()", () => {
  describe("encoding", () => {
    it.each<[string, any, string]>([
      ["a string", "test string", "eJxTKkktLlEoLinKzEtXAgAhFQS8"],
      ["an array", [1, 2, 3, 4], "eJyLNtQx0jHWMYkFAAoVAgc="],
      [
        "an object",
        { foo: 1, bar: 2, baz: 3 },
        "eJyrVkrLz1eyMtRRSkosUrIyAtFVSlbGtQBdegcX",
      ],
      ["null", null, "eJzLK83JAQAEXwG8"],
      ["100 nulls", Array(100).fill(null), "eJyLzivNydEZJUYSEQsAG/y+yQ=="],
    ])("should encode %s", (_, data, expected) => {
      expect(serializeHash(data)).toBe(expected);
    });
  });

  describe("decoding", () => {
    it.each<[string, string, any]>([
      ["a string", "eJxTKkktLlEoLinKzEtXAgAhFQS8", "test string"],
      ["an array", "eJyLNtQx0jHWMYkFAAoVAgc=", [1, 2, 3, 4]],
      [
        "an object",
        "eJyrVkrLz1eyMtRRSkosUrIyAtFVSlbGtQBdegcX",
        { foo: 1, bar: 2, baz: 3 },
      ],
      ["null", "eJzLK83JAQAEXwG8", null],
      ["100 nulls", "eJyLzivNydEZJUYSEQsAG/y+yQ==", Array(100).fill(null)],
    ])("should decode %s", (_, data, expected) => {
      expect(deserializeHash(data)).toEqual(expected);
    });
  });
});

export const data = {
  "/rosout/get_loggers": {
    title: "/rosout/get_loggers",
    type: "string",
  },
  "/agent/integer": {
    title: "/agent/integer",
    type: "object",
    properties: {
      devices: {
        title: "Devices",
        type: "integer",
      },
    },
  },
  "/agent/Array": {
    title: "/agent/integer",
    type: "object",
    properties: {
      devices: {
        title: "Stream",
        type: "array",
      },
    },
  },
  "/rosout/set_logger_level": {
    title: "/rosout/set_logger_level",
    type: "object",
    properties: {
      logger: {
        title: "Logger",
        type: "string",
      },
      level: {
        title: "Level",
        type: "string",
      },
    },
  },
  "/formant_bridge_node/get_loggers": {
    title: "/formant_bridge_node/get_loggers",
    type: "string",
  },
  "/formant_bridge_node/set_logger_level": {
    title: "/formant_bridge_node/set_logger_level",
    type: "object",
    properties: {
      logger: {
        title: "Logger",
        type: "boolean",
      },
      level: {
        title: "Level",
        type: "string",
      },
    },
  },
  "/random_server/get_loggers": {
    title: "/random_server/get_loggers",
    type: "string",
  },
  "/random_server/set_logger_level": {
    title: "/random_server/set_logger_level",
    type: "object",
    properties: {
      logger: {
        title: "Logger",
        type: "string",
      },
      level: {
        title: "Level",
        type: "string",
      },
    },
  },
  "/random": {
    title: "/random",
    type: "object",
    properties: {
      header: {
        type: "object",
        title: "Header",
        properties: {
          "header.seq": {
            type: "integer",
            title: "Header.seq",
          },
          "header.stamp": {
            type: "string",
            title: "Header.stamp",
          },
          "header.frame_id": {
            type: "string",
            title: "Header.frame_id",
          },
        },
      },
      seq: {
        type: "string",
        title: "Seq",
      },
      twist: {
        type: "object",
        title: "Twist",
        properties: {
          "twist.linear": {
            type: "object",
            title: "Twist.linear",
            properties: {
              nestedObj: {
                type: "object",
                title: "NestedObj",
                properties: {
                  "nestedObj.x": {
                    type: "string",
                    title: "NestedObj.x",
                  },
                },
              },
              "twist.linear.x": {
                type: "string",
                title: "Twist.linear.x",
              },
              "twist.linear.y": {
                type: "string",
                title: "Twist.linear.y",
              },
              "twist.linear.z": {
                type: "string",
                title: "Twist.linear.z",
              },
            },
          },
          "twist.angular": {
            type: "object",
            title: "Twist.angular",
            properties: {
              "twist.angular.x": {
                type: "string",
                title: "X",
              },
              "twist.angular.y": {
                type: "string",
                title: "Y",
              },
              "twist.angular.z": {
                type: "string",
                title: "Z",
              },
            },
          },
        },
      },
      // my_int_array: {
      //   type: "array",
      //   title: "My_int_array",
      //   items: {
      //     title: "Items",
      //     type: "integer",
      //   },
      // },
    },
  },
};

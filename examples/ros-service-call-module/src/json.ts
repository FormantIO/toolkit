let data = {
  "/rosout/get_loggers": {
    title: "/rosout/get_loggers",
    type: "object",
    properties: {},
  },
  "/rosout/set_logger_level": {
    title: "/rosout/set_logger_level",
    type: "object",
    properties: {
      logger: { type: "string", title: "logger" },
      level: { type: "string", title: "level" },
    },
  },
  "/formant_bridge_node/get_loggers": {
    title: "/formant_bridge_node/get_loggers",
    type: "object",
    properties: {},
  },
  "/formant_bridge_node/set_logger_level": {
    title: "/formant_bridge_node/set_logger_level",
    type: "object",
    properties: {
      logger: { type: "string", title: "logger" },
      level: { type: "string", title: "level" },
    },
  },
  "/random_server/get_loggers": {
    title: "/random_server/get_loggers",
    type: "object",
    properties: {},
  },
  "/random_server/set_logger_level": {
    title: "/random_server/set_logger_level",
    type: "object",
    properties: {
      logger: { type: "string", title: "logger" },
      level: { type: "string", title: "level" },
    },
  },
  "/random": {
    title: "/random",
    type: "object",
    properties: {
      header: {
        type: "object",
        properties: {
          "header.seq": { type: "integer", title: "header.seq" },
          "header.stamp": {
            type: "object",
            properties: {
              "header.stamp.secs": {
                type: "integer",
                title: "header.stamp.secs",
              },
              "header.stamp.nsecs": {
                type: "integer",
                title: "header.stamp.nsecs",
              },
            },
          },
          "header.frame_id": { type: "string", title: "header.frame_id" },
        },
      },
      seq: {
        type: "object",
        properties: {
          "seq.secs": { type: "integer", title: "seq.secs" },
          "seq.nsecs": { type: "integer", title: "seq.nsecs" },
        },
      },
      twist: {
        type: "object",
        properties: {
          "twist.linear": {
            type: "object",
            properties: {
              "twist.linear.x": { type: "number", title: "twist.linear.x" },
              "twist.linear.y": { type: "number", title: "twist.linear.y" },
              "twist.linear.z": { type: "number", title: "twist.linear.z" },
            },
          },
          "twist.angular": {
            type: "object",
            properties: {
              "twist.angular.x": { type: "number", title: "twist.angular.x" },
              "twist.angular.y": { type: "number", title: "twist.angular.y" },
              "twist.angular.z": { type: "number", title: "twist.angular.z" },
            },
          },
        },
      },
      my_int_array: {
        type: "array",
        items: { type: "integer" },
        title: "my_int_array",
      },
    },
  },
};

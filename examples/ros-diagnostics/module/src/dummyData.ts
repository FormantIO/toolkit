export const data = [
    { name: "/rosout_agg", type: "rosgraph_msgs/Log", hz: 0 },
    { name: "/rosout", type: "rosgraph_msgs/Log", hz: 0 },
    {
      name: "/msa_dummy_data",
      type: "std_msgs/String",
      hz: 9.99999651689454,
    },
    {
      name: "/auto_manual_status",
      type: "std_msgs/Bool",
      hz: 9.999996594436864,
    },
    {
      name: "/can/rpm_feedback",
      type: "std_msgs/Float32MultiArray",
      hz: 9.99999651689454,
    },
    {
      name: "/stacklight_toggle",
      type: "robot_msgs/StackLight",
      hz: 9.999996398157858,
    },
    {
      name: "/safety_state",
      type: "robot_msgs/SafetyState",
      hz: 9.999996458737797,
    },
    {
      name: "/formant_ui/mission_status",
      type: "ui_msgs/MissionStatusMsg",
      hz: 9.999999875447557,
    },
    {
      name: "/formant_ui/vehicle_status",
      type: "ui_msgs/VehicleStatusMsg",
      hz: 10.000005242836753,
    },
    {
      name: "/formant_ui/flexisoft_status",
      type: "ui_msgs/FlexisoftStatusMsg",
      hz: 10.00000655378912,
    },
    { name: "/formant/ingest", type: "std_msgs/Int32", hz: 0 },
    { name: "/formant/command", type: "std_msgs/String", hz: 0 },
  ]
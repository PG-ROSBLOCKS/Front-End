export let toolbox = {
    kind: 'categoryToolbox',
    contents: [
      {
        kind: 'category',
        name: 'Nodes',
        contents: [
          { kind: 'block', type: 'ros2_timer' },
          { kind: 'block', type: 'ros2_log' },
        ],
      },
      {
        kind: 'category',
        name: 'Topics',
        contents: [
          { kind: 'block', type: 'ros2_create_publisher' },
          { kind: 'block', type: 'ros2_create_subscriber' },
          { kind: 'block', type: 'ros2_subscriber_msg_data' },
          { kind: 'block', type: 'ros2_print_msg_type' },
          { kind: 'block', type: 'ros2_publish_message' },
        ],
      },
      {
        kind: 'category',
        name: 'Services',
        contents: [
          { kind: 'block', type: 'ros2_service_block' },
          { kind: 'block', type: 'ros2_named_message' },
          { kind: 'block', type: 'ros_create_server' },
          { kind: 'block', type: 'ros2_service_available' },
        ],
      },
      {
        kind: 'category',
        name: 'Clients',
        contents: [
          { kind: 'block', type: 'ros_create_client' },
          { kind: 'block', type: 'ros_send_request' },
        ],
      },
      {
        kind: 'category',
        name: 'Messages',
        contents: [
          { kind: 'block', type: 'ros2_message_block' },
          { kind: 'block', type: 'ros2_named_message' }
        ],
      },
      {
        kind: 'category',
        name: 'Conditionals',
        contents: [
          { kind: 'block', type: 'controls_if' },
          { kind: 'block', type: 'controls_ifelse' },
          { kind: 'block', type: 'logic_compare' },
          { kind: 'block', type: 'logic_operation' },
          { kind: 'block', type: 'logic_negate' },
          { kind: 'block', type: 'logic_boolean' },
          { kind: "block", type: "logic_ternary" },
        ],
      },
      {
        kind: 'category',
        name: 'Cycles',
        contents: [
          { kind: "block", type: "controls_repeat" },
          { kind: "block", type: "controls_repeat_ext" },
          { kind: "block", type: "controls_whileUntil" },
          { kind: "block", type: "controls_for" },
          { kind: "block", type: "controls_forEach" },
          { kind: "block", type: "ros2_sleep" },
        ],
      },
      {
        kind: 'category',
        name: 'Operations',
        contents: [
          { kind: "block", type: "math_number" },
          { kind: "block", type: "math_arithmetic" },
          { kind: "block", type: "math_single" },
          { kind: "block", type: "math_trig" },
          { kind: "block", type: "math_round" },
          { kind: "block", type: "math_random_int" },
          { kind: "block", type: "math_modulo" },
        ],
      },
      {
        kind: 'category',
        name: 'Variables',
        contents: [
          { kind: "block", type: "variables_get" },
          { kind: "block", type: "variables_set" },
        ],
      },
      {
        kind: 'category',
        name: 'Functions',
        contents: [
          { kind: "block", type: "procedures_defnoreturn" },
          { kind: "block", type: "procedures_defreturn" },
          { kind: "block", type: "procedures_callnoreturn" },
          { kind: "block", type: "procedures_callreturn" },
          { kind: "block", type: "procedures_ifreturn" },
        ],
      },
      {
        kind: 'category',
        name: 'Text',
        contents: [
          { kind: "block", type: "text" },
          { kind: "block", type: "text_join" },
          { kind: "block", type: "text_append" },
          { kind: "block", type: "text_length" },
          { kind: "block", type: "text_isEmpty" },
          { kind: "block", type: "text_indexOf" },
          { kind: "block", type: "text_charAt" },
          { kind: "block", type: "text_getSubstring" },
          { kind: "block", type: "text_changeCase" },
          { kind: "block", type: "text_trim" },
          { kind: "block", type: "text_print" },
        ]
      },
      {
        kind: 'category',
        name: 'Turtlesim',
        contents: [
          { kind: "block", type: "ros2_turtlesim_publisher" },
          { kind: "block", type: "ros2_publish_twist" },
          { kind: "block", type: "ros2_turtle_set_pose" },
          { kind: "block", type: "ros2_kill_turtle" },
          { kind: "block", type: "ros2_spawn_turtle" },
          { kind: "block", type: "ros2_turtle_set_pen" },
          { kind: "block", type: "ros2_turtle_rotate" },
        ]
      }
    ],
  };
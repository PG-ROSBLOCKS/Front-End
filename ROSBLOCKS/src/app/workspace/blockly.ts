import { blockColors } from "../blocks/color-palette";

export let toolbox = {
  kind: 'categoryToolbox',
  contents: [
    {
      kind: 'category',
      name: 'ROS 2 Blocks',
      contents: [
        {
          kind: 'category',
          name: 'Turtlesim',
          colour: blockColors.Turtlesim,
          contents: [
            { kind: "block", type: "ros2_turtlesim_publisher" },
            { kind: "block", type: "ros2_publish_twist" },
            { kind: "block", type: "ros2_turtle_set_pose" },
            { kind: "block", type: "ros2_kill_turtle" },
            { kind: "block", type: "ros2_spawn_turtle" },
            { kind: "block", type: "ros2_turtle_set_pen" },
            { kind: "block", type: "ros2_turtle_rotate" },
            { kind: "block", type: "ros2_publish_twist_full" },
          ]
        },
      {
        kind: 'category',
        name: 'Publishers and Subscribers',
        colour: blockColors.Topics,
        contents: [
          { kind: 'block', type: 'ros2_create_publisher' },
          { kind: 'block', type: 'ros2_publish_message' },
          { kind: 'block', type: 'ros2_create_subscriber' },
          { kind: 'block', type: 'ros2_subscriber_msg_data' },
        ],
      },
      {
        kind: 'category',
        name: 'Services',
        colour: blockColors.Services,
        contents: [
          { kind: 'block', type: 'ros_create_server' },
          { kind: 'block', type: 'ros2_named_message' },
        ],
      },
      {
        kind: 'category',
        name: 'Clients',
        colour: blockColors.Clients,
        contents: [
          { kind: 'block', type: 'ros_create_client' },
          { kind: 'block', type: 'ros_send_request' },
        ],
      },
      // {
      //   kind: 'category',
      //   name: 'Custom srv types',
      //   colour: blockColors.Messages,
      //   contents: [
      //     { kind: 'block', type: 'ros2_service_block' },
      //     { kind: 'block', type: 'ros2_named_message' },
      //   ],
      // },
      // {
      //   kind: 'category',
      //   name: 'Custom msgs types',
      //   colour: blockColors.Messages,
      //   contents: [
      //     { kind: 'block', type: 'ros2_message_block' },
      //     { kind: 'block', type: 'ros2_named_message' },
      //   ],
      // },
      {
        kind: 'category',
        name: 'Common Utilities',
        colour: blockColors.Utils,
        contents: [
          { kind: 'block', type: 'ros2_timer' },
          { kind: 'block', type: 'ros2_log' },
          { kind: "block", type: "ros2_cast_type" },
          { kind: "block", type: "ros2_sleep" },
          { kind: 'block', type: 'srv_response_set_field' },
        ],
      },
      {
        kind: 'category',
        name: 'ROS 2 msg and srv types',
        contents: [
            {
              kind: 'category',
              name: 'ROS 2 srv types',
              colour: blockColors.Services,
              contents: [
                {
                  kind: 'category',
                  name: 'Create custom srv',
                  colour: blockColors.Services,
                  contents: [
                    { kind: 'block', type: 'ros2_service_block' },
                    { kind: 'block', type: 'ros2_named_message' },
                  ]
                }
              ]
            },
            {
            kind: 'category',
            name: 'ROS 2 msg types',
            colour: blockColors.Messages,
            contents: [
              {
                kind: 'category',
                name: 'Create custom msg',
                colour: blockColors.Messages,
                contents: [
                  { kind: 'block', type: 'ros2_message_block' },
                  { kind: 'block', type: 'ros2_named_message' },
                ]
              }
            ]
          },
        ],
      }]
    },
    {
      kind: 'category',
      name: 'Programming Blocks',
      contents: [
        {
          kind: 'category',
          name: 'Conditionals',
          colour: blockColors.Conditionals,
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
          colour: '#3E7E7E',
          contents: [
            { kind: "block", type: "controls_repeat" },
            { kind: "block", type: "controls_repeat_ext" },
            { kind: "block", type: "controls_whileUntil" },
            { kind: "block", type: "controls_for" },
            { kind: "block", type: "controls_forEach" },
          ],
        },
        {
          kind: 'category',
          name: 'Operations',
          colour: '#569BBD',
          contents: [
            { kind: "block", type: "integer_number" },
            { kind: "block", type: "float_number" },
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
          colour: '#3D65A8',
          contents: [
            { kind: "block", type: "variables_get" },
            { kind: "block", type: "variables_set" },
          ],
        },
        {
          kind: 'category',
          name: 'Text',
          colour: '#6835BB',
          contents: [
            { kind: "block", type: "text" },
            { kind: "block", type: "text_char_to_ascii" },
            { kind: "block", type: "text_ascii_to_char" },
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
      ]
    }
  ]
};

export function updateNestedCategoryInToolbox(
  toolboxObj: any,
  rootCategory: string,
  parentCategory: string,
  dynamicCategoryName: string,
  newDynamicCategory: any
): void {
  const root = toolboxObj.contents.find((c: any) => c.name === rootCategory);
  if (!root) {
    console.warn(`Categoría raíz "${rootCategory}" no encontrada.`);
    return;
  }

  const parent = root.contents.find((c: any) => c.name === parentCategory);
  if (!parent) {
    console.warn(`Subcategoría "${parentCategory}" no encontrada dentro de "${rootCategory}".`);
    return;
  }

  const existing = parent.contents.findIndex((c: any) => c.name === dynamicCategoryName);
  if (existing !== -1) {
    parent.contents[existing] = newDynamicCategory;
  } else {
    parent.contents.push(newDynamicCategory);
  }
}

export function updateDeepCategoryInToolbox(
  toolboxObj: any,
  categoryPath: string[], // ['ROS 2 Blocks', 'ROS 2 msg and srv types', 'ROS 2 msg types']
  dynamicCategoryName: string,
  newDynamicCategory: any
): void {
  let currentLevel = toolboxObj.contents;

  for (const name of categoryPath) {
    const next = currentLevel.find((c: any) => c.name === name);
    if (!next) {
      console.warn(`Categoría "${name}" no encontrada en ruta [${categoryPath.join(' > ')}]`);
      return;
    }
    currentLevel = next.contents;
  }

  const existingIndex = currentLevel.findIndex((c: any) => c.name === dynamicCategoryName);
  if (existingIndex !== -1) {
    currentLevel[existingIndex] = newDynamicCategory;
  } else {
    currentLevel.push(newDynamicCategory);
  }
}


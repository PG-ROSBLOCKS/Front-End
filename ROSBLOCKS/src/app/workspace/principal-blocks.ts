/**
 * This file stores the main blocks.
 */

export enum PrincipalBlock {
    ROS2_SERVICE_BLOCK = "ros2_service_block",
    ROS_CREATE_SERVER = "ros_create_server",
    ROS_CREATE_CLIENT = "ros_create_client",
    ROS2_PUBLISH_MESSAGE = "ros2_publish_message",
    ROS2_SUBSCRIBER_MSG_DATA = "ros2_subscriber_msg_data",
    ROS2_CREATE_PUBLISHER = "ros2_create_publisher",
    ROS2_MINIMAL_PUBLISHER = "ros2_minimal_publisher",
    ROS2_CREATE_SUBSCRIBER = "ros2_create_subscriber",
}

export const principalBlocks: string[] = [
    PrincipalBlock.ROS2_SERVICE_BLOCK,
    PrincipalBlock.ROS_CREATE_SERVER,
    PrincipalBlock.ROS_CREATE_CLIENT,
    PrincipalBlock.ROS2_PUBLISH_MESSAGE,
    PrincipalBlock.ROS2_SUBSCRIBER_MSG_DATA,
    PrincipalBlock.ROS2_CREATE_PUBLISHER,
    PrincipalBlock.ROS2_MINIMAL_PUBLISHER,
    PrincipalBlock.ROS2_CREATE_SUBSCRIBER,
];
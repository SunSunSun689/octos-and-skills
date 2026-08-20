#!/usr/bin/env python
# -*- coding: utf-8 -*-
import dora
from dora import Node

ros2_context = dora.Ros2Context()
ros2_node = ros2_context.new_node(
    "turtle_teleop", # 名字
    "/ros2_demo", # 名字空间
    dora.Ros2NodeOptions(rosout=True),
)

# 定义一个 ROS2 QOS
topic_qos = dora.Ros2QosPolicies(
    reliable=True, max_blocking_time=0.1
)

# 创建一个发布者至 cmd_vel 主题
turtle_twist_topic = ros2_node.create_topic(
    "/turtle1/cmd_vel", "geometry_msgs/Twist", topic_qos
)
twist_writer = ros2_node.create_publisher(turtle_twist_topic)

# 创建一个监听者至 pose 主题
turtle_pose_topic = ros2_node.create_topic(
    "/turtle1/pose", "turtlesim/Pose", topic_qos
)
pose_reader = ros2_node.create_subscription(turtle_pose_topic)

# 创建一个 dora 节点
dora_node = Node()

# 在同一个循环上监听两个流，因为 Python 不能很好处理多进程
dora_node.merge_external_events(pose_reader)

for i in range(500):
    event = dora_node.next()
    if event is None:
        break
    match event["kind"]:

        # Dora 事件
        case "dora":
            match event["type"]:
                case "INPUT":
                    match event["id"]:
                        case "direction":
                            twist_writer.publish(event["value"])

        # 在本 case 分支中为 ROS2 事件
        case "external":
            pose = event["value"][0].as_py()
            dora_node.send_output("turtle_pose", event["value"])
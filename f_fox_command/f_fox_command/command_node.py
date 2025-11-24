import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from f_interfaces.msg import PersonAction, FoxCommand


class CommandNode(Node):
    # map gesture labels to command labels
    LABEL_COMMAND = {
        'cross arms': 'salute',
        'stop': 'stop',
        'come closer': 'come closer',
        'come to me': 'come to me',
    }

    COMMAND_DESCRIPTIONS = {
        'salute': "Greete the human",
        'stop': "",
        'come closer': "",
        'come to me': "",
    }

    def __init__(self):
        super().__init__("command_node")

        # wait at least this time before increasing the counter on a prediction
        self.declare_parameter('prediction_cooldown', 0.0)
        # count so many times before sending a command
        self.declare_parameter('prediction_successes', 2)
        # minimum confidence of a prediction
        self.declare_parameter('prediction_threshold', 0.6)
        # wait before publishing another command
        self.declare_parameter('command_cooldown', 10.0)

        self.prediction_cooldown = self.get_parameter('prediction_cooldown').get_parameter_value().double_value
        self.prediction_successes = self.get_parameter('prediction_successes').get_parameter_value().integer_value
        self.prediction_threshold = self.get_parameter('prediction_threshold').get_parameter_value().double_value
        self.command_cooldown = self.get_parameter('command_cooldown').get_parameter_value().double_value

        self.prepared_command = None
        self.prepared_counter = 0
        self.last_command_time = 0.
        self.last_prediction_time = 0.

        self.sub_gesture = self.create_subscription(PersonAction, "/f_gesture_recognition/actions", self.gesture_cb, 10)
        self.pub_command = self.create_publisher(FoxCommand, "/f_fox_command/command", 10)

        self.get_logger().info("Waiting for predictions...")

    def gesture_cb(self, msg: PersonAction):
        t = self.get_clock().now().nanoseconds / 10**9
        if t - self.last_prediction_time < self.prediction_cooldown:
            return
        self.last_prediction_time = t
        if msg.label not in CommandNode.LABEL_COMMAND:
            self.prepared_command = None
            self.prepared_counter = 0
            return
        command = CommandNode.LABEL_COMMAND[msg.label]
        if msg.confidence >= self.prediction_threshold:
            if command == self.prepared_command:
                self.prepared_counter += 1
            else:
                self.prepared_command = command
                self.prepared_counter = 1
        elif command != self.prepared_command:
            self.prepared_command = None
            self.prepared_counter = 0
        if self.prepared_counter >= self.prediction_successes and t - self.last_command_time >= self.command_cooldown:
            self.last_command_time = t
            command_msg = FoxCommand()
            command_msg.header = Header()
            command_msg.header.stamp = self.get_clock().now().to_msg()
            command_msg.command = command
            command_msg.description = CommandNode.COMMAND_DESCRIPTIONS[command] \
                if command in CommandNode.COMMAND_DESCRIPTIONS \
                else None
            self.pub_command.publish(command_msg)
            self.get_logger().info(f"Posting {command_msg.command}: {command_msg.description}")
            self.prepared_command = None
            self.prepared_counter = 0


def main():
    rclpy.init()
    node = CommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

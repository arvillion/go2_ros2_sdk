import argparse
import base64
import os
import random
import socket
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from requests.exceptions import Timeout, RequestException

import requests


class QueryClientNode(Node):
    def __init__(self):
        super().__init__('query_client_node')

        self.declare_parameter('query', 'find a global stand in the corridor')
        self.declare_parameter('server_ip', '10.16.2.104')
        self.declare_parameter('server_port', 12345)

        self.query = self.get_parameter('query').get_parameter_value().string_value
        self.server_ip = self.get_parameter('server_ip').get_parameter_value().string_value
        self.server_port = self.get_parameter('server_port').get_parameter_value().integer_value

        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.query_server()

    def send_request(self, url: str, **kwargs):
        response = {}
        for attempt in range(5):
            try:
                response = self._send_request(url, **kwargs)
                break
            except Exception as e:
                if attempt == 4:
                    self.get_logger().error(f"Error: {e}. Giving up.")
                    exit()
                else:
                    self.get_logger().info(f"Error: {e}. Retrying in 20-30 seconds...")
                    time.sleep(20 + random.random() * 10)

        return response

    def _send_request(self, url: str, **kwargs):
        lockfiles_dir = "lockfiles"
        if not os.path.exists(lockfiles_dir):
            os.makedirs(lockfiles_dir)
        filename = url.replace("/", "_").replace(":", "_") + ".lock"
        filename = filename.replace("localhost", socket.gethostname())
        filename = os.path.join(lockfiles_dir, filename)
        try:
            while True:
                while os.path.exists(filename):
                    time.sleep(0.05)
                    try:
                        if time.time() - os.path.getmtime(filename) > 120:
                            os.remove(filename)
                    except FileNotFoundError:
                        pass

                rand_str = str(random.randint(0, 1000000))

                with open(filename, "w") as f:
                    f.write(rand_str)
                time.sleep(0.05)
                try:
                    with open(filename, "r") as f:
                        if f.read() == rand_str:
                            break
                except FileNotFoundError:
                    pass

            payload = {k: v for k, v in kwargs.items()}
            headers = {"Content-Type": "application/json"}

            start_time = time.time()
            while True:
                try:
                    resp = requests.post(url, headers=headers, json=payload, timeout=20)
                    if resp.status_code == 200:
                        result = resp.json()
                        break
                    else:
                        raise Exception("Request failed")
                except (Timeout, RequestException) as e:
                    self.get_logger().info(f"Error: {e}. Retrying...")
                    if time.time() - start_time > 20:
                        raise Exception("Request timed out after 20 seconds")

            os.remove(filename)

        except Exception as e:
            try:
                os.remove(filename)
            except FileNotFoundError:
                pass
            raise e

        return result

    def send_query(self, port: int, query: str):
        url = f"http://{self.server_ip}:{port}/hovsg_query"
        payload = {"query": query}
        response = self.send_request(url, **payload)
        return response

    def query_server(self):
        self.get_logger().info(f"Sending query: '{self.query}'")
        result = self.send_query(port=self.server_port, query=self.query)

        # Assuming result is a dict with coordinates (x, y, z)
        if 'center' in result:
            self.get_logger().info(f"Received coordinates: {result}")
            target_pose = PoseStamped()
            target_pose.header.stamp = self.get_clock().now().to_msg()
            target_pose.header.frame_id = 'map'
            target_pose.pose.position.x = result['center'][0]
            target_pose.pose.position.y = result['center'][1]
            target_pose.pose.position.z = result['center'][2]
            target_pose.pose.orientation.w = 1.0  # Assuming no rotation for simplicity
            self.publisher.publish(target_pose)
            self.get_logger().info("Published target pose.")
        else:
            self.get_logger().error("Failed to receive valid coordinates from the server.")


def main(args=None):
    rclpy.init(args=args)

    query_client_node = QueryClientNode()

    # rclpy.spin(query_client_node)

    query_client_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
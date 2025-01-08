import argparse
import base64
import os
import random
import socket
import time
import asyncio
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from requests.exceptions import Timeout, RequestException
import requests


class QueryClientNode(Node):
    def __init__(self):
        super().__init__('query_client_node')

        # Declare parameters with defaults
        self.declare_parameter('query', 'find a global stand in the corridor')
        self.declare_parameter('server_ip', '10.16.2.104')
        self.declare_parameter('server_port', 12345)

        self.query = self.get_parameter('query').get_parameter_value().string_value
        self.server_ip = self.get_parameter('server_ip').get_parameter_value().string_value
        self.server_port = self.get_parameter('server_port').get_parameter_value().integer_value

        # Action client for NavigateToPose
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    async def send_request(self, url: str, **kwargs):
        """
        Send a request to the specified URL with retry logic and return the response.
        """
        response = {}
        for attempt in range(5):
            try:
                response = await self._send_request(url, **kwargs)
                break
            except Exception as e:
                if attempt == 4:
                    self.get_logger().error(f"Error: {e}. Giving up.")
                    return None
                else:
                    self.get_logger().info(f"Error: {e}. Retrying in 20-30 seconds...")
                    await asyncio.sleep(20 + random.random() * 10)
        return response

    async def _send_request(self, url: str, **kwargs):
        """
        Perform the actual HTTP request to the given URL.
        """
        lockfiles_dir = "lockfiles"
        if not os.path.exists(lockfiles_dir):
            os.makedirs(lockfiles_dir)
        filename = url.replace("/", "_").replace(":", "_") + ".lock"
        filename = filename.replace("localhost", socket.gethostname())
        filename = os.path.join(lockfiles_dir, filename)

        while os.path.exists(filename):
            await asyncio.sleep(0.05)
            if time.time() - os.path.getmtime(filename) > 120:
                os.remove(filename)

        rand_str = str(random.randint(0, 1000000))

        with open(filename, "w") as f:
            f.write(rand_str)
        await asyncio.sleep(0.05)
        
        try:
            with open(filename, "r") as f:
                if f.read() == rand_str:
                    pass
        except FileNotFoundError:
            pass

        payload = {k: v for k, v in kwargs.items()}
        headers = {"Content-Type": "application/json"}

        start_time = time.time()
        while True:
            try:
                resp = await asyncio.to_thread(requests.post, url, headers=headers, json=payload, timeout=20)
                if resp.status_code == 200:
                    return resp.json()
                else:
                    raise Exception("Request failed")
            except (Timeout, RequestException) as e:
                self.get_logger().info(f"Error: {e}. Retrying...")
                if time.time() - start_time > 20:
                    raise Exception("Request timed out after 20 seconds")

    async def send_query(self, port: int, query: str):
        """
        Send a query to the server and get the response.
        """
        url = f"http://{self.server_ip}:{port}/hovsg_query"
        payload = {"query": query}
        return await self.send_request(url, **payload)

    async def send_goal(self, pose: PoseStamped):
        """
        Send a goal to the navigation server via action.
        """
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available!')
            return
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        result = await send_goal_future

        if result.accepted:
            self.get_logger().info('Goal accepted!')
            feedback_future = self._action_client.get_feedback_async(result.goal_handle)
            feedback = await feedback_future
            self.get_logger().info(f"Feedback: {feedback.feedback}")
        else:
            self.get_logger().error('Goal rejected!')

    async def query_server(self):
        """
        Query the server for coordinates and send the goal to Nav2.
        """
        self.get_logger().info(f"Sending query: '{self.query}'")
        # result = await self.send_query(port=self.server_port, query=self.query)
        result = {
            'center': [5.803216666963672, 0.21148694242887453]
        }

        if result and 'center' in result:
            self.get_logger().info(f"Received coordinates: {result}")
            target_pose = PoseStamped()
            target_pose.header.stamp = self.get_clock().now().to_msg()
            target_pose.header.frame_id = 'map'
            target_pose.pose.position.x = result['center'][0]
            target_pose.pose.position.y = result['center'][1]
            # 5.803216666963672, 0.21148694242887453
            # target_pose.pose.position.x = 5.803216666963672
            # target_pose.pose.position.y = 0.21148694242887453
            target_pose.pose.position.z = 0.0
            target_pose.pose.orientation.w = 1.0  # Assuming no rotation for simplicity

            await self.send_goal(target_pose)

            self.get_logger().info("Goal has been sent. Shutting down node.")
            await self.shutdown()

        else:
            self.get_logger().error("Failed to receive valid coordinates from the server.")
            await self.shutdown()

    async def shutdown(self):
        """
        Shutdown the node gracefully.
        """
        self.destroy_node()  # Destroy the node
        rclpy.shutdown()  # Shutdown rclpy


def main(args=None):
    rclpy.init(args=args)

    query_client_node = QueryClientNode()

    asyncio.run(query_client_node.query_server())


if __name__ == "__main__":
    main()
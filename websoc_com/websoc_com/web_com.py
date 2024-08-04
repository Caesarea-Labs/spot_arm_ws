import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import asyncio
import websockets
import threading
import uuid

# auto release = "wss://lb8yivbqrd.execute-api.eu-central-1.amazonaws.com/release/";
# auto beta = "wss://lb8yivbqrd.execute-api.eu-central-1.amazonaws.com/beta/";



class Websoc(Node):

    def __init__(self):
        super().__init__('websoc')
        self.websocket_url = "wss://lb8yivbqrd.execute-api.eu-central-1.amazonaws.com/beta/"
        # Generate the id --
        self.listener_id = str(uuid.uuid4())
        self.target = ""
        self.event_type = "onSimulationStageSelected"  # Replace with your actual event type
        self.initial_message = "sub:onSimulationStageSelected:" + self.listener_id + "::[]"
        self.get_logger().info("Starting WebSocket node")


        # Start the WebSocket connection in a separate thread
        threading.Thread(target=self.start_websocket).start()

        self.publisher_ = self.create_publisher(String, 'test_cmd', 10)
        timer_period = 0.5  # seconds


    def start_websocket(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self.connect())

    async def connect(self):
        async with websockets.connect(self.websocket_url) as websocket:
            await websocket.send(self.initial_message)
            self.get_logger().info(f"Sent: {self.initial_message}")
            while True:
                response = await websocket.recv()
                self.get_logger().info(f"Received: {response}")
                self.handle_message(response)

    def handle_message(self, message):
        self.get_logger().info(f"Received: {message}")
        # Check if the message is in the format event:<listener-id>:<body>
        if message.startswith("event:"):
            parts = message.split(":")
            if len(parts) == 3:
                listener_id = parts[1]
                body = parts[2]
                self.process_event(listener_id, body)
            else:
                self.get_logger().warn(f"Invalid message format: {message}")
        else:
            self.get_logger().warn(f"Unrecognized message format: {message}")

    def process_event(self, listener_id, body):
        # Implement your custom logic to handle the event here
        self.get_logger().info(f"Processing event for listener {listener_id} with body: {body}")
        msg = String()
        msg.data = body
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    async def send_unsub_message(self):
        if self.websocket:
            unsub_message = f"unsub:{self.event_type}:{self.listener_id}"
            await self.websocket.send(unsub_message)
            self.get_logger().info(f"Sent: {unsub_message}")


    def destroy_node(self):
        self.get_logger().info('Destroy node')
        asyncio.run(self.send_unsub_message())
        self.get_logger().info('Unsbscribe message sent to websocket')
        super().destroy_node()



def main(args=None):
    rclpy.init(args=args)

    ws_node = Websoc()

    rclpy.spin(ws_node)
    ws_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
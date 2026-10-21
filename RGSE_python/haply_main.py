import argparse
import json
import asyncio
import websockets
import rclpy
import csv
import math
from datetime import datetime, timedelta
from rclpy.node import Node
import time
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

URI = "ws://localhost:10001"    # WebSocket URI
PRINT_EVERY_MS = 5000            # print telemetry once per N milliseconds

class JointPositionPublisher(Node):
    def __init__(self):
        super().__init__("position_publisher")

        # Setup of defaults
        self.future = rclpy.task.Future()
        self.joint_names = []
        self.initial_positions = []
        self.target = []
        self.period = float(10.0)  # seconds
        self.minperiod = float(3.0)  # seconds

        #Create readout for the joint states
        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )

        # Create a publisher for the joint trajectory
        self.traj_pub = self.create_publisher(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10
        )
        self.publish_timer = self.create_timer(0.1, self.publish_trajectory_list)

    def joint_state_callback(self, msg):
        # Readout of current joint state
        self.joint_names = list(msg.name)
        self.initial_positions = list(msg.position)

    def publish_trajectory_list(self):
        # Period correction
        self.period = max(self.period, self.minperiod)

        # Wait for initial joint state to be received
        if len(self.initial_positions) == 0:
            self.get_logger().info("Waiting for initial joint states...")
            return

        # Print initial joint state
        initial_positions_print = [round(val,3) for val in self.initial_positions]
        self.get_logger().info(
            f"\nCaptured initial joint states: \n {initial_positions_print}\n"
        )

        # Initiate message
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        # Add trajectory target to message
        t = self.period
        point = JointTrajectoryPoint()
        point.time_from_start.sec = int(t)
        point.time_from_start.nanosec = int((t - int(t)) * 1e9)
        point.positions = self.target
        msg.points.append(point)

        self.traj_pub.publish(msg)


        # Print target position
        target_print = [round(val,3) for val in self.target]
        self.get_logger().info(f"\nPublished position: \n {target_print}")
        print("----------------------------------------------------------\n")
        self.publish_timer.cancel()
        self.future.set_result(True)

async def fanuc_main():

    rclpy.init()
        
    print("------------------Position List Control---------------------")

    '''INPUTS'''
    
    list_name = str(input('\nEnter the name of the position list (with .csv): '))
    period = float(input('\nEnter the period for each motion (in seconds): '))

    '''CORE'''

    print("\n----------------------------------------------------------")

    # Read CSV
    target_list = []
    try:
        with open('/home/rgse/ws_fanuc/src/RGSE_python/position_lists/' + list_name, newline='') as csvfile:
            read_csv = csv.reader(csvfile, delimiter=',')
            for row in read_csv:
                for i in range(len(row)):
                    row[i] = math.radians(float(row[i]))
                target_list.append(list(row))

    except FileNotFoundError:
        print(f"Error: File not found: {list_name}")
        return

    # Target list readout loop
    count = 0
    for target in target_list:
        count += 1

        # Confirm to go to next position

        while True:
            Confirm = input('Continue to next pose? (y): ')
            if Confirm == "Y" or Confirm == "y":
                print()
                break
            else:
                print('Wrong input\n')

        #Creating & Running trajectory publisher node

        print("----------------------------------------------------------")
        print(f"Position {count}\n")

        node = JointPositionPublisher()

        node.period = period
        node.target = target

        rclpy.spin_until_future_complete(node, node.future)

        node.destroy_node()

    print('\nEnd of list')

    # Shutdown sequence to allow zeroing robot before exit

    while True:
        confirm_zero = input('\nZero the robot? (y/n): ')
        if confirm_zero == "y":

            node = JointPositionPublisher()

            node.period = period
            node.target = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

            rclpy.spin_until_future_complete(node, node.future)

            node.destroy_node()
            print('\nZero complete, Exiting program\n')
            break

        elif confirm_zero == "n":
            print('\nExiting program\n')
            break

        else:
            print('\nWrong input\n')
            return

    rclpy.shutdown()


def get_first_inverse3_device_id(data):
    """Return the first Inverse3 device ID from a state frame, or '' if none."""
    if "inverse3" not in data or not isinstance(data["inverse3"], list):
        return ""
    inverse3s = data["inverse3"]
    if not inverse3s:
        return ""
    return inverse3s[0].get("device_id", "")


def print_state(state):
    """Pretty-print one Inverse3 state frame on a single self-rewriting line."""
    p = state.get("cursor_position", {"x": 0.0, "y": 0.0, "z": 0.0})
    v = state.get("cursor_velocity", {"x": 0.0, "y": 0.0, "z": 0.0})
    f = state.get("current_cursor_force", {"x": 0.0, "y": 0.0, "z": 0.0})
    # The workspace transform uses partial-update semantics: sub-fields equal to
    # their identity default (position={0,0,0}, rotation=identity, scale={1,1,1})
    # are omitted from the payload to save bandwidth. Supply defaults here for
    # display. Enable serialization/explicit_fields to always receive all fields.
    t  = state.get("transform", {})
    tp = t.get("position", {"x": 0.0, "y": 0.0, "z": 0.0})
    tr = t.get("rotation", {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0})
    ts = t.get("scale",    {"x": 1.0, "y": 1.0, "z": 1.0})
    print(
        f"\rpos:({p['x']:.6f} {p['y']:.6f} {p['z']:.6f})  "
        f"vel:({v['x']:.6f} {v['y']:.6f} {v['z']:.6f})  "
        f"force:({f['x']:.3f} {f['y']:.3f} {f['z']:.3f})  "
        f"ws.pos:({tp['x']:+.4f} {tp['y']:+.4f} {tp['z']:+.4f})  "
        f"ws.rot:(w:{tr['w']:+.3f} x:{tr['x']:+.3f} y:{tr['y']:+.3f} z:{tr['z']:+.3f})  "
        f"ws.scale:({ts['x']:.3f} {ts['y']:.3f} {ts['z']:.3f})  ",
        end="",
        flush=True
    )


async def connect(full, query_config):
    first_message = True
    device_id = ""
    request_msg = {}
    print_delay = timedelta(milliseconds=PRINT_EVERY_MS)
    last_print = datetime.now()

    async with websockets.connect(URI) as websocket:
        print("WebSocket has been opened")

        try:
            while True:
                # Step 1 — Wait for a state frame from the service
                msg = await websocket.recv()
                data = json.loads(msg)

                if first_message:
                    first_message = False
                    # Step 2 — Pick the first Inverse3 and prepare the first command
                    device_id = get_first_inverse3_device_id(data)
                    if not device_id:
                        print("No Inverse3 found.")
                        await websocket.close()
                        return

                    print(f"✅ Connected to device: {device_id}")

                    request_msg = {
                        # Identify this session to Haply Hub so it can recognize the simulation,
                        # let you tweak device settings (workspace, basis, mount, ...) from the
                        # Hub UI, and automatically re-apply those tweaks on the next run.
                        "session": {
                            "configure": {
                                "profile": {"name": "co.haply.inverse.tutorials:print-inverse3"}
                            }
                        },
                        "inverse3": [
                            {
                                "device_id": device_id,
                                # Zero force keeps the session ticking (state frames keep flowing)
                                # without commanding any motion.
                                "commands": {
                                    "set_cursor_force": {"vector": {"x": 0.0, "y": 0.0, "z": 0.0}}
                                },
                            }
                        ]
                    }
                else:
                    # Step 3 — Print throttled telemetry; resend the zero-force keepalive
                    now = datetime.now()
                    if now > last_print + print_delay:
                        last_print = now
                        inverse3 = data.get("inverse3")
                        if inverse3:
                            state = inverse3[0].get("state")
                            if state:
                                if full:
                                    # Raw JSON dump — handy for exploring the full payload shape.
                                    print(json.dumps(state, indent=2))
                                else:
                                    print_state(state)
                        else:
                            print("\rDevice lost. Waiting...", end="", flush=True)

                # Send the request message back to the server
                await websocket.send(json.dumps(request_msg))

                # Session profile is a one-shot handshake — strip it so subsequent
                # ticks only send the keepalive command.
                request_msg.pop("session", None)

                # When --query-config is on, ask the service to re-emit a full
                # snapshot on the next tick so `config` keeps arriving.
                if query_config:
                    request_msg["session"] = {"force_render_full_state": {}}

        except (websockets.ConnectionClosed, asyncio.CancelledError):
            print("\nWebSocket has been closed")

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--full", action="store_true",
                        help="Print the full raw JSON payload instead of the one-line summary")
    parser.add_argument("--query-config", action="store_true",
                        help="Request session.force_render_full_state on every tick so every frame includes the `config` block")
    args = parser.parse_args()
    asyncio.run(connect(args.full, args.query_config))
    asyncio.run(fanuc_main())
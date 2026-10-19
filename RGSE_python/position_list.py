import rclpy
import math
import csv
from rclpy.node import Node
import time
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class JointPositionPublisher(Node):
    def __init__(self):
        super().__init__("position_publisher")

        self.future = rclpy.task.Future()
        self.joint_names = []
        self.initial_positions = []
        self.target = []

        self.period = float(10.0)  # seconds

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
        self.joint_names = list(msg.name)
        self.initial_positions = list(msg.position)

    def publish_trajectory_list(self):

        if len(self.initial_positions) == 0:
            self.get_logger().info("Waiting for initial joint states...")
            return

        initial_positions_print = [round(val,3) for val in self.initial_positions]
        self.get_logger().info(
            f"\nCaptured initial joint states: \n {initial_positions_print}\n"
        )

        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        t = self.period
        point = JointTrajectoryPoint()
        point.time_from_start.sec = int(t)
        point.time_from_start.nanosec = int((t - int(t)) * 1e9)
        point.positions = self.target
        msg.points.append(point)

        self.traj_pub.publish(msg)

        target_print = [round(val,3) for val in self.target]
        self.get_logger().info(f"\nPublished position: \n {target_print}")
        print("----------------------------------------------------------\n")
        self.publish_timer.cancel()
        self.future.set_result(True)

####################################################33

def main():
    rclpy.init()
    
    print("------------------Position List Control---------------------")

    '''INPUTS'''
    
    
    list_name = input('\nEnter the name of the position list (with .csv): ')
    period = float(input('\nEnter the period for each motion (in seconds): '))


    '''CORE'''

    print("\n----------------------------------------------------------")

    target_list = []
    with open('/home/rgse/ws_fanuc/src/RGSE_python/position_lists/' + list_name, newline='') as csvfile:
        read_csv = csv.reader(csvfile, delimiter=',')
        for row in read_csv:
            for i in range(len(row)):
                row[i] = math.radians(float(row[i]))
            target_list.append(list(row))

    count = 0
    for target in target_list:
        count += 1
        while True:
            Confirm = input('Continue to next pos? (y): ')
            if Confirm == "Y" or Confirm == "y":
                print()
                break
            else:
                print('Wrong input\n')

        #Creating & Running Node

        print("----------------------------------------------------------")
        print(f"Position {count}\n")

        node = JointPositionPublisher()

        node.period = period
        node.target = target

        rclpy.spin_until_future_complete(node, node.future)

        node.destroy_node()

    print('\nEnd of list')

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

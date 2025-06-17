import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, NavigationResult
from geometry_msgs.msg import PoseStamped
import time

def make_pose(x, y, theta, frame_id="map"):
    from tf_transformations import quaternion_from_euler
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = rclpy.time.Time().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    q = quaternion_from_euler(0, 0, theta)
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    return pose

class GraspNavNode(Node):
    def __init__(self):
        super().__init__('grasp_nav_node')
        self.navigator = BasicNavigator()
        self.get_logger().info("导航节点已启动，等待导航栈就绪...")

        # 位姿信息
        self.init_pose = make_pose(-0.027, 0.005, 0.0)
        self.grasp_pose = make_pose(0.043, -0.070, 0.045)
        self.place_pose = make_pose(0.043, -0.070, 0.04)

        self.run_sequence()

    def run_sequence(self):
        # 1. 设置初始位姿
        self.get_logger().info("设置初始位姿")
        self.navigator.setInitialPose(self.init_pose)
        self.navigator.waitUntilNav2Active()
        time.sleep(1.0)

        # 2. 导航到抓取点
        self.get_logger().info("导航到底盘抓取点")
        self.navigator.goToPose(self.grasp_pose)
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(f"导航进度: {feedback.distance_remaining:.2f}米")
            time.sleep(0.5)
        result = self.navigator.getResult()
        if result == NavigationResult.SUCCEEDED:
            self.get_logger().info("已到达抓取点")
        else:
            self.get_logger().warn("导航到抓取点失败")
            return

        # 3. 询问是否抓取（当前直接跳过）
        self.get_logger().info("是否执行抓取操作？[回车继续，当前自动跳过]")
        # input()  # 你可以取消注释让其手动确认
        self.get_logger().info("跳过抓取，继续导航。")

        # 4. 导航到放置点
        self.get_logger().info("导航到放置点")
        self.navigator.goToPose(self.place_pose)
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(f"导航进度: {feedback.distance_remaining:.2f}米")
            time.sleep(0.5)
        result = self.navigator.getResult()
        if result == NavigationResult.SUCCEEDED:
            self.get_logger().info("已到达放置点")
        else:
            self.get_logger().warn("导航到放置点失败")
            return

        # 5. 询问是否放置（当前直接跳过）
        self.get_logger().info("是否执行放置操作？[回车继续，当前自动跳过]")
        # input()
        self.get_logger().info("流程结束。")

def main(args=None):
    rclpy.init(args=args)
    node = GraspNavNode()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

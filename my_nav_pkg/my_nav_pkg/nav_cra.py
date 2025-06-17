#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
import time

class ArmNavigation(Node):
    def __init__(self):
        super().__init__('arm_navigation_node')
        
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        
        self.get_logger().info("等待导航服务器...")
        self.nav_client.wait_for_server()
        self.get_logger().info("导航服务器已连接!")

    def set_initial_pose(self, x, y, qz=0.0, qw=1.0):
        """设置初始位姿"""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        
        # 发布多次确保接收
        for i in range(5):
            self.initial_pose_pub.publish(msg)
            time.sleep(0.5)
        
        self.get_logger().info(f"初始位姿设置完成: ({x}, {y})")
        time.sleep(2)

    def navigate_to_point(self, name, x, y, qz=0.0, qw=1.0):
        """导航到指定点 - 简化版本"""
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw
        
        self.get_logger().info(f"导航到{name}: ({x:.2f}, {y:.2f})")
        
        # 发送目标并等待
        future = self.nav_client.send_goal_async(goal)
        
        # 等待目标被接受
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'导航到{name}被拒绝!')
            return False
        
        self.get_logger().info('目标已接受，正在导航...')
        
        # 获取结果
        result_future = goal_handle.get_result_async()
        
        # 等待导航完成
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        
        result = result_future.result()
        
        # 检查状态 (4 = SUCCEEDED)
        if result.status == 4:
            self.get_logger().info(f"成功到达{name}!")
            return True
        else:
            self.get_logger().error(f"导航到{name}失败! 状态码: {result.status}")
            return False

def main():
    rclpy.init()
    node = ArmNavigation()
    
    try:
        # 设置初始位姿
        node.set_initial_pose(-0.15222, -0.0005)
        time.sleep(5)
        # 导航到抓取点
        if node.navigate_to_point("抓取点", -1.2, -0.27379, 0.99989, 0.014501):
            node.get_logger().info("执行抓取操作... TODO")
            time.sleep(3)
            node.get_logger().info("抓取完成!")
        
        # 导航到放置点
        if node.navigate_to_point("放置点", 3.5227, -0.089603, 0.033768, 0.99943):
            node.get_logger().info("执行放置操作...TODO")
            time.sleep(3)
            node.get_logger().info("放置完成!")
        
        node.get_logger().info("任务完成!")
        
    except KeyboardInterrupt:
        node.get_logger().info("程序被中断")
    except Exception as e:
        node.get_logger().error(f"发生错误: {str(e)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

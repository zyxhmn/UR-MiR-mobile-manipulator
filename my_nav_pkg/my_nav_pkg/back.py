#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
import time

class ArmNavigation(Node):
    def __init__(self):
        super().__init__('arm_navigation_node')
        
        # 初始化导航动作客户端
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 初始化初始位姿发布器
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/initialpose', 
            10
        )
        
        self.get_logger().info("导航节点已启动，等待导航服务器...")
        self.nav_client.wait_for_server()
        self.get_logger().info("导航服务器已连接!")

    def set_initial_pose(self, x, y, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        """设置初始位姿"""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        
        self.initial_pose_pub.publish(msg)
        self.get_logger().info(f"初始位姿设置完成: position=({x}, {y}, 0.0), orientation=({qx}, {qy}, {qz}, {qw})")
        time.sleep(5)  # 确保消息发布

    def navigate_to_point(self, point_name, x, y, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        """导航到指定点"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw
        
        self.get_logger().info(f"开始导航到 {point_name}...")
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"导航到 {point_name} 被拒绝!")
            return False
        
        self.get_logger().info(f"正在前往 {point_name}...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        status = result_future.result().status
        if status == 4:  # GoalStatus.STATUS_SUCCEEDED
            self.get_logger().info(f"成功到达 {point_name}!")
            return True
        else:
            self.get_logger().error(f"导航失败，状态码: {status}")
            return False

    def run_mission(self):
        """执行完整导航任务"""
        # 1. 设置初始位姿
        self.set_initial_pose(x=-0.15222, y=-0.0005, qz=0.000)
        
        # 2. 导航到抓取点
        grab_position = (-1.2,-0.27379)
        grab_orientation = (0,0, 0.99989, 0.014501)  # 四元数表示
        if self.navigate_to_point("抓取点", 
                                 x=grab_position[0], 
                                 y=grab_position[1], 
                                 qz=grab_orientation[2], 
                                 qw=grab_orientation[3]):
            input("已到达抓取点，按Enter确认执行抓取 (等待机械臂)...")
            # TODO: 此处添加机械臂抓取代码
            self.get_logger().info("抓取操作完成 (模拟)")
        
        # 3. 导航到放置点
        place_position = (3.5227,-0.089603)
        place_orientation = (0, 0, 0.033768, 0.99943)  # 四元数表示
        if self.navigate_to_point("放置点", 
                                 x=place_position[0], 
                                 y=place_position[1], 
                                 qz=place_orientation[2], 
                                 qw=place_orientation[3]):
            input("已到达放置点，按Enter确认执行放置 (等待机械臂)...")
            # TODO: 此处添加机械臂放置代码
            self.get_logger().info("放置操作完成 (模拟)")
        
        self.get_logger().info("任务完成!")

def main(args=None):
    rclpy.init(args=args)
    nav_node = ArmNavigation()
    
    # 延迟启动以确保所有连接建立
    nav_node.create_timer(1.0, lambda: nav_node.run_mission())
    
    try:
        rclpy.spin(nav_node)
    except KeyboardInterrupt:
        pass
    finally:
        nav_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
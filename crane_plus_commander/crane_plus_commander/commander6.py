import threading
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient, ActionServer, CancelResponse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from threading import Event
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from airobot_interfaces.action import StringCommand
from crane_plus_commander.kinematics import (
    from_gripper_ratio, gripper_in_range)


# 他からアクションのリクエストを受け付け，CRANE+ V2用のアクションへリクエストを送るノード
class Commander(Node):

    def __init__(self):
        super().__init__('commander')
        self.joint_names = [
            'crane_plus_joint1',
            'crane_plus_joint2',
            'crane_plus_joint3',
            'crane_plus_joint4']
        self.callback_group = ReentrantCallbackGroup()
        self.action_client_joint = ActionClient(
            self, FollowJointTrajectory,
            'crane_plus_arm_controller/follow_joint_trajectory',
            callback_group=self.callback_group)
        self.gripper_names = [
            'crane_plus_joint_hand']
        self.action_client_gripper = ActionClient(
            self, FollowJointTrajectory,
            'crane_plus_gripper_controller/follow_joint_trajectory',
            callback_group=self.callback_group)
        # 文字列とポーズの組を保持する辞書
        self.poses = {}
        self.poses['zeros'] = [0, 0, 0, 0]
        self.poses['ones'] = [1, 1, 1, 1]
        self.poses['home'] = [0.0, -1.16, -2.01, -0.73]
        self.poses['carry'] = [-0.00, -1.37, -2.52, 1.17]
        
        # アクションサーバ
        self.action_server = ActionServer(
            self,
            StringCommand,
            'manipulation/command',
            self.execute_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            callback_group=self.callback_group,
        )
        self.action_done_event = Event()
        self.server_goal_handle = None
        self.goal_lock = threading.Lock()
        self.execute_lock = threading.Lock()

        # アクションクライアント
        self.client_goal_handle = None

    def handle_accepted_callback(self, server_goal_handle):
        with self.goal_lock:
            if self.server_goal_handle is not None and self.server_goal_handle.is_active:
                self.get_logger().info('サーバ： 前の処理を中止')
                self.server_goal_handle.abort()
                self.action_done_event.set()
            self.server_goal_handle = server_goal_handle
        server_goal_handle.execute()

    def execute_callback(self, server_goal_handle):
        with self.execute_lock:
            self.get_logger().info(f'command: {server_goal_handle.request.command}')
            server_result = StringCommand.Result()
            words = server_goal_handle.request.command.split()
            if words[0] == 'set_pose':
                self.set_pose(words, server_result, server_goal_handle)
            elif words[0] == 'set_gripper':
                self.set_gripper(words, server_result, server_goal_handle)
            else:
                server_result.answer = f'NG {words[0]} not supported'
            self.get_logger().info(f'answer: {server_result.answer}')
            if server_result.answer.startswith('OK'):
                server_goal_handle.succeed()
            else:
                server_goal_handle.abort()
            return server_result

    def cancel_callback(self, server_goal_handle):
        self.get_logger().info('サーバ： キャンセル受信')
        self.action_done_event.set()
        return CancelResponse.ACCEPT

    def set_pose(self, words, server_result, server_goal_handle):
        if len(words) < 2:
            server_result.answer = f'NG {words[0]} argument required'
            return
        if not words[1] in self.poses:
            server_result.answer = f'NG {words[1]} not found'
            return
        dt = 3.0
        client_result = self.send_goal_joint(self.poses[words[1]], dt)
        if self.check_action_result(client_result, server_result, server_goal_handle):
            return
        server_result.answer = 'OK'

    def set_gripper(self, words, server_result, server_goal_handle):
        if len(words) < 2:
            server_result.answer = f'NG {words[0]} argument required'
            return
        try:
            gripper_ratio = float(words[1])
        except ValueError:
            server_result.answer = f'NG {words[1]} unsuitable'
            return
        gripper = from_gripper_ratio(gripper_ratio)
        if not gripper_in_range(gripper):
            server_result.answer = 'NG out of range'
            return
        dt = 1.0
        client_result = self.send_goal_gripper(gripper, dt)
        if self.check_action_result(client_result, server_result, server_goal_handle):
            return
        server_result.answer = 'OK'

    def check_action_result(self, client_result, server_result, server_goal_handle, message=''):
        if message != '':
            message += ' '
        if client_result is None:
            if not server_goal_handle.is_active:
                self.get_logger().warn('サーバ： 中止')
                server_result.answer = f'NG {message}aborted'
            elif server_goal_handle.is_cancel_requested:
                server_goal_handle.canceled()
                self.cancel()
                self.get_logger().warn('サーバ： キャンセル')
                server_result.answer = f'NG {message}canceled'
            else:
                self.get_logger().warn('クライアント： タイムアウト')
                server_result.answer = f'NG {message}timeout'
            return True
        if client_result.result.error_code != 0:
            server_result.answer = f'NG {message}error_code: {client_result.result.error_code}'
            return True
        return False

    def send_goal_joint(self, q, time):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = JointTrajectory()
        goal_msg.trajectory.joint_names = self.joint_names
        goal_msg.trajectory.points = [JointTrajectoryPoint()]
        goal_msg.trajectory.points[0].positions = [
            float(q[0]), float(q[1]), float(q[2]), float(q[3])]
        goal_msg.trajectory.points[0].time_from_start = Duration(
            seconds=int(time), nanoseconds=(time-int(time))*1e9).to_msg()
        self.action_done_event.clear()
        send_goal_future = self.action_client_joint.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
        self.action_result = None
        self.action_done_event.wait(time*2)
        return self.action_result

    def send_goal_gripper(self, gripper, time):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = JointTrajectory()
        goal_msg.trajectory.joint_names = self.gripper_names
        goal_msg.trajectory.points = [JointTrajectoryPoint()]
        goal_msg.trajectory.points[0].positions = [float(gripper)]
        goal_msg.trajectory.points[0].time_from_start = Duration(
            seconds=int(time), nanoseconds=(time-int(time))*1e9).to_msg()
        self.action_done_event.clear()
        send_goal_future = self.action_client_gripper.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
        self.action_result = None
        self.action_done_event.wait(time*2)
        return self.action_result

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('クライアント: サーバがゴール拒否')
            return
        self.client_goal_handle = goal_handle
        self.get_logger().info('クライアント： サーバがゴール受け付け')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.action_result = future.result()
        self.client_goal_handle = None
        self.action_done_event.set()
        self.get_logger().info('クライアント： リザルト受信')

    def cancel(self):
        if self.client_goal_handle is None:
            self.get_logger().warn('クライアント： キャンセル対象なし')
            return
        self.get_logger().info('クライアント： キャンセル送信')
        future = self.client_goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_done)

    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('クライアント： キャンセル成功')
            self.client_goal_handle = None
        else:
            self.get_logger().warn('クライアント： キャンセル失敗')


def main():
    print('開始')

    # ROSクライアントの初期化
    rclpy.init()

    # ノードクラスのインスタンス
    commander = Commander()

    # 初期ポーズへゆっくり移動させる
    commander.send_goal_joint(commander.poses['home'], 5)
    commander.send_goal_gripper(from_gripper_ratio(1), 1)
    print('サービスサーバ待機')

    # Ctrl+CでエラーにならないようにKeyboardInterruptを捕まえる
    try:
        executor = MultiThreadedExecutor()
        rclpy.spin(commander, executor)
    except KeyboardInterrupt:
        pass

    print('サービスサーバ停止')
    # 終了ポーズへゆっくり移動させる
    commander.send_goal_joint(commander.poses['zeros'], 5)
    commander.send_goal_gripper(from_gripper_ratio(0), 1)

    rclpy.try_shutdown()
    print('終了')

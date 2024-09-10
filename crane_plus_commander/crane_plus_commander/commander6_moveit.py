import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from threading import Event
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from airobot_interfaces.action import StringCommand
import threading
from pymoveit2 import MoveIt2, GripperInterface
from math import radians

GRIPPER_MIN = -radians(40.62) + 0.001
GRIPPER_MAX = radians(38.27) - 0.001

def to_gripper_ratio(gripper):
    ratio = (gripper - GRIPPER_MIN) / (GRIPPER_MAX - GRIPPER_MIN)
    return ratio

def from_gripper_ratio(ratio):
    gripper = GRIPPER_MIN + ratio * (GRIPPER_MAX - GRIPPER_MIN)
    return gripper

def gripper_in_range(gripper):
    return GRIPPER_MIN <= gripper <= GRIPPER_MAX


# 他からアクションのリクエストを受け付け，CRANE+ V2用のアクションへリクエストを送るノード
class CommanderMoveit(Node):

    def __init__(self):
        super().__init__('commander_moveit')
        self.joint_names = [
            'crane_plus_joint1',
            'crane_plus_joint2',
            'crane_plus_joint3',
            'crane_plus_joint4']
        callback_group = ReentrantCallbackGroup()
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=self.joint_names,
            base_link_name='crane_plus_base',
            end_effector_name='crane_plus_link_tcp',
            group_name='arm_tcp',
            callback_group=callback_group,
        )
        self.moveit2.planner_id = 'RRTConnectkConfigDefault'
        self.moveit2.max_velocity = 0.5
        self.moveit2.max_acceleration = 0.5
        self.cancel_after_secs = 0.0

        gripper_joint_names = ['crane_plus_joint_hand']
        self.gripper_interface = GripperInterface(
            node=self,
            gripper_joint_names=gripper_joint_names,
            open_gripper_joint_positions=[GRIPPER_MIN],
            closed_gripper_joint_positions=[GRIPPER_MAX],
            gripper_group_name='gripper',
            callback_group=callback_group,
            gripper_command_action_name='gripper_action_controller/gripper_cmd',
        )

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
            callback_group=callback_group,
        )
        self.action_done_event = Event()
        self.goal_handle = None
        self.goal_lock = threading.Lock()
        self.execute_lock = threading.Lock()

    def execute_callback(self, goal_handle):
        with self.execute_lock:
            self.get_logger().info(f'command: {goal_handle.request.command}')
            result = StringCommand.Result()
            words = goal_handle.request.command.split()
            if words[0] == 'set_pose':
                self.set_pose(words, result)
            elif words[0] == 'set_gripper':
                self.set_gripper(words, result)
            else:
                result.answer = f'NG {words[0]} not supported'
            self.get_logger().info(f'answer: {result.answer}')
            if goal_handle.is_active:
                if result.answer.startswith('OK'):
                    goal_handle.succeed()
                else:
                    goal_handle.abort()
            return result

    def set_pose(self, words, result):
        if len(words) < 2:
            result.answer = f'NG {words[0]} argument required'
            return
        if not words[1] in self.poses:
            result.answer = f'NG {words[1]} not found'
            return
        self.set_max_velocity(0.5)
        success = self.move_joint(self.poses[words[1]])
        if success:
            result.answer = 'OK'
        else:
            result.answer = f'NG {words[0]} move_joint() failed'

    def set_gripper(self, words, result):
        if len(words) < 2:
            result.answer = f'NG {words[0]} argument required'
            return
        try:
            gripper_ratio = float(words[1])
        except ValueError:
            result.answer = f'NG {words[1]} unsuitable'
            return
        gripper = from_gripper_ratio(gripper_ratio)
        if not gripper_in_range(gripper):
            result.answer = 'NG out of range'
            return
        self.set_max_velocity(0.5)
        success = self.move_gripper(gripper)
        if success:
            result.answer = 'OK'
        else:
            result.answer = f'NG {words[0]} move_gripper() failed'

    def cancel_callback(self, goal_handle):
        self.get_logger().info('サーバ： キャンセル受信')
        self.action_done_event.set()
        return CancelResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        with self.goal_lock:
            if self.goal_handle is not None and self.goal_handle.is_active:
                self.get_logger().info('サーバ： 前の処理を中止')
                self.goal_handle.abort()
                self.action_done_event.set()
            self.goal_handle = goal_handle
        goal_handle.execute()

    def move_joint(self, q):
        joint_positions = [
            float(q[0]), float(q[1]), float(q[2]), float(q[3])]
        self.moveit2.move_to_configuration(joint_positions)
        return self.moveit2.wait_until_executed()

    def move_gripper(self, q):
        position = float(q)
        self.gripper_interface.move_to_position(position)
        return self.gripper_interface.wait_until_executed()

    def set_max_velocity(self, v):
        self.moveit2.max_velocity = float(v)


def main():
    print('開始')

    # ROSクライアントの初期化
    rclpy.init()

    # ノードクラスのインスタンス
    commander = CommanderMoveit()

    # 別のスレッドでrclpy.spin()を実行する
    executor = MultiThreadedExecutor()
    thread = threading.Thread(target=rclpy.spin, args=(commander,executor,))
    threading.excepthook = lambda x: ()
    thread.start()

    # 初期ポーズへゆっくり移動させる
    commander.set_max_velocity(0.2)
    commander.move_joint(commander.poses['home'])
    commander.move_gripper(GRIPPER_MAX)
    print('サービスサーバ待機')

    # Ctrl+CでエラーにならないようにKeyboardInterruptを捕まえる
    try:
        input('Enterキーを押すと終了\n')
    except KeyboardInterrupt:
        thread.join()
    else:
        print('サービスサーバ停止')
        # 終了ポーズへゆっくり移動させる
        commander.set_max_velocity(0.2)
        commander.move_joint(commander.poses['zeros'])
        commander.move_gripper(GRIPPER_MIN)

    rclpy.try_shutdown()
    print('終了')

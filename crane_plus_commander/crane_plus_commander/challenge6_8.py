import threading
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from airobot_interfaces.action import StringCommand
import threading
from pymoveit2 import MoveIt2, GripperInterface, MoveIt2State
from math import radians, atan2

GRIPPER_MIN = -radians(40.62) + 0.001
GRIPPER_MAX = radians(38.27) - 0.001

def to_gripper_ratio(gripper):
    ratio = (gripper - GRIPPER_MIN) / (GRIPPER_MAX - GRIPPER_MIN)
    return ratio

def from_gripper_ratio(ratio):
    gripper = GRIPPER_MIN + ratio * (GRIPPER_MAX - GRIPPER_MIN)
    return gripper


# 他からアクションのリクエストを受け付け，CRANE+ V2用のMoveItへ指令を送るノード
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
            end_effector_name='crane_plus_link_endtip',
            group_name='arm',
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
        self.goal_handle = None
        self.goal_lock = threading.Lock()
        self.execute_lock = threading.Lock()

        # tf
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def execute_callback(self, goal_handle):
        with self.execute_lock:
            self.get_logger().info(f'command: {goal_handle.request.command}')
            result = StringCommand.Result()
            words = goal_handle.request.command.split()
            if words[0] == 'set_pose':
                self.set_pose(words, result)
            elif words[0] == 'set_endtip':
                self.set_endtip(words, result)
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
        if gripper_ratio < 0.0 or 1.0 < gripper_ratio:
            result.answer = 'NG out of range'
            return
        gripper = from_gripper_ratio(gripper_ratio)
        self.set_max_velocity(0.5)
        success = self.move_gripper(gripper)
        if success:
            result.answer = 'OK'
        else:
            result.answer = f'NG {words[0]} move_gripper() failed'

    def set_endtip(self, words, result):
        if len(words) < 2:
            result.answer = f'NG {words[0]} argument required'
            return
        target = words[1]
        xyzrpy = self.get_frame_position(target)
        if xyzrpy is None:
            result.answer = f'NG cannot get frame {target}'
            return
        [x, y, z] = xyzrpy[0:3]
        pitch = 0.0
        self.set_max_velocity(0.5)
        success = self.move_endtip([x, y, z, pitch])
        if success:
            result.answer = 'OK'
        else:
            result.answer = 'NG move_endip() failed'

    def get_frame_position(self, frame_id):
        try:
            when = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'crane_plus_base',
                frame_id,
                when,
                timeout=Duration(seconds=1.0))
        except TransformException as ex:
            self.get_logger().info(f'{ex}')
            return None
        t = trans.transform.translation
        r = trans.transform.rotation
        roll, pitch, yaw = euler_from_quaternion([r.x, r.y, r.z, r.w])
        return [t.x, t.y, t.z, roll, pitch, yaw]

    def cancel_callback(self, goal_handle):
        self.get_logger().info('キャンセル受信')
        self.cancel_joint_and_gripper()
        return CancelResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        with self.goal_lock:
            if self.goal_handle is not None and self.goal_handle.is_active:
                self.get_logger().info('前の処理を中止')
                self.goal_handle.abort()
                self.cancel_joint_and_gripper()
            self.goal_handle = goal_handle
        goal_handle.execute()

    def cancel_joint_and_gripper(self):
        if self.moveit2.query_state() == MoveIt2State.EXECUTING:
            self.moveit2.cancel_execution()
        if self.gripper_interface.query_state() == MoveIt2State.EXECUTING:
            self.gripper_interface.cancel_execution()

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

    def move_endtip(self, endtip):
        position = [float(endtip[0]), float(endtip[1]), float(endtip[2])]
        yaw = atan2(position[1], position[0])
        pitch = float(endtip[3])
        quat_xyzw = quaternion_from_euler(0.0, pitch, yaw)
        self.moveit2.move_to_pose(
            position=position,
            quat_xyzw=quat_xyzw
        )
        return self.moveit2.wait_until_executed()


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
    print('アクションサーバ待機')

    # Ctrl+CでエラーにならないようにKeyboardInterruptを捕まえる
    try:
        input('Enterキーを押すと終了\n')
    except KeyboardInterrupt:
        thread.join()
    else:
        print('アクションサーバ停止')
        # 終了ポーズへゆっくり移動させる
        commander.set_max_velocity(0.2)
        commander.move_joint(commander.poses['zeros'])
        commander.move_gripper(GRIPPER_MIN)

    rclpy.try_shutdown()
    print('終了')

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time
import threading
from pymoveit2 import MoveIt2
from pymoveit2 import GripperInterface
from math import radians


class MoveitTest(Node):

    def __init__(self, timer=False):
        super().__init__('moveit_test')

        joint_names = [
            'crane_plus_joint1',
            'crane_plus_joint2',
            'crane_plus_joint3',
            'crane_plus_joint4']
        callback_group = ReentrantCallbackGroup()
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=joint_names,
            base_link_name='crane_plus_base',
            end_effector_name='crane_plus_link4',
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
            open_gripper_joint_positions=[-radians(40.62) + 0.001],
            closed_gripper_joint_positions=[radians(38.27) - 0.001],
            gripper_group_name='gripper',
            callback_group=callback_group,
            gripper_command_action_name="gripper_action_controller/gripper_cmd",
        )
        self.joint_names = joint_names + gripper_joint_names

    def move_joint(self, q):
        joint_positions = [
            float(q[0]), float(q[1]), float(q[2]), float(q[3])]
        self.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions)}}}")
        self.moveit2.move_to_configuration(joint_positions)
        self.moveit2.wait_until_executed()

    def open_gripper(self):
        self.get_logger().info('Performing gripper action "open"')
        self.gripper_interface.open()
        self.gripper_interface.wait_until_executed()
         
    def close_gripper(self):
        self.get_logger().info('Performing gripper action "close"')
        self.gripper_interface.close()
        self.gripper_interface.wait_until_executed()

    def get_joint(self):
        msg = self.moveit2.joint_state
        self.get_logger().info(f'{msg=}')
        d = {}
        for i, name in enumerate(msg.name):
            d[name] = msg.position[i]
        joint = [d[x] for x in self.joint_names]
        return joint

def main():
    # ROSクライアントの初期化
    rclpy.init()

    # ノードクラスのインスタンス
    node = MoveitTest()

    executor = MultiThreadedExecutor()

    # 別のスレッドでrclpy.spin()を実行する
    thread = threading.Thread(target=rclpy.spin, args=(node,executor,))
    threading.excepthook = lambda x: ()
    thread.start()

    # 最初の指令をパブリッシュする前に少し待つ
    time.sleep(1.0)

    # 文字列とポーズの組を保持する辞書
    goals = {}
    goals['zeros'] = [0, 0, 0, 0]
    goals['ones'] = [1, 1, 1, 1]
    goals['home'] = [0.0, -1.16, -2.01, -0.73]
    goals['carry'] = [-0.00, -1.37, -2.52, 1.17]

    # 初期ポーズへゆっくり移動させる
    joint = [0.0, 0.0, 0.0, 0.0]
    node.move_joint(joint)
    node.open_gripper()

    # Ctrl+CでエラーにならないようにKeyboardInterruptを捕まえる
    try:
        while True:
            for key, item in goals.items():
                print(f'{key:8} {item}')
            name = input('目標の名前を入力: ')
            if name == '':
                break
            if name not in goals:
                print(f'{name}は登録されていません')
                continue

            print('目標を送って結果待ち…')
            node.move_joint(goals[name])
            j = node.get_joint()
            print(f'[{j[0]:.2f}, {j[1]:.2f}, {j[2]:.2f}, {j[3]:.2f}, {j[4]:.2f}]]')
            print('')
    except KeyboardInterrupt:
        thread.join()
    else:
        # 終了ポーズへゆっくり移動させる
        joint = [0.0, 0.0, 0.0, 0.0]
        node.move_joint(joint)
        node.close_gripper()

    rclpy.try_shutdown()
    print('終了')

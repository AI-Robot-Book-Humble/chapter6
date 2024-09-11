import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time
import threading
from crane_plus_commander.kbhit import KBHit
from pymoveit2 import MoveIt2, GripperInterface
from math import radians, atan2
from tf_transformations import euler_from_quaternion, quaternion_from_euler

GRIPPER_MIN = -radians(40.62) + 0.001
GRIPPER_MAX = radians(38.27) - 0.001

def to_gripper_ratio(gripper):
    ratio = (gripper - GRIPPER_MIN) / (GRIPPER_MAX - GRIPPER_MIN)
    return ratio

def from_gripper_ratio(ratio):
    gripper = GRIPPER_MIN + ratio * (GRIPPER_MAX - GRIPPER_MIN)
    return gripper


# CRNAE+ V2用のMoveItで逆運動学を計算し関節値の指令を送るノード
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

        self.object_id = 'table_top'
        self.moveit2.add_collision_box(
            id=self.object_id, size=[1.0, 1.0, 0.01],
            position=[0.0, 0.0, 0.0], quat_xyzw=[0.0, 0.0, 0.0, 1.0])

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

    def forward_kinematics(self, joint):
        pose_stamped = self.moveit2.compute_fk(joint)
        p = pose_stamped.pose.position
        [x, y, z] = [p.x, p.y, p.z]
        q = pose_stamped.pose.orientation
        [_, pitch,_] = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return [x, y, z, pitch]

    def inverse_kinematics(self, endtip, elbow_up):
        [x, y, z, pitch] = endtip
        position = [x, y, z]
        yaw = atan2(y, x)
        quat_xyzw = quaternion_from_euler(0.0, pitch, yaw)
        if elbow_up:
            start_joint_state = [0.0, 0.0, -1.57, 0.0]
        else:
            start_joint_state = [0.0, 0.0, 1.57, 0.0]
        joint_state = self.moveit2.compute_ik(
            position, quat_xyzw, start_joint_state=start_joint_state)
        if joint_state is None:
            return None
        d = {}
        for i, name in enumerate(joint_state.name):
            d[name] = joint_state.position[i]
        joint = [d[x] for x in self.joint_names]
        return joint


def main():
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
    joint = [0.0, -1.16, -2.01, -0.73]
    gripper = 0
    commander.set_max_velocity(0.2)
    commander.move_joint(joint)
    commander.move_gripper(gripper)

    # 逆運動学の解の種類
    elbow_up = True

    # キー読み取りクラスのインスタンス
    kb = KBHit()

    print('1, 2, 3, 4, 5, 6, 7, 8, 9, 0キーを押して関節を動かす')
    print('a, z, s, x, d, c, f, v, g, bキーを押して手先を動かす')
    print('eキーを押して逆運動学の解を切り替える')
    print('スペースキーを押して起立状態にする')
    print('Escキーを押して終了')

    # Ctrl+CでエラーにならないようにKeyboardInterruptを捕まえる
    try:
        while True:
            # 順運動学
            [x, y, z, pitch] = commander.forward_kinematics(joint)
            ratio = to_gripper_ratio(gripper)

            # 変更前の値を保持
            joint_prev = joint.copy()
            gripper_prev = gripper
            elbow_up_prev = elbow_up

            commander.set_max_velocity(1.0)

            # キーが押されているか？
            if kb.kbhit():
                c = kb.getch()
                # 押されたキーによって場合分けして処理
                if c == '1':
                    joint[0] -= 0.1
                elif c == '2':
                    joint[0] += 0.1
                elif c == '3':
                    joint[1] -= 0.1
                elif c == '4':
                    joint[1] += 0.1
                elif c == '5':
                    joint[2] -= 0.1
                elif c == '6':
                    joint[2] += 0.1
                elif c == '7':
                    joint[3] -= 0.1
                elif c == '8':
                    joint[3] += 0.1
                elif c == '9':
                    gripper -= 0.1
                elif c == '0':
                    gripper += 0.1
                elif c == 'a':
                    x += 0.01
                elif c == 'z':
                    x -= 0.01
                elif c == 's':
                    y += 0.01
                elif c == 'x':
                    y -= 0.01
                elif c == 'd':
                    z += 0.01
                elif c == 'c':
                    z -= 0.01
                elif c == 'f':
                    pitch += 0.1
                elif c == 'v':
                    pitch -= 0.1
                elif c == 'g':
                    ratio += 0.1
                elif c == 'b':
                    ratio -= 0.1
                elif c == 'e':
                    elbow_up = not elbow_up
                    print(f'elbow_up: {elbow_up}')
                    commander.set_max_velocity(0.2)
                elif c == ' ':  # スペースキー
                    joint = [0.0, 0.0, 0.0, 0.0]
                    gripper = 0
                    commander.set_max_velocity(0.2)
                elif ord(c) == 27:  # Escキー
                    break

                # 逆運動学
                if c in 'azsxdcfve':
                    print('inverse_kinematics() ...', end='')
                    joint = commander.inverse_kinematics([x, y, z, pitch], elbow_up)
                    print(' done')
                    if joint is None:
                        print('逆運動学の解なし')
                        joint = joint_prev.copy()
                        elbow_up = elbow_up_prev
                elif c in 'gb':
                    gripper = from_gripper_ratio(ratio)

                if not (GRIPPER_MIN <= gripper <= GRIPPER_MAX):
                    print('グリッパ指令値が範囲外')
                    gripper = gripper_prev

                # 変化があれば指令を送る
                if joint != joint_prev:
                    print((f'joint: [{joint[0]:.2f}, {joint[1]:.2f}, '
                           f'{joint[2]:.2f}, {joint[3]:.2f}]'))
                    success = commander.move_joint(joint)
                    if not success:
                        print('move_joint()失敗')
                        joint = joint_prev.copy()
                if gripper != gripper_prev:
                    print(f'gripper: {gripper:.2f}')
                    success = commander.move_gripper(gripper)
                    if not success:
                        print('move_gripper()失敗')
                        gripper = gripper_prev

            time.sleep(0.01)
    except KeyboardInterrupt:
        thread.join()
    else:
        print('終了')
        # 終了ポーズへゆっくり移動させる
        joint = [0.0, 0.0, 0.0, 0.0]
        gripper = 0
        commander.set_max_velocity(0.2)
        commander.move_joint(joint)
        commander.move_gripper(gripper)

    rclpy.try_shutdown()

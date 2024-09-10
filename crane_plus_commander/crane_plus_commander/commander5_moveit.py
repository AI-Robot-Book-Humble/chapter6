import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion
import time
import threading
from math import sqrt, radians, atan2
from crane_plus_commander.kbhit import KBHit
from pymoveit2 import MoveIt2, GripperInterface
import tf_transformations as tft

GRIPPER_MIN = -radians(40.62) + 0.001
GRIPPER_MAX = radians(38.27) - 0.001

# CRNAE+ V2用のMoveItで逆運動学を計算しtfのフレームで与えられた点へ手先を位置決めするノード
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

        # tf
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def move_joint(self, q):
        joint_positions = [
            float(q[0]), float(q[1]), float(q[2]), float(q[3])]
        self.moveit2.move_to_configuration(joint_positions)
        return self.moveit2.wait_until_executed()

    def move_endtip(self, endtip):
        position = [float(endtip[0]), float(endtip[1]), float(endtip[2])]
        yaw = atan2(position[1], position[0])
        pitch = float(endtip[3])
        quat_xyzw = tft.quaternion_from_euler(0.0, pitch, yaw)
        self.moveit2.move_to_pose(
            position=position,
            quat_xyzw=quat_xyzw
        )
        return self.moveit2.wait_until_executed()

    def open_gripper(self):
        self.gripper_interface.open()
        self.gripper_interface.wait_until_executed()
         
    def close_gripper(self):
        self.gripper_interface.close()
        self.gripper_interface.wait_until_executed()

    def set_max_velocity(self, v):
        self.moveit2.max_velocity = float(v)

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


# リストで表された3次元座標かんの距離を計算する
def dist(a, b):
    return sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2 + (a[2] - b[2])**2)

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
    joint = [0.0, -1.16, -2.01, -0.73]
    commander.set_max_velocity(0.2)
    commander.move_joint(joint)
    commander.open_gripper()

    # 逆運動学の解の種類
    elbow_up = True

    # キー読み取りクラスのインスタンス
    kb = KBHit()

    # 状態
    INIT = 0
    WAIT = 1
    DONE = 2
    state = INIT

    # Ctrl+CでエラーにならないようにKeyboardInterruptを捕まえる
    try:
        while True:
            # キーが押されているか？
            if kb.kbhit():
                c = kb.getch()
                if ord(c) == 27:  # Escキー
                    break

            position = commander.get_frame_position('target')
            if position is None:
                print('対象のフレームが見つからない')
            else:
                xyz_now = position[0:3]
                time_now = time.time()
                if state == INIT:
                    xyz_first = xyz_now
                    time_first = time_now
                    state = WAIT
                elif state == WAIT:
                    if dist(xyz_now, xyz_first) > 0.01:
                        state = INIT
                    elif time_now - time_first > 1.0:
                        commander.set_max_velocity(1.0)
                        pitch = 0
                        sucess = commander.move_endtip(xyz_now + [pitch])
                        if not sucess:
                            print('move_endtip()失敗')
                        else:
                            print('move_endtip()成功')
                            state = DONE
                elif state == DONE:
                    if dist(xyz_now, xyz_first) > 0.01:
                        state = INIT
            time.sleep(0.01)            
    except KeyboardInterrupt:
        thread.join()
    else:
        # 終了ポーズへゆっくり移動させる
        joint = [0.0, 0.0, 0.0, 0.0]
        commander.set_max_velocity(0.2)
        commander.move_joint(joint)
        commander.close_gripper()

    rclpy.try_shutdown()
    print('終了')
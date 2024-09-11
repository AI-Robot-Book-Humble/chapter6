import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
import time
import threading
from crane_plus_commander.kbhit import KBHit
from crane_plus_commander.kinematics import gripper_in_range, joint_in_range


# CRANE+ V2用のトピックへ指令をパブリッシュしノードに
# CRANE+ V2用のアクションへリクエストを送る機能を追加
# さらにポーズの登録機能も追加
class Commander(Node):

    def __init__(self):
        super().__init__('commander')
        self.joint_names = [
            'crane_plus_joint1',
            'crane_plus_joint2',
            'crane_plus_joint3',
            'crane_plus_joint4']
        self.publisher_joint = self.create_publisher(
            JointTrajectory,
            'crane_plus_arm_controller/joint_trajectory', 10)
        self.publisher_gripper = self.create_publisher(
            JointTrajectory,
            'crane_plus_gripper_controller/joint_trajectory', 10)
        self.action_client_joint = ActionClient(
            self, FollowJointTrajectory,
            'crane_plus_arm_controller/follow_joint_trajectory')

    def publish_joint(self, q, time):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        msg.points = [JointTrajectoryPoint()]
        msg.points[0].positions = [
            float(q[0]), float(q[1]), float(q[2]), float(q[3])]
        msg.points[0].time_from_start = Duration(
            seconds=int(time), nanoseconds=(time-int(time))*1e9).to_msg()
        self.publisher_joint.publish(msg)

    def publish_gripper(self, gripper, time):
        msg = JointTrajectory()
        msg.joint_names = ['crane_plus_joint_hand']
        msg.points = [JointTrajectoryPoint()]
        msg.points[0].positions = [float(gripper)]
        msg.points[0].time_from_start = Duration(
            seconds=int(time), nanoseconds=(time-int(time))*1e9).to_msg()
        self.publisher_gripper.publish(msg)

    def send_goal_joint(self,  q, time):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = JointTrajectory()
        goal_msg.trajectory.joint_names = self.joint_names
        goal_msg.trajectory.points = [JointTrajectoryPoint()]
        goal_msg.trajectory.points[0].positions = [
            float(q[0]), float(q[1]), float(q[2]), float(q[3])]
        goal_msg.trajectory.points[0].time_from_start = Duration(
            seconds=int(time), nanoseconds=(time-int(time))*1e9).to_msg()
        self.action_client_joint.wait_for_server()
        return self.action_client_joint.send_goal(goal_msg)


# 数値のリストの小数点以下の桁数を揃えて文字列に変換
def list_to_str(x):
    return '[' + ', '.join([f'{i:.2f}' for i in x]) + ']'


def main():

    # アクション通信で入力されたポーズ名の指令を送る（関数内関数）
    def send_pose():
        # 登録内容の一覧表示
        for n, j in goals.items():
            print(f'{n:8}{list_to_str(j)}')
        while True:
            kb.set_normal_term()  # キー入力を標準に戻す
            name = input('目標の名前を入力: ')
            kb.set_term()  # キー入力をブロックなしに設定
            if name == '':
                print('中止します')
                return None
            elif name not in goals:
                print(f'{name}は登録されていません')
            else:
                break
        print('目標を送って結果待ち…')
        dt = 3.0
        r = commander.send_goal_joint(goals[name], dt)
        print(f'r.result.error_code: {r.result.error_code}')
        # 辞書の要素の参照ではなくコピーを返す
        return goals[name].copy()

    # 現在のポーズに名前を付けて登録する（関数内関数）
    def register_pose():
        print('現在のポーズを登録します')
        while True:
            kb.set_normal_term()  # キー入力を標準に戻す
            name = input('新たなポーズの名前を入力: ')
            kb.set_term()  # キー入力をブロックなしに設定
            if name == '':
                print('中止します')
                return
            if name not in goals:
                break
            print(f'{name}は既に登録されています')
        goals[name] = joint.copy()
        print(f'{name}: {list_to_str(goals[name])}を登録しました')

    # ROSクライアントの初期化
    rclpy.init()

    # ノードクラスのインスタンス
    commander = Commander()

    # 別のスレッドでrclpy.spin()を実行する
    thread = threading.Thread(target=rclpy.spin, args=(commander,))
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
    gripper = 0
    dt = 5
    commander.publish_joint(joint, dt)
    commander.publish_gripper(gripper, dt)

    # キー読み取りクラスのインスタンス
    kb = KBHit()

    print('1, 2, 3, 4, 5, 6, 7, 8, 9, 0キーを押して関節を動かす')
    print('スペースキーを押して起立状態にする')
    print('pキー： ポーズ名を入力してアクション通信で指令を送る')
    print('rキー： 現在のポーズに名前を付けて登録')
    print('Escキーを押して終了')

    # Ctrl+CでエラーにならないようにKeyboardInterruptを捕まえる
    try:
        while True:
            # 変更前の値を保持
            joint_prev = joint.copy()
            gripper_prev = gripper

            # 目標関節値とともに送る目標時間
            dt = 0.2

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
                elif c == ' ':  # スペースキー
                    joint = [0.0, 0.0, 0.0, 0.0]
                    gripper = 0
                    dt = 1.0
                elif ord(c) == 27:  # Escキー
                    break
                elif c == 'p':
                    j = send_pose()
                    if j is not None:
                        joint = j
                elif c == 'r':
                    register_pose()

                # 指令値を範囲内に収める
                if not all(joint_in_range(joint)):
                    print('関節指令値が範囲外')
                    joint = joint_prev.copy()
                if not gripper_in_range(gripper):
                    print('グリッパ指令値が範囲外')
                    gripper = gripper_prev

                # 変化があればパブリッシュ
                publish = False
                if joint != joint_prev:
                    print(f'joint: {list_to_str(joint)}')
                    commander.publish_joint(joint, dt)
                    publish = True
                if gripper != gripper_prev:
                    print(f'gripper: {gripper:.2f}')
                    commander.publish_gripper(gripper, dt)
                    publish = True
                # パブリッシュした場合は設定時間の分停止
                if publish:
                    time.sleep(dt)
            time.sleep(0.01)
    except KeyboardInterrupt:
        thread.join()
    else:
        print('終了')
        # 終了ポーズへゆっくり移動させる
        joint = [0.0, 0.0, 0.0, 0.0]
        gripper = 0
        dt = 5
        commander.publish_joint(joint, dt)
        commander.publish_gripper(gripper, dt)

    rclpy.try_shutdown()

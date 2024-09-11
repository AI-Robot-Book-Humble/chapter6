# CRANE+ V2用のROS2ノード群を利用する簡単なノード（Humble版）

## 概要

- アールティ社が公開している同社のロボットアーム[CRANE+ V2用のROS 2ノード群 crane_plus](https://github.com/rt-net/crane_plus)を利用するノード．
- 実機とシミュレーションの両方に対応．
- ノードのプログラムは，すべてPythonで記述．
- MoveItは使わずに各関節へ指令値を送るものと，MoveItを利用するものがある．
- MoveIt 2のPythonインタフェースとして[pymoveit2](https://github.com/AndrejOrsula/pymoveit2)を利用．
- crane_plusは，[アールティ社のオリジナル](https://github.com/rt-net/crane_plus)ではなく，そこから[フォークしたもの](https://github.com/AI-Robot-Book-Humble/crane_plus)を使う．
- Ubuntu 22.04, ROS Humbleで作成・確認．

## 準備

- CRANE+ V2の実機を使う場合には，[crane_plus_controlのREADME](https://github.com/rt-net/crane_plus/blob/master/crane_plus_control/README.md)に沿って設定を行う．  
  - 要点
    - 1 USB通信ポートの設定（`sudo chmod 666 /dev/ttyUSB0`など）
    - 2 USB通信ポートのlatency_timerの変更
    - 3 ロボットの各アクチュエータのReturn Delay Timeの設定
  - 1と2については，`/etc/udev/rules.d`に設定ファイルを追加すると，USBに接続するたびに自動的に設定される
  （[詳しくはROBOTIS社のサイトを参照](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/#copy-rules-file)）．
  - Linuxをホストにして，この本のDockerイメージを利用する場合は，ホスト側でCRANE+ V2と接続する設定と動作確認を行う．

## インストール

- ROSのワークスペースを`~/airobot_ws`とする．
  ```
  cd ~/airobot_ws/src
  ```

- crane_plusパッケージは，アールティ社のものではなく，[AI-Robot-Book-Humbleにフォークしたもの](https://github.com/AI-Robot-Book-Humble/crane_plus)をクローンする．
  ```
  git clone https://github.com/AI-Robot-Book-Humble/crane_plus.git
  ```

- [crane_plusのREADME](https://github.com/AI-Robot-Book-Humble/crane_plus/blob/master/README.md)に沿って作業する．
  ```
  rosdep install -r -y -i --from-paths .
  cd ~/airobot_ws
  colcon build --symlink-install
  source install/setup.bash
  ```

- [pymoveit2](https://github.com/AndrejOrsula/pymoveit2)パッケージを入手し，ビルド
  ```
  cd ~/airobot_ws/src
  git clone https://github.com/AndrejOrsula/pymoveit2.git
  rosdep install -y -r -i --rosdistro ${ROS_DISTRO} --from-paths .
  cd ~/airobot_ws
  colcon build --merge-install --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"
  source install/setup.bash
  ```

- このパッケージを含むリポジトリを入手
  ```
  cd ~/airobot_ws/src
  git clone https://github.com/AI-Robot-Book-Humble/chapter6.git
  ```

- アクションのインタフェースを定義しているパッケージを含むリポジトリを入手
  ```
  git clone https://github.com/AI-Robot-Book-Humble/chapter2.git
  ```

- パッケージをビルド
  ```
  cd ~/airobot_ws
  colcon build --symlink-install --packages-select airobot_interfaces crane_plus_commander
  source install/setup.bash
  ```

## 実行（MoveItなし）

- 端末1
  - オーバレイの設定
    ```
    cd ~/airobot_ws
    source install/setup.bash
    ```

  - 実機の場合
    ```
    ros2 launch crane_plus_examples no_moveit_demo.launch.py
    ```
  - 実機の代わりIgnition Gazeboを使う場合
    ```
    ros2 launch crane_plus_gazebo no_moveit_crane_plus_with_table.launch.py 
    ```

- 端末2
  - オーバレイの設定
    ```
    cd ~/airobot_ws
    source install/setup.bash
    ```
  - キー操作で関節値を変化させる場合
    ```
    ros2 run crane_plus_commander commander1
    ```

  - キー操作で手先位置も変化させる場合
    ```
    ros2 run crane_plus_commander commander2
    ```

  - キー操作で関節値を変化させつつ，関節の状態を表示する場合
    ```
    ros2 run crane_plus_commander commander3
    ```

  - 同期的な（結果を待つ）アクションクライアントの場合
    ```
    ros2 run crane_plus_commander commander4
    ```

  - tfのフレームで与えられた点へ手先を位置決めする場合
    ```
    ros2 run crane_plus_commander commander5
    ```

## 実行（MoveItあり）

- 端末1
  - オーバレイの設定
    ```
    cd ~/airobot_ws
    source install/setup.bash
    ```

  - 実機の場合
    ```
    ros2 launch crane_plus_examples endtip_demo.launch.py 
    ```
  - 実機の代わりIgnition Gazeboを使う場合
    ```
    ros2 launch crane_plus_gazebo endtip_crane_plus_with_table.launch.py 
    ```

- 端末2
  - オーバレイの設定
    ```
    cd ~/airobot_ws
    source install/setup.bash
    ```
  - キー操作で手先位置も変化させる場合（MoveIt利用）
    ```
    ros2 run crane_plus_commander commander2_moveit
    ```

  - tfのフレームで与えられた点へ手先を位置決めする場合（MoveIt利用）
    ```
    ros2 run crane_plus_commander commander5_moveit
    ```

  - アクションサーバとして使う場合（MoveIt利用）
    ```
    ros2 run crane_plus_commander commander6
    ```

- 端末3（アクションサーバをテストする場合）
  - テスト用のクライアント
    ```
    cd ~/airobot_ws
    source install/setup.bash
    ros2 run crane_plus_commander test_client
    ```

## ヘルプ

## 著者

升谷 保博

## 履歴

- 2024-09-15: MoveItの導入など
- 2023-10-15: ROS Humbleに対応
- 2022-08-23: ライセンス・ドキュメントの整備

## ライセンス

Copyright (c) 2022, 2024 MASUTANI Yasuhiro  
All rights reserved.  
This project is licensed under the Apache License 2.0 license found in the LICENSE file in the root directory of this project.

## 参考文献

- https://github.com/rt-net/crane_plus
- https://github.com/AndrejOrsula/pymoveit2

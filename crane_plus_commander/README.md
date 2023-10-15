# CRANE+ V2用のROS2ノード群を利用する簡単なノード（Humble版）

## 概要

- アールティ社が公開している同社のロボットアーム[CRANE+ V2用のROS2ノード群 crane_plus](https://github.com/rt-net/crane_plus)を利用するノード．
- ノードのプログラムは，Pythonで記述．
- MoveIt2は使わずに，各関節へ指令値を送る．
- [crane_plus_ignition](https://github.com/rt-net/crane_plus/tree/master/crane_plus_ignition)を使うことによってIgnition Gazebo内のCRANE+ V2を同じように動かすこともできる．
- Ubuntu 22.04, ROS Humbleで作成・確認

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

- crane_plusパッケージをインストールする．[crane_plusのREADME](https://github.com/rt-net/crane_plus/blob/master/README.md)に沿って作業する．

- ROSのワークスペースを`~/airobot_ws`とする．
  ```
  cd ~/airobot_ws/src
  ```

- このパッケージを含むリポジトリを入手
  ```
  git clone https://github.com/AI-Robot-Book-Humble/chapter6
  ```

- サービスのインタフェースを定義しているパッケージを含むリポジトリを入手
  ```
  git clone https://github.com/AI-Robot-Book-Humble/chapter2
  ```

- パッケージをビルド
  ```
  cd ~/airobot_ws
  colcon build --packages-select airobot_interfaces crane_plus_commander
  ```

## 実行


- 端末1
  - オーバレイの設定
    ```
    cd ~/airobot_ws
    source install/setup.bash
    ```

  - 実機の場合（robot_state_publisher付き）
    ```
    ros2 launch crane_plus_commander crane_plus_control_rsp.launch.py
    ```
  - 実機の代わりIgnition Gazeboを使う場合
    ```
    ros2 launch crane_plus_commander crane_plus_gazebo_no_moveit.launch.py 
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

  - tfを利用して順運動学の計算をする場合
    ```
    ros2 run crane_plus_commander commander5
    ```

  - アクションクライアント＋サービスサーバとして使う場合
    ```
    ros2 run crane_plus_commander commander6
    ```

- 端末3（サービスサーバをテストする場合）
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

- 2023-10-15: ROS Humbleに対応
- 2022-08-23: ライセンス・ドキュメントの整備

## ライセンス

Copyright (c) 2022, 2023 MASUTANI Yasuhiro  
All rights reserved.  
This project is licensed under the Apache License 2.0 license found in the LICENSE file in the root directory of this project.

## 参考文献

- https://github.com/rt-net/crane_plus

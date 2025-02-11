# 簡単な2自由度ロボットアームのモデル（改訂第2版）

## 概要

- ロボットアームがどのようなものかを知るためのもの
- URDFの見本
- RVizでロボットモデルを表示する見本

## インストール

- ROSのワークスペースを`~/airobot_ws`とする．
  ```
  cd ~/airobot_ws/src
  ```

- このパッケージを含むリポジトリを入手
  ```
  git clone https://github.com/AI-Robot-Book-Humble/chapter6
  ```

- パッケージのビルド
  ```
  sudo apt install ros-humble-joint-state-publisher-gui
  cd ~/airobot_ws
  colcon build --packages-select simple_arm_description
  ```

## 実行

- 端末で以下を実行
  ```
  source install/setup.bash
  ros2 launch simple_arm_description display.launch.py
  ```
- `joint_state_publisher_gui`のウインドウのスライダを操作．

## ヘルプ

## 著者

升谷 保博

## 履歴

- 2023-10-13: ROS Humbleで動作確認
- 2022-08-23: ライセンス・ドキュメントの整備

## ライセンス

Copyright (c) 2022-2025, MASUTANI Yasuhiro  
All rights reserved.  
This project is licensed under the Apache License 2.0 license found in the LICENSE file in the root directory of this project.

## 参考文献

# Physical AI Ball Launcher
ROS 2 and Gazebo ball launcher that learns from landing errors.

失敗から学習するPhysical AI投球ロボットです。

ROS 2とGazeboを用いてボールを投射し、目標位置と着地点の
X・Y方向の誤差から、次の投射力を補正します。

## Features

- X・Y方向の初期投射力をランダムに設定
- 着地誤差を使ったオンライン補正
- 通常学習率: 0.5
- 初回成功後の精度向上学習率: 0.2
- 10回の投球を継続
- 成功判定: 2D中心誤差 0.07 m以下
- ROS 2 Humble / Gazebo Classic 11
- Mac Apple M1上のDocker環境で実行

## Files

- `learn_2d.py`: 2次元自動学習・投球プログラム
- `predict_force.py`: 投射力の予測プログラム
- `ai_ball_launcher.world`: Gazeboワールド
- `results/`: 実験結果とコンソールログ

## Run

Gazeboを起動します。

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 launch gazebo_ros gazebo.launch.py \
  world:=/home/ubuntu/ai_ball_launcher.world

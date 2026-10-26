#!/usr/bin/env python3

import sys
import numpy as np

# 学習用データ
# force_x: X方向へ加えた力
# landing_x: ログから抽出した近似着地点
force_x = np.array([10.0, 15.0, 20.0])
landing_x = np.array([0.3655, 1.0240, 1.6830])

# 着地点 = a × 力 + b の線形モデルを学習
a, b = np.polyfit(force_x, landing_x, 1)

# コマンドラインに目標値があれば使用
# 指定がなければ X=1.30 m を使用
target_x = float(sys.argv[1]) if len(sys.argv) > 1 else 1.30

# target_x = a × predicted_force + b を逆算
predicted_force = (target_x - b) / a
predicted_landing = a * predicted_force + b

print("=== Learning data ===")
for force, landing in zip(force_x, landing_x):
    print(f"force={force:.2f} N -> landing_x={landing:.4f} m")

print()
print("=== Learned model ===")
print(f"landing_x = {a:.6f} * force_x + {b:.6f}")

print()
print("=== Prediction ===")
print(f"target_x         : {target_x:.4f} m")
print(f"predicted_force  : {predicted_force:.4f} N")
print(f"predicted_landing: {predicted_landing:.4f} m")

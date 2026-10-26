#!/usr/bin/env python3

import csv
import math
import os
import random
import subprocess
import sys
import time
from pathlib import Path

# ============================================================
# 動画・課題用に変更してよい設定
# ============================================================

TARGET_X = 3.84514
TARGET_Y = 0.34887

LEARNING_RATE = 0.5
PRECISION_LEARNING_RATE = 0.2
SUCCESS_RADIUS = 0.07
MAX_TRIALS = 10

# 初期力を選ぶ乱数範囲
INITIAL_FORCE_X_RANGE = (20.0, 26.0)
INITIAL_FORCE_Y_RANGE = (3.0, 7.0)

# 動画を撮り直しても同じ初期値を再現できるよう固定
RANDOM_SEED = 42

# 上向きの力と適用時間は固定
FORCE_Z = 10.0
FORCE_DURATION_NSEC = 200_000_000

# 校正実験から求めた「1 Nあたりの着地点移動量」
DISTANCE_PER_FORCE_X = 0.13175
DISTANCE_PER_FORCE_Y = 0.13160

# 安全のため投射力の範囲を制限
FORCE_X_LIMIT = (0.0, 50.0)
FORCE_Y_LIMIT = (-15.0, 15.0)

# 球の中心高さがこの値以下なら着地付近とみなす
LANDING_Z = 0.505

OUTPUT_DIR = Path("/home/ubuntu/tamain_ai/results")
OUTPUT_CSV = OUTPUT_DIR / "learning_results.csv"


def run_command(command, timeout=20, show_output=False):
    """外部コマンドを実行し、失敗したら停止する。"""
    result = subprocess.run(
        command,
        text=True,
        capture_output=True,
        timeout=timeout,
    )

    if show_output and result.stdout:
        print(result.stdout.strip())

    if result.returncode != 0:
        print("ERROR: command failed", file=sys.stderr)
        print(" ".join(command), file=sys.stderr)
        print(result.stdout, file=sys.stderr)
        print(result.stderr, file=sys.stderr)
        sys.exit(1)

    return result.stdout


def call_empty_service(service_name):
    """pause、reset、unpauseなど空のROSサービスを呼び出す。"""
    output = run_command(
        [
            "timeout",
            "15s",
            "ros2",
            "service",
            "call",
            service_name,
            "std_srvs/srv/Empty",
            "{}",
        ],
        timeout=18,
    )

    if "response:" not in output:
        print(f"ERROR: no response from {service_name}")
        print(output)
        sys.exit(1)


def apply_force(force_x, force_y):
    """球へX・Y・Z方向の力を0.2秒間設定する。"""
    request = (
        "{"
        "link_name: unit_sphere::link, "
        'reference_frame: "", '
        "reference_point: {x: 0.0, y: 0.0, z: 0.0}, "
        "wrench: {"
        f"force: {{x: {force_x:.6f}, y: {force_y:.6f}, z: {FORCE_Z:.6f}}}, "
        "torque: {x: 0.0, y: 0.0, z: 0.0}"
        "}, "
        "start_time: {sec: 0, nanosec: 0}, "
        f"duration: {{sec: 0, nanosec: {FORCE_DURATION_NSEC}}}"
        "}"
    )

    output = run_command(
        [
            "timeout",
            "15s",
            "ros2",
            "service",
            "call",
            "/apply_link_wrench",
            "gazebo_msgs/srv/ApplyLinkWrench",
            request,
        ],
        timeout=18,
    )

    if "success=True" not in output:
        print("ERROR: apply_link_wrench was not successful")
        print(output)
        sys.exit(1)


def get_ball_position(max_attempts=3):
    """
    Gazeboから球の現在位置を取得する。
    一時的な応答遅延に備えて最大3回再試行する。
    """
    last_error = None

    for attempt in range(1, max_attempts + 1):
        try:
            output = run_command(
                ["gz", "model", "-m", "unit_sphere", "-p"],
                timeout=15,
            ).strip()

            values = output.split()

            if len(values) < 3:
                raise RuntimeError(
                    f"Unexpected pose output: {output}"
                )

            return (
                float(values[0]),
                float(values[1]),
                float(values[2]),
            )

        except Exception as error:
            last_error = error
            print(
                f"Pose read retry {attempt}/{max_attempts}: "
                f"{error}",
                flush=True,
            )
            time.sleep(0.5)

    raise RuntimeError(
        f"Failed to read ball position after "
        f"{max_attempts} attempts: {last_error}"
    )


def advance_to_landing():
    """
    シミュレーションを一時停止したまま758ステップ進める。
    ball_drop.worldのstep sizeは1 msなので、約0.758秒に相当する。
    """
    run_command(
        ["gz", "world", "-m", "758"],
        timeout=15,
    )


def get_landing_position():
    """
    物理演算を接触付近まで進めた後、球の位置を1回取得する。
    """
    advance_to_landing()
    return get_ball_position()


def clamp(value, lower, upper):
    """値を安全範囲内に制限する。"""
    return max(lower, min(upper, value))


def print_trial(
    trial,
    force_x,
    force_y,
    landing_x,
    landing_y,
    error_x,
    error_y,
    distance_error,
    result,
):
    print()
    print("=" * 58)
    print(f"TRIAL {trial}")
    print("=" * 58)
    print(f"Force       : Fx={force_x:8.4f} N, Fy={force_y:8.4f} N")
    print(f"Target      : X ={TARGET_X:8.4f} m, Y ={TARGET_Y:8.4f} m")
    print(f"Landing     : X ={landing_x:8.4f} m, Y ={landing_y:8.4f} m")
    print(f"Error       : Ex={error_x:8.4f} m, Ey={error_y:8.4f} m")
    print(f"2D distance : {distance_error:8.4f} m")
    print(f"Result      : {result}")


def main():
    os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    random.seed(RANDOM_SEED)

    force_x = random.uniform(*INITIAL_FORCE_X_RANGE)
    force_y = random.uniform(*INITIAL_FORCE_Y_RANGE)

    print("AI SMART BALL LAUNCHER")
    print("Learning from failed attempts in Gazebo")
    print()
    print(f"Target             : X={TARGET_X:.3f}, Y={TARGET_Y:.3f}")
    print(f"Learning rate      : {LEARNING_RATE:.2f}")
    print(f"Success radius     : {SUCCESS_RADIUS:.3f} m")
    print(f"Maximum trials     : {MAX_TRIALS}")
    print(f"Random seed        : {RANDOM_SEED}")
    print(
        f"Random initial force: "
        f"Fx={force_x:.4f} N, Fy={force_y:.4f} N"
    )

    results = []
    success = False
    first_success_trial = None
    total_successes = 0
    consecutive_successes = 0

    for trial in range(1, MAX_TRIALS + 1):
        # 前回の動作を止めて、初期状態と時刻をリセット
        call_empty_service("/pause_physics")
        call_empty_service("/reset_simulation")

        # 今回の投射力を予約
        apply_force(force_x, force_y)

        # 一時停止したまま、着地付近まで物理演算を進める
        try:
            landing_x, landing_y, landing_z = get_landing_position()
        except Exception as error:
            print(f"ERROR: Failed to obtain landing position: {error}")
            sys.exit(1)

        error_x = TARGET_X - landing_x
        error_y = TARGET_Y - landing_y
        distance_error = math.hypot(error_x, error_y)

        success = distance_error <= SUCCESS_RADIUS
        result_text = "SUCCESS" if success else "MISS"

        print_trial(
            trial,
            force_x,
            force_y,
            landing_x,
            landing_y,
            error_x,
            error_y,
            distance_error,
            result_text,
        )

        results.append(
            {
                "trial": trial,
                "force_x": force_x,
                "force_y": force_y,
                "force_z": FORCE_Z,
                "landing_x": landing_x,
                "landing_y": landing_y,
                "landing_z": landing_z,
                "error_x": error_x,
                "error_y": error_y,
                "distance_error": distance_error,
                "result": result_text,
            }
        )

        if success:
            total_successes += 1
            consecutive_successes += 1

            if first_success_trial is None:
                first_success_trial = trial
                print()
                print(
                    "AI reached the target zone "
                    "for the first time."
                )
                print(
                    "Switching to precision mode."
                )
            else:
                print()
                print(
                    "AI maintained a successful throw."
                )

            print(
                f"Consecutive successes: "
                f"{consecutive_successes}"
            )

            active_learning_rate = (
                PRECISION_LEARNING_RATE
            )

        else:
            consecutive_successes = 0
            active_learning_rate = LEARNING_RATE

            print()
            print(
                "AI recognized the failed attempt."
            )

        # 成功前は0.5、成功後は0.2で誤差を修正
        correction_x = (
            active_learning_rate
            * error_x
            / DISTANCE_PER_FORCE_X
        )
        correction_y = (
            active_learning_rate
            * error_y
            / DISTANCE_PER_FORCE_Y
        )

        next_force_x = clamp(
            force_x + correction_x,
            *FORCE_X_LIMIT,
        )
        next_force_y = clamp(
            force_y + correction_y,
            *FORCE_Y_LIMIT,
        )

        print()
        print(
            f"Active learning rate: "
            f"{active_learning_rate:.2f}"
        )
        print(
            f"Correction   : dFx={correction_x:+.4f} N, "
            f"dFy={correction_y:+.4f} N"
        )

        if trial < MAX_TRIALS:
            print(
                f"Next force   : Fx={next_force_x:.4f} N, "
                f"Fy={next_force_y:.4f} N"
            )
        else:
            print(
                "All 10 throws have been completed."
            )

        force_x = next_force_x
        force_y = next_force_y

        # 動画で各試行を見分けやすくする間隔
        time.sleep(0.5)

    with OUTPUT_CSV.open("w", newline="") as csv_file:
        fieldnames = [
            "trial",
            "force_x",
            "force_y",
            "force_z",
            "landing_x",
            "landing_y",
            "landing_z",
            "error_x",
            "error_y",
            "distance_error",
            "result",
        ]

        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(results)

    print()
    print("=" * 58)
    print("LEARNING SUMMARY")
    print("=" * 58)
    print(f"Trials completed : {len(results)}")
    print(f"Total successes  : {total_successes}")
    print(
        f"First success    : "
        f"{first_success_trial if first_success_trial else 'None'}"
    )
    print(
        f"Final result     : "
        f"{'SUCCESS' if success else 'MISS'}"
    )
    print(f"CSV saved to    : {OUTPUT_CSV}")

    if first_success_trial is None:
        print(
            "The target was not reached in 10 trials."
        )
    elif success:
        print(
            "The AI reached the target and maintained "
            "successful performance through the final trial."
        )
    else:
        print(
            "The AI reached the target at least once, "
            "but the final trial was outside the target."
        )


if __name__ == "__main__":
    main()

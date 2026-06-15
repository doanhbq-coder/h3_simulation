#!/usr/bin/env python3
"""
Đánh giá sai số thuật toán detect PiTag.

Hai chế độ:
  --mode repeatability   : tag đứng yên, đo jitter/noise qua N frame
  --mode ground_truth    : so sánh với khoảng cách thực đo bằng thước

Chạy:
  ros2 run cob_fiducials evaluate_accuracy.py --mode repeatability --n 200
  ros2 run cob_fiducials evaluate_accuracy.py --mode ground_truth --distance 0.500 --n 100
"""

import argparse
import sys
import math
import csv
import datetime

import rclpy
from rclpy.node import Node
from cob_object_detection_msgs.msg import DetectionArray


def quat_to_euler_deg(qx, qy, qz, qw):
    """Quaternion → Euler angles (roll, pitch, yaw) in degrees."""
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.degrees(math.asin(sinp))

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

    return roll, pitch, yaw


class AccuracyEvaluator(Node):
    def __init__(self, args):
        super().__init__('pitag_accuracy_evaluator')
        self.args = args
        self.samples = {}   # {tag_id: list of (x,y,z,roll,pitch,yaw,reproj)}

        self.sub = self.create_subscription(
            DetectionArray,
            '/fiducials/detect_fiducials',
            self._callback,
            10
        )
        self.get_logger().info(
            f"Thu thập {args.n} mẫu cho mỗi tag | mode={args.mode}"
        )
        if args.mode == 'ground_truth':
            self.get_logger().info(
                f"Khoảng cách thực (ground truth Z): {args.distance:.4f} m"
            )

    def _callback(self, msg: DetectionArray):
        for det in msg.detections:
            tag_id = det.id
            if tag_id not in self.samples:
                self.samples[tag_id] = []

            if len(self.samples[tag_id]) >= self.args.n:
                continue

            p = det.pose.pose.position
            q = det.pose.pose.orientation
            roll, pitch, yaw = quat_to_euler_deg(q.x, q.y, q.z, q.w)
            reproj = float(det.score)

            self.samples[tag_id].append((p.x, p.y, p.z, roll, pitch, yaw, reproj))

            collected = len(self.samples[tag_id])
            if collected % 20 == 0 or collected == self.args.n:
                self.get_logger().info(
                    f"Tag {tag_id}: {collected}/{self.args.n} mẫu  "
                    f"(reproj={reproj:.2f} px)"
                )

    def done(self):
        return all(len(v) >= self.args.n for v in self.samples.values()) \
               and len(self.samples) > 0

    # ------------------------------------------------------------------
    def _stats(self, values):
        n = len(values)
        mean = sum(values) / n
        std = math.sqrt(sum((v - mean) ** 2 for v in values) / n)
        rmse = math.sqrt(sum(v ** 2 for v in values) / n)
        return mean, std, rmse, min(values), max(values)

    def report(self):
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = f"pitag_accuracy_{self.args.mode}_{ts}.csv"

        print("\n" + "=" * 70)
        print(f"  KẾT QUẢ ĐÁNH GIÁ SAI SỐ  —  mode: {self.args.mode.upper()}")
        print("=" * 70)

        rows = []
        for tag_id, samples in sorted(self.samples.items()):
            xs    = [s[0] for s in samples]
            ys    = [s[1] for s in samples]
            zs    = [s[2] for s in samples]
            rolls = [s[3] for s in samples]
            pits  = [s[4] for s in samples]
            yaws  = [s[5] for s in samples]
            rps   = [s[6] for s in samples]

            x_m, x_s, *_ = self._stats(xs)
            y_m, y_s, *_ = self._stats(ys)
            z_m, z_s, *_ = self._stats(zs)
            r_m, r_s, *_ = self._stats(rolls)
            p_m, p_s, *_ = self._stats(pits)
            w_m, w_s, *_ = self._stats(yaws)
            rp_m, rp_s, rp_rms, rp_min, rp_max = self._stats(rps)

            print(f"\n  Tag ID = {tag_id}  ({len(samples)} mẫu)")
            print(f"  {'Trục':<10} {'Trung bình':>12} {'Std Dev':>12} {'Đơn vị'}")
            print(f"  {'-'*50}")
            print(f"  {'X':<10} {x_m*1000:>11.2f} mm  ±{x_s*1000:.2f} mm")
            print(f"  {'Y':<10} {y_m*1000:>11.2f} mm  ±{y_s*1000:.2f} mm")
            print(f"  {'Z (depth)':<10} {z_m*1000:>11.2f} mm  ±{z_s*1000:.2f} mm")
            print(f"  {'Roll':<10} {r_m:>11.3f}°   ±{r_s:.3f}°")
            print(f"  {'Pitch':<10} {p_m:>11.3f}°   ±{p_s:.3f}°")
            print(f"  {'Yaw':<10} {w_m:>11.3f}°   ±{w_s:.3f}°")
            print(f"  {'Reproj err':<10} {rp_m:>11.3f} px  ±{rp_s:.3f} px  "
                  f"(min={rp_min:.2f}, max={rp_max:.2f})")

            if self.args.mode == 'ground_truth':
                gt_z = self.args.distance
                z_error_mm = (z_m - gt_z) * 1000
                z_rmse_mm  = math.sqrt(sum((z - gt_z)**2 for z in zs) / len(zs)) * 1000
                print(f"\n  ── Ground Truth Z = {gt_z*1000:.1f} mm ──")
                print(f"  Sai số trung bình (bias) Z : {z_error_mm:+.2f} mm")
                print(f"  RMSE Z                     : {z_rmse_mm:.2f} mm")
                print(f"  Sai số tương đối           : {abs(z_error_mm)/(gt_z*1000)*100:.2f} %")

            # Tổng hợp đánh giá
            pos_noise_mm = math.sqrt(x_s**2 + y_s**2 + z_s**2) * 1000
            ang_noise_deg = math.sqrt(r_s**2 + p_s**2 + w_s**2)
            print(f"\n  ── Tổng hợp ──")
            print(f"  Noise vị trí 3D (|std|)    : {pos_noise_mm:.2f} mm")
            print(f"  Noise góc 3D (|std|)       : {ang_noise_deg:.3f}°")
            print(f"  Reprojection error trung bình: {rp_m:.3f} px")
            if rp_m < 1.0:
                print(f"  → Chất lượng: TỐT (< 1 px)")
            elif rp_m < 2.5:
                print(f"  → Chất lượng: KHÁ (1–2.5 px)")
            else:
                print(f"  → Chất lượng: KÉM (> 2.5 px) — kiểm tra calibration")

            rows.append({
                'tag_id': tag_id,
                'n_samples': len(samples),
                'x_mean_mm': x_m*1000, 'x_std_mm': x_s*1000,
                'y_mean_mm': y_m*1000, 'y_std_mm': y_s*1000,
                'z_mean_mm': z_m*1000, 'z_std_mm': z_s*1000,
                'roll_mean_deg': r_m, 'roll_std_deg': r_s,
                'pitch_mean_deg': p_m, 'pitch_std_deg': p_s,
                'yaw_mean_deg': w_m, 'yaw_std_deg': w_s,
                'reproj_mean_px': rp_m, 'reproj_std_px': rp_s,
                'reproj_rmse_px': rp_rms,
                'gt_z_mm': self.args.distance * 1000 if self.args.mode == 'ground_truth' else '',
                'z_bias_mm': (z_m - self.args.distance)*1000 if self.args.mode == 'ground_truth' else '',
            })

        # Lưu CSV
        if rows:
            with open(csv_path, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=rows[0].keys())
                writer.writeheader()
                writer.writerows(rows)
            print(f"\n  Đã lưu kết quả: {csv_path}")

        print("=" * 70)


def main():
    parser = argparse.ArgumentParser(
        description='Đánh giá sai số PiTag detector')
    parser.add_argument('--mode', choices=['repeatability', 'ground_truth'],
                        default='repeatability',
                        help='Chế độ đánh giá')
    parser.add_argument('--n', type=int, default=100,
                        help='Số mẫu thu thập (mặc định: 100)')
    parser.add_argument('--distance', type=float, default=0.0,
                        help='Khoảng cách thực Z [mét] cho mode ground_truth')

    # Tách args trước khi rclpy parse
    idx = sys.argv.index('--ros-args') if '--ros-args' in sys.argv else len(sys.argv)
    args = parser.parse_args(sys.argv[1:idx])

    if args.mode == 'ground_truth' and args.distance <= 0:
        parser.error("--mode ground_truth yêu cầu --distance > 0 (đơn vị: mét)")

    rclpy.init()
    node = AccuracyEvaluator(args)

    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    if node.samples:
        node.report()
    else:
        print("[!] Không nhận được detection nào. Kiểm tra detector đang chạy.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

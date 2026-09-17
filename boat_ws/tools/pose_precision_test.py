#!/usr/bin/env python3

import argparse
import csv
import math
import statistics
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (
        q.w * q.z
        + q.x * q.y
    )

    cosy_cosp = 1.0 - 2.0 * (
        q.y * q.y
        + q.z * q.z
    )

    return math.atan2(
        siny_cosp,
        cosy_cosp
    )


def percentile(values, fraction):
    values = sorted(values)

    if not values:
        return math.nan

    index = fraction * (len(values) - 1)

    lower = int(math.floor(index))
    upper = int(math.ceil(index))

    if lower == upper:
        return values[lower]

    weight = index - lower

    return (
        values[lower] * (1.0 - weight)
        + values[upper] * weight
    )


class PosePrecisionTest(Node):

    def __init__(self, topic, samples, output):
        super().__init__('pose_precision_test')

        self.target_samples = samples
        self.output = output

        self.data = []

        self.subscription = self.create_subscription(
            PoseStamped,
            topic,
            self.pose_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info(
            f'Collecting {samples} samples from {topic}'
        )

    def pose_callback(self, msg):

        p = msg.pose.position
        q = msg.pose.orientation

        yaw = quaternion_to_yaw(q)

        self.data.append(
            (
                time.time(),
                float(p.x),
                float(p.y),
                float(p.z),
                float(yaw),
            )
        )

        count = len(self.data)

        if (
            count == 1
            or count % 25 == 0
            or count == self.target_samples
        ):
            self.get_logger().info(
                f'{count}/{self.target_samples} samples'
            )

        if count >= self.target_samples:
            self.calculate_results()
            rclpy.shutdown()

    def calculate_results(self):

        timestamps = [
            row[0]
            for row in self.data
        ]

        xs = [
            row[1]
            for row in self.data
        ]

        ys = [
            row[2]
            for row in self.data
        ]

        zs = [
            row[3]
            for row in self.data
        ]

        yaws = [
            row[4]
            for row in self.data
        ]

        mean_x = statistics.mean(xs)
        mean_y = statistics.mean(ys)
        mean_z = statistics.mean(zs)

        std_x = statistics.stdev(xs)
        std_y = statistics.stdev(ys)
        std_z = statistics.stdev(zs)

        # Circular mean for yaw.
        mean_sin = statistics.mean(
            math.sin(yaw)
            for yaw in yaws
        )

        mean_cos = statistics.mean(
            math.cos(yaw)
            for yaw in yaws
        )

        mean_yaw = math.atan2(
            mean_sin,
            mean_cos
        )

        yaw_errors = [
            math.atan2(
                math.sin(yaw - mean_yaw),
                math.cos(yaw - mean_yaw)
            )
            for yaw in yaws
        ]

        yaw_std = math.sqrt(
            statistics.mean(
                error * error
                for error in yaw_errors
            )
        )

        radial_errors = [
            math.hypot(
                x - mean_x,
                y - mean_y
            )
            for x, y in zip(xs, ys)
        ]

        rms_horizontal = math.sqrt(
            statistics.mean(
                error * error
                for error in radial_errors
            )
        )

        p50 = percentile(
            radial_errors,
            0.50
        )

        p95 = percentile(
            radial_errors,
            0.95
        )

        max_error = max(
            radial_errors
        )

        duration = (
            timestamps[-1]
            - timestamps[0]
        )

        rate = (
            (len(self.data) - 1) / duration
            if duration > 0.0
            else math.nan
        )

        print()
        print('==========================================')
        print(' MAVROS LOCAL POSE PRECISION TEST')
        print('==========================================')
        print(f'Samples:             {len(self.data)}')
        print(f'Duration:            {duration:.2f} s')
        print(f'Average rate:        {rate:.2f} Hz')
        print()

        print('MEAN POSITION')
        print(f'X:                   {mean_x:.4f} m')
        print(f'Y:                   {mean_y:.4f} m')
        print(f'Z:                   {mean_z:.4f} m')
        print()

        print('AXIS PRECISION')
        print(f'X std dev:           {std_x:.4f} m')
        print(f'Y std dev:           {std_y:.4f} m')
        print(f'Z std dev:           {std_z:.4f} m')
        print()

        print('HORIZONTAL PRECISION')
        print(f'RMS radial scatter:  {rms_horizontal:.4f} m')
        print(f'50% radius:          {p50:.4f} m')
        print(f'95% radius:          {p95:.4f} m')
        print(f'Max deviation:       {max_error:.4f} m')
        print()

        print('HEADING PRECISION')
        print(
            f'Mean yaw:            '
            f'{math.degrees(mean_yaw):.2f} deg'
        )
        print(
            f'Yaw RMS deviation:   '
            f'{math.degrees(yaw_std):.2f} deg'
        )
        print('==========================================')
        print()

        with open(
            self.output,
            'w',
            newline=''
        ) as file:

            writer = csv.writer(file)

            writer.writerow(
                [
                    'time',
                    'x',
                    'y',
                    'z',
                    'yaw_rad',
                    'yaw_deg',
                    'horizontal_error_from_mean',
                ]
            )

            for row, error in zip(
                self.data,
                radial_errors
            ):
                writer.writerow(
                    [
                        row[0],
                        row[1],
                        row[2],
                        row[3],
                        row[4],
                        math.degrees(row[4]),
                        error,
                    ]
                )

        print(
            f'Raw samples saved to: {self.output}'
        )


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        '--samples',
        type=int,
        default=300
    )

    parser.add_argument(
        '--topic',
        default='/mavros/local_position/pose'
    )

    parser.add_argument(
        '--output',
        default='pose_precision.csv'
    )

    args = parser.parse_args()

    rclpy.init()

    node = PosePrecisionTest(
        args.topic,
        args.samples,
        args.output
    )

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if rclpy.ok():
            rclpy.shutdown()

        node.destroy_node()


if __name__ == '__main__':
    main()

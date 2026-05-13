#!/usr/bin/env python3
"""
Excavation run logger — logs actuator positions + stage labels to CSV.
Triggers ros2 bag record on excavation start, stops on completion.
Lives in logger_pkg.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from interfaces_pkg.msg import MotorHealth
from msg_pkg.action import Excavation

import csv
import os
import subprocess
import signal
import time
from datetime import datetime


LOG_DIR = os.path.expanduser('~/excavation_logs')
MISALIGN_THRESH = 0.12


class ExcavationLogger(Node):
    def __init__(self):
        super().__init__('excavation_logger')

        os.makedirs(LOG_DIR, exist_ok=True)

        self.current_stage  = 'IDLE'
        self.run_number     = 0
        self.csv_file       = None
        self.csv_writer     = None
        self.bag_proc       = None
        self.run_active     = False
        self.t_run_start    = None

        # subscribe to health at full rate
        self.health_sub = self.create_subscription(
            MotorHealth, '/health_topic', self.on_health, 10)

        # subscribe to excavation action feedback
        self.feedback_sub = self.create_subscription(
            Excavation.Impl.FeedbackMessage,
            '/excavation_action/_action/feedback',
            self.on_feedback, 10)

        # subscribe to excavation action status
        self.status_sub = self.create_subscription(
            Excavation.Impl.GoalStatusMessage,
            '/excavation_action/_action/status',
            self.on_status, 10)

        self.get_logger().info('Excavation logger ready — waiting for dig run')

    # ── FEEDBACK → stage label ────────────────────────────────────────────────
    def on_feedback(self, msg):
        feedback_text = msg.feedback.feedback_message

        # map feedback strings from excavation_node.cpp to short labels
        if 'Stage 1' in feedback_text:
            self._set_stage('STAGE_1_APPROACH')
        elif 'Stage 2' in feedback_text and 'complete' not in feedback_text:
            self._set_stage('STAGE_2_ENTRY_DRIVE')
        elif 'Stage 2 complete' in feedback_text:
            self._set_stage('STAGE_2_COMPLETE')
        elif 'Stage 3' in feedback_text and 'complete' not in feedback_text:
            self._set_stage('STAGE_3_SCOOP')
        elif 'Stage 3 complete' in feedback_text:
            self._set_stage('STAGE_3_COMPLETE')
        elif 'Stage 6' in feedback_text and 'complete' not in feedback_text:
            self._set_stage('STAGE_6_RETURN')
        elif 'Excavation complete' in feedback_text:
            self._set_stage('COMPLETE')

        # start logging on first feedback
        if not self.run_active:
            self._start_run()

    # ── ACTION STATUS → detect completion ────────────────────────────────────
    def on_status(self, msg):
        for status in msg.status_list:
            # 4 = SUCCEEDED, 5 = CANCELED, 6 = ABORTED
            if status.status in [4, 5, 6] and self.run_active:
                final = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}[status.status]
                self._set_stage(f'RUN_{final}')
                self._stop_run()

    # ── HEALTH → continuous position log ─────────────────────────────────────
    def on_health(self, msg):
        if not self.run_active or self.csv_writer is None:
            return

        now = time.time()
        elapsed = now - self.t_run_start

        ll = msg.left_lift_position
        rl = msg.right_lift_position
        lt = msg.left_tilt_position
        rt = msg.right_tilt_position

        lift_delta = abs(ll - rl)
        tilt_delta = abs(lt - rt)

        lift_warn = 'MISALIGNED' if lift_delta > MISALIGN_THRESH else 'OK'
        tilt_warn = 'MISALIGNED' if tilt_delta > MISALIGN_THRESH else 'OK'

        self.csv_writer.writerow({
            'timestamp':        f'{elapsed:.4f}',
            'stage':            self.current_stage,
            'left_lift_pos':    f'{ll:.4f}',
            'right_lift_pos':   f'{rl:.4f}',
            'lift_delta':       f'{lift_delta:.4f}',
            'lift_status':      lift_warn,
            'left_tilt_pos':    f'{lt:.4f}',
            'right_tilt_pos':   f'{rt:.4f}',
            'tilt_delta':       f'{tilt_delta:.4f}',
            'tilt_status':      tilt_warn,
            'left_lift_curr':   f'{msg.left_lift_current:.3f}',
            'right_lift_curr':  f'{msg.right_lift_current:.3f}',
            'left_tilt_curr':   f'{msg.left_tilt_current:.3f}',
            'right_tilt_curr':  f'{msg.right_tilt_current:.3f}',
            'left_drive_vel':   f'{msg.left_motor_velocity:.1f}',
            'right_drive_vel':  f'{msg.right_motor_velocity:.1f}',
        })

        if lift_warn == 'MISALIGNED' or tilt_warn == 'MISALIGNED':
            self.get_logger().warn(
                f'[{self.current_stage}] '
                f'{"LIFTS MISALIGNED Δ="+str(round(lift_delta,3)) if lift_warn=="MISALIGNED" else ""}  '
                f'{"TILTS MISALIGNED Δ="+str(round(tilt_delta,3)) if tilt_warn=="MISALIGNED" else ""}')

    # ── START RUN ─────────────────────────────────────────────────────────────
    def _start_run(self):
        self.run_number  += 1
        self.run_active   = True
        self.t_run_start  = time.time()
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')

        # CSV log
        csv_path = os.path.join(LOG_DIR, f'run_{self.run_number:03d}_{ts}.csv')
        self.csv_file   = open(csv_path, 'w', newline='')
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=[
            'timestamp','stage',
            'left_lift_pos','right_lift_pos','lift_delta','lift_status',
            'left_tilt_pos','right_tilt_pos','tilt_delta','tilt_status',
            'left_lift_curr','right_lift_curr',
            'left_tilt_curr','right_tilt_curr',
            'left_drive_vel','right_drive_vel',
        ])
        self.csv_writer.writeheader()

        # ros2 bag record alongside the CSV
        bag_path = os.path.join(LOG_DIR, f'bag_run_{self.run_number:03d}_{ts}')
        self.bag_proc = subprocess.Popen([
            'ros2', 'bag', 'record',
            '/health_topic',
            '/excavation_action/_action/feedback',
            '/excavation_action/_action/status',
            '/odometry/filtered',
            '-o', bag_path,
        ])

        self.get_logger().info(
            f'Run {self.run_number} started — CSV: {csv_path}  BAG: {bag_path}')

    # ── STOP RUN ──────────────────────────────────────────────────────────────
    def _stop_run(self):
        self.run_active = False

        if self.csv_file:
            self.csv_file.close()
            self.csv_file   = None
            self.csv_writer = None

        if self.bag_proc:
            self.bag_proc.send_signal(signal.SIGINT)
            self.bag_proc.wait()
            self.bag_proc = None

        self.get_logger().info(f'Run {self.run_number} saved to {LOG_DIR}')
        self.current_stage = 'IDLE'

    def _set_stage(self, stage):
        if stage != self.current_stage:
            self.get_logger().info(f'Stage → {stage}')
            self.current_stage = stage


def main():
    rclpy.init()
    node = ExcavationLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.run_active:
            node._stop_run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
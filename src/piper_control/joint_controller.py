import logging
import threading
import time

import numpy as np

from piper_control.piper_control import PiperControl

logger = logging.getLogger(__name__)


class JointController:
    def __init__(self, robot_config, controller_config, interface_name):
        self.start_time_ns_ = time.perf_counter_ns()
        self.robot_config = robot_config
        self.controller_config = controller_config
        self.piper_control = PiperControl(can_port=interface_name)
        self.init_robot()
        self.background_send_recv_thread_ = threading.Thread(
            target=self.background_send_recv
        )
        self.background_send_recv_running_ = False
        logger.info(
            "Background send/recv thread is running", self.background_send_recv_running_
        )
        self.gain = Gain(self.robot_config.joint_dof)

        self.cmd_mutex_ = threading.Lock()
        self.output_joint_cmd_ = JointState(self.robot_config.joint_dof)

        self.state_mutex_ = threading.Lock()
        self.joint_state_ = JointState(self.robot_config.joint_dof)

    def __del__(self):
        if self.controller_config.shutdown_to_passive:
            logger.info("Shutting down to passive")
            damping_gain = Gain(self.robot_config.joint_dof)
            damping_gain.kd = self.controller_config.default_kd
            self.set_gain(damping_gain)

            with self.cmd_mutex_:
                self.output_joint_cmd_.vel = np.zeros(self.robot_config.joint_dof)
                self.output_joint_cmd_.torque = np.zeros(self.robot_config.joint_dof)
                self.interpolator.init_fixed(self.output_joint_cmd_)

            self.background_send_recv_running_ = True
            self.controller_config.gravity_compensation = False
            time.sleep(2.0)

        else:
            logger.info("Disconnect motors without setting to damping")
            self.piper_control.disable()
            self.destroy_background_threads = True
            self.background_send_recv_thread_.join()
            logger.info("Background send/recv thread joined")

    def get_joint_cmd(self):
        with self.cmd_mutex_:
            return self.output_joint_cmd_

    def get_joint_state(self):
        with self.state_mutex_:
            return self.joint_state_

    def set_gain(self, new_gain: Gain):
        if np.all(self.gain.kp == 0) and not np.all(new_gain.kp == 0):
            joint_state = self.get_joint_state()
            joint_cmd = self.get_joint_cmd()
            max_pos_error = np.max(np.abs(joint_state.pos - joint_cmd.pos))
            pos_error_threshold = 0.2
            kp_threshold = 1.0

            if (
                max_pos_error > pos_error_threshold
                and np.max(new_gain.kp) > kp_threshold
            ):
                logger.error(
                    "Cannot set kp too large when the joint pos cmd is far from the joint pos"
                )
                logger.error(
                    f"Target max kp: {np.max(new_gain.kp)}, kp threshold: {kp_threshold}. Current pos {self.joint_state_.pos}, target pos {joint_cmd.pos}"
                )
                self.background_send_recv_running_ = False
                raise RuntimeError(
                    "Cannot set kp to non-zero when the joint pos cmd is far from the joint pos"
                )

        with self.cmd_mutex_:
            self.gain = new_gain

    def get_gain(self) -> Gain:
        with self.cmd_mutex_:
            return self.gain

    def get_timestamp(self) -> float:
        return (time.perf_counter_ns() - self.start_time_ns_) / 1e9

    def reset_to_home(self):
        init_state = self.get_joint_state()
        init_gain = self.get_gain()

        target_gain = Gain(self.robot_config.joint_dof)
        if np.all(init_gain.kp == 0):
            logger.info("Current gain is 0, using default gain")
            target_gain = Gain(
                self.controller_config.default_kp, self.controller_config.default_kd
            )
        else:
            target_gain = init_gain

        target_state = JointState(self.robot_config.joint_dof)
        max_pos_error = np.max(np.abs(init_state.pos - target_state.pos))
        wait_time = max(max_pos_error, 0.5)
        step_num = int(wait_time / self.controller_config.dt)
        logger.info(
            f"Start reset to home in {wait_time + 0.5:.3f}s, max pos error: {max_pos_error:.3f}"
        )

        prev_running = self.background_send_recv_running_
        self.background_send_recv_running_ = True
        target_state.timestamp = self.get_timestamp() + wait_time

        with self.cmd_mutex_:
            start_state = JointState(self.robot_config.joint_dof)
            start_state.pos = init_state.pos
            start_state.gripper_pos = init_state.gripper_pos
            start_state.timestamp = self.get_timestamp()
            self.interpolator.init(start_state, target_state)
            logger.info("Interpolator initialized")

        new_gain = Gain(self.robot_config.joint_dof)
        for i in range(step_num):
            alpha = i / step_num
            new_gain = init_gain * (1 - alpha) + target_gain * alpha
            self.set_gain(new_gain)
            time.sleep(self.controller_config.dt)

        logger.info("New gain updated")

        target_state.timestamp = self.get_timestamp() + 0.5
        with self.cmd_mutex_:
            self.interpolator.override_waypoint(self.get_timestamp(), target_state)

        logger.info("Finish reset to home")
        self.background_send_recv_running_ = prev_running

    def set_to_damping(self):
        damping_gain = Gain(self.robot_config.joint_dof)
        damping_gain.kd = self.controller_config.default_kd
        self.set_gain(damping_gain)
        time.sleep(0.01)
        joint_state = self.get_joint_state()
        with self.cmd_mutex_:
            joint_state.vel = np.zeros(self.robot_config.joint_dof)
            joint_state.torque = np.zeros(self.robot_config.joint_dof)
            self.interpolator.init_fixed(joint_state)

    def init_robot(self):
        logger.info("Initializing robot")
        self.piper_control.reset()
        logger.info("Robot initialized")

        gain = Gain(self.robot_config.joint_dof)
        gain.kd = self.controller_config.default_kd

        init_joint_state = self.get_joint_state()
        init_joint_state.vel = np.zeros(self.robot_config.joint_dof)
        init_joint_state.torque = np.zeros(self.robot_config.joint_dof)
        self.set_gain(gain)

        if (self.joint_state_.pos == 0.0).all():
            logger.error(
                "Robot motors are not initialized, check the connection or state of the arm."
            )
            raise RuntimeError()

        logger.info("Motors enabled. Starting to send and receive..")
        for j in range(10):
            self.send_recv_()
            self.check_joint_state_sanity_()
            self.over_current_protection_()

    def send_recv_(self):
        self.update_output_cmd()
        with self.cmd_mutex_:
            self.piper_control.set_joint_mit_ctrl(
                self.output_joint_cmd_.pos,
                self.output_joint_cmd_.vel,
                self.gain.kp,
                self.gain.kd,
                self.output_joint_cmd_.torque,
            )

        time.sleep(0.0001)

        if self.robot_config.gripper_on:
            pass

        self.update_joint_state()

    def check_joint_state_sanity_(self):
        # TODO
        pass

    def over_current_protection_(self):
        # TODO
        pass

    def background_send_recv(self):
        while not self.destroy_background_threads:
            start_time_ns = time.perf_counter_ns()
            if self.background_send_recv_running_:
                self.over_current_protection_()
                self.check_joint_state_sanity_()
                self.send_recv_()
            elapsed_time_ns = time.perf_counter_ns() - start_time_ns
            sleep_time_ns = int(self.controller_config.dt * 1e9) - elapsed_time_ns
            if sleep_time_ns > 0:
                time.sleep(sleep_time_ns / 1e9)
            elif sleep_time_ns < -500 * 1e3:  # 500us
                logger.warning(
                    f"Send/recv thread is taking too long, time: {elapsed_time_ns / 1e9:.3f}s"
                )

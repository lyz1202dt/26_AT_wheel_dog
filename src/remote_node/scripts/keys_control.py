#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_interfaces.msg import MoveCmd
from pynput import keyboard

class ProKeyboardTeleop(Node):
    def __init__(self):
        super().__init__('pro_keyboard_teleop')
        self.pub = self.create_publisher(MoveCmd, 'robot_move_cmd', 10)

        # 当前实际速度（平滑后）
        self.vx = 0.0      # 前后 (W/S)
        self.vy = 0.0      # 左右 (A/D)
        self.vz = 0.0      # 旋转 (Q/E)

        # 当前按下的方向键集合（用于组合）
        self.pressed_keys = set()

        self.mode = 0
        self.shift_pressed = False
        self.ctrl_pressed = False

        # 初始化目标速度（防止 update 中属性不存在）
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_vz = 0.0

        # 参数
        self.accel = 3.5
        self.fast_accel = 4.0
        self.max_v = 0.7
        self.fast_max_v = 1.0
        self.decay = 3.0

        self.last_time = self.get_clock().now()

        # 键盘监听
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        self.listener.start()

        # 定时器 20ms
        self.timer = self.create_timer(0.02, self.update)

        self.get_logger().info("ProKeyboardTeleop started. Use WASD+QE, Shift for fast, 0-9 for mode, Ctrl+0-9 for mode 10-19.")

    def on_press(self, key):
        if key in (keyboard.Key.shift, keyboard.Key.shift_l, keyboard.Key.shift_r):
            self.shift_pressed = True
            return
        if key in (keyboard.Key.ctrl_l, keyboard.Key.ctrl_r):
            self.ctrl_pressed = True
            return

        try:
            if hasattr(key, 'char') and key.char is not None:
                char = key.char.lower()
            else:
                return
        except:
            return

        if self.ctrl_pressed and char.isdigit():
            digit = int(char)
            self.mode = 10 + digit
            self.get_logger().info(f"Mode changed to {self.mode} (Ctrl+{digit})")
            return

        if char.isdigit():
            self.mode = int(char)
            self.get_logger().info(f"Mode changed to {self.mode}")
            return

        if char in ['w', 'a', 's', 'd', 'q', 'e']:
            self.pressed_keys.add(char)
            self._update_target_from_keys()

    def on_release(self, key):
        if key in (keyboard.Key.shift, keyboard.Key.shift_l, keyboard.Key.shift_r):
            self.shift_pressed = False
            self._update_target_from_keys()
            return
        if key in (keyboard.Key.ctrl_l, keyboard.Key.ctrl_r):
            self.ctrl_pressed = False
            return

        try:
            if hasattr(key, 'char') and key.char is not None:
                char = key.char.lower()
            else:
                return
        except:
            return

        if char in ['w', 'a', 's', 'd', 'q', 'e']:
            self.pressed_keys.discard(char)
            self._update_target_from_keys()

    def _update_target_from_keys(self):
        fast = self.shift_pressed
        cur_max = self.fast_max_v if fast else self.max_v

        dx = 0.0   # 左右 (A/D)
        dy = 0.0   # 前后 (W/S)
        dz = 0.0   # 旋转 (Q/E)

        if 'w' in self.pressed_keys:
            dy += 1.0
        if 's' in self.pressed_keys:
            dy -= 1.0
        if 'a' in self.pressed_keys:
            dx -= 1.0
        if 'd' in self.pressed_keys:
            dx += 1.0
        if 'q' in self.pressed_keys:
            dz -= 1.0
        if 'e' in self.pressed_keys:
            dz += 1.0

        # length = (dx*dx + dy*dy + dz*dz) ** 0.5
        # if length > 1e-6:
        #     dx /= length
        #     dy /= length
        #     dz /= length

        lateral_gain = 0.7   # 用户可根据手感调整

        self.target_vx = dy * cur_max
        self.target_vy = dx * cur_max * lateral_gain   # 横向乘以增益
        self.target_vz = dz * cur_max

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        dt = min(dt, 0.05)
        self.last_time = now

        fast = self.shift_pressed
        cur_accel = self.fast_accel if fast else self.accel
        cur_max = self.fast_max_v if fast else self.max_v

        self.vx = self._approach(self.vx, self.target_vx, cur_accel * dt)
        self.vy = self._approach(self.vy, self.target_vy, cur_accel * dt)
        self.vz = self._approach(self.vz, self.target_vz, cur_accel * dt)

        if not self.pressed_keys:
            self.vx = self._approach(self.vx, 0.0, self.decay * dt)
            self.vy = self._approach(self.vy, 0.0, self.decay * dt)
            self.vz = self._approach(self.vz, 0.0, self.decay * dt)

        self.vx = max(-cur_max, min(cur_max, self.vx))
        self.vy = max(-cur_max, min(cur_max, self.vy))
        self.vz = max(-cur_max, min(cur_max, self.vz))

        msg = MoveCmd()
        msg.vx = float(self.vx)
        msg.vy = float(-self.vy)
        msg.vz = float(-self.vz)
        msg.step_mode = self.mode

        msg.vx = max(-1.0, min(1.0, msg.vx))
        msg.vy = max(-0.3, min(0.3, msg.vy))
        msg.vz = max(-1.0, min(1.0, msg.vz))

        self.pub.publish(msg)
        self.get_logger().debug(f"vx={self.vx:.2f}, vy={self.vy:.2f}, vz={self.vz:.2f}, mode={self.mode}")

    def _approach(self, v, target, step):
        if v < target:
            v += step
            if v > target:
                v = target
        elif v > target:
            v -= step
            if v < target:
                v = target
        return v

    def __del__(self):
        if hasattr(self, 'listener') and self.listener.is_alive():
            self.listener.stop()

def main(args=None):
    rclpy.init(args=args)
    node = ProKeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    
    
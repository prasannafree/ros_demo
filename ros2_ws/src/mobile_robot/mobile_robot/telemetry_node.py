"""Keyboard teleoperation node for mobile robot control."""

import os
import select
import sys
import termios
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String

KEY_MAPPINGS = {
    '\x1b[A': 'UP',
    '\x1b[B': 'DOWN',
    '\x1b[C': 'RIGHT',
    '\x1b[D': 'LEFT',
    '\x1bOA': 'UP',
    '\x1bOB': 'DOWN',
    '\x1bOC': 'RIGHT',
    '\x1bOD': 'LEFT',
    'w': 'UP',
    'W': 'UP',
    's': 'DOWN',
    'S': 'DOWN',
    'a': 'LEFT',
    'A': 'LEFT',
    'd': 'RIGHT',
    'D': 'RIGHT',
    ' ': 'SPACE',
    '\x1b': 'ESC',
    'q': 'QUIT',
    'Q': 'QUIT',
    '\x03': 'QUIT',
}


class TelemetryNode(Node):
    """ROS 2 node for keyboard teleoperation via terminal input."""

    def __init__(self):
        """Initialize the keyboard teleoperation node."""
        super().__init__('keyboard_teleop_node')

        self.publisher = self.create_publisher(
            String,
            '/keyboard_command',
            10
        )

        self.speed_level = 3
        self.emergency_stop = False
        self.running = True

        self.key_press_times = {}
        self.key_timeout = 0.4
        self.last_published_data = None

        self.is_tty = sys.stdin.isatty()
        self.old_settings = None

        if self.is_tty:
            try:
                fd = sys.stdin.fileno()
                self.old_settings = termios.tcgetattr(fd)
                new_settings = termios.tcgetattr(fd)
                # Non-canonical mode, no echo, keep output processing
                new_settings[3] &= ~(termios.ICANON | termios.ECHO)
                new_settings[6][termios.VMIN] = 0
                new_settings[6][termios.VTIME] = 0
                termios.tcsetattr(fd, termios.TCSANOW, new_settings)
            except (termios.error, OSError) as e:
                self.get_logger().warn(
                    f'Could not set terminal mode: {e}. '
                    'Falling back to standard input.'
                )
                self.is_tty = False

        self.input_thread = threading.Thread(
            target=self._read_keyboard_loop,
            daemon=True
        )
        self.input_thread.start()

        self.timer = self.create_timer(
            0.1,
            self.publish_command
        )

        self.get_logger().info(
            'Keyboard teleoperation started.\n'
            '  Move        : Arrow Keys or WASD\n'
            '  Speed (0-9) : Set speed level\n'
            '  Space       : Emergency STOP\n'
            '  Esc / q     : Quit'
        )

    def _parse_keys(self, data):
        """Parse raw input bytes into recognized key events."""
        events = []
        i = 0
        n = len(data)
        while i < n:
            # Check 3-character escape sequences first
            if i + 3 <= n and data[i:i + 3] in KEY_MAPPINGS:
                events.append(KEY_MAPPINGS[data[i:i + 3]])
                i += 3
            elif data[i] in KEY_MAPPINGS:
                events.append(KEY_MAPPINGS[data[i]])
                i += 1
            elif data[i].isdigit():
                events.append(data[i])
                i += 1
            else:
                i += 1
        return events

    def _read_keyboard_loop(self):
        """Continuously poll for keyboard input in a background thread."""
        fd = sys.stdin.fileno() if self.is_tty else None

        while self.running and rclpy.ok():
            if not self.is_tty or fd is None:
                time.sleep(0.05)
                continue

            rlist, _, _ = select.select([fd], [], [], 0.05)
            if not rlist:
                continue

            try:
                raw_bytes = os.read(fd, 32)
                if not raw_bytes:
                    continue
                data = raw_bytes.decode('utf-8', errors='ignore')
            except (OSError, IOError):
                continue

            events = self._parse_keys(data)
            for key in events:
                if key in ('ESC', 'QUIT'):
                    self.running = False
                    self.get_logger().info('Quit requested; stop published.')
                    if rclpy.ok():
                        stop_msg = String()
                        stop_msg.data = 'keys=STOP;speed_level=0'
                        self.publisher.publish(stop_msg)
                        rclpy.shutdown()
                    return
                elif key == 'SPACE':
                    self.emergency_stop = True
                    self.key_press_times.clear()
                    self.get_logger().warn(
                        'Emergency STOP engaged (Space pressed).'
                    )
                elif key in ('UP', 'DOWN', 'LEFT', 'RIGHT'):
                    self.emergency_stop = False
                    self.key_press_times[key] = time.time()
                elif key.isdigit():
                    self.speed_level = int(key)
                    self.emergency_stop = False
                    self.get_logger().info(
                        f'Speed level set to {self.speed_level}.'
                    )

    def publish_command(self):
        """Publish the current keyboard command string at a fixed rate."""
        if not self.running:
            return

        msg = String()
        if self.emergency_stop:
            msg.data = 'keys=STOP;speed_level=0'
        else:
            now = time.time()
            active_keys = [
                k for k, t in self.key_press_times.items()
                if (now - t) <= self.key_timeout
            ]
            if active_keys:
                key_text = ','.join(sorted(active_keys))
            else:
                key_text = 'NONE'

            msg.data = f'keys={key_text};speed_level={self.speed_level}'

        self.publisher.publish(msg)

        if msg.data != self.last_published_data:
            self.get_logger().info(f'Command: {msg.data}')
            self.last_published_data = msg.data

    def restore_terminal(self):
        """Restore original terminal attributes."""
        if self.old_settings is not None and self.is_tty:
            try:
                termios.tcsetattr(
                    sys.stdin.fileno(),
                    termios.TCSADRAIN,
                    self.old_settings
                )
            except (termios.error, OSError):
                pass
            self.old_settings = None

    def destroy_node(self):
        """Clean up the node and restore terminal configuration."""
        self.running = False
        self.restore_terminal()
        super().destroy_node()


def main(args=None):
    """Run the keyboard teleoperation node."""
    rclpy.init(args=args)
    node = TelemetryNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.restore_terminal()
        if rclpy.ok():
            stop_msg = String()
            stop_msg.data = 'keys=STOP;speed_level=0'
            node.publisher.publish(stop_msg)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

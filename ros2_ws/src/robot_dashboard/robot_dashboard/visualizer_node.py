"""Telemetry visualizer GUI node for the mobile robot dashboard."""

import subprocess
import threading
import time
import tkinter as tk

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray


class VisualizerNode(Node):
    """ROS 2 node that subscribes to /keyboard_command and feeds the GUI."""

    def __init__(self, state_lock, state):
        """Initialize the visualizer node."""
        super().__init__('robot_dashboard_node')
        self._state_lock = state_lock
        self._state = state

        self.create_subscription(
            Int32MultiArray,
            '/keyboard_command',
            self._command_cb,
            10,
        )
        self.get_logger().info(
            'Robot Dashboard node started. Subscribing to /keyboard_command.'
        )

    def _command_cb(self, msg):
        """Handle incoming [speed, up, down, left, right] array messages."""
        data = list(msg.data)
        if len(data) < 5:
            return
        speed, up, down, left, right = data[:5]
        with self._state_lock:
            self._state['speed'] = int(speed)
            self._state['up'] = bool(up)
            self._state['down'] = bool(down)
            self._state['left'] = bool(left)
            self._state['right'] = bool(right)
            self._state['raw'] = data
            self._state['last_rcv'] = time.time()
            self._state['count'] += 1


class DashboardGUI:
    """Tkinter GUI dashboard displaying live robot telemetry."""

    # ── Colours ──────────────────────────────────────────────────────
    C = {
        'bg': '#0f1117',
        'card': '#181b27',
        'border': '#272d45',
        'text': '#eef0f8',
        'dim': '#828aa6',
        'cyan': '#00e5ff',
        'green': '#00e676',
        'red': '#ff1744',
        'yellow': '#ffd600',
        'pad_off': '#232738',
        'pad_on': '#00e5ff',
        'pad_on_fg': '#0a0c14',
    }

    def __init__(self, root, state_lock, state):
        """Build the full dashboard UI."""
        self.root = root
        self._lock = state_lock
        self._state = state

        root.title('Robot Dashboard')
        root.geometry('560x700')
        root.minsize(480, 600)
        root.configure(bg=self.C['bg'])

        self._build()
        self._tick()

    # ── UI construction ───────────────────────────────────────────────

    def _build(self):
        # Header
        hdr = tk.Frame(self.root, bg=self.C['bg'], pady=14)
        hdr.pack(fill=tk.X, padx=22)

        tk.Label(
            hdr,
            text='🤖  Robot Telemetry Dashboard',
            font=('Ubuntu', 18, 'bold'),
            fg=self.C['text'],
            bg=self.C['bg'],
        ).pack(anchor=tk.W)

        self._status_lbl = tk.Label(
            hdr,
            text='● Waiting for /keyboard_command…',
            font=('Ubuntu', 10),
            fg=self.C['dim'],
            bg=self.C['bg'],
        )
        self._status_lbl.pack(anchor=tk.W, pady=(4, 0))

        # Speed card
        speed_card = self._card(self.root)
        tk.Label(
            speed_card,
            text='SPEED LEVEL',
            font=('Ubuntu', 9, 'bold'),
            fg=self.C['dim'],
            bg=self.C['card'],
        ).pack(anchor=tk.W)

        row = tk.Frame(speed_card, bg=self.C['card'])
        row.pack(fill=tk.X, pady=(6, 8))

        self._speed_lbl = tk.Label(
            row,
            text='0',
            font=('Ubuntu', 40, 'bold'),
            fg=self.C['cyan'],
            bg=self.C['card'],
        )
        self._speed_lbl.pack(side=tk.LEFT)

        tk.Label(
            row,
            text=' / 9',
            font=('Ubuntu', 16),
            fg=self.C['dim'],
            bg=self.C['card'],
        ).pack(side=tk.LEFT, pady=(18, 0))

        self._speed_bar = tk.Canvas(
            speed_card,
            height=14,
            bg='#202436',
            highlightthickness=0,
        )
        self._speed_bar.pack(fill=tk.X)

        # Direction HUD card
        hud_card = self._card(self.root, expand=True)
        tk.Label(
            hud_card,
            text='DIRECTIONAL HUD',
            font=('Ubuntu', 9, 'bold'),
            fg=self.C['dim'],
            bg=self.C['card'],
        ).pack(anchor=tk.W, pady=(0, 10))

        grid = tk.Frame(hud_card, bg=self.C['card'])
        grid.pack()

        self._btn_up = self._pad(grid, '▲\nFORWARD', 0, 1)
        self._btn_left = self._pad(grid, '◄\nLEFT', 1, 0)
        self._center_lbl = self._pad(grid, 'STOP', 1, 1, is_center=True)
        self._btn_right = self._pad(grid, '►\nRIGHT', 1, 2)
        self._btn_down = self._pad(grid, '▼\nBACKWARD', 2, 1)

        self._motion_lbl = tk.Label(
            hud_card,
            text='State: IDLE',
            font=('Ubuntu', 12, 'bold'),
            fg=self.C['text'],
            bg=self.C['card'],
        )
        self._motion_lbl.pack(pady=(14, 0))

        # Diagnostics card
        diag_card = self._card(self.root)
        left_col = tk.Frame(diag_card, bg=self.C['card'])
        left_col.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self._raw_lbl = tk.Label(
            left_col,
            text='Array: [0, 0, 0, 0, 0]',
            font=('Monospace', 10),
            fg=self.C['green'],
            bg=self.C['card'],
        )
        self._raw_lbl.pack(anchor=tk.W)

        self._count_lbl = tk.Label(
            left_col,
            text='Topic: /keyboard_command  |  Messages: 0',
            font=('Ubuntu', 9),
            fg=self.C['dim'],
            bg=self.C['card'],
        )
        self._count_lbl.pack(anchor=tk.W, pady=(3, 0))

        tk.Button(
            diag_card,
            text='📈  Open rqt_graph',
            font=('Ubuntu', 10, 'bold'),
            bg='#252c4a',
            fg=self.C['cyan'],
            activebackground='#334',
            activeforeground='#fff',
            relief=tk.FLAT,
            padx=12,
            pady=7,
            cursor='hand2',
            command=self._open_rqt,
        ).pack(side=tk.RIGHT)

    def _card(self, parent, expand=False):
        """Return a styled dark card frame."""
        f = tk.Frame(
            parent,
            bg=self.C['card'],
            highlightbackground=self.C['border'],
            highlightthickness=1,
            padx=20,
            pady=16,
        )
        f.pack(
            fill=tk.BOTH if expand else tk.X,
            expand=expand,
            padx=22,
            pady=(0, 14),
        )
        return f

    def _pad(self, parent, text, row, col, is_center=False):
        """Create a directional pad cell."""
        lbl = tk.Label(
            parent,
            text=text,
            font=('Ubuntu', 11, 'bold'),
            width=9,
            height=3,
            bg='#2a1f2d' if is_center else self.C['pad_off'],
            fg=self.C['yellow'] if is_center else self.C['dim'],
        )
        lbl.grid(row=row, column=col, padx=7, pady=7)
        return lbl

    # ── Helpers ───────────────────────────────────────────────────────

    def _open_rqt(self):
        """Launch rqt_graph in a background process."""
        try:
            subprocess.Popen(['ros2', 'run', 'rqt_graph', 'rqt_graph'])
        except Exception as e:
            print(f'Could not open rqt_graph: {e}')

    def _set_pad(self, widget, active):
        if active:
            widget.config(bg=self.C['pad_on'], fg=self.C['pad_on_fg'])
        else:
            widget.config(bg=self.C['pad_off'], fg=self.C['dim'])

    # ── Render loop ───────────────────────────────────────────────────

    def _tick(self):
        with self._lock:
            s = self._state.copy()

        now = time.time()
        live = s['last_rcv'] > 0 and (now - s['last_rcv']) < 1.0

        # Status badge
        if live:
            self._status_lbl.config(
                text='● Connected — receiving from /keyboard_command',
                fg=self.C['green'],
            )
        else:
            self._status_lbl.config(
                text='○ Waiting for messages on /keyboard_command…',
                fg=self.C['dim'],
            )

        # Speed
        spd = s['speed']
        self._speed_lbl.config(text=str(spd))
        w = self._speed_bar.winfo_width()
        if w > 1:
            self._speed_bar.delete('all')
            pct = max(0.0, min(1.0, spd / 9.0))
            colour = self.C['cyan']
            if spd >= 7:
                colour = self.C['red']
            elif spd >= 4:
                colour = self.C['yellow']
            self._speed_bar.create_rectangle(0, 0, w * pct, 14, fill=colour, width=0)

        # Directional pads
        self._set_pad(self._btn_up, s['up'])
        self._set_pad(self._btn_down, s['down'])
        self._set_pad(self._btn_left, s['left'])
        self._set_pad(self._btn_right, s['right'])

        moving = s['up'] or s['down'] or s['left'] or s['right']
        if moving:
            dirs = []
            if s['up']:
                dirs.append('FORWARD')
            if s['down']:
                dirs.append('BACKWARD')
            if s['left']:
                dirs.append('LEFT')
            if s['right']:
                dirs.append('RIGHT')
            self._center_lbl.config(text='MOVING', bg='#17352f', fg=self.C['green'])
            self._motion_lbl.config(
                text=f'State: {" + ".join(dirs)}  (Speed {spd})',
                fg=self.C['cyan'],
            )
        else:
            self._center_lbl.config(text='STOP', bg='#3b1d24', fg=self.C['red'])
            self._motion_lbl.config(text='State: IDLE / STOPPED', fg=self.C['dim'])

        # Diagnostics
        self._raw_lbl.config(text=f'Array: {s["raw"]}')
        self._count_lbl.config(
            text=f'Topic: /keyboard_command  |  Messages: {s["count"]}'
        )

        # Schedule next frame ~30 FPS
        self.root.after(33, self._tick)


def main(args=None):
    """Entry point for the robot dashboard visualizer."""
    rclpy.init(args=args)

    lock = threading.Lock()
    state = {
        'speed': 0,
        'up': False,
        'down': False,
        'left': False,
        'right': False,
        'raw': [0, 0, 0, 0, 0],
        'last_rcv': 0.0,
        'count': 0,
    }

    node = VisualizerNode(lock, state)

    # ROS spin in background thread — GUI must stay on the main thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    root = tk.Tk()
    DashboardGUI(root, lock, state)

    def _on_close():
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        root.destroy()

    root.protocol('WM_DELETE_WINDOW', _on_close)

    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()

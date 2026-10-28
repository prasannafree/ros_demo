"""Robot Dashboard — live telemetry + 2-D position simulation."""

import math
import subprocess
import threading
import time
import tkinter as tk

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Int32MultiArray
from nav_msgs.msg import Odometry


# ──────────────────────────────────────────────────────────────────────────────
# ROS 2 Node — subscribes to every topic and writes into a shared state dict
# ──────────────────────────────────────────────────────────────────────────────

class VisualizerNode(Node):
    """Subscribe to all robot topics and populate the shared GUI state."""

    def __init__(self, state_lock, state):
        super().__init__('robot_dashboard_node')
        self._lock = state_lock
        self._state = state

        # /keyboard_command  →  [speed, up, down, left, right]
        self.create_subscription(
            Int32MultiArray,
            '/keyboard_command',
            self._cmd_cb,
            10,
        )

        # /left_wheel_speed  and  /right_wheel_speed  (rad/s)
        self.create_subscription(
            Float64,
            '/left_wheel_speed',
            self._left_cb,
            10,
        )
        self.create_subscription(
            Float64,
            '/right_wheel_speed',
            self._right_cb,
            10,
        )

        # /odom  →  pose (x, y, theta) + twist
        self.create_subscription(
            Odometry,
            '/odom',
            self._odom_cb,
            10,
        )

        self.get_logger().info(
            'Dashboard node started — subscribed to '
            '/keyboard_command, /left_wheel_speed, /right_wheel_speed, /odom'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cmd_cb(self, msg):
        data = list(msg.data)
        if len(data) < 5:
            return
        speed, up, down, left, right = data[:5]
        with self._lock:
            self._state['speed'] = int(speed)
            self._state['up'] = bool(up)
            self._state['down'] = bool(down)
            self._state['left'] = bool(left)
            self._state['right'] = bool(right)
            self._state['raw'] = data
            self._state['last_cmd'] = time.time()
            self._state['cmd_count'] += 1

    def _left_cb(self, msg):
        with self._lock:
            self._state['left_vel'] = msg.data
            self._state['last_wheel'] = time.time()

    def _right_cb(self, msg):
        with self._lock:
            self._state['right_vel'] = msg.data
            self._state['last_wheel'] = time.time()

    def _odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        theta = 2.0 * math.atan2(qz, qw)
        lin = msg.twist.twist.linear.x
        ang = msg.twist.twist.angular.z
        with self._lock:
            self._state['x'] = x
            self._state['y'] = y
            self._state['theta'] = theta
            self._state['lin_vel'] = lin
            self._state['ang_vel'] = ang
            self._state['last_odom'] = time.time()
            self._state['odom_count'] += 1
            # Append trail point (keep last 2000)
            self._state['trail'].append((x, y))
            if len(self._state['trail']) > 2000:
                self._state['trail'].pop(0)


# ──────────────────────────────────────────────────────────────────────────────
# GUI
# ──────────────────────────────────────────────────────────────────────────────

class DashboardGUI:
    """Two-panel dashboard: left = teleop HUD, right = 2-D sim map."""

    C = {
        'bg':        '#0d1117',
        'panel':     '#161b27',
        'card':      '#1c2233',
        'border':    '#2a3350',
        'text':      '#e6eaf4',
        'dim':       '#6b7699',
        'cyan':      '#00e5ff',
        'green':     '#00e676',
        'red':       '#ff1744',
        'yellow':    '#ffd600',
        'purple':    '#bb86fc',
        'pad_off':   '#252d45',
        'pad_on':    '#00e5ff',
        'pad_on_fg': '#050810',
        # map colours
        'map_bg':    '#090d18',
        'grid':      '#17213a',
        'trail':     '#00e5ff',
        'robot':     '#00e676',
        'heading':   '#ff1744',
        'origin':    '#ffd600',
    }

    # How many metres one pixel represents at default zoom
    _PX_PER_M = 60.0

    def __init__(self, root, lock, state):
        self.root = root
        self._lock = lock
        self._state = state

        # Map view offset (metres from world origin to canvas centre)
        self._map_ox = 0.0
        self._map_oy = 0.0
        self._zoom = self._PX_PER_M     # pixels per metre
        self._follow = True             # auto-follow robot

        root.title('🤖  Robot Dashboard')
        root.geometry('1200x760')
        root.minsize(900, 600)
        root.configure(bg=self.C['bg'])

        self._build()
        self._tick()

    # ── Layout ────────────────────────────────────────────────────────────────

    def _build(self):
        # ── Top header bar ────────────────────────────────────────────
        hdr = tk.Frame(self.root, bg=self.C['bg'], pady=10)
        hdr.pack(fill=tk.X, padx=18)

        tk.Label(
            hdr,
            text='🤖  Mobile Robot Dashboard',
            font=('Ubuntu', 20, 'bold'),
            fg=self.C['text'],
            bg=self.C['bg'],
        ).pack(side=tk.LEFT)

        self._status_lbl = tk.Label(
            hdr,
            text='○  Waiting…',
            font=('Ubuntu', 10),
            fg=self.C['dim'],
            bg=self.C['bg'],
        )
        self._status_lbl.pack(side=tk.LEFT, padx=24)

        # ── Main body: left panel + right map ─────────────────────────
        body = tk.Frame(self.root, bg=self.C['bg'])
        body.pack(fill=tk.BOTH, expand=True, padx=14, pady=(0, 12))

        # LEFT column (fixed width ~320 px)
        left = tk.Frame(body, bg=self.C['bg'], width=330)
        left.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
        left.pack_propagate(False)

        self._build_speed(left)
        self._build_hud(left)
        self._build_wheels(left)
        self._build_pose(left)
        self._build_diag(left)

        # RIGHT column: 2-D simulation map
        right = tk.Frame(body, bg=self.C['bg'])
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self._build_map(right)

    # ── Left panel cards ──────────────────────────────────────────────────────

    def _card(self, parent, title):
        outer = tk.Frame(
            parent,
            bg=self.C['card'],
            highlightbackground=self.C['border'],
            highlightthickness=1,
        )
        outer.pack(fill=tk.X, pady=(0, 10))
        if title:
            tk.Label(
                outer,
                text=title,
                font=('Ubuntu', 8, 'bold'),
                fg=self.C['dim'],
                bg=self.C['card'],
            ).pack(anchor=tk.W, padx=14, pady=(8, 0))
        inner = tk.Frame(outer, bg=self.C['card'], padx=14, pady=8)
        inner.pack(fill=tk.X)
        return inner

    def _build_speed(self, parent):
        c = self._card(parent, 'SPEED LEVEL')

        row = tk.Frame(c, bg=self.C['card'])
        row.pack(fill=tk.X)

        self._speed_lbl = tk.Label(
            row, text='0',
            font=('Ubuntu', 44, 'bold'),
            fg=self.C['cyan'], bg=self.C['card'],
        )
        self._speed_lbl.pack(side=tk.LEFT)

        tk.Label(
            row, text=' / 9',
            font=('Ubuntu', 18),
            fg=self.C['dim'], bg=self.C['card'],
        ).pack(side=tk.LEFT, pady=(20, 0))

        self._speed_bar = tk.Canvas(
            c, height=12, bg='#141925', highlightthickness=0,
        )
        self._speed_bar.pack(fill=tk.X, pady=(6, 0))

    def _build_hud(self, parent):
        c = self._card(parent, 'DIRECTIONAL HUD')

        grid = tk.Frame(c, bg=self.C['card'])
        grid.pack()

        self._btn_up    = self._pad_btn(grid, '▲\nFWD',  0, 1)
        self._btn_left  = self._pad_btn(grid, '◄\nLEFT', 1, 0)
        self._center_lbl= self._pad_btn(grid, 'STOP',    1, 1, center=True)
        self._btn_right = self._pad_btn(grid, '►\nRGHT', 1, 2)
        self._btn_down  = self._pad_btn(grid, '▼\nBCK',  2, 1)

        self._motion_lbl = tk.Label(
            c, text='IDLE',
            font=('Ubuntu', 11, 'bold'),
            fg=self.C['dim'], bg=self.C['card'],
        )
        self._motion_lbl.pack(pady=(8, 0))

    def _pad_btn(self, parent, text, row, col, center=False):
        lbl = tk.Label(
            parent, text=text,
            font=('Ubuntu', 10, 'bold'),
            width=7, height=3,
            bg='#261c30' if center else self.C['pad_off'],
            fg=self.C['yellow'] if center else self.C['dim'],
        )
        lbl.grid(row=row, column=col, padx=5, pady=5)
        return lbl

    def _build_wheels(self, parent):
        c = self._card(parent, 'WHEEL SPEEDS  (rad/s)')

        for side in ('LEFT', 'RIGHT'):
            row = tk.Frame(c, bg=self.C['card'])
            row.pack(fill=tk.X, pady=3)

            tk.Label(
                row, text=f'{side}',
                font=('Ubuntu', 9), width=6,
                fg=self.C['dim'], bg=self.C['card'],
            ).pack(side=tk.LEFT)

            bar_bg = tk.Canvas(
                row, height=16, bg='#141925', highlightthickness=0,
            )
            bar_bg.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(6, 8))

            val_lbl = tk.Label(
                row, text='0.000',
                font=('Monospace', 10, 'bold'),
                fg=self.C['cyan'], bg=self.C['card'], width=7,
            )
            val_lbl.pack(side=tk.LEFT)

            if side == 'LEFT':
                self._lw_bar = bar_bg
                self._lw_lbl = val_lbl
            else:
                self._rw_bar = bar_bg
                self._rw_lbl = val_lbl

    def _build_pose(self, parent):
        c = self._card(parent, 'ODOMETRY  —  CURRENT POSE')

        grid = tk.Frame(c, bg=self.C['card'])
        grid.pack(fill=tk.X)

        labels = [
            ('X',     'm',   'cyan'),
            ('Y',     'm',   'cyan'),
            ('θ',     '°',   'purple'),
            ('v',     'm/s', 'green'),
            ('ω',     'r/s', 'yellow'),
        ]
        self._pose_vals = {}
        for i, (name, unit, colour) in enumerate(labels):
            tk.Label(
                grid, text=name,
                font=('Ubuntu', 10, 'bold'),
                fg=self.C['dim'], bg=self.C['card'], width=3,
            ).grid(row=i, column=0, sticky=tk.W, pady=2)

            val = tk.Label(
                grid, text='0.000',
                font=('Monospace', 13, 'bold'),
                fg=self.C[colour], bg=self.C['card'], width=9, anchor=tk.E,
            )
            val.grid(row=i, column=1, sticky=tk.E)

            tk.Label(
                grid, text=unit,
                font=('Ubuntu', 9),
                fg=self.C['dim'], bg=self.C['card'],
            ).grid(row=i, column=2, sticky=tk.W, padx=(4, 0))

            self._pose_vals[name] = val

        # Odom msg counter
        self._odom_count_lbl = tk.Label(
            c, text='/odom  msgs: 0',
            font=('Ubuntu', 8),
            fg=self.C['dim'], bg=self.C['card'],
        )
        self._odom_count_lbl.pack(anchor=tk.W, pady=(6, 0))

    def _build_diag(self, parent):
        c = self._card(parent, 'DIAGNOSTICS')

        self._raw_lbl = tk.Label(
            c, text='/keyboard_command:  [0,0,0,0,0]',
            font=('Monospace', 9),
            fg=self.C['green'], bg=self.C['card'], anchor=tk.W,
        )
        self._raw_lbl.pack(fill=tk.X)

        self._cmd_count_lbl = tk.Label(
            c, text='cmd msgs: 0',
            font=('Ubuntu', 8),
            fg=self.C['dim'], bg=self.C['card'], anchor=tk.W,
        )
        self._cmd_count_lbl.pack(fill=tk.X, pady=(3, 0))

        tk.Button(
            c,
            text='📈  rqt_graph',
            font=('Ubuntu', 9, 'bold'),
            bg='#1e2740', fg=self.C['cyan'],
            activebackground='#2a3558',
            activeforeground='#fff',
            relief=tk.FLAT, padx=10, pady=5,
            cursor='hand2',
            command=self._open_rqt,
        ).pack(anchor=tk.W, pady=(8, 0))

    # ── Map panel ─────────────────────────────────────────────────────────────

    def _build_map(self, parent):
        hdr = tk.Frame(parent, bg=self.C['bg'])
        hdr.pack(fill=tk.X, pady=(0, 6))

        tk.Label(
            hdr,
            text='2-D POSITION MAP',
            font=('Ubuntu', 9, 'bold'),
            fg=self.C['dim'], bg=self.C['bg'],
        ).pack(side=tk.LEFT)

        # Zoom controls
        tk.Button(
            hdr, text='＋', font=('Ubuntu', 11, 'bold'),
            bg='#1e2740', fg=self.C['cyan'],
            relief=tk.FLAT, padx=8, pady=2, cursor='hand2',
            command=self._zoom_in,
        ).pack(side=tk.RIGHT, padx=(4, 0))

        tk.Button(
            hdr, text='－', font=('Ubuntu', 11, 'bold'),
            bg='#1e2740', fg=self.C['cyan'],
            relief=tk.FLAT, padx=8, pady=2, cursor='hand2',
            command=self._zoom_out,
        ).pack(side=tk.RIGHT, padx=(4, 0))

        self._follow_btn = tk.Button(
            hdr, text='⊙ Follow ON',
            font=('Ubuntu', 9, 'bold'),
            bg='#17352f', fg=self.C['green'],
            relief=tk.FLAT, padx=8, pady=2, cursor='hand2',
            command=self._toggle_follow,
        )
        self._follow_btn.pack(side=tk.RIGHT, padx=(4, 0))

        tk.Button(
            hdr, text='⟳ Reset',
            font=('Ubuntu', 9, 'bold'),
            bg='#1e2740', fg=self.C['dim'],
            relief=tk.FLAT, padx=8, pady=2, cursor='hand2',
            command=self._reset_view,
        ).pack(side=tk.RIGHT, padx=(4, 0))

        # Canvas
        map_frame = tk.Frame(
            parent,
            bg=self.C['map_bg'],
            highlightbackground=self.C['border'],
            highlightthickness=1,
        )
        map_frame.pack(fill=tk.BOTH, expand=True)

        self._map = tk.Canvas(
            map_frame,
            bg=self.C['map_bg'],
            highlightthickness=0,
        )
        self._map.pack(fill=tk.BOTH, expand=True)

        # Mouse drag to pan
        self._drag_start = None
        self._map.bind('<ButtonPress-1>',   self._drag_begin)
        self._map.bind('<B1-Motion>',       self._drag_move)
        self._map.bind('<MouseWheel>',      self._scroll_zoom)   # Windows/Mac
        self._map.bind('<Button-4>',        self._scroll_zoom)   # Linux up
        self._map.bind('<Button-5>',        self._scroll_zoom)   # Linux down

    # ── Map interaction ───────────────────────────────────────────────────────

    def _zoom_in(self):
        self._zoom = min(self._zoom * 1.3, 500.0)

    def _zoom_out(self):
        self._zoom = max(self._zoom / 1.3, 5.0)

    def _toggle_follow(self):
        self._follow = not self._follow
        if self._follow:
            self._follow_btn.config(text='⊙ Follow ON',  bg='#17352f', fg=self.C['green'])
        else:
            self._follow_btn.config(text='⊙ Follow OFF', bg='#251c1c', fg=self.C['dim'])

    def _reset_view(self):
        self._map_ox = 0.0
        self._map_oy = 0.0
        self._zoom = self._PX_PER_M
        self._follow = True
        self._follow_btn.config(text='⊙ Follow ON', bg='#17352f', fg=self.C['green'])
        # Clear trail
        with self._lock:
            self._state['trail'].clear()
            self._state['x'] = 0.0
            self._state['y'] = 0.0
            self._state['theta'] = 0.0

    def _drag_begin(self, event):
        self._drag_start = (event.x, event.y)
        self._follow = False
        self._follow_btn.config(text='⊙ Follow OFF', bg='#251c1c', fg=self.C['dim'])

    def _drag_move(self, event):
        if self._drag_start is None:
            return
        dx = event.x - self._drag_start[0]
        dy = event.y - self._drag_start[1]
        self._map_ox -= dx / self._zoom
        self._map_oy += dy / self._zoom
        self._drag_start = (event.x, event.y)

    def _scroll_zoom(self, event):
        if event.num == 4 or event.delta > 0:
            self._zoom_in()
        else:
            self._zoom_out()

    # ── World ↔ canvas coordinate conversion ─────────────────────────────────

    def _w2c(self, wx, wy):
        """World (m) → canvas (px)."""
        cw = self._map.winfo_width()  or 400
        ch = self._map.winfo_height() or 400
        cx = cw / 2 + (wx - self._map_ox) * self._zoom
        cy = ch / 2 - (wy - self._map_oy) * self._zoom   # Y-axis flipped
        return cx, cy

    # ── Map drawing ───────────────────────────────────────────────────────────

    def _draw_map(self, rx, ry, theta):
        c = self._map
        cw = c.winfo_width()  or 400
        ch = c.winfo_height() or 400

        if self._follow:
            self._map_ox = rx
            self._map_oy = ry

        c.delete('all')

        # Grid lines
        grid_m = 1.0            # 1-metre grid
        if self._zoom > 200:
            grid_m = 0.25
        elif self._zoom < 20:
            grid_m = 5.0

        # Visible world extents
        left_w  =  self._map_ox - cw / (2 * self._zoom)
        right_w =  self._map_ox + cw / (2 * self._zoom)
        bot_w   =  self._map_oy - ch / (2 * self._zoom)
        top_w   =  self._map_oy + ch / (2 * self._zoom)

        import math as _m
        x0 = _m.floor(left_w  / grid_m) * grid_m
        y0 = _m.floor(bot_w   / grid_m) * grid_m

        gx = x0
        while gx <= right_w + grid_m:
            px, _ = self._w2c(gx, 0)
            c.create_line(px, 0, px, ch, fill=self.C['grid'], width=1)
            # Label every major line
            if abs(round(gx / grid_m) % 5) == 0:
                c.create_text(px + 3, ch - 12, text=f'{gx:.1f}',
                              fill=self.C['dim'], font=('Ubuntu', 7), anchor=tk.W)
            gx += grid_m

        gy = y0
        while gy <= top_w + grid_m:
            _, py = self._w2c(0, gy)
            c.create_line(0, py, cw, py, fill=self.C['grid'], width=1)
            if abs(round(gy / grid_m) % 5) == 0:
                c.create_text(8, py - 3, text=f'{gy:.1f}',
                              fill=self.C['dim'], font=('Ubuntu', 7), anchor=tk.W)
            gy += grid_m

        # Axes
        ox, oy = self._w2c(0, 0)
        c.create_line(ox, 0, ox, ch, fill=self.C['origin'], width=1, dash=(4, 4))
        c.create_line(0, oy, cw, oy, fill=self.C['origin'], width=1, dash=(4, 4))
        c.create_text(ox + 4, oy - 4, text='O', fill=self.C['origin'],
                      font=('Ubuntu', 9, 'bold'))

        # Trail
        with self._lock:
            trail = list(self._state['trail'])

        if len(trail) >= 2:
            pts = []
            for wx, wy in trail:
                px, py = self._w2c(wx, wy)
                pts.extend([px, py])
            c.create_line(*pts, fill=self.C['trail'], width=2, smooth=True)

        # Robot body
        R = max(8, self._zoom * 0.18)   # radius in pixels, scales with zoom
        rpx, rpy = self._w2c(rx, ry)

        # Glow
        for r, alpha in [(R * 2.2, '#0a3040'), (R * 1.5, '#0d4f60')]:
            c.create_oval(rpx - r, rpy - r, rpx + r, rpy + r,
                          fill=alpha, outline='')

        # Body circle
        c.create_oval(rpx - R, rpy - R, rpx + R, rpy + R,
                      fill=self.C['robot'], outline='#ffffff', width=2)

        # Heading arrow (direction the robot faces)
        hlen = R * 1.8
        hx = rpx + hlen * math.cos(theta)
        hy = rpy - hlen * math.sin(theta)      # canvas Y is inverted
        c.create_line(rpx, rpy, hx, hy,
                      fill=self.C['heading'], width=3, arrow=tk.LAST,
                      arrowshape=(10, 12, 4))

        # Coordinate overlay
        deg = math.degrees(theta)
        overlay = (
            f'x = {rx:+.3f} m\n'
            f'y = {ry:+.3f} m\n'
            f'θ = {deg:+.1f}°'
        )
        c.create_text(
            cw - 10, 10,
            text=overlay,
            fill=self.C['cyan'],
            font=('Monospace', 11, 'bold'),
            anchor=tk.NE,
            justify=tk.RIGHT,
        )

        # Scale bar (bottom-left)
        bar_m = 1.0
        if self._zoom > 200:
            bar_m = 0.25
        elif self._zoom < 20:
            bar_m = 5.0
        bar_px = bar_m * self._zoom
        bx1, by = 20, ch - 22
        c.create_line(bx1, by, bx1 + bar_px, by, fill=self.C['text'], width=3)
        c.create_line(bx1, by - 5, bx1, by + 5, fill=self.C['text'], width=2)
        c.create_line(bx1 + bar_px, by - 5, bx1 + bar_px, by + 5,
                      fill=self.C['text'], width=2)
        c.create_text(bx1 + bar_px / 2, by - 8,
                      text=f'{bar_m} m', fill=self.C['text'],
                      font=('Ubuntu', 8))

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _open_rqt(self):
        try:
            subprocess.Popen(['ros2', 'run', 'rqt_graph', 'rqt_graph'])
        except Exception as e:
            print(f'Could not open rqt_graph: {e}')

    def _set_pad(self, widget, active):
        if active:
            widget.config(bg=self.C['pad_on'], fg=self.C['pad_on_fg'])
        else:
            widget.config(bg=self.C['pad_off'], fg=self.C['dim'])

    def _draw_bar(self, canvas, value, max_val=10.0):
        w = canvas.winfo_width()
        if w < 2:
            return
        canvas.delete('all')
        pct = max(0.0, min(1.0, abs(value) / max_val))
        col = self.C['cyan'] if value >= 0 else self.C['yellow']
        canvas.create_rectangle(0, 0, w * pct, 16, fill=col, width=0)

    # ── Render loop ───────────────────────────────────────────────────────────

    def _tick(self):
        with self._lock:
            s = dict(self._state)

        now = time.time()

        # Status
        cmd_live  = s['last_cmd']  > 0 and (now - s['last_cmd'])  < 1.0
        odom_live = s['last_odom'] > 0 and (now - s['last_odom']) < 1.0

        if cmd_live and odom_live:
            self._status_lbl.config(
                text='● cmd  ● odom  — all topics live',
                fg=self.C['green'])
        elif cmd_live:
            self._status_lbl.config(
                text='● cmd  ○ odom  — waiting for /odom',
                fg=self.C['yellow'])
        elif odom_live:
            self._status_lbl.config(
                text='○ cmd  ● odom  — waiting for /keyboard_command',
                fg=self.C['yellow'])
        else:
            self._status_lbl.config(
                text='○ Waiting for topics…',
                fg=self.C['dim'])

        # Speed
        spd = s['speed']
        self._speed_lbl.config(text=str(spd))
        w = self._speed_bar.winfo_width()
        if w > 1:
            self._speed_bar.delete('all')
            pct = max(0.0, min(1.0, spd / 9.0))
            col = (self.C['red'] if spd >= 7
                   else self.C['yellow'] if spd >= 4
                   else self.C['cyan'])
            self._speed_bar.create_rectangle(0, 0, w * pct, 12, fill=col, width=0)

        # Directional pads
        self._set_pad(self._btn_up,    s['up'])
        self._set_pad(self._btn_down,  s['down'])
        self._set_pad(self._btn_left,  s['left'])
        self._set_pad(self._btn_right, s['right'])

        moving = s['up'] or s['down'] or s['left'] or s['right']
        if moving:
            dirs = (
                (['FWD'] if s['up']    else []) +
                (['BCK'] if s['down']  else []) +
                (['LFT'] if s['left']  else []) +
                (['RGT'] if s['right'] else [])
            )
            self._center_lbl.config(text='GO', bg='#17352f', fg=self.C['green'])
            self._motion_lbl.config(
                text=f'{" + ".join(dirs)}  (spd {spd})',
                fg=self.C['cyan'])
        else:
            self._center_lbl.config(text='STOP', bg='#3b1d24', fg=self.C['red'])
            self._motion_lbl.config(text='IDLE / STOPPED', fg=self.C['dim'])

        # Wheel speed bars
        lv = s['left_vel']
        rv = s['right_vel']
        self._lw_lbl.config(text=f'{lv:+.3f}')
        self._rw_lbl.config(text=f'{rv:+.3f}')
        self._draw_bar(self._lw_bar, lv)
        self._draw_bar(self._rw_bar, rv)

        # Pose readout
        rx, ry, theta = s['x'], s['y'], s['theta']
        self._pose_vals['X'].config(text=f'{rx:+.4f}')
        self._pose_vals['Y'].config(text=f'{ry:+.4f}')
        self._pose_vals['θ'].config(text=f'{math.degrees(theta):+.2f}')
        self._pose_vals['v'].config(text=f'{s["lin_vel"]:+.4f}')
        self._pose_vals['ω'].config(text=f'{s["ang_vel"]:+.4f}')
        self._odom_count_lbl.config(text=f'/odom  msgs: {s["odom_count"]}')

        # Diagnostics
        self._raw_lbl.config(text=f'/keyboard_command: {s["raw"]}')
        self._cmd_count_lbl.config(text=f'cmd msgs: {s["cmd_count"]}')

        # 2-D map
        self._draw_map(rx, ry, theta)

        # ~30 FPS
        self.root.after(33, self._tick)


# ──────────────────────────────────────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────────────────────────────────────

def main(args=None):
    """Launch the robot dashboard."""
    rclpy.init(args=args)

    lock = threading.Lock()
    state = {
        # keyboard
        'speed': 0, 'up': False, 'down': False, 'left': False, 'right': False,
        'raw': [0, 0, 0, 0, 0],
        'last_cmd': 0.0, 'cmd_count': 0,
        # wheels
        'left_vel': 0.0, 'right_vel': 0.0,
        'last_wheel': 0.0,
        # odometry
        'x': 0.0, 'y': 0.0, 'theta': 0.0,
        'lin_vel': 0.0, 'ang_vel': 0.0,
        'last_odom': 0.0, 'odom_count': 0,
        'trail': [],
    }

    node = VisualizerNode(lock, state)

    # ROS spin in a background thread (GUI must run on the main thread)
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True
    )
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



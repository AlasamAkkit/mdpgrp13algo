"""Interactive, dependency-free simulator for the SC2079 MDP Task 1 planner.

Run from the ``Algorithm`` directory with::

    python -m task1_simulator

The UI deliberately stays offline: it never opens the RPi socket used by the
submitted ``main.txt``.  It consumes the pure planner in :mod:`planner` and
animates the resulting command stream.
"""

from __future__ import annotations

import argparse
import json
import math
import time
import tkinter as tk
from dataclasses import replace
from pathlib import Path
from tkinter import messagebox, ttk
from typing import Iterable, Optional

try:  # Package execution: python -m task1_simulator
    from .planner import (
        GridPose,
        Heading,
        Motion,
        Obstacle,
        PlanResult,
        PoseSample,
        Task1Planner,
    )
except ImportError:  # Direct execution while developing.
    from planner import (  # type: ignore
        GridPose,
        Heading,
        Motion,
        Obstacle,
        PlanResult,
        PoseSample,
        Task1Planner,
    )


ARENA_CELLS = 20
START_ZONE_CELLS = 4
TASK_TIMEOUT_S = 6 * 60
START_POSE = GridPose(1, 1, Heading.N)


COLORS = {
    "bg": "#08111f",
    "panel": "#101c30",
    "panel_alt": "#15243b",
    "panel_lift": "#1b2c47",
    "text": "#eef5ff",
    "muted": "#91a5bf",
    "grid": "#2b3b55",
    "grid_major": "#435776",
    "border": "#587092",
    "start": "#3b3324",
    "start_line": "#d7ae55",
    "cyan": "#42d4e6",
    "cyan_soft": "#245566",
    "green": "#46dfa6",
    "amber": "#ffca67",
    "red": "#ff6f78",
    "obstacle": "#273750",
    "obstacle_selected": "#405777",
    "white": "#ffffff",
}


PRESETS = {
    "Checklist demo - 5": [
        (1, 4, 8, "N", 11),
        (2, 10, 13, "S", 20),
        (3, 15, 15, "W", 36),
        (4, 13, 5, "E", 27),
        (5, 6, 17, "S", 40),
    ],
    "Task 1 stress - 8": [
        (1, 5, 8, "S", 14),
        (2, 9, 15, "E", 22),
        (3, 15, 12, "W", 31),
        (4, 13, 4, "N", 38),
        (5, 3, 15, "S", 11),
        (6, 18, 8, "W", 20),
        (7, 8, 5, "N", 36),
        (8, 6, 18, "S", 40),
    ],
    "Open arena - 4": [
        (1, 5, 8, "S", 14),
        (2, 9, 15, "E", 22),
        (3, 15, 12, "W", 31),
        (4, 13, 4, "N", 38),
    ],
}


def heading_from_text(value: str) -> Heading:
    return Heading[value.strip().upper()]


def heading_degrees(heading: Heading) -> float:
    """Return mathematical degrees (east=0, north=90)."""

    mapping = {Heading.E: 0.0, Heading.N: 90.0, Heading.W: 180.0, Heading.S: -90.0}
    return mapping[heading]


def shortest_angle_delta(a: float, b: float) -> float:
    return (b - a + 180.0) % 360.0 - 180.0


class Task1Simulator(tk.Tk):
    """Tk application that edits an arena and animates a :class:`PlanResult`."""

    def __init__(self) -> None:
        super().__init__()
        self.title("MDP Task 1 · Algorithm Simulator")
        self.geometry("1260x760")
        self.minsize(1080, 660)
        self.configure(bg=COLORS["bg"])

        self.obstacles: list[Obstacle] = []
        self.selected_obstacle_id: Optional[int] = None
        self.plan_result: Optional[PlanResult] = None
        self.current_pose = PoseSample(
            float(START_POSE.x),
            float(START_POSE.y),
            heading_degrees(START_POSE.heading),
        )
        self.motion_index = 0
        self.motion_elapsed_s = 0.0
        self.elapsed_sim_s = 0.0
        self.playing = False
        self.last_tick = time.perf_counter()
        self.recognized_ids: set[int] = set()
        self.logged_scan_motions: set[int] = set()
        self.event_messages: list[str] = ["Waiting for route playback…"]
        self._drag_obstacle_id: Optional[int] = None
        self._syncing_controls = False
        self._canvas_geometry = (36.0, 18.0, 25.0)

        self._configure_styles()
        self._build_layout()
        self._bind_shortcuts()
        self._load_preset("Checklist demo - 5")
        self.after(16, self._animation_tick)

    # ------------------------------------------------------------------ UI
    def _configure_styles(self) -> None:
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure("App.TFrame", background=COLORS["bg"])
        style.configure("Panel.TFrame", background=COLORS["panel"])
        style.configure("Lift.TFrame", background=COLORS["panel_alt"])
        style.configure(
            "Title.TLabel",
            background=COLORS["bg"],
            foreground=COLORS["text"],
            font=("Segoe UI Semibold", 20),
        )
        style.configure(
            "Subtitle.TLabel",
            background=COLORS["bg"],
            foreground=COLORS["muted"],
            font=("Segoe UI", 10),
        )
        style.configure(
            "Section.TLabel",
            background=COLORS["panel"],
            foreground=COLORS["text"],
            font=("Segoe UI Semibold", 11),
        )
        style.configure(
            "Body.TLabel",
            background=COLORS["panel"],
            foreground=COLORS["text"],
            font=("Segoe UI", 9),
        )
        style.configure(
            "Muted.TLabel",
            background=COLORS["panel"],
            foreground=COLORS["muted"],
            font=("Segoe UI", 9),
        )
        style.configure(
            "StatValue.TLabel",
            background=COLORS["panel_alt"],
            foreground=COLORS["text"],
            font=("Cascadia Mono", 15, "bold"),
        )
        style.configure(
            "StatLabel.TLabel",
            background=COLORS["panel_alt"],
            foreground=COLORS["muted"],
            font=("Segoe UI", 8),
        )
        style.configure(
            "Primary.TButton",
            background=COLORS["cyan"],
            foreground=COLORS["bg"],
            borderwidth=0,
            font=("Segoe UI Semibold", 9),
            padding=(12, 8),
        )
        style.map(
            "Primary.TButton",
            background=[("active", "#73e2ef"), ("disabled", COLORS["grid_major"])],
            foreground=[("disabled", COLORS["muted"])],
        )
        style.configure(
            "Secondary.TButton",
            background=COLORS["panel_lift"],
            foreground=COLORS["text"],
            bordercolor=COLORS["border"],
            lightcolor=COLORS["panel_lift"],
            darkcolor=COLORS["panel_lift"],
            font=("Segoe UI", 9),
            padding=(10, 7),
        )
        style.map("Secondary.TButton", background=[("active", COLORS["obstacle_selected"])])
        style.configure(
            "Danger.TButton",
            background=COLORS["panel_lift"],
            foreground=COLORS["red"],
            bordercolor=COLORS["border"],
            font=("Segoe UI", 9),
            padding=(10, 7),
        )
        style.map("Danger.TButton", background=[("active", COLORS["obstacle_selected"])])
        style.configure(
            "TCombobox",
            fieldbackground=COLORS["panel_lift"],
            background=COLORS["panel_lift"],
            foreground=COLORS["text"],
            arrowcolor=COLORS["muted"],
            bordercolor=COLORS["border"],
            padding=6,
        )
        style.map(
            "TCombobox",
            fieldbackground=[("readonly", COLORS["panel_lift"])],
            foreground=[("readonly", COLORS["text"])],
        )
        style.configure(
            "TSpinbox",
            fieldbackground=COLORS["panel_lift"],
            foreground=COLORS["text"],
            arrowcolor=COLORS["muted"],
            bordercolor=COLORS["border"],
            padding=5,
        )
        style.configure(
            "Treeview",
            background=COLORS["panel_alt"],
            fieldbackground=COLORS["panel_alt"],
            foreground=COLORS["text"],
            rowheight=25,
            borderwidth=0,
            font=("Cascadia Mono", 9),
        )
        style.map("Treeview", background=[("selected", COLORS["obstacle_selected"])])
        style.configure(
            "Treeview.Heading",
            background=COLORS["panel_lift"],
            foreground=COLORS["muted"],
            borderwidth=0,
            font=("Segoe UI Semibold", 8),
        )
        style.configure(
            "Horizontal.TScale",
            background=COLORS["panel"],
            troughcolor=COLORS["panel_lift"],
        )
        style.configure(
            "TCheckbutton",
            background=COLORS["panel"],
            foreground=COLORS["text"],
            font=("Segoe UI", 9),
        )
        style.map("TCheckbutton", background=[("active", COLORS["panel"])])

    def _build_layout(self) -> None:
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(1, weight=1)

        header = ttk.Frame(self, style="App.TFrame", padding=(22, 14, 22, 10))
        header.grid(row=0, column=0, sticky="ew")
        header.grid_columnconfigure(0, weight=1)
        ttk.Label(header, text="MDP Task 1  /  Algorithm Simulator", style="Title.TLabel").grid(
            row=0, column=0, sticky="w"
        )
        ttk.Label(
            header,
            text="Hamiltonian order · car-like A* · command-derived animation",
            style="Subtitle.TLabel",
        ).grid(row=1, column=0, sticky="w", pady=(2, 0))
        header_metrics = ttk.Frame(header, style="App.TFrame")
        header_metrics.grid(row=0, column=1, rowspan=2, sticky="e", padx=(14, 12))
        header_metrics.grid_columnconfigure((0, 1, 2), weight=1)
        self.stat_recognized = self._stat_card(header_metrics, 0, "RECOGNIZED", "0 / 0")
        self.stat_timer = self._stat_card(header_metrics, 1, "TASK CLOCK", "00:00")
        self.stat_distance = self._stat_card(header_metrics, 2, "DISTANCE", "—")

        self.header_status = tk.Label(
            header,
            text="OFFLINE DEMO",
            bg=COLORS["cyan_soft"],
            fg=COLORS["cyan"],
            font=("Segoe UI Semibold", 9),
            padx=10,
            pady=5,
        )
        self.header_status.grid(row=0, column=2, rowspan=2, sticky="e")

        content = ttk.Frame(self, style="App.TFrame", padding=(18, 4, 18, 18))
        content.grid(row=1, column=0, sticky="nsew")
        content.grid_columnconfigure(0, weight=1)
        content.grid_columnconfigure(1, minsize=370)
        content.grid_rowconfigure(0, weight=1)

        arena_frame = ttk.Frame(content, style="Panel.TFrame", padding=10)
        arena_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 12))
        arena_frame.grid_columnconfigure(0, weight=1)
        arena_frame.grid_rowconfigure(1, weight=1)

        arena_bar = ttk.Frame(arena_frame, style="Panel.TFrame")
        arena_bar.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        arena_bar.grid_columnconfigure(0, weight=1)
        ttk.Label(arena_bar, text="2.0 m × 2.0 m movement area", style="Section.TLabel").grid(
            row=0, column=0, sticky="w"
        )
        self.cursor_label = ttk.Label(
            arena_bar, text="cell —  ·  robot (1, 1, N)", style="Muted.TLabel"
        )
        self.cursor_label.grid(row=0, column=1, sticky="e")
        self.command_label = ttk.Label(
            arena_bar, text="Ready · ROBOT, 1, 1, N", style="Muted.TLabel"
        )
        self.command_label.grid(row=1, column=0, sticky="w", pady=(3, 0))
        self.latest_event_label = ttk.Label(
            arena_bar, text="No recognitions yet", style="Muted.TLabel"
        )
        self.latest_event_label.grid(row=1, column=1, sticky="e", pady=(3, 0))

        self.canvas = tk.Canvas(
            arena_frame,
            bg=COLORS["panel"],
            highlightthickness=0,
            cursor="crosshair",
        )
        self.canvas.grid(row=1, column=0, sticky="nsew")
        self.canvas.bind("<Configure>", lambda _event: self._draw_arena())
        self.canvas.bind("<Motion>", self._on_canvas_motion)
        self.canvas.bind("<Leave>", lambda _event: self._update_cursor_label(None))
        self.canvas.bind("<Button-1>", self._on_canvas_press)
        self.canvas.bind("<B1-Motion>", self._on_canvas_drag)
        self.canvas.bind("<ButtonRelease-1>", self._on_canvas_release)
        self.canvas.bind("<Button-3>", self._on_canvas_right_click)

        legend = ttk.Frame(arena_frame, style="Panel.TFrame")
        legend.grid(row=2, column=0, sticky="ew", pady=(8, 0))
        self._legend_item(legend, 0, COLORS["start_line"], "start zone")
        self._legend_item(legend, 1, COLORS["cyan"], "forward path")
        self._legend_item(legend, 2, COLORS["amber"], "reverse path")
        self._legend_item(legend, 3, COLORS["green"], "recognized")
        ttk.Label(
            legend,
            text="Click/add · drag/move · right-click/rotate · Del/remove",
            style="Muted.TLabel",
        ).grid(row=0, column=8, sticky="e", padx=(18, 0))
        legend.grid_columnconfigure(8, weight=1)

        sidebar = ttk.Frame(content, style="Panel.TFrame", padding=14)
        sidebar.grid(row=0, column=1, sticky="nsew")
        sidebar.grid_columnconfigure(0, weight=1)

        self._build_scenario_section(sidebar)
        self._separator(sidebar, 1)
        self._build_editor_section(sidebar)
        self._separator(sidebar, 3)
        self._build_run_section(sidebar)
        self._separator(sidebar, 5)

    def _legend_item(self, parent: ttk.Frame, column: int, color: str, text: str) -> None:
        marker = tk.Canvas(parent, width=13, height=13, bg=COLORS["panel"], highlightthickness=0)
        marker.create_oval(2, 2, 11, 11, fill=color, outline="")
        marker.grid(row=0, column=column * 2, padx=(0 if column == 0 else 10, 4))
        ttk.Label(parent, text=text, style="Muted.TLabel").grid(row=0, column=column * 2 + 1)

    def _separator(self, parent: ttk.Frame, row: int) -> None:
        tk.Frame(parent, height=1, bg=COLORS["grid"]).grid(
            row=row, column=0, sticky="ew", pady=12
        )

    def _build_scenario_section(self, parent: ttk.Frame) -> None:
        frame = ttk.Frame(parent, style="Panel.TFrame")
        frame.grid(row=0, column=0, sticky="ew")
        frame.grid_columnconfigure(0, weight=1)
        ttk.Label(frame, text="Scenario", style="Section.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Label(frame, text="4–8 image obstacles", style="Muted.TLabel").grid(
            row=0, column=1, sticky="e"
        )
        self.preset_var = tk.StringVar(value="Checklist demo - 5")
        preset = ttk.Combobox(
            frame,
            textvariable=self.preset_var,
            values=list(PRESETS),
            state="readonly",
        )
        preset.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        preset.bind("<<ComboboxSelected>>", lambda _event: self._load_preset(self.preset_var.get()))

    def _build_editor_section(self, parent: ttk.Frame) -> None:
        frame = ttk.Frame(parent, style="Panel.TFrame")
        frame.grid(row=2, column=0, sticky="ew")
        frame.grid_columnconfigure(0, weight=1)
        ttk.Label(frame, text="Arena editor", style="Section.TLabel").grid(row=0, column=0, sticky="w")
        self.obstacle_count_label = ttk.Label(frame, text="0 / 8", style="Muted.TLabel")
        self.obstacle_count_label.grid(row=0, column=1, sticky="e")

        controls = ttk.Frame(frame, style="Panel.TFrame")
        controls.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(8, 8))
        controls.grid_columnconfigure(0, weight=1)
        controls.grid_columnconfigure(1, weight=1)
        ttk.Label(controls, text="Image face", style="Muted.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Label(controls, text="Target ID", style="Muted.TLabel").grid(row=0, column=1, sticky="w", padx=(8, 0))
        self.face_var = tk.StringVar(value="N")
        face = ttk.Combobox(
            controls,
            textvariable=self.face_var,
            values=("N", "E", "S", "W"),
            state="readonly",
            width=8,
        )
        face.grid(row=1, column=0, sticky="ew")
        face.bind("<<ComboboxSelected>>", self._apply_editor_controls)
        self.target_id_var = tk.IntVar(value=11)
        target = ttk.Spinbox(
            controls,
            from_=11,
            to=40,
            textvariable=self.target_id_var,
            width=8,
            command=self._apply_editor_controls,
        )
        target.grid(row=1, column=1, sticky="ew", padx=(8, 0))
        target.bind("<Return>", self._apply_editor_controls)
        target.bind("<FocusOut>", self._apply_editor_controls)

        columns = ("id", "cell", "face", "target")
        self.obstacle_tree = ttk.Treeview(
            frame,
            columns=columns,
            show="headings",
            height=5,
            selectmode="browse",
        )
        headings = {"id": "#", "cell": "cell", "face": "face", "target": "target"}
        widths = {"id": 35, "cell": 90, "face": 55, "target": 60}
        for key in columns:
            self.obstacle_tree.heading(key, text=headings[key])
            self.obstacle_tree.column(key, width=widths[key], anchor="center", stretch=key == "cell")
        self.obstacle_tree.grid(row=2, column=0, columnspan=2, sticky="ew")
        self.obstacle_tree.bind("<<TreeviewSelect>>", self._on_tree_select)

        actions = ttk.Frame(frame, style="Panel.TFrame")
        actions.grid(row=3, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        actions.grid_columnconfigure(0, weight=1)
        actions.grid_columnconfigure(1, weight=1)
        ttk.Button(
            actions,
            text="Remove selected",
            style="Danger.TButton",
            command=self._remove_selected,
        ).grid(row=0, column=0, sticky="ew", padx=(0, 4))
        ttk.Button(
            actions,
            text="Clear arena",
            style="Secondary.TButton",
            command=self._clear_arena,
        ).grid(row=0, column=1, sticky="ew", padx=(4, 0))

    def _build_run_section(self, parent: ttk.Frame) -> None:
        frame = ttk.Frame(parent, style="Panel.TFrame")
        frame.grid(row=4, column=0, sticky="ew")
        frame.grid_columnconfigure(0, weight=1)
        ttk.Label(frame, text="Route playback", style="Section.TLabel").grid(row=0, column=0, sticky="w")
        self.plan_state_label = ttk.Label(frame, text="Not planned", style="Muted.TLabel")
        self.plan_state_label.grid(row=0, column=1, sticky="e")

        self.calculate_button = ttk.Button(
            frame,
            text="Calculate shortest-time route",
            style="Primary.TButton",
            command=self._calculate_plan,
        )
        self.calculate_button.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(8, 6))

        row = ttk.Frame(frame, style="Panel.TFrame")
        row.grid(row=2, column=0, columnspan=2, sticky="ew")
        row.grid_columnconfigure((0, 1, 2), weight=1)
        self.play_button = ttk.Button(
            row, text="▶  Play", style="Secondary.TButton", command=self._toggle_play
        )
        self.play_button.grid(row=0, column=0, sticky="ew", padx=(0, 3))
        ttk.Button(row, text="Step", style="Secondary.TButton", command=self._step_motion).grid(
            row=0, column=1, sticky="ew", padx=3
        )
        ttk.Button(row, text="Reset", style="Secondary.TButton", command=self._reset_run).grid(
            row=0, column=2, sticky="ew", padx=(3, 0)
        )

        speed_header = ttk.Frame(frame, style="Panel.TFrame")
        speed_header.grid(row=3, column=0, columnspan=2, sticky="ew", pady=(9, 0))
        speed_header.grid_columnconfigure(0, weight=1)
        ttk.Label(speed_header, text="Playback speed", style="Muted.TLabel").grid(row=0, column=0, sticky="w")
        self.speed_var = tk.DoubleVar(value=8.0)
        self.speed_value_label = ttk.Label(speed_header, text="8×", style="Body.TLabel")
        self.speed_value_label.grid(row=0, column=1, sticky="e")
        speed = ttk.Scale(
            frame,
            from_=1.0,
            to=20.0,
            variable=self.speed_var,
            command=lambda value: self.speed_value_label.configure(text=f"{float(value):.0f}×"),
        )
        speed.grid(row=4, column=0, columnspan=2, sticky="ew")

        toggles = ttk.Frame(frame, style="Panel.TFrame")
        toggles.grid(row=5, column=0, columnspan=2, sticky="ew", pady=(7, 0))
        self.show_clearance_var = tk.BooleanVar(value=False)
        self.show_targets_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            toggles,
            text="Clearance",
            variable=self.show_clearance_var,
            command=self._draw_arena,
        ).grid(row=0, column=0, sticky="w")
        ttk.Checkbutton(
            toggles,
            text="Recognition poses",
            variable=self.show_targets_var,
            command=self._draw_arena,
        ).grid(row=0, column=1, sticky="w", padx=(14, 0))

    def _build_stats_section(self, parent: ttk.Frame) -> None:
        frame = ttk.Frame(parent, style="Panel.TFrame")
        frame.grid(row=6, column=0, sticky="ew")
        frame.grid_columnconfigure((0, 1, 2), weight=1)
        self.stat_recognized = self._stat_card(frame, 0, "RECOGNIZED", "0 / 5")
        self.stat_timer = self._stat_card(frame, 1, "TASK CLOCK", "00:00")
        self.stat_distance = self._stat_card(frame, 2, "DISTANCE", "—")

    def _stat_card(self, parent: ttk.Frame, column: int, label: str, value: str) -> ttk.Label:
        card = ttk.Frame(parent, style="Lift.TFrame", padding=(9, 7))
        card.grid(row=0, column=column, sticky="nsew", padx=(0 if column == 0 else 3, 0))
        ttk.Label(card, text=label, style="StatLabel.TLabel").grid(row=0, column=0, sticky="w")
        value_label = ttk.Label(card, text=value, style="StatValue.TLabel")
        value_label.grid(row=1, column=0, sticky="w")
        return value_label

    def _bind_shortcuts(self) -> None:
        self.bind("<Delete>", lambda _event: self._remove_selected())
        self.bind("<BackSpace>", lambda _event: self._remove_selected())
        self.bind("<space>", lambda _event: self._toggle_play())
        self.bind("<Control-r>", lambda _event: self._calculate_plan())

    # ---------------------------------------------------------- Arena editor
    def _load_preset(self, name: str) -> None:
        rows = PRESETS[name]
        self.obstacles = [
            Obstacle(item_id, x, y, heading_from_text(face), target)
            for item_id, x, y, face, target in rows
        ]
        self.selected_obstacle_id = None
        self._invalidate_plan("Preset loaded")
        self._refresh_obstacle_tree()
        self._draw_arena()

    def _clear_arena(self) -> None:
        if self.obstacles and not messagebox.askyesno(
            "Clear arena", "Remove every obstacle from this simulator scenario?"
        ):
            return
        self.obstacles.clear()
        self.selected_obstacle_id = None
        self._invalidate_plan("Arena cleared")
        self._refresh_obstacle_tree()
        self._draw_arena()

    def _remove_selected(self) -> None:
        if self.selected_obstacle_id is None:
            return
        self.obstacles = [o for o in self.obstacles if o.id != self.selected_obstacle_id]
        self.selected_obstacle_id = None
        self._renumber_obstacles()
        self._invalidate_plan("Obstacle removed")
        self._refresh_obstacle_tree()
        self._draw_arena()

    def _renumber_obstacles(self) -> None:
        self.obstacles = [replace(obstacle, id=index + 1) for index, obstacle in enumerate(self.obstacles)]

    def _refresh_obstacle_tree(self) -> None:
        current = self.selected_obstacle_id
        for item in self.obstacle_tree.get_children():
            self.obstacle_tree.delete(item)
        for obstacle in self.obstacles:
            item = self.obstacle_tree.insert(
                "",
                tk.END,
                iid=str(obstacle.id),
                values=(
                    obstacle.id,
                    f"({obstacle.x}, {obstacle.y})",
                    obstacle.face.name,
                    obstacle.target_id if obstacle.target_id is not None else "—",
                ),
            )
            if obstacle.id == current:
                self.obstacle_tree.selection_set(item)
        self.obstacle_count_label.configure(text=f"{len(self.obstacles)} / 8")
        self._update_stat_labels()

    def _on_tree_select(self, _event=None) -> None:
        selection = self.obstacle_tree.selection()
        if not selection:
            return
        self.selected_obstacle_id = int(selection[0])
        obstacle = self._selected_obstacle()
        if obstacle is not None:
            self._syncing_controls = True
            self.face_var.set(obstacle.face.name)
            self.target_id_var.set(obstacle.target_id or 11)
            self._syncing_controls = False
        self._draw_arena()

    def _selected_obstacle(self) -> Optional[Obstacle]:
        return next((o for o in self.obstacles if o.id == self.selected_obstacle_id), None)

    def _apply_editor_controls(self, _event=None) -> None:
        if self._syncing_controls:
            return
        obstacle = self._selected_obstacle()
        if obstacle is None:
            return
        try:
            target_id = max(11, min(40, int(self.target_id_var.get())))
        except (tk.TclError, ValueError):
            target_id = obstacle.target_id or 11
        updated = replace(
            obstacle,
            face=heading_from_text(self.face_var.get()),
            target_id=target_id,
        )
        self.obstacles = [updated if o.id == updated.id else o for o in self.obstacles]
        self._invalidate_plan("Obstacle annotation changed")
        self._refresh_obstacle_tree()
        self._draw_arena()

    def _on_canvas_motion(self, event: tk.Event) -> None:
        self._update_cursor_label(self._canvas_to_cell(event.x, event.y))

    def _on_canvas_press(self, event: tk.Event) -> None:
        cell = self._canvas_to_cell(event.x, event.y)
        if cell is None:
            self.selected_obstacle_id = None
            self._refresh_obstacle_tree()
            self._draw_arena()
            return
        obstacle = self._obstacle_at(*cell)
        if obstacle is not None:
            self.selected_obstacle_id = obstacle.id
            self._drag_obstacle_id = obstacle.id
            self._refresh_obstacle_tree()
            self._on_tree_select()
            return
        if len(self.obstacles) >= 8:
            self._set_status("Task 1 supports at most 8 obstacles", error=True)
            return
        if cell[0] < START_ZONE_CELLS and cell[1] < START_ZONE_CELLS:
            self._set_status("Start zone must remain clear", error=True)
            return
        next_id = max((o.id for o in self.obstacles), default=0) + 1
        try:
            target_id = max(11, min(40, int(self.target_id_var.get())))
        except (tk.TclError, ValueError):
            target_id = 11
        new_obstacle = Obstacle(
            next_id,
            cell[0],
            cell[1],
            heading_from_text(self.face_var.get()),
            target_id,
        )
        self.obstacles.append(new_obstacle)
        self.selected_obstacle_id = next_id
        self._drag_obstacle_id = next_id
        self._invalidate_plan("Obstacle added")
        self._refresh_obstacle_tree()
        self._draw_arena()

    def _on_canvas_drag(self, event: tk.Event) -> None:
        if self._drag_obstacle_id is None:
            return
        cell = self._canvas_to_cell(event.x, event.y)
        if cell is None or (cell[0] < START_ZONE_CELLS and cell[1] < START_ZONE_CELLS):
            return
        if any(o.id != self._drag_obstacle_id and (o.x, o.y) == cell for o in self.obstacles):
            return
        changed = False
        moved: list[Obstacle] = []
        for obstacle in self.obstacles:
            if obstacle.id == self._drag_obstacle_id and (obstacle.x, obstacle.y) != cell:
                moved.append(replace(obstacle, x=cell[0], y=cell[1]))
                changed = True
            else:
                moved.append(obstacle)
        if changed:
            self.obstacles = moved
            self._invalidate_plan("Obstacle moved")
            self._refresh_obstacle_tree()
            self._draw_arena()

    def _on_canvas_release(self, event: tk.Event) -> None:
        if self._drag_obstacle_id is not None and self._canvas_to_cell(event.x, event.y) is None:
            self.obstacles = [o for o in self.obstacles if o.id != self._drag_obstacle_id]
            self.selected_obstacle_id = None
            self._renumber_obstacles()
            self._invalidate_plan("Obstacle removed")
            self._refresh_obstacle_tree()
            self._draw_arena()
        self._drag_obstacle_id = None

    def _on_canvas_right_click(self, event: tk.Event) -> None:
        cell = self._canvas_to_cell(event.x, event.y)
        if cell is None:
            return
        obstacle = self._obstacle_at(*cell)
        if obstacle is None:
            return
        cycle = [Heading.N, Heading.E, Heading.S, Heading.W]
        face = cycle[(cycle.index(obstacle.face) + 1) % len(cycle)]
        updated = replace(obstacle, face=face)
        self.obstacles = [updated if o.id == updated.id else o for o in self.obstacles]
        self.selected_obstacle_id = obstacle.id
        self._invalidate_plan("Image face rotated")
        self._refresh_obstacle_tree()
        self._on_tree_select()

    def _obstacle_at(self, x: int, y: int) -> Optional[Obstacle]:
        return next((o for o in self.obstacles if o.x == x and o.y == y), None)

    # -------------------------------------------------------------- Planning
    def _calculate_plan(self) -> None:
        if not 4 <= len(self.obstacles) <= 8:
            self._set_status("Task 1 scenarios require 4–8 obstacles", error=True)
            messagebox.showwarning(
                "Task 1 scenario",
                "Add between 4 and 8 image obstacles before calculating a route.",
            )
            return
        self.playing = False
        self.play_button.configure(text="▶  Play")
        self.calculate_button.configure(text="Calculating…", state=tk.DISABLED)
        self.plan_state_label.configure(text="Searching")
        self.header_status.configure(text="PLANNING", bg=COLORS["cyan_soft"], fg=COLORS["cyan"])
        self.update_idletasks()
        started = time.perf_counter()
        try:
            result = Task1Planner(self.obstacles).plan(START_POSE)
        except (ValueError, RuntimeError) as exc:
            self.plan_result = None
            self._set_status(str(exc), error=True)
            messagebox.showerror("Unable to plan", str(exc))
            return
        finally:
            self.calculate_button.configure(text="Calculate shortest-time route", state=tk.NORMAL)

        self.plan_result = result
        self._reset_run(clear_plan=False)
        wall_ms = (time.perf_counter() - started) * 1000.0
        if result.complete:
            self.plan_state_label.configure(text=f"Ready · {wall_ms:.0f} ms")
            self.header_status.configure(text="ROUTE READY", bg="#173d36", fg=COLORS["green"])
        else:
            missing = ", ".join(str(item) for item in result.unreachable_ids)
            self.plan_state_label.configure(text=f"Partial · missing {missing}")
            self.header_status.configure(text="PARTIAL ROUTE", bg="#493426", fg=COLORS["amber"])
        self._draw_arena()
        self._update_stat_labels()

    def _invalidate_plan(self, reason: str) -> None:
        self.playing = False
        self.play_button.configure(text="▶  Play") if hasattr(self, "play_button") else None
        self.plan_result = None
        self.motion_index = 0
        self.motion_elapsed_s = 0.0
        self.elapsed_sim_s = 0.0
        self.recognized_ids.clear()
        self.logged_scan_motions.clear()
        self.current_pose = PoseSample(
            float(START_POSE.x), float(START_POSE.y), heading_degrees(START_POSE.heading)
        )
        if hasattr(self, "plan_state_label"):
            self.plan_state_label.configure(text="Not planned")
            self.header_status.configure(text="EDITING", bg=COLORS["panel_lift"], fg=COLORS["muted"])
            self.command_label.configure(text=reason)
            self.latest_event_label.configure(text="No recognitions yet")
            self.event_messages = ["Route changed · calculate again"]
            self._update_cursor_label(None)
            self._update_stat_labels()

    # ------------------------------------------------------------- Playback
    def _toggle_play(self) -> None:
        if self.plan_result is None:
            self._calculate_plan()
            if self.plan_result is None:
                return
        if self.motion_index >= len(self.plan_result.motions):
            self._reset_run(clear_plan=False)
        self.playing = not self.playing
        self.last_tick = time.perf_counter()
        self.play_button.configure(text="Ⅱ  Pause" if self.playing else "▶  Play")
        self.header_status.configure(
            text="RUNNING" if self.playing else "PAUSED",
            bg="#173d36" if self.playing else COLORS["panel_lift"],
            fg=COLORS["green"] if self.playing else COLORS["muted"],
        )

    def _reset_run(self, clear_plan: bool = False) -> None:
        self.playing = False
        self.play_button.configure(text="▶  Play")
        if clear_plan:
            self.plan_result = None
        self.motion_index = 0
        self.motion_elapsed_s = 0.0
        self.elapsed_sim_s = 0.0
        self.recognized_ids.clear()
        self.logged_scan_motions.clear()
        self.current_pose = PoseSample(
            float(START_POSE.x), float(START_POSE.y), heading_degrees(START_POSE.heading)
        )
        self.event_messages = ["Ready · press Play or Step"]
        self.command_label.configure(text="Ready · ROBOT, 1, 1, N")
        self.latest_event_label.configure(text="No recognitions yet")
        self._update_cursor_label(None)
        if self.plan_result is not None:
            self.header_status.configure(text="ROUTE READY", bg="#173d36", fg=COLORS["green"])
        self._update_stat_labels()
        self._draw_arena()

    def _step_motion(self) -> None:
        if self.plan_result is None:
            self._calculate_plan()
            if self.plan_result is None:
                return
        self.playing = False
        self.play_button.configure(text="▶  Play")
        if self.motion_index >= len(self.plan_result.motions):
            return
        motion = self.plan_result.motions[self.motion_index]
        self.motion_elapsed_s = max(0.0, motion.duration_s)
        self.elapsed_sim_s += max(0.0, motion.duration_s)
        self.current_pose = motion.samples[-1]
        self._complete_motion(self.motion_index, motion)
        self.motion_index += 1
        self.motion_elapsed_s = 0.0
        self._update_command_label()
        self._update_stat_labels()
        self._draw_arena()

    def _animation_tick(self) -> None:
        now = time.perf_counter()
        real_dt = min(0.1, max(0.0, now - self.last_tick))
        self.last_tick = now
        if self.playing and self.plan_result is not None:
            self._advance_playback(real_dt * self.speed_var.get())
        self.after(16, self._animation_tick)

    def _advance_playback(self, delta_s: float) -> None:
        assert self.plan_result is not None
        motions = self.plan_result.motions
        while delta_s > 0 and self.motion_index < len(motions):
            motion = motions[self.motion_index]
            duration = max(0.001, motion.duration_s)
            remaining = duration - self.motion_elapsed_s
            advance = min(delta_s, remaining)
            self.motion_elapsed_s += advance
            self.elapsed_sim_s += advance
            delta_s -= advance
            progress = min(1.0, self.motion_elapsed_s / duration)
            self.current_pose = self._sample_motion(motion, progress)
            if self.motion_elapsed_s + 1e-9 >= duration:
                self._complete_motion(self.motion_index, motion)
                self.motion_index += 1
                self.motion_elapsed_s = 0.0

        if self.motion_index >= len(motions):
            self.playing = False
            self.play_button.configure(text="↻  Replay")
            self.header_status.configure(text="RUN COMPLETE", bg="#173d36", fg=COLORS["green"])
            self.command_label.configure(text="Complete · all planned commands executed")
        else:
            self._update_command_label()
        self._update_cursor_label(None)
        self._update_stat_labels()
        self._draw_arena()

    def _sample_motion(self, motion: Motion, progress: float) -> PoseSample:
        samples = motion.samples
        if not samples:
            return self.current_pose
        if len(samples) == 1:
            return samples[0]
        position = progress * (len(samples) - 1)
        index = min(len(samples) - 2, int(position))
        local = position - index
        a, b = samples[index], samples[index + 1]
        angle = a.heading_deg + shortest_angle_delta(a.heading_deg, b.heading_deg) * local
        return PoseSample(
            a.x + (b.x - a.x) * local,
            a.y + (b.y - a.y) * local,
            angle,
        )

    def _complete_motion(self, index: int, motion: Motion) -> None:
        if motion.code == "SCAN" and index not in self.logged_scan_motions:
            self.logged_scan_motions.add(index)
            if motion.obstacle_id is not None:
                self.recognized_ids.add(motion.obstacle_id)
            stamp = self._format_clock(self.elapsed_sim_s)
            target = motion.target_id if motion.target_id is not None else "?"
            obstacle = motion.obstacle_id if motion.obstacle_id is not None else "?"
            if len(self.event_messages) == 1 and self.event_messages[0].startswith(("Ready", "Waiting")):
                self.event_messages.clear()
            self.event_messages.append(f"{stamp}  TARGET, {obstacle}, {target}")
            self.latest_event_label.configure(text=f"TARGET, {obstacle}, {target}")

    def _update_command_label(self) -> None:
        if self.plan_result is None or self.motion_index >= len(self.plan_result.motions):
            return
        motion = self.plan_result.motions[self.motion_index]
        heading = self._quantized_heading(self.current_pose.heading_deg)
        self.command_label.configure(
            text=f"{motion.code} · ROBOT, {round(self.current_pose.x)}, {round(self.current_pose.y)}, {heading}"
        )

    # --------------------------------------------------------------- Drawing
    def _draw_arena(self) -> None:
        if not hasattr(self, "canvas"):
            return
        canvas = self.canvas
        canvas.delete("all")
        width = max(200, canvas.winfo_width())
        height = max(200, canvas.winfo_height())
        left, top, bottom_margin = 38.0, 18.0, 34.0
        side = max(120.0, min(width - left - 18.0, height - top - bottom_margin))
        cell = side / ARENA_CELLS
        self._canvas_geometry = (left, top, cell)
        right, bottom = left + side, top + side

        canvas.create_rectangle(left, top, right, bottom, fill="#0c1728", outline="")
        sx0, sy0 = self._world_edge_to_canvas(0, START_ZONE_CELLS)
        sx1, sy1 = self._world_edge_to_canvas(START_ZONE_CELLS, 0)
        canvas.create_rectangle(
            sx0,
            sy0,
            sx1,
            sy1,
            fill=COLORS["start"],
            outline=COLORS["start_line"],
            width=2,
        )
        canvas.create_text(
            sx0 + 8,
            sy0 + 8,
            text="START",
            anchor="nw",
            fill=COLORS["start_line"],
            font=("Segoe UI Semibold", max(7, int(cell * 0.28))),
        )

        for i in range(ARENA_CELLS + 1):
            x = left + i * cell
            y = top + i * cell
            major = i % 5 == 0
            color = COLORS["grid_major"] if major else COLORS["grid"]
            line_width = 1.3 if major else 1
            canvas.create_line(x, top, x, bottom, fill=color, width=line_width)
            canvas.create_line(left, y, right, y, fill=color, width=line_width)
        canvas.create_rectangle(left, top, right, bottom, outline=COLORS["border"], width=2)

        label_font = ("Cascadia Mono", max(6, min(9, int(cell * 0.28))))
        for i in range(ARENA_CELLS):
            canvas.create_text(
                left + (i + 0.5) * cell,
                bottom + 13,
                text=str(i),
                fill=COLORS["muted"],
                font=label_font,
            )
            canvas.create_text(
                left - 14,
                bottom - (i + 0.5) * cell,
                text=str(i),
                fill=COLORS["muted"],
                font=label_font,
            )

        if self.show_clearance_var.get():
            for obstacle in self.obstacles:
                self._draw_clearance(obstacle)
        if self.plan_result is not None:
            self._draw_planned_path(self.plan_result)
        if self.show_targets_var.get():
            for obstacle in self.obstacles:
                self._draw_recognition_pose(obstacle)
        for obstacle in self.obstacles:
            self._draw_obstacle(obstacle)
        self._draw_robot(self.current_pose)
        self._draw_canvas_info_panel(width, right, top)

    def _draw_canvas_info_panel(self, canvas_width: float, grid_right: float, top: float) -> None:
        """Use the space beside the square arena for route and scan status."""

        panel_left = grid_right + 24
        available = canvas_width - panel_left - 14
        if available < 165:
            return
        canvas = self.canvas
        canvas.create_line(
            panel_left - 12,
            top,
            panel_left - 12,
            top + min(470, canvas.winfo_height() - top - 8),
            fill=COLORS["grid"],
            width=1,
        )
        canvas.create_text(
            panel_left,
            top + 2,
            text="ROUTE SUMMARY",
            anchor="nw",
            fill=COLORS["muted"],
            font=("Segoe UI Semibold", 9),
        )
        if self.plan_result is None:
            route_text = "Calculate a route to view\nvisit order and timing."
            timing_text = "—"
        else:
            route_text = " → ".join(str(item) for item in self.plan_result.visit_order) or "No reachable target"
            timing_text = (
                f"{self._format_clock(self.plan_result.estimated_seconds)} estimated\n"
                f"{self.plan_result.expanded_states:,} A* states"
            )
        canvas.create_text(
            panel_left,
            top + 28,
            text=route_text,
            anchor="nw",
            fill=COLORS["text"],
            font=("Cascadia Mono", 10, "bold"),
            width=available,
        )
        canvas.create_text(
            panel_left,
            top + 65,
            text=timing_text,
            anchor="nw",
            fill=COLORS["muted"],
            font=("Cascadia Mono", 8),
            width=available,
        )
        canvas.create_line(
            panel_left,
            top + 103,
            panel_left + available,
            top + 103,
            fill=COLORS["grid"],
        )
        canvas.create_text(
            panel_left,
            top + 118,
            text="RECOGNITION EVENTS",
            anchor="nw",
            fill=COLORS["muted"],
            font=("Segoe UI Semibold", 9),
        )
        y = top + 146
        for message in self.event_messages[-8:]:
            canvas.create_text(
                panel_left,
                y,
                text=message,
                anchor="nw",
                fill=COLORS["green"] if "TARGET," in message else COLORS["text"],
                font=("Cascadia Mono", 8),
                width=available,
            )
            y += 25

    def _draw_clearance(self, obstacle: Obstacle) -> None:
        # A 40x40 cm virtual obstacle centered on the 10x10 cm physical block.
        x0, y0 = self._world_edge_to_canvas(obstacle.x - 1.5, obstacle.y + 2.5)
        x1, y1 = self._world_edge_to_canvas(obstacle.x + 2.5, obstacle.y - 1.5)
        self.canvas.create_rectangle(
            x0,
            y0,
            x1,
            y1,
            fill="",
            outline=COLORS["red"],
            dash=(4, 4),
            width=1,
        )

    def _draw_planned_path(self, plan: PlanResult) -> None:
        completed = self.motion_index
        for index, motion in enumerate(plan.motions):
            if motion.code == "SCAN" or len(motion.samples) < 2:
                continue
            points: list[float] = []
            samples: Iterable[PoseSample] = motion.samples
            if index == completed and self.motion_elapsed_s > 0:
                duration = max(0.001, motion.duration_s)
                progress = min(1.0, self.motion_elapsed_s / duration)
                count = max(2, int(progress * (len(motion.samples) - 1)) + 1)
                samples = list(motion.samples[:count]) + [self.current_pose]
            for sample in samples:
                px, py = self._pose_to_canvas(sample.x, sample.y)
                points.extend((px, py))
            if len(points) < 4:
                continue
            reverse = motion.code.startswith("B") or motion.code in {"LB090", "RB090", "BL", "BR"}
            color = COLORS["amber"] if reverse else COLORS["cyan"]
            active = index < completed or (index == completed and self.motion_elapsed_s > 0)
            self.canvas.create_line(
                *points,
                fill=color if active else COLORS["cyan_soft"],
                width=3 if active else 2,
                smooth=True,
                splinesteps=12,
                dash=(6, 4) if reverse else (),
            )

    def _draw_recognition_pose(self, obstacle: Obstacle) -> None:
        pose = self._recognition_pose(obstacle)
        if pose is None:
            return
        x, y, heading = pose
        cx, cy = self._pose_to_canvas(x, y)
        cell = self._canvas_geometry[2]
        color = COLORS["green"] if obstacle.id in self.recognized_ids else COLORS["cyan"]
        self.canvas.create_oval(
            cx - cell * 0.22,
            cy - cell * 0.22,
            cx + cell * 0.22,
            cy + cell * 0.22,
            outline=color,
            width=2,
        )
        angle = math.radians(heading_degrees(heading))
        ex = cx + math.cos(angle) * cell * 0.55
        ey = cy - math.sin(angle) * cell * 0.55
        self.canvas.create_line(cx, cy, ex, ey, fill=color, width=2, arrow=tk.LAST)

    def _draw_obstacle(self, obstacle: Obstacle) -> None:
        x0, y0 = self._cell_top_left(obstacle.x, obstacle.y)
        cell = self._canvas_geometry[2]
        selected = obstacle.id == self.selected_obstacle_id
        fill = COLORS["obstacle_selected"] if selected else COLORS["obstacle"]
        outline = COLORS["white"] if selected else COLORS["border"]
        self.canvas.create_rectangle(
            x0 + 1,
            y0 + 1,
            x0 + cell - 1,
            y0 + cell - 1,
            fill=fill,
            outline=outline,
            width=2 if selected else 1,
        )
        face = obstacle.face
        pad = 1.5
        if face == Heading.N:
            coords = (x0 + pad, y0 + pad, x0 + cell - pad, y0 + pad)
        elif face == Heading.S:
            coords = (x0 + pad, y0 + cell - pad, x0 + cell - pad, y0 + cell - pad)
        elif face == Heading.E:
            coords = (x0 + cell - pad, y0 + pad, x0 + cell - pad, y0 + cell - pad)
        else:
            coords = (x0 + pad, y0 + pad, x0 + pad, y0 + cell - pad)
        face_color = COLORS["green"] if obstacle.id in self.recognized_ids else COLORS["cyan"]
        self.canvas.create_line(*coords, fill=face_color, width=max(3, int(cell * 0.16)))
        self.canvas.create_text(
            x0 + cell / 2,
            y0 + cell / 2 - 1,
            text=str(obstacle.id),
            fill=COLORS["white"],
            font=("Segoe UI Semibold", max(7, int(cell * 0.34))),
        )
        if cell >= 24:
            self.canvas.create_text(
                x0 + cell / 2,
                y0 + cell + 7,
                text=f"T{obstacle.target_id or '?'}",
                fill=COLORS["muted"],
                font=("Cascadia Mono", 7),
            )

    def _draw_robot(self, pose: PoseSample) -> None:
        cx, cy = self._pose_to_canvas(pose.x, pose.y)
        cell = self._canvas_geometry[2]
        angle = math.radians(pose.heading_deg)
        forward = (math.cos(angle), -math.sin(angle))
        right = (-forward[1], forward[0])

        # Conservative 30x30 cm planner envelope.
        half = 1.5 * cell
        self.canvas.create_rectangle(
            cx - half,
            cy - half,
            cx + half,
            cy + half,
            outline=COLORS["cyan"],
            dash=(4, 3),
            width=1,
        )

        # Physical 20x21 cm chassis, rotated continuously during turns.
        half_width = cell * 0.82
        half_length = cell * 0.94
        polygon: list[float] = []
        for longitudinal, lateral in (
            (half_length, -half_width),
            (half_length, half_width),
            (-half_length, half_width),
            (-half_length, -half_width),
        ):
            px = cx + forward[0] * longitudinal + right[0] * lateral
            py = cy + forward[1] * longitudinal + right[1] * lateral
            polygon.extend((px, py))
        self.canvas.create_polygon(
            *polygon,
            fill="#1f8395",
            outline=COLORS["cyan"],
            width=2,
        )
        front_x = cx + forward[0] * half_length
        front_y = cy + forward[1] * half_length
        self.canvas.create_line(
            cx,
            cy,
            front_x,
            front_y,
            fill=COLORS["white"],
            width=2,
            arrow=tk.LAST,
        )
        radius = max(2.5, cell * 0.13)
        self.canvas.create_oval(
            front_x - radius,
            front_y - radius,
            front_x + radius,
            front_y + radius,
            fill=COLORS["amber"],
            outline="",
        )

    # -------------------------------------------------------------- Helpers
    def _cell_top_left(self, x: int, y: int) -> tuple[float, float]:
        left, top, cell = self._canvas_geometry
        return left + x * cell, top + (ARENA_CELLS - 1 - y) * cell

    def _pose_to_canvas(self, x: float, y: float) -> tuple[float, float]:
        left, top, cell = self._canvas_geometry
        return left + (x + 0.5) * cell, top + (ARENA_CELLS - y - 0.5) * cell

    def _world_edge_to_canvas(self, x: float, y: float) -> tuple[float, float]:
        left, top, cell = self._canvas_geometry
        return left + x * cell, top + (ARENA_CELLS - y) * cell

    def _canvas_to_cell(self, x: float, y: float) -> Optional[tuple[int, int]]:
        left, top, cell = self._canvas_geometry
        local_x = (x - left) / cell
        local_y = (y - top) / cell
        if not (0 <= local_x < ARENA_CELLS and 0 <= local_y < ARENA_CELLS):
            return None
        return int(local_x), ARENA_CELLS - 1 - int(local_y)

    def _recognition_pose(self, obstacle: Obstacle) -> Optional[tuple[int, int, Heading]]:
        if obstacle.face == Heading.N:
            pose = (obstacle.x, obstacle.y + 4, Heading.S)
        elif obstacle.face == Heading.S:
            pose = (obstacle.x, obstacle.y - 4, Heading.N)
        elif obstacle.face == Heading.E:
            pose = (obstacle.x + 4, obstacle.y, Heading.W)
        else:
            pose = (obstacle.x - 4, obstacle.y, Heading.E)
        return pose if 0 <= pose[0] < ARENA_CELLS and 0 <= pose[1] < ARENA_CELLS else None

    def _update_cursor_label(self, cell: Optional[tuple[int, int]]) -> None:
        cell_text = "—" if cell is None else f"({cell[0]}, {cell[1]})"
        heading = self._quantized_heading(self.current_pose.heading_deg)
        self.cursor_label.configure(
            text=f"cell {cell_text}  ·  robot ({self.current_pose.x:.1f}, {self.current_pose.y:.1f}, {heading})"
        )

    @staticmethod
    def _quantized_heading(angle: float) -> str:
        normalized = angle % 360.0
        index = int((normalized + 45.0) // 90.0) % 4
        return ("E", "N", "W", "S")[index]

    @staticmethod
    def _format_clock(seconds: float) -> str:
        seconds = max(0, int(round(seconds)))
        return f"{seconds // 60:02d}:{seconds % 60:02d}"

    def _update_stat_labels(self) -> None:
        total = len(self.obstacles)
        self.stat_recognized.configure(text=f"{len(self.recognized_ids)} / {total}")
        self.stat_timer.configure(text=self._format_clock(self.elapsed_sim_s))
        if self.elapsed_sim_s > TASK_TIMEOUT_S:
            self.stat_timer.configure(foreground=COLORS["red"])
        else:
            self.stat_timer.configure(foreground=COLORS["text"])
        if self.plan_result is None:
            self.stat_distance.configure(text="—")
        else:
            self.stat_distance.configure(text=f"{self.plan_result.total_distance_cm / 100:.1f} m")

    def _update_cursor_from_pose(self) -> None:
        self._update_cursor_label(None)

    def _set_status(self, text: str, error: bool = False) -> None:
        self.command_label.configure(text=text)
        self.header_status.configure(
            text="CHECK ARENA" if error else "READY",
            bg="#4b2730" if error else COLORS["panel_lift"],
            fg=COLORS["red"] if error else COLORS["muted"],
        )


def export_preset_plan(preset_name: str, destination: Optional[Path] = None) -> dict:
    """Headless helper used by smoke tests and for demo-data inspection."""

    rows = PRESETS[preset_name]
    obstacles = [
        Obstacle(item_id, x, y, heading_from_text(face), target)
        for item_id, x, y, face, target in rows
    ]
    result = Task1Planner(obstacles).plan(START_POSE)
    payload = {
        "preset": preset_name,
        "complete": result.complete,
        "visit_order": list(result.visit_order),
        "unreachable_ids": list(result.unreachable_ids),
        "estimated_seconds": round(result.estimated_seconds, 3),
        "distance_cm": round(result.total_distance_cm, 1),
        "expanded_states": result.expanded_states,
        "commands": list(result.command_messages),
    }
    if destination is not None:
        destination.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return payload


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="MDP Task 1 algorithm simulator")
    parser.add_argument(
        "--headless-plan",
        choices=list(PRESETS),
        help="plan one bundled scenario, print JSON, and exit without opening the GUI",
    )
    args = parser.parse_args(argv)
    if args.headless_plan:
        print(json.dumps(export_preset_plan(args.headless_plan), indent=2))
        return 0
    app = Task1Simulator()
    app.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

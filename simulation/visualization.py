"""Interactive Matplotlib dashboard for decision and trajectory results."""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon
from matplotlib.widgets import Button, Slider

from simulation.simulator import SimulationResult


PIECE_COLORS = ("#0072B2", "#E69F00", "#009E73", "#CC79A7")
APPROACH_COLOR = "#2D6CDF"
TRANSFER_COLOR = "#16856B"
RISK_COLOR = "#C23B33"
TEXT_COLOR = "#22252A"
GRID_COLOR = "#D8DDE5"
TIMER_INTERVAL_MS = 40
PLAYBACK_RATE = 4.0


def _piece_at_pose(vertices: np.ndarray, pick, pose: np.ndarray) -> np.ndarray:
    yaw_delta = (pose[3] - pick.yaw_deg + 180.0) % 360.0 - 180.0
    angle = np.deg2rad(yaw_delta)
    rotation = np.array(
        [[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]
    )
    relative = vertices - [pick.x_mm, pick.y_mm]
    return relative @ rotation.T + pose[:2]


class SimulationView:
    def __init__(self, figure, axes: dict[str, object], result: SimulationResult):
        self.figure = figure
        self.axes = axes
        self.result = result
        self.current_time = 0.0
        self.playing = False
        self._updating_slider = False
        self._piece_patches: dict[int, Polygon] = {}
        self._piece_labels = {}

        self._draw_result()
        self._create_controls()
        self.timer = figure.canvas.new_timer(interval=TIMER_INTERVAL_MS)
        self.timer.add_callback(self._on_timer)
        self.set_time(0.0)

    @property
    def duration_s(self) -> float:
        return float(self.result.samples[-1].time_s)

    def _create_controls(self) -> None:
        play_ax = self.figure.add_axes((0.16, 0.075, 0.075, 0.04))
        reset_ax = self.figure.add_axes((0.245, 0.075, 0.075, 0.04))
        slider_ax = self.figure.add_axes((0.37, 0.082, 0.57, 0.028))

        self.play_button = Button(play_ax, "Play")
        self.reset_button = Button(reset_ax, "Reset")
        self.time_slider = Slider(
            slider_ax,
            "Time (s)",
            0.0,
            self.duration_s,
            valinit=0.0,
        )
        self.play_button.on_clicked(lambda _event: self.toggle_play())
        self.reset_button.on_clicked(lambda _event: self.reset())
        self.time_slider.on_changed(self._on_slider_changed)

    def _style_axis(self, axis) -> None:
        axis.set_facecolor("white")
        axis.grid(True, color=GRID_COLOR, linewidth=0.7, alpha=0.75)
        axis.tick_params(colors="#4B5058", labelsize=8)

    def _draw_result(self) -> None:
        board_ax = self.axes["board"]
        path_ax = self.axes["path"]
        pose_ax = self.axes["pose"]
        yaw_ax = self.axes["yaw"]
        limits_ax = self.axes["limits"]

        for axis in (board_ax, path_ax, pose_ax, yaw_ax, limits_ax):
            axis.clear()
        for axis in (board_ax, pose_ax, limits_ax):
            self._style_axis(axis)

        self.figure.suptitle(
            "Four-Servo Arm: Decision and Cartesian Trajectory Verification",
            fontsize=15,
            fontweight="bold",
            color=TEXT_COLOR,
        )
        self._draw_board_static(board_ax)
        self._draw_path_static(path_ax)
        self._draw_timeline_static(pose_ax, yaw_ax, limits_ax)

    def _draw_board_static(self, axis) -> None:
        target_polygons = (
            self.result.scenario.target_polygons
            if self.result.scenario.target_polygons
            else self.result.final_polygons
        )
        all_polygons = list(self.result.initial_polygons.values()) + list(
            target_polygons.values()
        )
        all_points = np.concatenate(all_polygons)
        span = np.maximum(all_points.max(axis=0) - all_points.min(axis=0), 1.0)
        padding = 0.10 * float(max(span))
        axis.set_xlim(all_points[:, 0].min() - padding, all_points[:, 0].max() + padding)
        axis.set_ylim(all_points[:, 1].min() - padding, all_points[:, 1].max() + padding)
        axis.set_aspect("equal", adjustable="box")
        axis.set_xlabel("X (mm)", fontsize=9)
        axis.set_ylabel("Y (mm)", fontsize=9)
        axis.set_title("Decision Board", fontsize=11, fontweight="bold")

        for piece_index, execution in enumerate(self.result.moves):
            piece_id = execution.piece_id
            color = PIECE_COLORS[piece_index % len(PIECE_COLORS)]
            target = target_polygons[piece_id]
            target_patch = Polygon(
                target,
                closed=True,
                fill=False,
                edgecolor=color,
                linewidth=1.4,
                linestyle="--",
                alpha=0.8,
            )
            axis.add_patch(target_patch)
            initial = self.result.initial_polygons[piece_id]
            patch = Polygon(
                initial,
                closed=True,
                facecolor=color,
                edgecolor="white",
                linewidth=1.2,
                alpha=0.78,
            )
            axis.add_patch(patch)
            self._piece_patches[piece_id] = patch
            axis.scatter(
                execution.move.pick.x_mm,
                execution.move.pick.y_mm,
                marker="x",
                s=36,
                linewidth=1.4,
                color=color,
                zorder=5,
            )
            center = initial.mean(axis=0)
            self._piece_labels[piece_id] = axis.text(
                center[0],
                center[1],
                str(piece_id),
                ha="center",
                va="center",
                fontsize=8,
                fontweight="bold",
                color="white",
                zorder=6,
            )

        self.board_effector, = axis.plot(
            [], [], marker="o", markersize=6, color="#111111", zorder=8
        )
        self.board_yaw, = axis.plot([], [], color="#111111", linewidth=2.0, zorder=8)

    def _draw_path_static(self, axis) -> None:
        axis.set_facecolor("white")
        axis.set_title("Cartesian Path", fontsize=11, fontweight="bold")
        axis.set_xlabel("X (mm)", fontsize=9)
        axis.set_ylabel("Y (mm)", fontsize=9)
        axis.set_zlabel("Z (mm)", fontsize=9)
        axis.tick_params(labelsize=8)
        axis.grid(True, color=GRID_COLOR, linewidth=0.7, alpha=0.7)

        for execution in self.result.moves:
            for phase_name in ("approach", "transfer"):
                phase_samples = [
                    sample
                    for sample in self.result.samples
                    if sample.move_index == execution.move_index
                    and sample.phase == phase_name
                ]
                points = np.array([sample.pose[:3] for sample in phase_samples])
                risk = phase_name == "approach" and any(
                    sample.low_height_risk for sample in phase_samples
                )
                color = (
                    RISK_COLOR
                    if risk
                    else APPROACH_COLOR
                    if phase_name == "approach"
                    else TRANSFER_COLOR
                )
                axis.plot(
                    points[:, 0],
                    points[:, 1],
                    points[:, 2],
                    color=color,
                    linewidth=2.2,
                    alpha=0.75,
                )

            # Mark the two endpoints plus the cruise-height lift poses, which is
            # what makes the "above both ends" shape visible.
            move = execution.move
            axis.scatter(
                [move.pick.x_mm, move.place.x_mm],
                [move.pick.y_mm, move.place.y_mm],
                [move.pick.z_mm, move.place.z_mm],
                s=18,
                color=(APPROACH_COLOR, TRANSFER_COLOR),
                depthshade=False,
            )
            axis.scatter(
                [move.pick_above.x_mm, move.place_above.x_mm],
                [move.pick_above.y_mm, move.place_above.y_mm],
                [move.pick_above.z_mm, move.place_above.z_mm],
                s=22,
                marker="^",
                color="#7B61A8",
                depthshade=False,
            )

        all_points = np.array([sample.pose[:3] for sample in self.result.samples])
        minimum = all_points.min(axis=0)
        maximum = all_points.max(axis=0)
        span = np.maximum(maximum - minimum, 1.0)
        padding = 0.08 * span
        axis.set_xlim(minimum[0] - padding[0], maximum[0] + padding[0])
        axis.set_ylim(minimum[1] - padding[1], maximum[1] + padding[1])
        axis.set_zlim(max(0.0, minimum[2] - padding[2]), maximum[2] + padding[2])
        axis.view_init(elev=25, azim=-58)
        self.completed_path, = axis.plot(
            [], [], [], color="#17191D", linewidth=1.0, alpha=0.55
        )
        self.path_effector, = axis.plot(
            [], [], [], marker="o", markersize=6, color="#17191D"
        )
        self.path_yaw, = axis.plot([], [], [], color="#17191D", linewidth=2.0)

    def _draw_timeline_static(self, pose_axis, yaw_axis, limits_axis) -> None:
        samples = self.result.samples
        times = np.array([sample.time_s for sample in samples])
        poses = np.array([sample.pose for sample in samples])
        pose_axis.set_title("Pose Timeline", fontsize=11, fontweight="bold")
        pose_axis.set_xlabel("Time (s)", fontsize=9)
        pose_axis.set_ylabel("Position (mm)", fontsize=9)
        pose_axis.plot(times, poses[:, 0], color="#0072B2", label="X")
        pose_axis.plot(times, poses[:, 1], color="#E69F00", label="Y")
        pose_axis.plot(times, poses[:, 2], color="#009E73", label="Z")
        yaw_axis.set_ylabel("Yaw (deg)", fontsize=9, color="#7B61A8")
        yaw_axis.plot(times, poses[:, 3], color="#7B61A8", linewidth=1.4, label="Yaw")
        yaw_axis.tick_params(axis="y", colors="#7B61A8", labelsize=8)
        pose_lines = pose_axis.get_lines() + yaw_axis.get_lines()
        pose_axis.legend(
            pose_lines,
            [line.get_label() for line in pose_lines],
            loc="upper right",
            ncol=4,
            fontsize=8,
        )
        self.pose_cursor = pose_axis.axvline(0.0, color="#222222", linewidth=1.1)
        self.yaw_cursor = yaw_axis.axvline(0.0, color="#222222", linewidth=1.1)

        limits = self.result.scenario.limits
        ratios = np.array(
            [
                [
                    sample.linear_speed / limits.max_linear_velocity_mm_s,
                    sample.linear_acceleration / limits.max_linear_acceleration_mm_s2,
                    sample.yaw_speed / limits.max_yaw_velocity_deg_s,
                    sample.yaw_acceleration / limits.max_yaw_acceleration_deg_s2,
                ]
                for sample in samples
            ]
        )
        grips = np.array([sample.grip for sample in samples], dtype=float)
        labels = ("Linear v", "Linear a", "Yaw v", "Yaw a")
        colors = ("#0072B2", "#E69F00", "#009E73", "#CC79A7")
        limits_axis.set_title("Limits & Grip", fontsize=11, fontweight="bold")
        limits_axis.set_xlabel("Time (s)", fontsize=9)
        limits_axis.set_ylabel("Limit ratio", fontsize=9)
        for index, label in enumerate(labels):
            limits_axis.plot(times, ratios[:, index], color=colors[index], label=label)
        limits_axis.step(
            times,
            grips,
            where="post",
            color="#222222",
            linewidth=1.2,
            alpha=0.75,
            label="Grip",
        )
        limits_axis.axhline(1.0, color=RISK_COLOR, linestyle="--", linewidth=1.0)
        limits_axis.set_ylim(0.0, max(1.2, float(ratios.max()) * 1.1))
        limits_axis.legend(loc="upper right", ncol=3, fontsize=7)
        self.limits_cursor = limits_axis.axvline(0.0, color="#222222", linewidth=1.1)
        self._constraint_ratios = ratios

        if hasattr(self, "status_text"):
            self.status_text.remove()
            self.warning_text.remove()
        self.status_text = self.figure.text(
            0.37, 0.035, "", fontsize=9, color=TEXT_COLOR, ha="left", va="center"
        )
        self.warning_text = self.figure.text(
            0.94,
            0.035,
            "",
            fontsize=9,
            fontweight="bold",
            color=RISK_COLOR,
            ha="right",
            va="center",
        )

    def _on_slider_changed(self, value: float) -> None:
        if not self._updating_slider:
            self.set_time(float(value), update_slider=False)

    def _on_timer(self) -> None:
        if not self.playing:
            return
        next_time = (
            self.current_time + TIMER_INTERVAL_MS / 1000.0 * PLAYBACK_RATE
        )
        if next_time > self.duration_s:
            next_time = 0.0
        self.set_time(next_time)

    def toggle_play(self) -> None:
        self.playing = not self.playing
        self.play_button.label.set_text("Pause" if self.playing else "Play")
        if self.playing:
            self.timer.start()
        else:
            self.timer.stop()

    def reset(self) -> None:
        if self.playing:
            self.toggle_play()
        self.set_time(0.0)

    def _sample_index(self, time_s: float) -> int:
        times = np.array([sample.time_s for sample in self.result.samples])
        return int(np.clip(np.searchsorted(times, time_s, side="right") - 1, 0, len(times) - 1))

    def set_time(self, time_s: float, update_slider: bool = True) -> None:
        self.current_time = float(np.clip(time_s, 0.0, self.duration_s))
        sample_index = self._sample_index(self.current_time)
        sample = self.result.samples[sample_index]

        for execution in self.result.moves:
            piece_id = execution.piece_id
            if execution.move_index < sample.move_index:
                polygon = self.result.final_polygons[piece_id]
            elif execution.move_index > sample.move_index:
                polygon = self.result.initial_polygons[piece_id]
            elif sample.phase == "transfer" and sample.state == 1:
                polygon = self.result.final_polygons[piece_id]
            elif sample.grip == 1:
                polygon = _piece_at_pose(
                    self.result.initial_polygons[piece_id],
                    execution.move.pick,
                    sample.pose,
                )
            else:
                polygon = self.result.initial_polygons[piece_id]
            self._piece_patches[piece_id].set_xy(polygon)
            self._piece_labels[piece_id].set_position(polygon.mean(axis=0))

        x, y, z, yaw = sample.pose
        yaw_rad = np.deg2rad(yaw)
        yaw_length = 15.0
        yaw_end = (x + yaw_length * np.cos(yaw_rad), y + yaw_length * np.sin(yaw_rad))
        self.board_effector.set_data([x], [y])
        self.board_yaw.set_data([x, yaw_end[0]], [y, yaw_end[1]])

        completed = np.array(
            [entry.pose[:3] for entry in self.result.samples[: sample_index + 1]]
        )
        self.completed_path.set_data(completed[:, 0], completed[:, 1])
        self.completed_path.set_3d_properties(completed[:, 2])
        self.path_effector.set_data([x], [y])
        self.path_effector.set_3d_properties([z])
        self.path_yaw.set_data([x, yaw_end[0]], [y, yaw_end[1]])
        self.path_yaw.set_3d_properties([z, z])

        for cursor in (self.pose_cursor, self.yaw_cursor, self.limits_cursor):
            cursor.set_xdata([self.current_time, self.current_time])

        state_name = "Complete" if sample.state == 1 else "Running"
        self.status_text.set_text(
            f"Piece: {sample.piece_id} ({sample.move_index + 1}/4)  |  "
            f"Phase: {sample.phase.title()}  |  State: {state_name}  |  "
            f"Grip: {sample.grip}  |  t={self.current_time:.2f}s"
        )
        constraint_exceeded = bool(np.max(self._constraint_ratios[sample_index]) > 1.0)
        if constraint_exceeded:
            self.warning_text.set_text("LIMIT EXCEEDED")
        elif sample.low_height_risk:
            self.warning_text.set_text("LOW-Z XY TRAVEL")
        else:
            self.warning_text.set_text("")

        if update_slider and hasattr(self, "time_slider"):
            self._updating_slider = True
            self.time_slider.set_val(self.current_time)
            self._updating_slider = False
        self.figure.canvas.draw_idle()

    def save_snapshot(self, path: Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        self.figure.savefig(path, dpi=150, facecolor="white")


def create_figure(result: SimulationResult):
    figure = plt.figure(figsize=(15, 9), facecolor="#F4F6F8")
    grid = figure.add_gridspec(
        2,
        2,
        left=0.055,
        right=0.97,
        bottom=0.18,
        top=0.91,
        width_ratios=(1.0, 1.15),
        hspace=0.32,
        wspace=0.22,
    )
    board_axis = figure.add_subplot(grid[0, 0])
    path_axis = figure.add_subplot(grid[0, 1], projection="3d")
    pose_axis = figure.add_subplot(grid[1, 0])
    yaw_axis = pose_axis.twinx()
    limits_axis = figure.add_subplot(grid[1, 1])
    axes = {
        "board": board_axis,
        "path": path_axis,
        "pose": pose_axis,
        "yaw": yaw_axis,
        "limits": limits_axis,
    }
    view = SimulationView(figure, axes, result)
    return figure, view

#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
live_plotter.py: live data plotting using matplotlib.
"""

import math

import matplotlib
import matplotlib.pylab as plt
import numpy as np

# 在Docker中使用X11转发时，使用TkAgg后端
# 如果没有显示，会自动回退到Agg后端
try:
    matplotlib.use("TkAgg")
except ImportError:
    matplotlib.use("Agg")
matplotlib.interactive(False)
current_cmap = matplotlib.cm.get_cmap()
current_cmap.set_bad(color="gray")

MAX_YLIM = math.inf  # set to inf for no effect.
MIN_YLIM = -math.inf  # set to -inf for no effect.

MAX_XLIM = math.inf  # set to inf for no effect.
MIN_XLIM = -math.inf  # set to -inf for no effect.

EPS = 1e-10


def add_colorbar(fig, ax, im, title=None):
    from mpl_toolkits.axes_grid1 import make_axes_locatable

    divider = make_axes_locatable(ax)
    cax = divider.append_axes("right", size="5%", pad=0.05)
    return cax


class LivePlotter(object):
    def __init__(
        self,
        max_ylim=MAX_YLIM,
        min_ylim=MIN_YLIM,
        log=True,
        label="",
        max_xlim=MAX_XLIM,
        min_xlim=MIN_XLIM,
        ax=None,
        fig=None,
        timestamp=None,
        logger=None,  # 添加logger参数
    ):
        self.max_ylim = max_ylim
        self.min_ylim = min_ylim
        self.max_xlim = max_xlim
        self.min_xlim = min_xlim
        self.vmin = np.inf
        self.vmax = -np.inf

        self.log = log
        self._current_timestamp = timestamp  # for saving plots
        self.logger = logger  # 存储传入的logger
        

        if (fig is None) and (ax is None):
            self.fig, self.ax = plt.subplots()
        else:
            self.fig = fig
            self.ax = ax

        self.fig.canvas.set_window_title(label)

        # containers for continuously updated data
        self.lines = {}
        self.axvlines = {}
        self.arrows = {}
        self.scatter = {}

        self.mesh = None
        self.colorbar = None
        
        # 添加图像保存功能
        self.save_counter = 0
        self.save_interval = 1  # 每10次更新保存一次
        self.save_dir = "AUDIOROS/tmp/plots"  # Docker容器中的保存路径
        import os
        os.makedirs(self.save_dir, exist_ok=True)

        self.fig.canvas.mpl_connect("close_event", self.handle_close)
        self.fig.canvas.mpl_connect("resize_event", self.handle_resize)

        # Need the block argument to make sure the script continues after
        # plotting the figure.
        plt.show(block=False)

    def save_plot(self, suffix="", timestamp=None):
        """保存当前图像到文件"""
        # 优先使用传入的timestamp，否则使用存储的_current_timestamp
        save_timestamp = timestamp if timestamp is not None else getattr(self, '_current_timestamp', None)
        if save_timestamp is None:
            filename = f"{self.save_dir}/plot_{self.save_counter:04d}{suffix}.png"
        else:
            filename = f"{self.save_dir}/plot_ts_{save_timestamp}{suffix}.png"
        self.fig.savefig(filename, dpi=100, bbox_inches='tight')
        # self.get_logger().info(f"Saved plot to {filename} with timestamp: {save_timestamp}")
        self.save_counter += 1

    def set_timestamp(self, timestamp):
        """设置当前时间戳，用于文件名"""
        self._current_timestamp = timestamp

    def get_logger(self):
        """获取logger，如果没有则返回None或使用print"""
        if self.logger is not None:
            return self.logger
        else:
            # 如果没有传入logger，创建一个模拟的logger
            class MockLogger:
                def info(self, msg):
                    print(f"[INFO] {msg}")
                def warn(self, msg):
                    print(f"[WARN] {msg}")
                def error(self, msg):
                    print(f"[ERROR] {msg}")
            return MockLogger()

    def handle_close(self, evt):
        plt.close("all")

    def handle_resize(self, evt):
        # TODO(FD) this is not called when we resize
        # the figure, need to figure out why.
        self.fig.canvas.draw()

    def clear(self):
        for i in range(len(self.ax.lines)):
            self.ax.lines.pop()
        self.lines = {}
        self.axvlines = {}

    def update_arrow(self, origin, angle_deg, label=None):
        """ Update arrow coordinates. 
        """
        xmin, xmax = self.ax.get_xlim()
        arrow_length = (xmax - xmin) / 5

        origin_x, origin_y = origin
        dx = arrow_length * math.cos(angle_deg * math.pi / 180)
        dy = arrow_length * math.sin(angle_deg * math.pi / 180)

        if label in self.arrows.keys():
            self.arrows[label]["pointer"].set_data(origin_x + dx, origin_y + dy)
            self.arrows[label]["line"].set_data(
                [origin_x, origin_x + dx], [origin_y, origin_y + dy]
            )
        else:
            self.arrows[label] = {}
            (line,) = self.ax.plot(
                origin_x + dx, origin_y + dy, marker=">", color=f"C{len(self.arrows)-1}"
            )
            self.arrows[label]["pointer"] = line
            (line,) = self.ax.plot(
                [origin_x, origin_x + dx],
                [origin_y, origin_y + dy],
                label=label,
                color=f"C{len(self.arrows)-1}",
            )
            self.arrows[label]["line"] = line

    def update_lines(self, row_matrix, x_data=None, labels=None, **kwargs):
        """ Plot each row of row_matrix as one line.
        """
        for i in range(row_matrix.shape[0]):
            if i in self.lines.keys():
                if x_data is not None:
                    self.lines[i].set_data(x_data, row_matrix[i, :])
                    self.ax.set_xlim(min(x_data), max(x_data))
                else:
                    self.lines[i].set_ydata(row_matrix[i, :])

                if labels is not None:
                    self.lines[i].set_label(labels[i])
            else:
                x_data = range(row_matrix.shape[1]) if x_data is None else x_data
                label = labels[i] if labels is not None else None
                if self.log:
                    (line,) = self.ax.semilogy(
                        x_data,
                        row_matrix[i, :],
                        color=f"C{i % 10}",
                        label=label,
                        **kwargs,
                    )
                else:
                    (line,) = self.ax.plot(
                        x_data,
                        row_matrix[i, :],
                        color=f"C{i % 10}",
                        label=label,
                        **kwargs,
                    )
                self.lines[i] = line

        self.reset_xlim()
        self.reset_ylim()

    def update_mesh(
        self,
        data_matrix,
        y_labels=None,
        x_labels=None,
        n_labels=5,
        colorbar=False,
        vmin=None,
        vmax=None,
        logger=None,
    ):

        """ Plot data_matrix as a heatmap. """

        # we round vmax and vmin so we don't have to recretae colorbars all the time for minor changes.
        if vmax is None:
            vmax = np.max(data_matrix)
            if self.log:
                if vmax == 0:
                    vmax = 2 * EPS  # 2 x to make it bigger than vmin
                vmax = np.log10(vmax)
            vmax = np.ceil(vmax)

        if vmin is None:
            if self.log:
                vals = data_matrix[data_matrix > 0]
                if len(vals):
                    vmin = np.min(vals)
                else:
                    vmin = EPS
                vmin = np.log10(vmin)
            else:
                vmin = np.min(data_matrix)
            vmin = np.floor(vmin)

        if vmin >= vmax:
            if logger:
                logger.error(f"Cannot plot from {vmin} to {vmax}")
            return

        if logger:
            logger.info(f"Plotting from {vmin} to {vmax}")

        if self.log:
            data = np.log10(data_matrix)
        else:
            data = data_matrix

        if self.mesh is None:
            self.mesh = self.ax.pcolorfast(data)
        else:
            self.mesh.set_data(data)

        self.mesh.set_clim(vmin, vmax)
        if colorbar:
            title = "log10-values" if self.log else "values"
            if self.colorbar is None:
                axcolorbar = add_colorbar(self.fig, self.ax, self.mesh)
                axcolorbar.set_ylabel(title, rotation=90)
                self.colorbar = self.fig.colorbar(
                    self.mesh, cax=axcolorbar, orientation="vertical"
                )

            self.colorbar.set_ticks(np.linspace(vmin, vmax, num=5))
            self.colorbar.draw_all()

        for axis, labels in zip(["x", "y"], [x_labels, y_labels]):
            ax = eval(f"self.ax.get_{axis}axis()")
            if labels is not None:
                if n_labels is None:
                    step = 1
                else:
                    step = max(len(labels) // n_labels, 1)
            ax.set_ticks(step / 2 + np.arange(len(labels), step=step))
            ax.set_ticklabels(labels[::step])

    def update_axvlines(self, vlines, **kwargs):
        for i, xcoord in enumerate(vlines):
            if i in self.axvlines.keys():
                self.axvlines[i].set_xdata(xcoord)
            else:
                color = kwargs.get("color", None)
                if color is None:
                    color = f"C{i % 10}"
                axvline = self.ax.axvline(xcoord, ls=":", **kwargs)
                self.axvlines[i] = axvline

    def update_scatter(self, x_data, y_data, label="position", **kwargs):
        """ Plot x_data and y_data as scattered points.
        """
        # 提取timestamp参数，不传递给matplotlib
        timestamp = kwargs.pop('timestamp', None)
        
        if label in self.scatter.keys():
            self.scatter[label].set_data(x_data, y_data)

        else:
            (line,) = self.ax.plot(
                x_data, y_data, label=label, linestyle="", marker="o", **kwargs
            )
            self.scatter[label] = line

        self.reset_xlim()
        self.reset_ylim()
        
        '''''
        # 定期保存图像（仅在geometry可视化时）
        if hasattr(self, 'save_counter') and self.save_counter % self.save_interval == 0:
            # 优先使用传入的timestamp，否则使用_current_timestamp
            save_timestamp = timestamp if timestamp is not None else getattr(self, '_current_timestamp', None)
            if save_timestamp is not None:
                self.save_plot("_geometry", timestamp=save_timestamp)
            else:
                self.save_plot("_geometry")
        '''
    def reset_ylim(self):
        # recompute the ax.dataLim
        self.ax.relim()

        # limit ymin and ymax to chosen window.
        ymin, ymax = self.ax.get_ylim()
        ymax_new = min(max(self.ax.dataLim.y1, ymax), self.max_ylim)
        ymin_new = max(min(self.ax.dataLim.y0, ymin), self.min_ylim)

        # update ax.viewLim using new ax.dataLim
        self.ax.set_ylim(ymin_new, ymax_new)

    def reset_xlim(self):
        # recompute the ax.dataLim
        self.ax.relim()

        # limit xmin and xmax to chosen window.
        xmin, xmax = self.ax.get_xlim()
        xmax_new = min(max(self.ax.dataLim.x1, xmax), self.max_xlim)
        xmin_new = max(min(self.ax.dataLim.x0, xmin), self.min_xlim)

        # update ax.viewLim using new ax.dataLim
        self.ax.set_xlim(xmin_new, xmax_new)


if __name__ == "__main__":
    test = LivePlotter()

#!/usr/bin/env python3
"""
四足机器人步行日志可视化 (阅读优化版，含mode背景)

时间戳自动转换为相对时间，横轴单位固定为 ms。
绘制:
  1. 关节角度 (rad) 随时间变化（背景按 mode 着色）
  2. 足端力 (N) 随时间变化（背景按 mode 着色）

用法:
    python plot_walk_optimized.py walk_000.csv
    python plot_walk_optimized.py walk_*.csv
    python plot_walk_optimized.py --subsample 10 walk_*.csv
"""

import argparse
import glob
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# ---------- 列名映射 (与 C++ logWorker 输出顺序一致) ----------
COLUMN_NAMES = [
    "t", "mode",
    "lf_0", "lf_1", "lf_2",
    "rf_0", "rf_1", "rf_2",
    "lb_0", "lb_1", "lb_2",
    "rb_0", "rb_1", "rb_2",
    "lf_fx", "lf_fy", "lf_fz",
    "rf_fx", "rf_fy", "rf_fz",
    "lb_fx", "lb_fy", "lb_fz",
    "rb_fx", "rb_fy", "rb_fz",
]

LEG_NAMES = ["lf", "rf", "lb", "rb"]
JOINT_NAMES = ["0", "1", "2"]
FORCE_AXES = ["fx", "fy", "fz"]

LEG_COLORS = {"lf": "C0", "rf": "C1", "lb": "C2", "rb": "C3"}
JOINT_LABELS = ["Hip", "Thigh", "Calf"]
JOINT_LINESTYLES = ["-", "--", "-."]   # 不同关节用不同线型，便于黑白打印区分

# 用于 mode 背景的颜色列表（柔和色）
# 使用高对比度、色盲友好的颜色（Tableau 10 调色板的一部分）
MODE_COLORS = [
    '#1f77b4',  # 蓝色
    '#ff7f0e',  # 橙色
    '#2ca02c',  # 绿色
    '#d62728',  # 红色
    '#9467bd',  # 紫色
    '#8c564b',  # 棕色
    '#e377c2',  # 粉色
    '#7f7f7f',  # 灰色
    '#bcbd22',  # 橄榄绿
    '#17becf',  # 青色
]


def load_data(file_patterns):
    """加载所有 CSV 并合并，将绝对时间戳转换为相对时间（毫秒）"""
    files = []
    for pat in file_patterns:
        matched = glob.glob(pat)
        if not matched:
            print(f"警告: 未找到 '{pat}'", file=sys.stderr)
        files.extend(matched)

    if not files:
        raise FileNotFoundError("没有找到日志文件")

    dfs = []
    for f in sorted(files):
        print(f"加载: {f}")
        df = pd.read_csv(f, comment="#", names=COLUMN_NAMES, skiprows=1, dtype=float)
        if dfs and not df.empty:
            # 多文件时累加时间，保证连续
            last_t = dfs[-1]["t"].iloc[-1]
            df["t"] += last_t
        dfs.append(df)

    full = pd.concat(dfs, ignore_index=True).dropna().sort_values("t").reset_index(drop=True)
    
    # 转换为相对时间（毫秒），从0开始
    t_start = full["t"].iloc[0]
    full["t"] = full["t"] - t_start
    
    total_ms = full["t"].max()
    total_s = total_ms / 1000.0
    print(f"数据量: {len(full)} 行, 时间跨度: {total_ms:.2f} ms ({total_s:.3f} s)")
    return full, files, total_ms


def get_mode_intervals(df):
    """
    根据 mode 列的变化，返回区间列表，每个元素为 (start_time, end_time, mode_value)
    """
    if df.empty:
        return []
    
    t = df["t"].values
    mode = df["mode"].values
    
    intervals = []
    start_idx = 0
    current_mode = mode[0]
    
    for i in range(1, len(mode)):
        if mode[i] != current_mode:
            intervals.append((t[start_idx], t[i-1], current_mode))
            start_idx = i
            current_mode = mode[i]
    # 最后一个区间
    intervals.append((t[start_idx], t[-1], current_mode))
    return intervals


def add_mode_background(ax, df):
    """在坐标轴上添加 mode 背景色块，并在右上角绘制 mode 图例"""
    intervals = get_mode_intervals(df)
    used_modes = set()
    
    for t_start, t_end, mode_val in intervals:
        color = MODE_COLORS[int(mode_val) % len(MODE_COLORS)]
        ax.axvspan(t_start, t_end, facecolor=color, alpha=0.25, zorder=0, edgecolor='none')
        used_modes.add(int(mode_val))
        # 在区间中央顶部添加 mode 数字
        mid = (t_start + t_end) / 2
        ax.text(mid, 1.01, f'{int(mode_val)}', transform=ax.get_xaxis_transform(),
                ha='center', va='bottom', fontsize=8, fontweight='bold',
                color=color, bbox=dict(boxstyle="round,pad=0.1", facecolor='white', alpha=0.8, edgecolor=color))
    # 在右上角添加 mode 色块图例
    if used_modes:
        legend_text = "Modes: " + ", ".join([f"{m}" for m in sorted(used_modes)])
        ax.text(0.98, 0.98, legend_text, transform=ax.transAxes,
                ha='right', va='top', fontsize=8,
                bbox=dict(boxstyle="round,pad=0.3", facecolor='white', alpha=0.9, edgecolor='gray'))
        # 可选：绘制小色块（简单起见用文字描述）

        
def format_time_axis(ax, total_ms):
    """固定使用毫秒作为横轴单位"""
    ax.set_xlabel("时间 (ms)")
    ax.xaxis.set_major_formatter(plt.ScalarFormatter())


def plot_joint_angles(df, subsample=1, title_suffix="", total_ms=0):
    """四条腿，每条腿三个关节，角度随时间变化"""
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    fig.suptitle(f"关节角度 (rad) 随时间变化 {title_suffix}", fontsize=14)

    idx = slice(None, None, subsample)
    t = df["t"].iloc[idx]

    for i, leg in enumerate(LEG_NAMES):
        ax = axes[i]
        for j, joint in enumerate(JOINT_NAMES):
            col = f"{leg}_{joint}"
            ax.plot(t, df[col].iloc[idx], 
                    label=JOINT_LABELS[j], 
                    color=f"C{j}",
                    linestyle=JOINT_LINESTYLES[j],
                    linewidth=1.2)
        ax.set_ylabel(f"{leg.upper()} (rad)")
        ax.grid(True, alpha=0.2, linestyle='--')
        ax.legend(loc="upper right", fontsize=9)
        # 添加腿部标签在左上角
        ax.text(0.02, 0.95, leg.upper(), transform=ax.transAxes,
                fontsize=11, fontweight='bold', va='top',
                bbox=dict(boxstyle="round,pad=0.2", facecolor='white', alpha=0.7))
    
    # 在所有子图上添加 mode 背景（使用第一个子图获取背景信息，但需应用到所有子图）
    for ax in axes:
        add_mode_background(ax, df)
    
    format_time_axis(axes[-1], total_ms)
    # 调整 y 轴范围以适应 mode 标签（已在 text 中使用 axes coordinates 不影响数据区）
    plt.tight_layout()
    return fig


def plot_forces(df, subsample=1, title_suffix="", total_ms=0):
    """四条腿，每条腿三个方向的力 (fx, fy, fz) 随时间变化"""
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    fig.suptitle(f"足端期望力 (N) 随时间变化 {title_suffix}", fontsize=14)

    idx = slice(None, None, subsample)
    t = df["t"].iloc[idx]

    for i, axis in enumerate(FORCE_AXES):
        ax = axes[i]
        for leg in LEG_NAMES:
            col = f"{leg}_f{axis[1]}"   # fx, fy, fz
            ax.plot(t, df[col].iloc[idx], 
                    label=leg.upper(), 
                    color=LEG_COLORS[leg],
                    linewidth=1.2)
        ax.set_ylabel(f"{axis.upper()} (N)")
        ax.grid(True, alpha=0.2, linestyle='--')
        ax.legend(loc="upper right", fontsize=9)
        # 添加方向标签
        ax.text(0.02, 0.95, f"Force {axis.upper()}", transform=ax.transAxes,
                fontsize=11, fontweight='bold', va='top',
                bbox=dict(boxstyle="round,pad=0.2", facecolor='white', alpha=0.7))
        # 对于 FZ 子图，添加零力参考线
        if axis == "fz":
            ax.axhline(0, color='gray', linestyle=':', linewidth=0.8, alpha=0.7)

    # 添加 mode 背景
    for ax in axes:
        add_mode_background(ax, df)

    format_time_axis(axes[-1], total_ms)
    plt.tight_layout()
    return fig


def main():
    parser = argparse.ArgumentParser(description="步行日志可视化（阅读优化版，含mode背景）")
    parser.add_argument("files", nargs="+", help="CSV 文件，支持通配符")
    parser.add_argument("--subsample", type=int, default=1, help="降采样因子")
    parser.add_argument("--no-show", action="store_true", help="不显示图形")
    parser.add_argument("--save", type=str, help="保存图片前缀")
    args = parser.parse_args()

    try:
        df, files, total_ms = load_data(args.files)
    except FileNotFoundError as e:
        print(e, file=sys.stderr)
        sys.exit(1)

    # 标题后缀
    if len(files) == 1:
        suffix = f"({Path(files[0]).name})"
    else:
        suffix = f"(共{len(files)}个文件)"

    figs = [
        ("angles", plot_joint_angles(df, args.subsample, suffix, total_ms)),
        ("forces", plot_forces(df, args.subsample, suffix, total_ms)),
    ]

    if args.save:
        for name, fig in figs:
            fname = f"{args.save}{name}.png"
            fig.savefig(fname, dpi=150, bbox_inches="tight")
            print(f"已保存: {fname}")

    if not args.no_show:
        plt.show()
    else:
        for _, fig in figs:
            plt.close(fig)


if __name__ == "__main__":
    main()
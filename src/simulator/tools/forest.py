import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

# ---------------- 配置路径 ----------------
nearest_csv_path_1 = "src/simulator/statistics/forest/find_nearest_stats_20251208_222834.csv"
nearest_csv_path_2 = "src/simulator/statistics/forest/find_nearest_stats_20251208_232332.csv"
nearest_csv_path_3 = "src/simulator/statistics/forest/find_nearest_stats_20251210_230321.csv"

nbv_csv_path_1 = "src/simulator/statistics/forest/nbvplanner_stats_20251208_221909.csv"
nbv_csv_path_2 = "src/simulator/statistics/forest/nbvplanner_stats_20251209_005243.csv"
nbv_csv_path_3 = "src/simulator/statistics/forest/nbvplanner_stats_20251211_095703.csv"

hpp_csv_path_1 = "src/simulator/statistics/forest/hpp_stats_20251210_141631.csv"
hpp_csv_path_2 = "src/simulator/statistics/forest/hpp_stats_20251210_142721.csv"
hpp_csv_path_3 = "src/simulator/statistics/forest/hpp_stats_20251211_100617.csv"

# ---------------- 读取并清洗数据 ----------------
def load_and_clean(path):
    df = pd.read_csv(path)
    df.columns = df.columns.str.strip()
    return df["time_elapsed"].to_numpy(), df["explored_cells"].to_numpy()

nearest_ts1, nearest_rate1 = load_and_clean(nearest_csv_path_1)
nearest_ts2, nearest_rate2 = load_and_clean(nearest_csv_path_2)
nearest_ts3, nearest_rate3 = load_and_clean(nearest_csv_path_3)

nbv_ts1, nbv_rate1 = load_and_clean(nbv_csv_path_1)
nbv_ts2, nbv_rate2 = load_and_clean(nbv_csv_path_2)
nbv_ts3, nbv_rate3 = load_and_clean(nbv_csv_path_3)

hpp_ts1, hpp_rate1 = load_and_clean(hpp_csv_path_1)
hpp_ts2, hpp_rate2 = load_and_clean(hpp_csv_path_2)
hpp_ts3, hpp_rate3 = load_and_clean(hpp_csv_path_3)

# ---------------- 关键修改 1: 分别计算每种方法的具体最大时间 ----------------

# 1. Nearest 的最大时间
nearest_max_time = max(
    nearest_ts1.max() if len(nearest_ts1) else 0,
    nearest_ts2.max() if len(nearest_ts2) else 0,
    nearest_ts3.max() if len(nearest_ts3) else 0
)

# 2. NBV 的最大时间
nbv_max_time = max(
    nbv_ts1.max() if len(nbv_ts1) else 0,
    nbv_ts2.max() if len(nbv_ts2) else 0,
    nbv_ts3.max() if len(nbv_ts3) else 0
)

# 3. HPP 的最大时间
hpp_max_time = max(
    hpp_ts1.max() if len(hpp_ts1) else 0,
    hpp_ts2.max() if len(hpp_ts2) else 0,
    hpp_ts3.max() if len(hpp_ts3) else 0
)

# ---------------- 关键修改 2: 生成 3 个独立的时间网格 ----------------
# 每个网格只延伸到该方法自己的最大时间
grid_points = 1000
nearest_time_grid = np.linspace(0, nearest_max_time, grid_points)
nbv_time_grid = np.linspace(0, nbv_max_time, grid_points)
hpp_time_grid = np.linspace(0, hpp_max_time, grid_points)

# ---------------- 关键修改 3: 分别插值 ----------------
def get_interp_rate(ts, rate, grid):
    if len(ts) == 0: return np.zeros_like(grid)
    # 依然保留 fill_value 策略：
    # 虽然现在 grid 变短了，但比如 Trial 1 比 Nearest最大时间 短的时候，
    # 依然需要保持 Trial 1 的最后值直到 Nearest最大时间，以便计算均值。
    f = interp1d(ts, rate, bounds_error=False, fill_value=(rate[0], rate[-1]))
    return f(grid)

# Nearest 插值到 nearest_time_grid
n_r1 = get_interp_rate(nearest_ts1, nearest_rate1, nearest_time_grid)
n_r2 = get_interp_rate(nearest_ts2, nearest_rate2, nearest_time_grid)
n_r3 = get_interp_rate(nearest_ts3, nearest_rate3, nearest_time_grid)

# NBV 插值到 nbv_time_grid
b_r1 = get_interp_rate(nbv_ts1, nbv_rate1, nbv_time_grid)
b_r2 = get_interp_rate(nbv_ts2, nbv_rate2, nbv_time_grid)
b_r3 = get_interp_rate(nbv_ts3, nbv_rate3, nbv_time_grid)

# HPP 插值到 hpp_time_grid
h_r1 = get_interp_rate(hpp_ts1, hpp_rate1, hpp_time_grid)
h_r2 = get_interp_rate(hpp_ts2, hpp_rate2, hpp_time_grid)
h_r3 = get_interp_rate(hpp_ts3, hpp_rate3, hpp_time_grid)

# ---------------- 3. 计算均值和标准差 ----------------
# Nearest Stats
nearest_mean = np.mean([n_r1, n_r2, n_r3], axis=0)
nearest_std = np.std([n_r1, n_r2, n_r3], axis=0)
nearest_max_val = np.max(nearest_mean)

# NBV Stats
nbv_mean = np.mean([b_r1, b_r2, b_r3], axis=0)
nbv_std = np.std([b_r1, b_r2, b_r3], axis=0)
nbv_max_val = np.max(nbv_mean)

# HPP Stats
hpp_mean = np.mean([h_r1, h_r2, h_r3], axis=0)
hpp_std = np.std([h_r1, h_r2, h_r3], axis=0)
hpp_max_val = np.max(hpp_mean)

print(f"Nearest Max: {nearest_max_val:.2f} at {nearest_max_time:.2f}s")
print(f"NBV Max:     {nbv_max_val:.2f} at {nbv_max_time:.2f}s")
print(f"HPP Max:     {hpp_max_val:.2f} at {hpp_max_time:.2f}s")

# ---------------- 4. 绘制折线图 ----------------
pixel_width = 4800
pixel_height = 2200
dpi = 100
fig_width = pixel_width / dpi
fig_height = pixel_height / dpi
plt.figure(figsize=(fig_width/2, fig_height/2), dpi=dpi)

# 绘制 Nearest (使用 nearest_time_grid)
plt.plot(nearest_time_grid, nearest_mean, label=f"Nearest", color="#f86c26")
plt.fill_between(nearest_time_grid, nearest_mean - nearest_std, nearest_mean + nearest_std, color="#f86c26", alpha=0.1)
plt.plot(nearest_time_grid, nearest_mean - nearest_std, "--", color="#f86c26", alpha=0.5, linewidth=1)
plt.plot(nearest_time_grid, nearest_mean + nearest_std, "--", color="#f86c26", alpha=0.5, linewidth=1)

# 绘制 NBV (使用 nbv_time_grid)
plt.plot(nbv_time_grid, nbv_mean, label=f"NBVP", color="#6bc656")
plt.fill_between(nbv_time_grid, nbv_mean - nbv_std, nbv_mean + nbv_std, color="#6bc656", alpha=0.1)
plt.plot(nbv_time_grid, nbv_mean - nbv_std, "--", color="#6bc656", alpha=0.5, linewidth=1)
plt.plot(nbv_time_grid, nbv_mean + nbv_std, "--", color="#6bc656", alpha=0.5, linewidth=1)

# 绘制 HPP (使用 hpp_time_grid)
plt.plot(hpp_time_grid, hpp_mean, label=f"HPP", color="#009eff")
plt.fill_between(hpp_time_grid, hpp_mean - hpp_std, hpp_mean + hpp_std, color="#009eff", alpha=0.1)
plt.plot(hpp_time_grid, hpp_mean - hpp_std, "--", color="#009eff", alpha=0.5, linewidth=1)
plt.plot(hpp_time_grid, hpp_mean + hpp_std, "--", color="#009eff", alpha=0.5, linewidth=1)

# 阈值线
threshold_y = 9500
plt.axhline(y=threshold_y, color="#373627", linestyle="--", linewidth=1.5, label="Threshold (9500)")

# ---------------- X 标记逻辑 (需要分别处理每个 grid) ----------------
marker_y_pos = 9800 

def plot_markers(time_grid, mean_rate, color):
    for idx in range(len(time_grid) - 1):
        if mean_rate[idx] < threshold_y and mean_rate[idx + 1] > threshold_y:
            plt.plot((time_grid[idx] + time_grid[idx + 1]) / 2, marker_y_pos, 
                     "x", color=color, markersize=14, markeredgewidth=3)


# ---------------- 5. 设置图表样式 ----------------
plt.ylim(0, 12000)
# X轴范围可以设为最长那个时间，或者让 matplotlib 自动处理
plt.xlim(0, max(nearest_max_time, nbv_max_time, hpp_max_time)) 

plt.tick_params(axis="both", labelsize=26)
plt.xlabel("Time (s)", fontsize=34)
plt.ylabel("Exploration Volume (Cells)", fontsize=34)
plt.legend(fontsize=24)
plt.grid(True)
plt.subplots_adjust(top=0.94, bottom=0.175, left=0.13, right=0.97)

plt.show()
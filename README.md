# AS2 温室巡检机器狗：仿真与 SLAM

当前发布版面向温室第 2、3、4、5 通道的四通道蛇形巡航：机器狗在 Isaac Sim 中依次巡航四条垄沟，经端区换道并沿底部通道返回起点。Windows 负责物理仿真、传感器和运动控制，Ubuntu 22.04 / ROS 2 Humble 负责 ICP+IMU 里程计与 RGB+激光 RTAB-Map 建图。实机阶段计划缩小为第 2、3 通道闭环。

## 目录

- `assets/`：AS2、SO-ARM101 及组合模型的必要网格、URDF 和 USD 资产。
- `work/`：当前仿真、在线估计、控制与验收代码；已移除历史版本和一次性实验脚本。
- `launch/`：四通道、两通道、短展示和 Ubuntu 建图/录包入口。
- `outputs/`：仅保留 2026-11-16 两次独立通过验收的四通道运行。
- `baselines/four-aisle-current/`：当前源码快照、可信结果哈希和一键校验程序。
- `field_data/`：首次有效室内实机静止采集及其审计，尚不属于自主巡航验收结果。
- `deliverables/final_review_20261117/`：结题展示图和讲解材料。
- `runtime/`：运行时临时目录；缓存会自动重建，不应写入论文成果包。

## 运行顺序

1. Windows 双击 `launch/four-aisle-slam.bat`。
2. Ubuntu 按窗口提示运行 `bash ~/as2_slam/start-one-path-live.sh`。
3. 需要四画面展示时，Ubuntu 另开终端运行 `bash ~/as2_slam/slam-screen/start-screen.sh`。
4. 任务完成后先在 Ubuntu 建图终端按 `Ctrl+C`，等待数据库和审计保存，再关闭仿真。

`launch/two-aisle-slam.bat` 是面向后续实机范围的 2、3 通道仿真入口；`launch/short-clip.bat` 仅用于短时录屏。当前 Windows 入口依赖 `E:\isaacsim`、项目位于 `E:\AS2_Greenhouse`，ROS 2 通信使用 Fast DDS、Domain ID 0。

## 已固化结果

两次保留运行均完成四条垄沟、返回起点并通过全局回环几何审计：

- 运行一：188.98 m；运动定位均值 2.005 cm、P95 4.707 cm；真值返航 0.496 cm、0.341°。
- 运行二：187.80 m；运动定位均值 2.027 cm、P95 4.545 cm；真值返航 0.357 cm、0.480°。

这说明当前固定场景和已知起点条件下结果可重复两次；它不等同于跨场景鲁棒性证明。论文应明确：局部巡航以激光为主，RGB 用于 RTAB-Map 视觉词袋/全局地点识别和植被投影；仿真真值只用于评价以及仍未完全替换的边界、姿态和无进展安全保护。

## 发布前校验

在项目根目录执行：

```powershell
E:\Anaconda\python.exe -B baselines\four-aisle-current\verify_release.py
```

输出 `RELEASE_VERIFY_OK` 后再压缩整个 `AS2_Greenhouse` 文件夹。不要把新的失败运行、`runtime` 缓存或 Ubuntu 临时日志加入成果 ZIP。


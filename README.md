# xgc2-slam

Public XGC-Team aggregator for ROS 1 Noetic SLAM packages.

This repository actively maintains one algorithm: Faster-LIO. Debug and CI
for that package live in [`XGC-Team/xgc2-faster-lio`](https://github.com/XGC-Team/xgc2-faster-lio).
Vehicle-specific work uses a `robot-sensor` branch on that fork
(for example `scout-helios16`). The release workflow here only builds
already-published Debian artifacts for the maintained set.

## Maintained

| Path | Repository | Debian package |
| --- | --- | --- |
| `faster_lio` | [xgc2-faster-lio](https://github.com/XGC-Team/xgc2-faster-lio) | `ros-noetic-xgc2-faster-lio` |

Clone recursively:

```bash
git clone --recurse-submodules git@github.com:XGC-Team/xgc2-slam.git
```

## Not maintained here

The trees under `temp/` are public upstream forks. They are parked for
reference and are not part of the current Debian set or aggregator CI.

| Path | Repository | Upstream |
| --- | --- | --- |
| `temp/fast_lio` | [xgc2-fast-lio](https://github.com/XGC-Team/xgc2-fast-lio) | hku-mars/FAST_LIO |
| `temp/lio_sam` | [xgc2-lio-sam](https://github.com/XGC-Team/xgc2-lio-sam) | TixiaoShan/LIO-SAM |
| `temp/point_lio` | [xgc2-point-lio](https://github.com/XGC-Team/xgc2-point-lio) | hku-mars/Point-LIO |
| `temp/swarm_lio2` | [xgc2-swarm-lio2](https://github.com/XGC-Team/xgc2-swarm-lio2) | hku-mars/Swarm-LIO2 |
| `temp/voxel_slam` | [xgc2-voxel-slam](https://github.com/XGC-Team/xgc2-voxel-slam) | hku-mars/Voxel-SLAM |

## Install

```bash
sudo apt update
sudo apt install ros-noetic-xgc2-slam
```

The `ros-noetic-xgc2-slam` meta package currently pulls Faster-LIO.

## Smoke Test

```bash
source /opt/ros/noetic/setup.bash
rospack find faster_lio
```

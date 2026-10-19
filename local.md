# SegFormer + 3D LiDAR + IMU 短时感知与融合 — 实施文档 (Markdown)

版本：1.0  
目标平台：轮式机器人，最高 2 m/s，Jetson Orin（200 TOPS），感知频率 10 Hz。  
目标：基于相机语义（SegFormer） + 3D LiDAR（点云几何）+ IMU（运动补偿），在局部范围内实时生成 traversability costmap（分辨率 0.05 m，前向 12 m × ±3 m），供本地规划器使用。

概述  
使用 SegFormer 做图像语义分割，LiDAR 提取几何特征并结合 IMU 做 motion compensation，把语义投影到 2.5D 高程栅格，按规则或轻量学习模型融合生成每帧 traversability costmap。

---

## 目录
1. 总体架构与数据流  
2. 硬件与安装建议  
3. 标定与时间同步  
4. ROS2 接口（Topic / Frame / Message）  
5. 点云预处理与 IMU 去畸变（motion compensation）  
6. SegFormer语义分割
7. 点云 ↔ 像素 投影（实现细节与置信度策略）  
8. 融合策略与代价函数（规则融合 + 学习融合可选）  
9. 输出、Planner 接口与安全策略  
10. 性能目标与优化建议（针对 Jetson Orin）  
11. 测试计划、评估指标与数据采集建议  
12. 分阶段工程里程碑与交付清单  
13. 常见问题与排查建议  
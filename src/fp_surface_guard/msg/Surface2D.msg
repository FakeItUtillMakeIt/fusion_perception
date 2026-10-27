# 一个经几何验证的语义表面候选（GlassGuard: 视觉提议, 几何验证, 覆盖写入）
std_msgs/Header header            # frame_id: 多边形所在坐标系(建议 base_link), stamp: 观测时刻
string class_name                 # glass | water | stair_down ...
float32 confidence                # 0..1
geometry_msgs/Polygon polygon     # 2D 闭合区域(多边形在 header.frame_id 下)
float32 min_height                # 表面下沿(离地 m), 参考信息
float32 max_height                # 表面上沿(离地 m), 参考信息

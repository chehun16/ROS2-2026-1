std_msgs/Header header
string class_name
float32 confidence

# 픽셀 좌표 (카메라 이미지 기준)
geometry_msgs/Point position

# LiDAR-Camera Fusion 결과
geometry_msgs/Point map_position   # map 프레임 기준 3D 위치
float32 distance                   # 로봇으로부터의 거리 (m)
bool position_valid                # fusion 성공 여부

float32 bbox_x
float32 bbox_y
float32 bbox_w
float32 bbox_h

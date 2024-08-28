# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import PointCloud2, PointField
import struct

def callback(data):
    # 确认字段索引
    x_index = y_index = z_index = -1
    for i, field in enumerate(data.fields):
        if field.name == 'x':
            x_index = i
        elif field.name == 'y':
            y_index = i
        elif field.name == 'z':
            z_index = i

    # 检查是否找到了x, y, z字段
    if x_index == -1 or y_index == -1 or z_index == -1:
        rospy.loginfo("Point cloud data does not have x, y, and z coordinates.")
        return

    # 读取每个点的位置信息
    points = []
    fmt = 'fff' if data.is_bigendian else '<fff'
    point_step = data.point_step
    for i in range(data.width * data.height):
        offset = i * point_step
        (x, y, z) = struct.unpack_from(fmt, data.data, offset)
        points.append((x, y, z))

    rospy.loginfo(f"Extracted {len(points)} points.")
    # 处理points列表，例如打印或存储

def listener():
    rospy.init_node('point_cloud_listener', anonymous=True)
    rospy.Subscriber("/table_top_points_subsampled", PointCloud2, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

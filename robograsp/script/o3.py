#!/usr/bin/env python
import rospy
import open3d as o3d
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg
import numpy as np

def ros_cloud_to_o3d(ros_cloud):
    # 将ROS PointCloud2转换为Open3D PointCloud
    cloud_points = list(pc2.read_points(ros_cloud, field_names=("x", "y", "z"), skip_nans=True))
    o3d_cloud = o3d.geometry.PointCloud()
    o3d_cloud.points = o3d.utility.Vector3dVector(cloud_points)
    return o3d_cloud

def o3d_cloud_to_ros(o3d_cloud, frame_id="arm_h"):
    # 将Open3D PointCloud转换为ROS PointCloud2
    ros_cloud = PointCloud2()
    ros_cloud.header.stamp = rospy.Time.now()
    ros_cloud.header.frame_id = frame_id

    # 从Open3D PointCloud提取点
    points = np.asarray(o3d_cloud.points)
    ros_cloud = pc2.create_cloud_xyz32(ros_cloud.header, points)
    return ros_cloud

def cloud_callback(ros_cloud):
    # 将ROS点云转换为Open3D点云
    o3d_cloud = ros_cloud_to_o3d(ros_cloud)
    # 打印原始点云的点数
    # print(f"原始点云中的点数: {len(o3d_cloud.points)}")

    # 体素下采样
    o3d_cloud_down = o3d_cloud.voxel_down_sample(voxel_size=0.004)

    # 打印下采样后点云的点数
    # print(f"下采样后的点云中的点数: {len(o3d_cloud_down.points)}")



    # 估计点云法线
    o3d_cloud_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    # 定义一个函数来检查法线是否指向X或Y轴（竖直面）
    def is_vertical(normal, vertical_threshold=0.8):
        # 检查法线与X轴或Y轴的点积
        return abs(np.dot(normal, [1, 0, 0])) > vertical_threshold or abs(
            np.dot(normal, [0, 1, 0])) > vertical_threshold

    # 筛选出非竖直面
    non_vertical_indices = [i for i, normal in enumerate(np.asarray(o3d_cloud_down.normals)) if not is_vertical(normal)]
    non_vertical_pcd = o3d_cloud_down.select_by_index(non_vertical_indices)

    # print(f"Number of points in the downsampled point cloud: {len(non_vertical_pcd.points)}")

    # 将处理后的点云转换回ROS点云
    ros_cloud_down = o3d_cloud_to_ros(non_vertical_pcd, frame_id=ros_cloud.header.frame_id)

    # 发布下采样后的点云
    pub.publish(ros_cloud_down)
    # print("发布下采样后的点云")

if __name__ == '__main__':
    rospy.init_node('point_cloud_processor', anonymous=True)
    pub = rospy.Publisher('/downsampled_points', PointCloud2, queue_size=1)
    rospy.Subscriber('/table_top_points', PointCloud2, cloud_callback, queue_size=1)
    rospy.spin()  # 保持节点运行，等待点云消息

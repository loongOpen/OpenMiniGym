#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from sklearn.cluster import DBSCAN
from sklearn.decomposition import PCA
import numpy as np
import tf
import tf2_ros
import geometry_msgs.msg

def calculate_orientation(points):
    pca = PCA(n_components=3)
    pca.fit(points)
    axes = pca.components_
    # extents = np.max(points, axis=0) - np.min(points, axis=0)


    x_axis = np.array([0, 0, -1])
    # 计算每个轴的范围（extent）
    extents = [np.ptp(points.dot(axis)) for axis in axes]

    # 假设x_axis_index是与-z轴最接近的轴的索引
    x_axis_index = np.argmin([axis[2] for axis in axes])

    # 移除x轴对应的范围值
    remaining_extents = np.delete(extents, x_axis_index)
    remaining_axes = np.delete(axes, x_axis_index, axis=0)

    # 找到剩余轴中范围最短的轴的索引
    y_axis_index = np.argmin(remaining_extents)

    # 选择y轴
    y_axis = remaining_axes[y_axis_index]

    
    # 确保y轴在世界坐标系下的y分量是正的
    # print(y_axis[2])
    if y_axis[1] < 0:
        y_axis = -y_axis
    
    # 将y轴投影到xy平面（即设置z分量为0）
    y_axis_projected = np.array([y_axis[0], y_axis[1], 0])
    if y_axis_projected[1] < 0:
        y_axis_projected = -y_axis_projected

    # 将投影的向量转换为单位向量
    y_axis_unit = y_axis_projected / np.linalg.norm(y_axis_projected)

    # 计算z轴以保持右手规则
    z_axis = np.cross(x_axis, y_axis_unit)

    return np.array([x_axis, y_axis_unit, z_axis]).T

def callback(data):
    points_list = []
    for point in pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True):
        points_list.append([point[0], point[1], point[2]])

    points_array = np.array(points_list)
    print(len(points_array))

    if len(points_array) > 200 and len(points_array) <4000:
        clustering = DBSCAN(eps=0.05, min_samples=100).fit(points_array)
        br = tf2_ros.TransformBroadcaster()

        # 存储标签和对应中心点的信息
        clusters = []
        # 打印聚类后的簇的个数（去除噪声点后）
        # print("Number of clusters:", len(unique_labels) - (1 if -1 in unique_labels else 0))
        unique_labels = set(clustering.labels_)
        for label in unique_labels:
            if label != -1:
                points_in_cluster = points_array[clustering.labels_ == label]
                center = points_in_cluster.mean(axis=0)
                distance = np.linalg.norm(center)  # 计算到原点的距离
                # 打印每个簇的点云数量
                # print("Cluster {} has {} points.".format(label, len(points_in_cluster)))

                clusters.append((label, center, distance))
                print(center)

        # 按照距离排序
        clusters.sort(key=lambda x: x[2])

        # 发送排序后的变换
        for label, center, distance in clusters:
            points_in_cluster = points_array[clustering.labels_ == label]
            orientation_matrix = calculate_orientation(points_in_cluster)
            quaternion = tf.transformations.quaternion_from_matrix(
                np.vstack([np.hstack([orientation_matrix, np.zeros((3, 1))]), [0, 0, 0, 1]])
            )

            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "arm_h"
            t.child_frame_id = "cluster_{}".format(label)
            t.transform.translation.x = center[0]
            t.transform.translation.y = center[1]
            t.transform.translation.z = center[2]
            t.transform.rotation.x = quaternion[0]
            t.transform.rotation.y = quaternion[1]
            t.transform.rotation.z = quaternion[2]
            t.transform.rotation.w = quaternion[3]
            br.sendTransform(t)
        

def listener():
    rospy.init_node('point_cloud_clustering', anonymous=True)
    rospy.Subscriber("/table_top_points_subsampled", PointCloud2, callback, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    listener()

#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from sklearn.cluster import DBSCAN
import numpy as np
import tf
import tf2_ros
import geometry_msgs.msg

def callback(data):
    # 解析点云数据
    points_list = []
    for point in pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True):
        points_list.append([point[0], point[1], point[2]])
    # 将列表转换为NumPy数组以进行聚类
    points_array = np.array(points_list)

    if len(points_array) > 0:
        print(len(points_array))
        # 使用DBSCAN算法进行聚类
        clustering = DBSCAN(eps=0.05, min_samples=100).fit(points_array)

        # 初始化tf广播器
        br = tf2_ros.TransformBroadcaster()

        # 获取每个簇的点数
        unique_labels = set(clustering.labels_)
        for label in unique_labels:
            if label != -1:  # 不包括噪声点
                points_in_cluster = points_array[clustering.labels_ == label]
                if len(points_in_cluster) > 2000:  # 忽略大于4000个点的簇
                    rospy.loginfo("Cluster {} ignored due to size > 2000".format(label))
                    continue  # 跳过这个簇的后续处理
                
                center = points_in_cluster.mean(axis=0)
                # rospy.loginfo("Center of cluster {}: {}".format(label, center))
                print(center)

                # 发布TF变换
                t = geometry_msgs.msg.TransformStamped()
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = "arm_h"
                t.child_frame_id = "cluster_{}".format(label)
                t.transform.translation.x = center[0]
                t.transform.translation.y = center[1]
                t.transform.translation.z = center[2]
                t.transform.rotation.x = 0
                t.transform.rotation.y = 0.707
                t.transform.rotation.z = 0
                t.transform.rotation.w = 0.707
                br.sendTransform(t)

def listener():
    rospy.init_node('means', anonymous=True)
    rospy.Subscriber("/table_top_points_subsampled", PointCloud2, callback, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    listener()

import os
os.environ["OMP_NUM_THREADS"] = '1'
from sklearn.cluster import KMeans
import numpy as np




def k_means(data: object, n_clusters: object = 2, ) -> object:
    # 定义聚类器, 设置迭代次数和终止条件
    kmeans = KMeans(n_clusters=2, n_init='auto', max_iter=1000, tol=1e-4)
    kmeans.fit(data)
    # 获取每个数据点的簇标签
    labels = kmeans.labels_
    # 获取每类数据的中心点
    cluster_centers = kmeans.cluster_centers_

    return labels, cluster_centers

if __name__ == "__main__":
    #test
    data = np.array([[1,1,1],[2,2,2],[1,1,1]])
    labels, centers = k_means(data,2)
    print(labels, centers)


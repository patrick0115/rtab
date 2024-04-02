import pcl
from pcl import pcl_visualization
import open3d as o3d
import pcl
import numpy as np
def extract_ground_from_pcd(pcl_cloud, distance_threshold=0.5):
    # 地面偵測
    seg = pcl_cloud.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)  # 這裡改正
    seg.set_method_type(pcl.SAC_RANSAC)  # 以及這裡
    seg.set_distance_threshold(distance_threshold)
    indices, model = seg.segment()
    ground = pcl_cloud.extract(indices, negative=False)

    # 可視化地面點雲
    visual = pcl_visualization.CloudViewing()
    visual.ShowMonochromeCloud(ground)
    v = True
    while v:
        v = not(visual.WasStopped())

    return ground



def filter_ground_plane(pcd, normal_search_radius, angle_threshold):

    # 法向量估計
    ne = pcd.make_NormalEstimation()
    tree = pcd.make_kdtree()
    ne.set_SearchMethod(tree)
    ne.set_RadiusSearch(normal_search_radius)
    normals = ne.compute()

    # 過濾點雲
    filtered_indices = []
    for i in range(normals.size):
        # 法向量
        normal = normals[i]
        # 計算法向量與Z軸的夾角
        angle = np.arccos(normal[2]) # 法向量的Z分量
        # 比較夾角與閾值
        if angle <= angle_threshold:
            filtered_indices.append(i)

    # 提取過濾後的點雲
    filtered_pcd = pcd.extract(filtered_indices, negative=False)
        # 可視化過濾後的點雲 (可選)
    visual = pcl.pcl_visualization.CloudViewing()
    visual.ShowMonochromeCloud(filtered_pcd)
    v = True
    while v:
        v = not(visual.WasStopped())
    
    return filtered_pcd


path="/home/lab606/rtab/src/rtab/map/b1/b1.pcd"
pcd = pcl.load(path)
# 過濾點雲
filtered_pcd = filter_ground_plane(pcd, normal_search_radius = 1 , angle_threshold = 0.1)


check_pcd_contents(path)
ground_pcd = extract_ground_from_pcd(filtered_pcd)

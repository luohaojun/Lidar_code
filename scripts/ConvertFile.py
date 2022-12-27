import numpy as np
import open3d as o3d
np.set_printoptions(suppress=True) # 取消默认的科学计数法
Data1 = np.loadtxt('/home/luo/Group_UAV/Lidar/dataset/sydney-urban-objects-dataset/objects/building.7.17589.csv',dtype=float,skiprows=0,
                   delimiter=',',usecols=(3,4,5),unpack=False)

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(Data1)
print(np.asarray(pcd.points))
o3d.visualization.draw_geometries([pcd])
o3d.io.write_point_cloud('/home/luo/Group_UAV/Lidar_code/building.7.17589.ply',pcd)
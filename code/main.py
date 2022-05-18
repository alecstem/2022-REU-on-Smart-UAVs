from laspy.file import File
import open3d as o3d
import numpy as np

las = File("points.las", mode="r")
point_data = np.stack([las.X, las.Y, las.Z]).transpose()
print(point_data)
geom = o3d.geometry.PointCloud()
geom.points = o3d.utility.Vector3dVector(point_data)
o3d.visualization.draw_geometries([geom])

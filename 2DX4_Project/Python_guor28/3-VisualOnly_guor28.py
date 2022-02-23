#Ruiyan Guo
#400256752
#guor28
#Python3.8.8
#
import numpy as np
import open3d as o3d

#Plot the points in 3D
pcd = o3d.io.read_point_cloud("Demo_cloud.xyz", format='xyz')
print(pcd)
print(np.asarray(pcd.points))
#o3d.visualization.draw_geometries([pcd])
#Connect the lines in 3D
po=0
lines = []
for x in range(10): #connect vertices within planes
    for pt in range(8):
        lines.append([pt+po,pt+1+po])
    po+=8  # 8 rotations per x
po=0
do=8  # index of same point on the next plane will be 8 indices from present point
for x in range(9): #connect vertices between planes
    for pt in range(8):
        lines.append([pt+po,pt+do+po])
    po+=8
line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)), lines=o3d.utility.Vector2iVector(lines))
o3d.visualization.draw_geometries([line_set])


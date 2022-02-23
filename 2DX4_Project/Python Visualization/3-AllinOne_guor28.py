#Ruiyan Guo
#400256752
#guor28
#Python3.8.8
#
import math
import serial
import numpy as np
import open3d as o3d
#Transform raw sensor data into xyz format
f = open("Demo_cloud.xyz", "w")
s = serial.Serial("COM3", 115200)
print('start\n')
arr = []
for m in range(10):   #Append the measured data into an array
    for n in range(8):
        str=""
        while True:
            x=s.read()
            c=x.decode()
            if c=='A':
                break
            str=str+c
        print(str+'\n')
        arr.append(str)
s.close()
print('Measuremrnt Finished\n')
print('Array generation Finished\n')
x=0
angle=-45
counter=1
#break the measurements into x,y,z by using sin/cos functions
#and write the x,y,z points into the xyz file
for i in arr:
    if ((counter==9) or (counter==17) or (counter==25) or (counter==33) or (counter==41) or (counter==49) or (counter==57) or (counter==65) or (counter==73)):
        angle=-45
        x+=0.5
    angle+=45
    temp=float(i)
    y=(temp/1000)*(math.sin(math.pi*angle/180))
    z=(temp/1000)*(math.cos(math.pi*angle/180))
    counter+=1
    f.write("{} {} {}\n".format(x, y, z))    
f.close()
print('xyz file generation finished\n')
print('visuaing\n')
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
    po+=8  # 8   rotations per x
po=0
do=8  # index of same point on the next plane will be 8 indices from present point
for x in range(9): #connect vertices between planes
    for pt in range(8):
        lines.append([pt+po,pt+do+po])
    po+=8
line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)), lines=o3d.utility.Vector2iVector(lines))
o3d.visualization.draw_geometries([line_set])

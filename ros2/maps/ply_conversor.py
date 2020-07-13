from plyfile import PlyData, PlyElement

plydata = PlyData.read('worldMap22.ply')

f = open("xyz_rotated.txt", "w")

array  = []

for i in plydata.elements[0].data:
    array.append(i[0])
    array.append(i[1])
    array.append(i[2])
    print('{}, {}, {}\n'.format(i[0], i[1], i[2]))  
    st = ('{} {} {}'.format(i[0], i[1], i[2]) + '\n')
    f.write(st) 

f.close() 


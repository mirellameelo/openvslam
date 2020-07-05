# this code convert the xyz.txt file with 3d points for worldMap0.ply file 

import numpy as np
from plyfile import PlyData, PlyElement
import os
import re

def get_paths(path, formato):
    files_r = []
    for r, d, f in os.walk(path):
        for file_tmp in f:
            if formato in file_tmp:
                files_r.append(file_tmp)
    
    return sorted_nicely(files_r)

def sorted_nicely( l ):
    convert = lambda text: int(text) if text.isdigit() else text
    alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)', key)]
    return sorted(l, key = alphanum_key)


def main():
    path = os.getcwd() 
    files = "xyz.txt"
    array = []
    count = 0
    countMaps = 0
    #import ipdb; ipdb.set_trace()

    if count % 3 == 0:
        lines = open(files)
        for line in lines.readlines():
            values = line.split(' ')
            array.append((values[0], values[1], values[2]))
        vertex = np.array(array, dtype=[('x', 'f4'), ('y', 'f4'),
                        ('z', 'f4')] )
        el = PlyElement.describe(vertex, 'vertex')
        PlyData([el]).write('worldMap'+ str(countMaps)+'.ply')
        array.clear()
        vertex = []
        el = []
        countMaps = countMaps + 1
    count = count + 1



    print("Waintg")

if __name__== "__main__":
    
    main()

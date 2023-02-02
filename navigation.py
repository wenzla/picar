import numpy as np
import math

def check_way(p1,p2):
    dec0=False
    dec1=False
    inc0=False
    inc1=False
    
    if p1[0] < p2[0]:
        inc0=True
    elif p1[0] > p2[0]:
        dec0=True

    if p1[1] < p2[1]:
        inc1=True
    elif p1[1] > p2[1]:
        dec1=True
    return inc0, dec0, inc1, dec1

def incremental_step(newp, p2, inc0, dec0, inc1, dec1):
    """creates the manhattan path from point1 to point2 incrementally"""
    add0 = 0
    add1 = 0
    if dec0:
        add0 = -1
    if dec1:
        add1 = -1
    if inc0:
        add0 = 1
    if inc1:
        add1 = 1
        
    if math.dist([newp[0]+add0, newp[1]], p2) > math.dist([newp[0], newp[1]+add1], p2):
        #if adding to x coor results greater distance, then we add to y.
        newp[1] += add1
    else:
        newp[0] += add0
        
    return newp

def get_manhattan(p1, p2):
    if (math.dist(p1,p2)) < 4:
        inc0, dec0, inc1, dec1 = check_way(p1,p2)
        
        newp = p1
        path_list = []
        times = 0
        
        while (math.dist(newp, p2) > 0) and (times < 6):
            newp = incremental_step(newp, p2, inc0, dec0, inc1, dec1)
            path_list.append(newp.copy())
            inc0, dec0, inc1, dec1 = check_way(p1,p2)
            times += 1
            
        if times == 7:
            return None
        else:
            return path_list[:-1]#remove last element
        
    return None

def make_scan_list(possible):
    
    scan_list = []
    temp = [list(p) for p in possible if p[0] != -1]
    
    for i in range(len(temp)-1):
        scan_list.append(temp[i])
        
        for path in get_manhattan(temp[i], temp[i+1]):
            scan_list.append(path)
        
    scan_list.append(temp[-1])
    
    return scan_list

def make_safety_zone(x,y,mapping):

    mapping[x-1][y-1] = 1
    mapping[x-1][y] = 1
    mapping[x-1][y+1] = 1
    
    mapping[x][y-1] = 1
    mapping[x][y] = 1
    mapping[x][y+1] = 1
    
    mapping[x+1][y-1] = 1
    mapping[x+1][y] = 1
    mapping[x+1][y+1] = 1
    
    return mapping


# !USAGE BELOW
# test = [26.9,26.93,27.26, 26.88, 27.72, 29.26, 42.37, 41.93, 42.47, 41.78, 42.67, 43.66, -2]
# angles = np.deg2rad(np.arange(-60,70,10)) #in radians
# xvals = (np.round((np.sin(angles) * test)/5, 0)+14).astype(int)
# yvals = ((np.cos(angles) * test)//5).astype(int)
# scan_list = make_scan_list(list(zip(yvals, xvals))) 
# for pair in scan_list:
#     if pair[0] != -1:
# #         print(pair[0],pair[1])
#         mapping[pair[0]][pair[1]] = 1
#         mapping = make_safety_zone(pair[0],pair[1],mapping)
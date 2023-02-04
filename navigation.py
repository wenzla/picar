import numpy as np
import math
from queue import PriorityQueue
from functools import partial
from car import left_turn, right_turn, forward_step, backward_step


#################### MAPPING FUNCTIONS ####################

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
        
        manhat = get_manhattan(temp[i], temp[i+1])
        
        if manhat is not None:
            for path in manhat:
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

# *USAGE BELOW
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


#################### ROUTING FUNCTIONS ####################

def _get_north(point):
    x, y = point
    return (x-1,y)

def _get_south(point):
    x, y = point
    return (x+1,y)

def _get_east(point):
    x, y = point
    return (x,y+1)

def _get_west(point):
    x, y = point
    return (x,y-1)

def _get_city_neighbor(point, xmax, ymax):
    x, y = point
    neighbors = []
    
    if x == 0:
        neighbors.append(_get_south(point))
    elif x == (xmax-1):
        neighbors.append(_get_north(point))
    else:
        neighbors.append(_get_south(point))
        neighbors.append(_get_north(point))
    
    if y == 0:
        neighbors.append(_get_east(point))
    elif y == (ymax-1):
        neighbors.append(_get_west(point))
    else:
        neighbors.append(_get_east(point))
        neighbors.append(_get_west(point))
        
    return neighbors
    
def get_neighbors(point, mapping):

    xmax, ymax = mapping.shape
    neighbors = _get_city_neighbor(point, xmax, ymax)
    
    weights = []
    for n in (neighbors):
        map_weight = mapping[n[0]][n[1]]
        if map_weight == 0:
            weights.append(1)
        else: #obstacle
            weights.append(999) 
    
    return neighbors, weights

def heuristic(start, goal): 
    x1, y1 = start
    x2, y2 = goal
    return abs(x1 - x2) + abs(y1 - y2)

# code below based off of https://www.redblobgames.com/pathfinding/a-star/implementation.html#python-astar
def a_star(start, goal, mapping):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    path = {}
    running_cost = {}
    
    path[start] = None
    running_cost[start] = 0
    
    while frontier.qsize() > 0:
        current = frontier.get()
        
        if current == goal:
            break
        
        neighbors, costs = get_neighbors(current, mapping)
        
        for i, nxt in enumerate(neighbors):
            cost = costs[i]
            new_cost = running_cost[current] + cost
            if (nxt not in path) or (new_cost < running_cost[nxt]):
                running_cost[nxt] = new_cost
                priority = new_cost + heuristic(nxt, goal)
                frontier.put(nxt, priority)
                path[nxt] = current  
    
    return path, running_cost    

def make_route(path, start, goal, route=[]):
    prev = path[goal]
    route.append(prev)
    
    if prev == start:
        return route
    else:    
        route = make_route(path, start, prev, route)
        return route

# *USAGE BELOW
# start = (0,14)
# goal = (20,14)
# path, cost_so_far = a_star(start, goal, mapping)
# route = make_route(path, start, goal)
# route.reverse()
# route.append(goal)

#################### INSTRUCTION FUNCTIONS ####################

def generate_instructions(route, start):
    lastx, lasty = start
    direction = 's'

    instructions = []
    for r in route[1:]:
        rx, ry = r
        if lastx == rx: #moving east-west
            if direction == 's':
                if lasty < ry:
                    instructions.append('right')
                    instructions.append('forward')
                    direction = 'w'
                else:
                    instructions.append('left')
                    instructions.append('forward')
                    direction = 'e'
            elif direction == 'n':
                if lasty < ry:
                    instructions.append('left')
                    instructions.append('forward')
                    direction = 'w'
                else:
                    instructions.append('right')
                    instructions.append('forward')
                    direction = 'e'
            elif direction == 'e':
                if lasty < ry:
                    instructions.append('reverse')
                    direction = 'e'
                else:
                    instructions.append('forward')
                    direction = 'e'
            else: #direction == 'w'
                if lasty < ry:
                    instructions.append('forward')
                    direction = 'w'
                else:
                    instructions.append('reverse')
                    direction = 'w'
        else: #lasty == ry; moving north-south
            if direction == 's':
                if lastx < rx:
                    instructions.append('forward')
                    direction = 's'
                else:
                    instructions.append('reverse')
                    direction = 's'
            elif direction == 'n':
                if lastx < rx:
                    instructions.append('reverse')
                    direction = 'n'
                else:
                    instructions.append('forward')
                    direction = 'n'
            elif direction == 'e':
                if lastx < rx:
                    instructions.append('right')
                    instructions.append('forward')
                    direction = 's'
                else:
                    instructions.append('left')
                    instructions.append('forward')
                    direction = 'n'
            else: #direction == 'w'
                if lastx < rx:
                    instructions.append('left')
                    instructions.append('forward')
                    direction = 's'
                else:
                    instructions.append('right')
                    instructions.append('forward')
                    direction = 'n'
    #     print(rx, ry, direction)

        lastx, lasty = rx, ry
    return instructions

def generate_movements(instructions):
    instruction_list = []
    last_inst = None
    dummy=None
    stepper = 1

    for inst in instructions:
        if inst == 'left':
            if last_inst == 'forward':
                instruction_list.append(partial(forward_step, stepper))
                instruction_list.append(partial(left_turn, dummy))
                stepper = 1
            else:
                instruction_list.append(partial(left_turn, dummy))
        elif inst == 'right':
            if last_inst == 'forward':
                instruction_list.append(partial(forward_step, stepper))
                instruction_list.append(partial(right_turn, dummy))
                stepper = 1
            else:
                instruction_list.append(partial(right_turn, dummy))
        elif inst == 'forward':
            if last_inst == 'forward':
                stepper += 1           
        else: #reverse should never happen with my current obstacle course
            None
        last_inst = inst

    if last_inst == 'forward':
        instruction_list.append(partial(forward_step, stepper))  
    return instruction_list
    
# *USAGE BELOW
# instructions = generate_instructions(route, start)
# generate_movements(instructions)
# for il in instruction_list:
#     il()
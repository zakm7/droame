# we make use of heap queue algo
import heapq

# base class
class Drone:
    def __init__(self, start, end, s_time):
        self.start = start
        self.end = end
        self.s_time = s_time

# get position logic
def get_adjacent_positions(position, mv_type):
    x, y = position
    
    if mv_type == 8:
        return [(x+i,y+j) for i in range(-1, 2) for j in range(-1, 2) if i!=0 or j!=0];
    
    elif mv_type == 4:
        return [(x+1, y),(x-1, y),(x, y+1),(x, y-1)];

#h'ristic function
def heuristic(position, end):
    return abs((position[1]-end[1])+(position[0]-end[0]))

# logic
def a_star(start, end, grid, mv_type, collision_penal):
    
    #init
    queue = [(0, start)]
    came_from = {}
    cost_so_far = {start: 0}
    
    # while in heap queue
    while queue:
        _, current = heapq.heappop(queue)

        if current == end:
            break

        for next_pos in get_adjacent_positions(current, mv_type):
            
            if not (0 <= next_pos[0] < len(grid) and 0 <= next_pos[1] < len(grid[0])):
                continue
            
            if grid[next_pos[0]][next_pos[1]]!=None and grid[next_pos[0]][next_pos[1]] <= cost_so_far[current]:
                cost = cost_so_far[current] + collision_penal
                
            else:
                cost = cost_so_far[current] + 1
                
            if next_pos not in cost_so_far or cost < cost_so_far[next_pos] :
                cost_so_far[next_pos] = cost
                priority = cost + heuristic(next_pos, end)
                heapq.heappush(queue, (priority, next_pos))
                came_from[next_pos] = current

        grid[current[0]][current[1]] = cost_so_far[current]
    
    #result
    path = [end]
    current = end
    
    while current != start:
        current = came_from[current]
        path.append(current)
    
    path.reverse()
    return path


def get_paths(drones, grid_size, mv_type=8, collision_penal=100):
    grid = [[None for _ in range(grid_size[1])] for _ in range(grid_size[0])]
    
    #init drones
    active_drones = []
    
    for drone in drones:
        active_drones.append((drone.s_time, drone))
    active_drones.sort()

    for time_step, drone in active_drones:
        path = a_star(drone.start, drone.end, grid, mv_type, collision_penal)
        for pos in path:
            grid[pos[0]][pos[1]] = time_step

        drone.path = path

    return [drone.path for _, drone in active_drones]

drones = [Drone((1, 1), (3, 5), 0), Drone((5, 5), (3, 6), 3)]
grid_size = (15, 15)
paths = get_paths(drones, grid_size)
print(paths)

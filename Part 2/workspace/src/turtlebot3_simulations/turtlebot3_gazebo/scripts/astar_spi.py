from collections import deque
import numpy as np
import time
import matplotlib.path as mplPath
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
plt.rcParams["figure.figsize"] = [7.00, 3.50]
plt.rcParams["figure.autolayout"] = True

def lin_comb(arr1:np.array, arr2:np.array):
    lin_comb = np.array(np.meshgrid(arr1,arr2)).T.reshape(-1,2)
    return lin_comb

class Node(object):

    parent = None
    state = None
    idx = None
    action_set = [(1,0), (-1,0), (0,1), (0,-1), (1,1), (-1,1), (1,-1), (-1,-1)]
    c2c = None
    map_shape = (250,600)

    def __init__(self, parent, state:tuple, goal:tuple, idx:int, c2c = 0):
        self.parent = parent
        self.state = state
        self.idx = idx
        self.c2c = c2c

    def __repr__(self):
        return f"State = {self.state} | C2C = {self.c2c}"

    def actions(self, obst_map:set, goal:tuple):
        nodes = []
        
        for a in self.action_set:
            y = self.state[0] + a[0]
            x = self.state[1] + a[1]
            state = (y, x)

            if y < 0 or y > 250 or x < 0 or x > 600:
                nodes.append(None)
            
            elif state in obst_map:
                nodes.append(None)
            
            else:
                parent = self
                idx = self.idx + 1
                new_c2c = self.c2c + np.linalg.norm(np.array(a))
                nodes.append(Node(parent, state=state, goal=goal, idx=idx, c2c=new_c2c))
        
        return nodes
    
class Map:

    #obstacle map attributes
    map = np.ones((251,601))
    obst_space_clr = set()
    obst_space = set()
    x = np.arange(0,601,1)
    y = np.arange(0,251,1)
    pts = lin_comb(y,x)

    #tree attributes
    tree = []
    goal_node = None
    initial = None
    goal = None
    explored_path = []
    visited = set()
    tree = {}
    flag = False

    #class methods
    def init_map(self, initial:tuple, goal:tuple) :
        self.goal = goal
        self.initial = initial

    def get_obstacle_space(self):

        a1 = [0, 100]
        a2 = [0, 150]
        a3 = [100, 150]
        a4 = [100, 100]

        b1 = [150, 100]
        b2 = [150, 150]
        b3 = [250, 150]
        b4 = [250, 100]

        rect1_path = mplPath.Path(np.array([a1,a2,a3,a4]))
        rect2_path = mplPath.Path(np.array([b1,b2,b3,b4]))

        # Hexagon
        h1 = [50, 300]
        h2 = [(50 + 75*np.sin(np.deg2rad(30))), (300 + 75*np.cos(np.deg2rad(30)))]
        h3 = [(50 + 75*np.sin(np.deg2rad(30)) + 75), (300 + 75*np.cos(np.deg2rad(30)))]
        h4 = [50 + 150, 300]
        h5 = [(50 + 75*np.sin(np.deg2rad(30)) + 75), (300 - 75*np.cos(np.deg2rad(30)))]
        h6 = [(50 + 75*np.sin(np.deg2rad(30))), (300 - 75*np.cos(np.deg2rad(30)))]

        hex_path = mplPath.Path(np.array([h1, h2, h3, h4, h5 ,h6]))

        # Triangle
        t1 = [25, 460]
        t2 = [125, 510]
        t3 = [225, 460]
        tri_path = mplPath.Path(np.array([t1, t2, t3]))

        # Boundaries
        y_r_1 = np.arange(0,5,1)
        y_r_2 = np.arange(246,251,1)
        x_r_1 = np.arange(0,5,1)
        x_r_2 = np.arange(596,601,1)

        uw = lin_comb(y_r_1, self.x)
        lw = lin_comb(self.y, x_r_1)
        bw = lin_comb(y_r_2, self.x)
        rw = lin_comb(self.y, x_r_2)

        walls = np.vstack((uw,lw,bw,rw))

        # Adding to obstacle space

        for coord in walls: #Adding bloated walls
            self.obst_space_clr.add((coord[0],coord[1]))

        for pt in self.pts: #Adding bloated obstacles
            for rad in [-0.01, -11]:
                bool1 = rect1_path.contains_point(pt, radius=rad)
                bool2 = rect2_path.contains_point(pt, radius=rad)
                bool3 = hex_path.contains_point(pt, radius=rad)
                bool4 = tri_path.contains_point(pt, radius=rad)

                if (bool1 or bool2 or bool3 or bool4) and (rad == -11) :
                    self.obst_space_clr.add((pt[0],pt[1]))
                
                elif(bool1 or bool2 or bool3 or bool4):
                    self.obst_space.add((pt[0],pt[1]))

    def populate_graph(self):

        root = Node(parent=None, state=self.initial, goal=self.goal, idx=0, c2c=0)

        self.tree[root.state] = root
        nodes = deque([root])

        if np.array_equal(root.state, np.array(self.goal)):
            print("\nSUCCESS WOOHOO!\n")
            self.goal_node = root
            return self.tree
        
        start_time = time.time()

        n = 0

        while(True):

            n+=1
            
            p_node = nodes.popleft()

            self.explored_path.append(p_node.state)
            del self.tree[p_node.state]
            self.visited.add(p_node.state)
            c_nodes = p_node.actions(self.obst_space_clr, goal=self.goal)

            for node in c_nodes:
                if node is None:
                    continue

                if node.state not in self.visited:
                    if node.state == self.goal:
                        self.goal_node = node
                        self.flag = True
                        print("\nSUCCESS WOOHOO!\n")
                        print("--- Time elapsed to find solution using Djikstra: %s seconds ---" % (time.time() - start_time))
                        break

                    if node.state not in self.tree.keys():
                        self.tree[node.state] = node
                    
                    elif node.c2c < self.tree[node.state].c2c:
                        self.tree[node.state] = node

                    else:
                        continue

            if self.flag:
                break
        
            nodes = deque(sorted(self.tree.values(), key=lambda x: x.c2c))

            if len(nodes) == 0:
                print("\nAlgorithm terminated : No solutions found!\n")
                return True, np.array(self.explored_path)
        
        return False, np.array(self.explored_path)
    
    def backtrack(self):
        node = self.goal_node
        path = [node.state]
        while True:
            node = node.parent
            if node is None:
                break
            path.append(node.state)
        
        return path[::-1]
    
def get_inputs(pt:str, map):
    
    # Getting inputs

    while(True): # start inputs
        x = float(input("\nPlease enter the x coordinate of the " + pt + " point (positive integer) : "))
        y = float(input("\nPlease enter the y coordinate of the " + pt + " point (positive integer) : "))

        bool1, bool2, bool3 = False, False, False

        if x - int(x) != 0 or y - int(y) != 0 or x<0 or y<0:
            bool1 = True

        x = int(x)
        y = 250 - int(y)

        if (y, x) in map.obst_space_clr:
            bool2 = True

        if y < 0 or y > 250 or x < 0 or x > 600:
            bool3 = True

        if not(bool1 or bool2 or bool3):
            print("\n" + pt + " coordinates successfully entered!\n")
            break

        else:
            print("\nInvalid " + pt + " coordinates! Please re-enter\n")

    return (y,x)

def main():

    # Map Creation

    print("\nCreating map with obstacles...\n")

    map = Map()
    map.get_obstacle_space()

    print("\nObstacle map successfully created!\n")

    # Getting inputs

    init = get_inputs("start", map)
    goal = get_inputs("goal", map)
    map.init_map(init, goal)

    # Running Djikstra

    print("\nExecuting Dijkstra to solve for path planning...\n")
    flag, exp_path = map.populate_graph()
    if not flag:
        path = np.array(map.backtrack())

    # Plotting

    obstacles_clr = np.array(list(map.obst_space_clr - map.obst_space))
    obstacles = np.array(list(map.obst_space))

    canvas = map.map.copy()
    canvas[obstacles_clr[:,0],obstacles_clr[:,1]] = 0.5
    canvas[obstacles[:,0],obstacles[:,1]] = 0

    fig, ax = plt.subplots()

    def update(i):
        im = canvas
        if i < exp_path.shape[0]//1000:
            ax.imshow(im, cmap='gray')
            ax.scatter(exp_path[i*1000:(i+1)*1000, 1], exp_path[i*1000:(i+1)*1000 ,0], c = "red", s = 1)
            plt.axis('off')

        else :
            j = i - exp_path.shape[0]//1000
            if j == 0:
                ax.scatter(exp_path[:, 1], exp_path[: ,0], c = "red", s = 1)
            ax.scatter(path[j*10 : (j+1)*10 , 1], path[ j*10 : (j+1)*10 , 0], c = "green", s=1)
            plt.axis('off')

    buffer = path.shape[0]%10
    if flag: anim = FuncAnimation(fig, update, frames= exp_path.shape[0]//1000 + buffer, interval=1, repeat = False)
    else: anim = FuncAnimation(fig, update, frames= exp_path.shape[0]//1000 + path.shape[0]//10 + buffer, interval=1, repeat = False)
    # anim.save('dijkstra.gif', writer = 'imagemagick', fps=60) #uncomment to save the animation
    plt.show()

    return

if __name__ == "__main__":
    main()
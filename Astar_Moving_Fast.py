import math
import matplotlib.pyplot as plt
import numpy as np
import timing
""" Tags indicate lines of code solely dedicated to the task which the tag belongs to"""
""" There are two types of grids. One is the Original Grid,denoted by primary grid, [the grid plt.plot() works on] and another one
is the Artificial Grid enclosed by boundaries which are presented as part of the obstacles. The artificial grid
(denoted by secondary grid) ranges from X[0-xwidth] and Y[0 to ywidth].
The values of xwidth and ywidth is simply determined by the position of secondary grid boundary
obstacles. The self.calc_primary_index method converts secondary grid indices into primary grid indices and the method
self.calc_secondary_index method transforms primary grid indices into secondary grid indices."""
show_animation = True
breakWall=False #Breaks wall at  specified location as part of updating obstacles after each iteration, if True. [Tag: BW]
wall_padding=1

class AStarPlanner:

    def __init__(self, ox, oy, reso, rr):
        """
        Initialize grid map for a star planning
        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        reso: grid resolution [m]
        rr: robot radius[m]
        """

        self.reso = reso
        self.rr = rr

        self.motion = self.get_motion_model()

    class Node:
        def __init__(self, x, y, cost, pind):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost #cost 
            self.pind = pind # parent index of each node. NOT the index of the node itself.

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)

    def planning(self, sx, sy, gx, gy,ox,oy):
        """
        A star path search
        input:
            sx: start x position [m]
            sy: start y position [m]
            gx: goal x position [m]
            gx: goal x position [m]
        output:
            rx: x position list of the final path
            ry: y position list of the final path
        
        """
        self.map_secondary_grid(ox, oy)
        self.obsmap(ox,oy)

        nstart = self.Node(self.calc_secondary_index(sx, self.minx),
                           self.calc_secondary_index(sy, self.miny), 0.0, -1) # the start node doesn't have any parent. So we set the parent index to be -1
        ngoal = self.Node(self.calc_secondary_index(gx, self.minx),
                          self.calc_secondary_index(gy, self.miny), 0.0, -1) # we don't know the parent of the goal yet. so the parent index is -1 for now

        open_set, closed_set = dict(), dict()
    
        open_set[self.get_secondary_index(nstart)] = nstart 
        searched_grid=[]
        Failure_status=False

        while 1:
            if len(open_set) == 0:
                Failure_status=True
                break
            #Find the index of the node with minimum f cost in the open set
            c_id = min(
                open_set, key=lambda o: open_set[o].cost + self.calc_heuristic(ngoal, open_set[o]))

            current = open_set[c_id]

            # plot the current node
            if show_animation:
                    searched_grid.extend(plt.plot(self.calc_primary_index(current.x, self.minx),
                        self.calc_primary_index(current.y, self.miny), "xc"))

            if current.x == ngoal.x and current.y == ngoal.y:
                ngoal.pind = current.pind
                ngoal.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search and calculate cost based on motion model.
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.get_secondary_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node,ox,oy):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node
       

        rx, ry= self.calc_final_path(ngoal, closed_set)

        return rx, ry, searched_grid,Failure_status

    def calc_final_path(self, ngoal, closedset):
        """ generate final course for the current iteration """

        """ Let's say, before breaking the loop, current node 3 [index:(x3,y3) and pind:(x2,y2)] reaches goal node. It doesn't get added to the closed set.
         But the pind attribute of the goal node gets overridden as ngoal.pind=(x2,y2).
        Now, let's look at the closed set. In the closed set, every node is stored against its OWN index. Again, every node has its parent index associated with 
        it as an attribute. Below the closed set, a dictionary, is presented with the parent index of each node in square brackets. Note: The square brackets don't
        represent List.
        closed_set={ (sx,sy):Start_Node[-1] ,(x1,y1):Node_1[sx,sy], (x2,y2):Node_2[x1,y1] }

        Now let's run the loop to understand how it works considering this test case

        """
        rx, ry = [self.calc_primary_index(ngoal.x, self.minx)], [
            self.calc_primary_index(ngoal.y, self.miny)] # put the goal index in. rx,ry=[x3],[y3]
        pind = ngoal.pind #get the parent of the goal index. pind=(x2,y2)
        while pind != -1:
            n = closedset[pind]
            # iter 1: n= closedset[(x2,y2)]= Node_2      iter 2: n=closedset[(x1,y1)]=Node_1       iter 3: n=closedset[(sx,sy)]=Start_node
            rx.append(self.calc_primary_index(n.x, self.minx)) 
            # iter 1: rx=[x3,x2]                         iter 2: rx=[x3,x2,x1]                     iter 3: rx=[x3,x2,x1,sx]
            ry.append(self.calc_primary_index(n.y, self.miny))
             # iter 1: ry=[y3,y2]                        iter 2: ry=[y3,y2,y1]                     iter 3: ry=[y3,y2,sy]
            pind = n.pind
             # iter 1: pind=n.pind=Node_2.pind=[x1,y1]   iter 2: pind=n.pind=Node_1.pind=[sx,sy]   iter 3: pind=n.pind=Start_Node.pind=[-1] **loop executed**
 
        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):

        #calculate the f value
        w = 5  # weight of heuristic
        d = w * math.sqrt((n1.x - n2.x) ** 2 + (n1.y - n2.y) ** 2)
        return d

    def calc_primary_index(self, index, minp):
        """converts secondary index to primary grid index"""
        pos = index * self.reso + minp
        return pos

    def calc_secondary_index(self, position, min_pos):
        """ converts primary index to secondary grid index"""
        return round((position - min_pos) / self.reso)

    def get_secondary_index(self, node):
        """Returns a tuple of the node's own index which 
        will be used to index them in the open and closed list"""
        return (node.x,node.y)

    def verify_node(self, node,ox,oy):
        """Verify if the node falls out of the secondary grid
        or collides with the obstacles"""
        px = self.calc_primary_index(node.x, self.minx)
        py = self.calc_primary_index(node.y, self.miny)

        if px < self.minx:
            return False
        elif py < self.miny:
            return False
        elif px >= self.maxx:
            return False
        elif py >= self.maxy:
            return False

        # collision check
        if self.obmap[node.x][node.y]:
            return False

        return True

    def map_secondary_grid(self, ox, oy):

        """ Get the boundary values (min,max,width) associated with the secondary
        grid (which is part of the obstacles)"""

        self.minx = round(min(ox))
        self.miny = round(min(oy))
        self.maxx = round(max(ox))
        self.maxy = round(max(oy))

        self.xwidth = round((self.maxx - self.minx) / self.reso)
        # xwidth=35 as defined by the current ox,oy mapping. It indicates the width of the secondary grid
        self.ywidth = round((self.maxy - self.miny) / self.reso)
        # ywidth=35 as defined by the current ox,oy mapping. It indicates the height of the secondary grid

    def obsmap(self,ox,oy):
        """For each point in the secondary grid, set a boolean value to it  
        based on it being too close to the obstacles"""
        x=np.arange(0,self.xwidth,dtype=int)
        y=np.arange(0,self.ywidth,dtype=int)
        xx,yy=np.meshgrid(x,y,sparse=False)
        xx=np.ravel(xx)
        yy=np.ravel(yy)

        x_prim=self.calc_primary_index(xx,self.minx)
        y_prim=self.calc_primary_index(yy,self.miny)

        ox_array=np.transpose(np.array([ox]))
        oy_array=np.transpose(np.array([oy]))

        ox_temp=np.ones((len(ox),len(xx)),dtype=float)*ox_array
        oy_temp=np.ones((len(oy),len(yy)),dtype=float)*oy_array

        d1=np.sqrt((ox_temp-x_prim)**2+(oy_temp-y_prim)**2)
        d2=self.rr+wall_padding
        temp=d1<=d2
        temp=temp.sum(axis=0)>0
        self.obmap=np.reshape(temp,(self.xwidth,self.ywidth)).T



    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1], # Right ,cost=1
                  [0, 1, 1], # Up ,cost=1
                  [-1, 0, 1], # Left, cost=1
                  [0, -1, 1], #Down, cost=1
                  [-1, -1, math.sqrt(2)], # Bottom left ,ost=sqrt(2)
                  [-1, 1, math.sqrt(2)], # Top Left ,cost=sqrt(2)
                  [1, -1, math.sqrt(2)], # Bottom Right, cost=sqrt(2)
                  [1, 1, math.sqrt(2)]] # Top Right, cost=sqrt(2)

        return motion

def obsplan():
    """The initial obstacles"""
    # Secondary grid boundary [start]
    ox, oy = [], []
    for i in range(-10, 60):
        ox.append(i)
        oy.append(-10.0)
    for i in range(-10, 60):
        ox.append(60.0)
        oy.append(i)
    for i in range(-10, 61):
        ox.append(i)
        oy.append(60.0)
    for i in range(-10, 61):
        ox.append(-10.0)
        oy.append(i)
    # Secondary grid boundary[end]

    #Other initial obstacles [start]
    for i in range(-10, 40):
        ox.append(20.0)
        oy.append(i)
    for i in range(0, 40):
        ox.append(40.0)
        oy.append(60.0 - i)
    #Other initial obstacles [end]
    return ox,oy


def scan_Obstacles(ox,oy,i,newObstacle_LastPos):
    """Build new obstacles as the robot moves (which means,per iteration of the main loop) and return their positions
    NewObstacle_Lastpos is used to record references and to update the new obstacles"""
    if i is not 0:
        start_pos=[(20,30),(30,40),(20,40),(40,20),(40,30),(60,40)]
        lowerLimit=[1,13,14,33,53,60] # Indicates the iteration number the method starts building each new obsatcle 
        upperLimit=[13,19,24,45,59,66] #Indicates the iteration number the method finishes building each new obsatcle
        growth_rate=[1,2,1,1,2,2] # The rate at which each new obstacle grows
        direction=['left','up','right','left','right','left']

        

        #New Obstacle: 1 (Horizontal)
        if i>=lowerLimit[0] and i<=upperLimit[0]:
            if i is not lowerLimit[0]:
                start_pos[0]=newObstacle_LastPos[0]
            ox,oy,last_pos=horizontal_line(ox,oy,start_pos[0],growth_rate[0],direction[0])
            newObstacle_LastPos.update({0:last_pos})
        #New Obstacle: 2 (Vertical)
        if i>=lowerLimit[1] and i<=upperLimit[1]:
            if i is not lowerLimit[1]:
                start_pos[1]=newObstacle_LastPos[1]
            ox,oy,last_pos=vertical_line(ox,oy,start_pos[1],growth_rate[1],direction[1])
            newObstacle_LastPos.update({1:last_pos})
        #New Obstacle: 3 (Horizontal)
        if i>=lowerLimit[2] and i<=upperLimit[2]:
            if i is not lowerLimit[2]:
                start_pos[2]=newObstacle_LastPos[2]
            ox,oy,last_pos=horizontal_line(ox,oy,start_pos[2],growth_rate[2],direction[2])
            newObstacle_LastPos.update({2:last_pos})
        #New Obstacle: 4 (Horizontal)
        if i>=lowerLimit[3] and i<=upperLimit[3]:
            if i is not lowerLimit[3]:
                start_pos[3]=newObstacle_LastPos[3]
            ox,oy,last_pos=horizontal_line(ox,oy,start_pos[3],growth_rate[3],direction[3])
            newObstacle_LastPos.update({3:last_pos})
        #New Obstacle: 5 (Horizontal)
        if i>=lowerLimit[4] and i<=upperLimit[4]:
            if i is not lowerLimit[4]:
                start_pos[4]=newObstacle_LastPos[4]
            ox,oy,last_pos=horizontal_line(ox,oy,start_pos[4],growth_rate[4],direction[4])
            newObstacle_LastPos.update({4:last_pos})
        #New Obstacle: 6 (Horizontal)
        if i>=lowerLimit[5] and i<=upperLimit[5]:
            if i is not lowerLimit[5]:
                start_pos[5]=newObstacle_LastPos[5]
            ox,oy,last_pos=horizontal_line(ox,oy,start_pos[5],growth_rate[5],direction[5])
            newObstacle_LastPos.update({5:last_pos})


        ### BW Start ###
        if breakWall:
            if i is 35:
                indices=[]
                tempy=[y for y in range(42,47)]
                tempx=[40]*len(range(42,47))
                break_location=[k for k in zip(tempx,tempy)]
                initial_wall=[k for k in zip(ox,oy)]
                for position in initial_wall:
                    if position in break_location:
                        ind=initial_wall.index(position)
                        plt.plot(ox[ind],oy[ind],'w+')
                        indices.append(ind)
                del ox[min(indices):max(indices)]
                del oy[min(indices):max(indices)]
    return ox,oy,newObstacle_LastPos
        ### BW End ###

def horizontal_line(ox,oy,start_pos,growth_rate,direction):
    """" Builds a horizontal obstacles per iteration. 
    If growth_rate is 1, it will build one obstacle per iteration in the specified direction in the
    start_pos. If it's n, then n obstacles will be built per iteration in the specified direction."""
    x_start,y_start=start_pos[0],start_pos[1]
    xnew,ynew=[x_start],[y_start]
    if direction is'left':
        temp1=[x*(-1) for x in range(1,growth_rate+1)]
        temp2=[x_start]*growth_rate
        xnew=xnew+[sum(x) for x in zip(temp1,temp2)]
        ynew=ynew+[y_start]*growth_rate
    if direction is'right':
        temp1=[x*(1) for x in range(1,growth_rate+1)]
        temp2=[x_start]*growth_rate
        xnew=xnew+[sum(x) for x in zip(temp1,temp2)]
        ynew=ynew+[y_start]*growth_rate

    last_pos=(xnew.pop(),ynew.pop())
    ox=ox+xnew
    oy=oy+ynew
    if show_animation:
        plt.plot(xnew,ynew,'ks')

    return ox,oy,last_pos

def vertical_line(ox,oy,start_pos,growth_rate,direction):
    """" Builds a vertical obstacles per iteration. 
    If growth_rate is 1, it will build one obstacle per iteration in the specified direction in the
    start_pos. If it's n, then n obstacles will be built per iteration in the specified direction."""
    x_start,y_start=start_pos[0],start_pos[1]
    xnew,ynew=[x_start],[y_start]
    if direction is'down':
        temp1=[y*(-1) for y in range(1,growth_rate+1)]
        temp2=[y_start]*growth_rate
        ynew=ynew+[sum(y) for y in zip(temp1,temp2)]
        xnew=xnew+[x_start]*growth_rate
    if direction is'up':
        temp1=[y*(1) for y in range(1,growth_rate+1)]
        temp2=[y_start]*growth_rate
        ynew=ynew+[sum(y) for y in zip(temp1,temp2)]
        xnew=xnew+[x_start]*growth_rate

    last_pos=(xnew.pop(),ynew.pop())
    ox=ox+xnew
    oy=oy+ynew
    if show_animation:
        plt.plot(xnew,ynew,'ks')

    return ox,oy,last_pos

def Initialize_Grid(sx,sy,gx,gy,ox,oy,robot_radius):
    """Plot the start position, goal position and the initial obstacles"""
    plt.plot(ox, oy,'ks')
    border=plt.Circle((sx,sy),robot_radius,color='m')
    ax=plt.gca()
    ax.add_artist(border)
    object_position,=plt.plot(sx, sy, ".k") 
    plt.plot(gx, gy, "xb")
    plt.grid(True)
    plt.axis("equal")
    return object_position,border

def move(object_position,border,new_sx,new_sy,robot_radius):
    """Move the robot to new position in the plot"""
    object_position.remove()
    border.remove()
    object_position,=plt.plot(new_sx,new_sy,'.k')
    border=plt.Circle((new_sx,new_sy),robot_radius,edgecolor='m',facecolor='w')
    ax=plt.gca()
    ax.add_artist(border)

    return object_position,border

def flush_old_plan(path,searched_grid):
    """ Remove the path and searched grid inidcations plotted in the previous iteration""" 
    for p in searched_grid:
        p.remove()
    path.remove()







def main():
    print(__file__ + " start!!")

    # start and goal position
    sx = 10.0  # [m]
    sy = 10.0  # [m]
    gx = 50.0  # [m]
    gy = 50.0  # [m]
    grid_size = 2.0  # [m]
    robot_radius = 1.5  # [m]
    initial_message="Initial Start position:({}.{})\nGoal Position:({},{})"
    print(initial_message.format(sx,sy,gx,gy))

    new_sx,new_sy=sx,sy
    ox,oy=obsplan()

    if show_animation:
        object_position,border=Initialize_Grid(sx,sy,gx,gy,ox,oy,robot_radius)
    i=0
    path=None
    searched_grid=None
    newObstacle_LastPos={}
    ultimate_path_x=[sx]
    ultimate_path_y=[sy]
    while 1:

        # Remove the previous path and searched grid points from plot  
        if show_animation:     
            if new_sx is not sx and new_sy is not sy:
                flush_old_plan(path,searched_grid) 
         # Scan and update obstacle list
        ox,oy,newObstacle_LastPos=scan_Obstacles(ox,oy,i,newObstacle_LastPos)
        #Get new planned path according to the current robot position and current obstacles
        a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
        rx,ry,searched_grid,Failure_status= a_star.planning(new_sx, new_sy, gx, gy,ox,oy)
        if Failure_status:
            print("Failed!!\nOpen set is empty..")
            break
        if show_animation:
            path,=plt.plot(rx, ry, "-r") #Plot the updated path
        
        # Move the robot to the immediate child of the current start position. We got this child from the path planned using the 
        # current start position and current obstacle list. So the child position is safe
        new_sx,new_sy=rx[len(rx)-2],ry[len(ry)-2] 
        #Move it in the plot as well
        if show_animation:
            object_position,border=move(object_position,border,new_sx,new_sy,robot_radius)

        print("Iteration:{}\nRobot Position:({},{})".format(i,new_sx,new_sy))
        
        plt.pause(0.00001)

        i+=1
        #Record the path travelled by the robot from start position to the goal position
        ultimate_path_x.append(new_sx)
        ultimate_path_y.append(new_sy)
        if new_sx==gx and new_sy==gy:
            print("Successfull!!")
            break
    #Plot the path travelled by the robot from start position to the goal position
    if show_animation and Failure_status is False:
        plt.plot(sx,sy,'og')
        plt.plot(ultimate_path_x,ultimate_path_y,'-y')
        plt.show()

if __name__ == '__main__':
    main()

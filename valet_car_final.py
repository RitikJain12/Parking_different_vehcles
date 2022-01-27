from os import path
from matplotlib.patches import Rectangle
import matplotlib.pyplot as plt
from numpy import pi, sqrt
import numpy as np
import math



#parameters for displaying output
obstacle_posx = 70
obstacle_posy = 90
obstacle_width = 50
obstacle_height = 50
car1_posx = 30
car1_posy = 10
car2_posx = 130
car2_posy = 10
agent_posx = 0
agent_posy = 180
agent_theta = 0
car_width = 30
car_height = 20 
padding = 5

#parameters for motion planning
agent_start = [agent_posx + 1+5, agent_posy + car_height/2,0]
agent_goal = [car1_posx+car_width+15+1+5,10 + car_height/2,0]

#kinematic parameters
wheelbase = 28
steering_angle = 30
vel = 1

#parameters for collision checking
agent_bound = [[agent_posx,agent_posy,1],[agent_posx+car_width,agent_posy,1],[agent_posx+car_width,agent_posy+car_height,1],[agent_posx,agent_posy+car_height,1]]
obstacle = [[obstacle_posx-padding,obstacle_posy-padding],[obstacle_posx+obstacle_width+padding,obstacle_posy-padding],[obstacle_posx+obstacle_width+padding,obstacle_posy+obstacle_height+padding],[obstacle_posx-padding,obstacle_posy+obstacle_height+padding]]
car1 = [[car1_posx-padding,car1_posy-padding],[car1_posx+car_width+padding,car1_posy-padding],[car1_posx+car_width+padding,car1_posy+car_height+padding],[car1_posx-padding,car1_posy+car_height+padding]]
car2 = [[car2_posx-padding,car2_posy-padding],[car2_posx+car_width+padding,car2_posy-padding],[car2_posx+car_width+padding,car2_posy+car_height+padding],[car2_posx-padding,car2_posy+car_height+padding]]

# use to display the car at different points 
agent_bound_T = [[-1,29,29,-1],[-10,-10,10,10],[1,1,1,1]]

# dislpaying the animation
def world(x,y,theta):
    fig = plt.figure("Animation of Car")
    ax = fig.add_subplot(111)

    obstacle_mid = Rectangle((obstacle_posx, obstacle_posy),obstacle_width,obstacle_height,color ='black')

    car1 = Rectangle((car1_posx, car1_posy),car_width, car_height,color ='red')
    car2 = Rectangle((car2_posx, car2_posy),car_width, car_height,color ='red')

    agent = Rectangle((agent_posx, agent_posy),car_width, car_height,color ='green')

    ax.add_patch(obstacle_mid)
    ax.add_patch(car1)
    ax.add_patch(car2)
    ax.add_patch(agent)
    ax.add_patch( Rectangle((car1_posx+car_width+15, 5),car_width+10, car_height+10,fc ='none',ec ='g',lw = 2) )#goal
    plt.scatter(agent_goal[0],agent_goal[1])
    plt.plot(x,y,"sk")
    boundary = get_boundary(x,y,theta) # get the boundary of car
    
    X = []
    Y = []
    for x,y in boundary:
        X.append(x)
        Y.append(y)

    plt.plot(X,Y)
    plt.xlim([0, 200])
    plt.ylim([-20, 200])

    return

#defining the neighbour points using the kinematic equation
def get_neighbours(x,y,theta):
    neighbour = []
    for i in range(-steering_angle,steering_angle+1,5):
        x_dot = vel*math.cos(theta*(pi/180))
        y_dot = vel*math.sin(theta*(pi/180))
        theta_dot = (vel*math.tan(i*(pi/180))/wheelbase)*(180/pi)
        if(valid_point(x+x_dot,y+y_dot,theta+theta_dot)): # to check if the neighbour position is a valid one before adding it to the list of neighbour
            neighbour.append([round(x+x_dot,2),round(y+y_dot,2),(round(theta+theta_dot,2))%360,1,i])
        if(valid_point(x-x_dot,y-y_dot,theta-theta_dot)): # to check if the neighbour position is a valid one before adding it to the list of neighbour
            neighbour.append([round(x-x_dot,2),round(y-y_dot,2),(round(theta-theta_dot,2)+360)%360,-1,i])
    return neighbour

# g(x) function 
def cost_function(x1,y1,x2,y2):
    distance = sqrt((pow(x1-x2,2)+pow(y1-y2,2)))
    return distance

# h(x) function
def hurestic_function(x,y,theta):
    theta_ = 0
    theta = (theta+360)%360 #changing any negative theta to range of 0-360
    distance = sqrt((pow(agent_goal[0]-x,2)+pow(agent_goal[1]-y,2))) # for x,y # distance of the back axle
    distance += sqrt(((pow((agent_goal[0]+car_width)-(x+car_width*math.cos(theta*(pi/180))),2)+pow((agent_goal[1]+car_height)-(y+car_width*math.sin(theta*(pi/180))),2)))) # distance of the front axle
    if straight_available(x,y) and not(x>agent_goal[0]-5 and y>agent_goal[1]-5 and x <agent_goal[0]+5 and y <agent_goal[1]+5): # if straight path exist it should take that
        theta_ = abs((360 + (math.atan2(y-agent_goal[1],x-agent_goal[0]))*(180/pi))%360 - theta+180) # for theta
    hurestic = distance+theta_
    return hurestic

# to check if collision free straight path exists between the robot and the goal
def straight_available(x,y):
    boundary_line = [[x,y],[agent_goal[0],agent_goal[1]],[agent_goal[0]+1,agent_goal[1]],[x+1,y]]
    if do_polygons_intersect(boundary_line,obstacle):
        return False
    if do_polygons_intersect(boundary_line,car1):
        return False
    return True

# to get the boundary of the car from the  back axle position using homogeneous transformations 
def get_boundary(x,y,theta):
    tx = x 
    ty = y 
    th = theta-agent_start[2]
    homogeneous_matrix = [[math.cos(th*(pi/180)),-math.sin(th*(pi/180)),tx],[math.sin(th*(pi/180)),math.cos(th*(pi/180)),ty]]
    mat_mul = np.dot(homogeneous_matrix,agent_bound_T)
    new_boundary = [[mat_mul[0][0],mat_mul[1][0]],[mat_mul[0][1],mat_mul[1][1]],[mat_mul[0][2],mat_mul[1][2]],[mat_mul[0][3],mat_mul[1][3]]]
    return new_boundary

# to check if the position of the car is valid or not
def valid_point(x,y,theta): 
    boundary = get_boundary(x,y,theta)
    if x < 1 or y < car_height or x > 200-car_width or y > 200-car_height/2.0:
        return False 
    #collision conditions between boundary and different obstacles
    if do_polygons_intersect(boundary,obstacle):
        return False
    if do_polygons_intersect(boundary,car1):
        return False
    if do_polygons_intersect(boundary,car2):
        return False
    
    return True

# to check if two pollygons intersect to check for collision
def do_polygons_intersect(a, b):
    polygons = [a, b]
    minA, maxA, projected, i, i1, j, minB, maxB = None, None, None, None, None, None, None, None

    for i in range(len(polygons)):

        # for each polygon, look at each edge of the polygon, and determine if it separates
        # the two shapes
        polygon = polygons[i]
        for i1 in range(len(polygon)):

            # grab 2 vertices to create an edge
            i2 = (i1 + 1) % len(polygon)
            p1 = polygon[i1]
            p2 = polygon[i2]

            # find the line perpendicular to this edge
            normal = { 'x': p2[1] - p1[1], 'y': p1[0] - p2[0] }

            minA, maxA = None, None
            # for each vertex in the first shape, project it onto the line perpendicular to the edge
            # and keep track of the min and max of these values
            for j in range(len(a)):
                projected = normal['x'] * a[j][0] + normal['y'] * a[j][1];
                if (minA is None) or (projected < minA): 
                    minA = projected

                if (maxA is None) or (projected > maxA):
                    maxA = projected

            # for each vertex in the second shape, project it onto the line perpendicular to the edge
            # and keep track of the min and max of these values
            minB, maxB = None, None
            for j in range(len(b)): 
                projected = normal['x'] * b[j][0] + normal['y'] * b[j][1]
                if (minB is None) or (projected < minB):
                    minB = projected

                if (maxB is None) or (projected > maxB):
                    maxB = projected

            # if there is no overlap between the projects, the edge we are looking at separates the two
            # polygons, and we know there is no overlap
            if (maxA < minB) or (maxB < minA):
                #print("polygons don't intersect!")
                return False

    return True

def priority(queue): #find the path with shortest distance that will be selected from the priority queue
    min = math.inf
    index = 0
    for check in range(len(queue)):
        _,value,_,_ = queue[check]
        if value<min:
            min = value
            index = check #index of the shortest path
    return index

# to check for visited nodes for the A* algorithm
def check_visited(current,visited):
    for x,y,th in visited:
        if current[0]== x and current[1]== y and current[2]==th :
            return True
    return False

#A* algorithm to find the shortest path from the start orientation to goal orientation
def A_star():
    open_set = []
    visited = []
    start = agent_start
    tcost = 0
    gcost = 0
    path = [start]
    open_set.append((start,tcost,gcost,path))
    while len(open_set)>0:
        index = priority(open_set)
        (shortest,_,gvalue,path) = open_set[index] #select the node with lowest distance
        open_set.pop(index)
        if not (check_visited([round(shortest[0]),round(shortest[1]),round(shortest[2])],visited)): # check if already visited
            visited.append([round(shortest[0]),round(shortest[1]),round(shortest[2])])
            if round(shortest[0]) <= agent_goal[0]+5 and round(shortest[0]) >= agent_goal[0]-5 and round(shortest[1]) <= agent_goal[1]+5 and round(shortest[1]) >= agent_goal[1]-5 and shortest[2] <= agent_goal[2]+15 and shortest[2] >= agent_goal[2]-15: #goal condition
                return path
            neighbours= get_neighbours(shortest[0],shortest[1],shortest[2]) # get valid neighbours using tehe kinematic equation
            for neighbour in neighbours:#calculate cost of each neighbor
                vel = neighbour[3]
                turn = neighbour[4]
                temp_gcost = gvalue+(0.1*cost_function(shortest[0],shortest[1],neighbour[0],neighbour[1]))
                temp_tcost = temp_gcost+(0.9*hurestic_function(neighbour[0],neighbour[1],neighbour[2]))
                open_set.append((neighbour,temp_tcost,temp_gcost,path+ [neighbour]))
    print("not working")      
    return path


path = A_star()
print("reached")
print(path[-1])#goal point

#animation of the final path
for points in path:
    plt.cla()
    world(points[0],points[1],points[2])
    # print(points)
    plt.pause(0.00001)

# ploting the axle center path
plt.figure("Back Axle path")
plt.xlim([0, 200])
plt.ylim([-20, 200])
for points in path:
    plt.scatter(points[0],points[1],color = 'black',s=1)  

plt.show()

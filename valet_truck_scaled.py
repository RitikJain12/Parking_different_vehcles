from os import path
from matplotlib.patches import Rectangle
import matplotlib.pyplot as plt
from numpy import pi, sqrt
import numpy as np
import math
import matplotlib as mpl


#parameters for displaying output
obstacle_posx = 70
obstacle_posy = 90
obstacle_width = 50
obstacle_height = 50
car1_posx = 30
car1_posy = 10
# car2_posx = 130
# car2_posy = 10
agent_posx = 49
agent_posy = 180
agent_trailer_posx = agent_posx-25+2
agent_trailer_posy = agent_posy
agent_theta = 0
agent_trailer_theta = 0
car_width = 22
car_height = 17.5 
padding = 2

#parameters for motion planning
agent_start = [agent_posx, agent_posy + car_height/2,0]
agent_trailer_start = [agent_trailer_posx,agent_trailer_posy + car_height/2,0]
agent_goal = [car1_posx+car_width+15+1+5+car_width,10 + car_height/2,180]
# agent_goal = [car2_posx,10 + car_height*2,0]
wheelbase = 15
hitch_lenght = 25
steering_angle = 30
vel = 1

#parameters for collision checking
agent_bound = [[agent_posx,agent_posy,1],[agent_posx+car_width,agent_posy,1],[agent_posx+car_width,agent_posy+car_height,1],[agent_posx,agent_posy+car_height,1]]
agent_bound_T = [[-1,21,21,-1],[-7.5,-7.5,7.5,7.5],[1,1,1,1]]
agent_trailer_bound_T = [[-1,11,11,-1],[-7.5,-7.5,7.5,7.5],[1,1,1,1]]
# agent_trailer_bound_T = [[-1,29,29,-1],[-10,-10,10,10],[1,1,1,1]]
obstacle = [[obstacle_posx-padding,obstacle_posy-padding],[obstacle_posx+obstacle_width+padding,obstacle_posy-padding],[obstacle_posx+obstacle_width+padding,obstacle_posy+obstacle_height+padding],[obstacle_posx-padding,obstacle_posy+obstacle_height+padding]]
car1 = [[car1_posx-padding,car1_posy-padding],[car1_posx+car_width+padding,car1_posy-padding],[car1_posx+car_width+padding,car1_posy+car_height+padding],[car1_posx-padding,car1_posy+car_height+padding]]
# car2 = [[car2_posx-padding,car2_posy-padding],[car2_posx+car_width+padding,car2_posy-padding],[car2_posx+car_width+padding,car2_posy+car_height+padding],[car2_posx-padding,car2_posy+car_height+padding]]
# parking_spot = [[car1_posx+car_width+15, 5],[car1_posx+car_width+15+car_width+10, 5],[car1_posx+car_width+15+car_width+10, 5+car_height+10],[car1_posx+car_width+15, 5+car_height+10]]
# lower_boundary = [[0,0],[200,0],[200,0],[0,0]]
# world_border = [[0,0],[0,220],[220,220],[220,0]]

def world(x,y,theta,xt,yt,theta_t):
    fig = plt.figure("1")
    ax = fig.add_subplot(111)

    obstacle_mid = Rectangle((obstacle_posx, obstacle_posy),obstacle_width,obstacle_height,color ='black')

    car1 = Rectangle((car1_posx, car1_posy),car_width, car_height,color ='red')
    # car2 = Rectangle((car2_posx, car2_posy),car_width, car_height,color ='red')

    agent = Rectangle((agent_posx, agent_posy),car_width, car_height,color ='green')
    agent_trailer = Rectangle((agent_trailer_posx,agent_trailer_posy),10,car_height,color = 'green')

    ax.add_patch(obstacle_mid)
    ax.add_patch(car1)
    # ax.add_patch(car2)
    ax.add_patch(agent)
    ax.add_patch(agent_trailer)
    ax.add_patch( Rectangle((car1_posx+car_width+15, 0),car_width+car_width+10+20, car_height+car_height,fc ='none',ec ='g',lw = 2) )
    # plt.scatter(agent_goal[0],agent_goal[1])
    plt.plot(x,y,"sk")
    boundary = get_boundary(x,y,theta)
    boundary_t = get_boundary(xt,yt,theta_t,"trailer")
    # boundary = Rectangle((x-1, y-(car_height/2)),car_width, car_height,color ='green')
    # t = mpl.transforms.Affine2D().rotate_deg(theta) + ax.transData
    # boundary.set_transform(t)
    # ax.add_patch(boundary)
    #print(boundary[0][0],boundary[0][0])
    X = []
    Y = []
    for x,y in boundary:
        X.append(x)
        Y.append(y)
    plt.plot(X,Y,color = 'blue')

    X = []
    Y = []
    plt.plot(X,Y)
    for x,y in boundary_t:
        X.append(x)
        Y.append(y)

    plt.plot(X,Y,color = 'blue')
    plt.xlim([0, 200])
    plt.ylim([-20, 200])

    return



def get_neighbours(x,y,theta,xt,yt,theta_t):
    neighbour = []
    for i in range(-steering_angle,steering_angle+1,5):
        x_dot = vel*math.cos(theta*(pi/180))
        y_dot = vel*math.sin(theta*(pi/180))
        theta_dot = (vel*math.tan(i*(pi/180))/wheelbase)*(180/pi)
        xt_dot = vel*math.cos(theta_t*(pi/180))
        yt_dot = vel*math.sin(theta_t*(pi/180))
        theta_t_dot = (vel*math.sin((theta-theta_t)*(pi/180))/hitch_lenght)*(180/pi)
        #theta_dot = (360 + theta_dot*(180/pi))%360
        #print(x,y,theta)
        #print(x_dot,y_dot,theta_dot)
        if(valid_point(x+x_dot,y+y_dot,theta+theta_dot,xt+xt_dot,yt+yt_dot,theta_t+theta_t_dot)):
            neighbour.append([round(x+x_dot,2),round(y+y_dot,2),(round(theta+theta_dot,2))%360,round(xt+xt_dot,2),round(yt+yt_dot,2),(round(theta_t+theta_t_dot,2)+360)%360,1,i])
        if(valid_point(x-x_dot,y-y_dot,theta-theta_dot,xt-xt_dot,yt-yt_dot,theta_t-theta_t_dot)):
            neighbour.append([round(x-x_dot,2),round(y-y_dot,2),(round(theta-theta_dot,2)+360)%360,round(xt-xt_dot,2),round(yt-yt_dot,2),(round(theta_t-theta_t_dot,2)+360)%360,-1,i])
    return neighbour

def cost_function(x1,y1,x2,y2,theta1,theta2,vel): #g(x)
    cost = 0
    reverse_penalty = 0
    turn_penalty = 0
    distance = sqrt((pow(x1-x2,2)+pow(y1-y2,2)))
    #hypotaneous = sqrt((pow(agent_goal[0]-x2,2)+pow(agent_goal[1]-y2,2)))
    # perpendicular = agent_goal[1]-y2
    # adjacent = agent_goal[0]-x2
    # target_theta = math.atan2(perpendicular,adjacent)
    # turn_penalty = abs(abs(target_theta)-abs(theta2))
    # if vel < 0:
    #     reverse_penalty = 10
    # if theta1-theta2!=0:
    #     turn_penalty = 10
    #turn_penalty = -180+(math.atan2(y2-agent_goal[1],x2-agent_goal[0]))*(180/pi)+theta2
    cost = distance+reverse_penalty+turn_penalty
    return cost

def hurestic_function(x,y,theta,xt,yt,theta_t,vel,turn): #h(x)
    theta_ = 0
    theta = (theta+360)%360
    theta_t = (theta_t+360)%360
    if theta_t == 0:
        theta_t=360
    reverse_penalty = 0 
    turn_penalty = 0
    obstacle_penalty = 0 
    distance = sqrt((pow(agent_goal[0]-x,2)+pow(agent_goal[1]-y,2))) # for x,y
    distance += sqrt(((pow((agent_goal[0]-car_width)-(x+car_width*math.cos(theta*(pi/180))),2)+pow((agent_goal[1]+car_height)-(y+car_width*math.sin(theta*(pi/180))),2))))
    # distance += sqrt((pow(agent_goal[0]+50-2-xt,2)+pow(agent_goal[1]-yt,2)))
    #distance = abs(agent_goal[0]-x) + abs(agent_goal[1]-y)
    # if (x>agent_goal[0]-5 and y>agent_goal[1]-5 and x <agent_goal[0]+5 and y <agent_goal[1]+5):
        # distance += sqrt(((pow((agent_goal[0]-car_width)-(x+car_width*math.cos(theta*(pi/180))),2)+pow((agent_goal[1]+car_height)-(y+car_width*math.sin(theta*(pi/180))),2))))
        # theta_ = abs(theta-agent_goal[2])
    # if not(x>agent_goal[0]-5 and y>agent_goal[1]-5 and x <agent_goal[0]+5 and y <agent_goal[1]+5):
    # if not do_polygons_intersect(get_boundary(x,y,theta),parking_spot):
    if straight_available(x,y,xt,yt) and not(x>agent_goal[0]-5 and y>agent_goal[1]-5 and x <agent_goal[0]+5 and y <agent_goal[1]+15):
        theta_ = abs((360 + (math.atan2(y-agent_goal[1],x-agent_goal[0]))*(180/pi))%360 - theta+180) # for theta
    else:
        theta_ = 180
    if do_polygons_intersect([[x-15,y],[x+200*math.cos(theta*(pi/180))-15,y+200*math.sin(theta*(pi/180))],[15+x+200*math.cos(theta*(pi/180)),y+200*math.sin(theta*(pi/180))],[x+15,y]],obstacle):
        obstacle_penalty +=10
    if do_polygons_intersect([[x-15,y],[x+200*math.cos(theta*(pi/180))-15,y+200*math.sin(theta*(pi/180))],[15+x+200*math.cos(theta*(pi/180)),y+200*math.sin(theta*(pi/180))],[x+15,y]],obstacle):
        obstacle_penalty +=10
    if vel < 0:
        reverse_penalty = 1
    if abs(theta-theta_t)>15 and not(x>agent_goal[0]-5 and y>agent_goal[1]-5 and x <agent_goal[0]+5 and y <agent_goal[1]+15):
        turn_penalty = 5
        # turn_penalty = abs(theta-theta_t)
    # if turn!=0 :
    #     turn_penalty = 1
    hurestic = distance+theta_+reverse_penalty+turn_penalty+obstacle_penalty
    return hurestic

def straight_available(x,y,xt,yt):
    boundary_line = [[x,y],[agent_goal[0],agent_goal[1]],[agent_goal[0]+1,agent_goal[1]],[x+1,y]]
    boundary_line_t = [[xt,yt],[agent_goal[0]+50,agent_goal[1]],[agent_goal[0]+1+50,agent_goal[1]],[xt+1,yt]]
    if do_polygons_intersect(boundary_line,obstacle) and do_polygons_intersect(boundary_line_t,obstacle):
        return False
    if do_polygons_intersect(boundary_line,car1) and do_polygons_intersect(boundary_line_t,car1):
        return False
    return True

def get_boundary(x,y,theta,car_type="car"):
    # tx = x-agent_start[0]
    # ty = y-agent_start[1]
    tx = x 
    ty = y 
    th = theta-agent_start[2]
    homogeneous_matrix = [[math.cos(th*(pi/180)),-math.sin(th*(pi/180)),tx],[math.sin(th*(pi/180)),math.cos(th*(pi/180)),ty]]
    if car_type == "car":
        mat_mul = np.dot(homogeneous_matrix,agent_bound_T)
    else:
        mat_mul = np.dot(homogeneous_matrix,agent_trailer_bound_T)
    new_boundary = [[mat_mul[0][0],mat_mul[1][0]],[mat_mul[0][1],mat_mul[1][1]],[mat_mul[0][2],mat_mul[1][2]],[mat_mul[0][3],mat_mul[1][3]]]
    return new_boundary

def valid_point(x,y,theta,xt,yt,theta_t): #collision checking current_pos(x,y,theta,vel)
    boundary = get_boundary(x,y,theta)
    boundary_t = get_boundary(xt,yt,theta_t,"trailer")
    #print(boundary)
    if x < 1 or y < car_height/2.0 or x > 200-car_width or y > 200-car_height/2.0:
        return False 
    if do_polygons_intersect(boundary,obstacle):# or do_polygons_intersect(boundary_t,obstacle):
        #print("obs")
        return False
    if do_polygons_intersect(boundary,car1) or do_polygons_intersect(boundary_t,car1):
        #print("car1")
        return False
    # if do_polygons_intersect(boundary,car2):
    #     #print("car2")
    #     return False
    # if do_polygons_intersect(boundary,lower_boundary):
    #     return False
    
    return True

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

def check_visited(current,visited):
    for x,y,th,xt,yt,th_t in visited:
        if current[0]== x and current[1]== y and current[2]==th and current[3]== xt and current[4]== yt and current[5]==th_t:
            return True
    return False

def A_star():
    open_set = []
    visited = []
    start = [agent_start[0],agent_start[1],agent_start[2],agent_trailer_start[0],agent_trailer_start[1],agent_trailer_start[2]]
    tcost = 0
    gcost = 0
    path = [start]
    open_set.append((start,tcost,gcost,path))
    itr =0
    while len(open_set)>0:
        itr+=1
        index = priority(open_set)
        (shortest,_,gvalue,path) = open_set[index] #select the node with lowest distance
        # if itr%10000 == 0:
        #     plt.cla()
        #     world(shortest[0],shortest[1],shortest[2],shortest[3],shortest[4],shortest[5])
        #     print(shortest)
        #     plt.pause(0.00000001)
        
        open_set.pop(index)
        if not (check_visited([round(shortest[0]),round(shortest[1]),round(shortest[2]),round(shortest[3]),round(shortest[4]),round(shortest[5])],visited)):
            # plt.cla()
            # world(shortest[0],shortest[1],shortest[2],shortest[3],shortest[4],shortest[5])
            # print(shortest)
            # plt.pause(0.00000001)
            visited.append([round(shortest[0]),round(shortest[1]),round(shortest[2]),round(shortest[3]),round(shortest[4]),round(shortest[5])])
            if round(shortest[0]) <= agent_goal[0]+5 and round(shortest[0]) >= agent_goal[0]-5 and round(shortest[1]) <= agent_goal[1]+5 and round(shortest[1]) >= agent_goal[1]-5 and shortest[2] <= agent_goal[2]+5 and shortest[2] >= agent_goal[2]-5:# and shortest[5] <= agent_goal[2]+10 and shortest[5] >= agent_goal[2]-10:
                return path
            neighbours= get_neighbours(shortest[0],shortest[1],shortest[2],shortest[3],shortest[4],shortest[5])
            for neighbour in neighbours:
                vel = neighbour[6]
                turn = neighbour[7]
                temp_gcost = gvalue+(0.1*cost_function(shortest[0],shortest[1],neighbour[0],neighbour[1],shortest[2],neighbour[2],vel))
                temp_tcost = temp_gcost+(0.9*hurestic_function(neighbour[0],neighbour[1],neighbour[2],neighbour[3],neighbour[4],neighbour[5],vel,turn))
                open_set.append((neighbour,temp_tcost,temp_gcost,path+ [neighbour]))
    print("not working")      
    return path


# world(agent_start[0],agent_start[1],agent_start[2],agent_start[0]-50,agent_start[1],agent_start[2])
path = A_star()
print("reached")
print(path[-1])
# world(path[-1][0],path[-1][1],path[-1][2],path[-1][3],path[-1][4],path[-1][5])
for points in path:
    plt.cla()
    world(points[0],points[1],points[2],points[3],points[4],points[5])
    # print(points)
    plt.pause(0.00001) 
plt.figure()
plt.xlim([0, 200])
plt.ylim([-20, 200])
for points in path:
    plt.scatter(points[0],points[1],color = 'black',s=1) 
plt.show()

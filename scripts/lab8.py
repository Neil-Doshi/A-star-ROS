#!/usr/bin/python3



import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray
from tf.transformations import euler_from_quaternion
from rospy.client import get_param
import matplotlib.pyplot as plt
startx= -8
starty= -2
stopx= float(get_param("goalx"))
stopy= float(get_param("goaly"))
#stopx = 4
#stopy = 9
m_startx= 10 - int(starty)
m_starty= 9 + int(startx)
m_stopx= 10 - int(stopy)
m_stopy= 9 + int(stopx)
global targetx
global targety
global line_to_goal
global x
global y
global bot_theta
global laser
intx = 0
inty = 0
line_to_goal = 0

#paths = [(-7.5,-2.5),(-6.5,-2.5),(-5.5,-2.5),(-4.5,-2.5),(-4.5,-3.5),(-3.5,-3.5),(-2.5,-3.5),(-2.5,-4.5),(-2.5,-5.5),(-2.5,-6.5),(-2.5,-7.5),(-2.5,-8.5)]
nc = 0
targetx = stopx
targety = stopy


k=0
map = np.array([
    [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0],
    [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0],
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [0,0,1,1,0,0,1,1,1,1,1,1,0,0,0,0,0,0],
    [0,0,1,1,0,0,1,1,1,1,1,1,0,0,0,0,0,0],
    [0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,1,1,0],
    [0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1],
    [0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1],
    [0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1],
    [0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0],
    [0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0],
    [0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0],
    [0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0],
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0],
    [0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0],
    [0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0],
    [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1]
    ])
blocked = np.zeros(np.shape(map))

def astar():
    global map, m_startx , m_starty, m_stopx, m_stopy , blocked, k
    #print(m_startx , m_starty, m_stopx, m_stopy)
    blockage = False
    node_count = 1
    x=m_startx 
    y=m_starty
    closed = np.zeros(np.shape(map))
    opener = np.zeros(np.shape(map))
    node = np.zeros(np.shape(map))
    h= np.zeros(np.shape(map))
    g= np.zeros(np.shape(map))
    f= np.zeros(np.shape(map))
    path=[]
    while not(x == m_stopx and y == m_stopy):

        closed[x,y]=1
        node[x,y]= node_count 
        node_count += 1
        neighbours= [[1,0],[-1,0],[0,1],[0,-1],[1,1],[-1,-1],[1,-1],[-1,1]]
        ar=[]
        for neighbour in neighbours:
            a = x + neighbour[0]
            b = y + neighbour[1]
            penality= 0 
            if a<0 or a>19 or b < 0 or b > 17:
                penality= 999  
            if opener[a,b] == 0:
                opener[a,b]=node[x,y]
            g[a,b] = g[x,y]+ math.sqrt(neighbour[0]**2+neighbour[1]**2)
            h[a,b]= hcost(a , b , m_stopx , m_stopy)
            f[a,b]= g[a,b]+h[a,b]+ 999*map[a,b] + 999*blocked[a,b] + 999*closed[a,b] + penality
            ar.append(f[a,b])
        min_val = np.min(ar)
        if min_val <999:
            arg_min=np.argmin(ar)

            x += neighbours[arg_min][0]
            y += neighbours[arg_min][1]
        else:
            blocked[x,y] = 1
            blockage = True
           
            break
       
    k += 1
    if blockage and k < 50:
        return astar()
    path.append([y-9,10-x])
    n=np.max(node)
    show_path = []
    while n!=1:
        a,b = np.where(node == n)
        a = int(a)
        b = int(b)
        t = []
        neighbours= [[1,0],[-1,0],[0,1],[0,-1],[1,1],[-1,-1],[1,-1],[-1,1]]
        for neighbour in neighbours:
            x = a + neighbour[0]
            y = b + neighbour[1]
            data = node[x][y]       
            t.append(data)
        t = np.sort(t)
        t = np.trim_zeros(t)
        t = min(t)
        n = t
        a,b = np.where(node == n)
        a = int(a)
        b = int(b)
        show_path.append([a,b])
        path.append([b-9,10-a])
    path = path[::-1]
    display_map = np.zeros(np.shape(map))
    display_map = map
    for paths in show_path:
        x,y = paths
        display_map[x][y] = 5
    #plt.imshow(display_map)
    #plt.show()
    return path, display_map

def hcost(a,b,c,d):
    dista= abs(a - c)
    distb= abs(b-d)
    return dista + distb

def call_odom(info, path):
    global x, y, bot_theta
    angle = info.pose.pose.orientation
    position = info.pose.pose.position
    angle = euler_from_quaternion([angle.x, angle.y, angle.z, angle.w])  
    bot_theta = math.radians(angle[2]*57.30)
    x = position.x
    y = position.y
    travel(path)
    return 

def dist(tx,x,ty,y):
    distance = math.sqrt((tx-x)**2 + (ty-y)**2)
    return distance

def travel(paths):
    global targetx , targety , nc
    move = Twist()
    global complete
    complete = False
    if nc >= len(paths):
        return
    tx = paths[nc][0]
    ty = paths[nc][1]
    theta = math.atan2(ty-y,tx-x)
    rotation = theta - bot_theta

    dis= dist(tx,x,ty,y)
    if dis < 0.3:
        nc += 1
    else:
        if abs(rotation)<0.05:
            move.linear.x=2
            move.angular.z=rotation
        else:
            move.angular.x=0
            move.angular.z= rotation
    pub.publish(move)


if __name__ == "__main__":
    rospy.init_node("bugalgorithm", anonymous=True)
    path, disp_map = astar()
    rospy.Subscriber("/odom", Odometry, call_odom, path)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
    plt.imshow(disp_map)
    plt.show()
    rospy.spin()

#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import math
from heapq import *
import sys
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# extra = 0
res = 1
r = 3.3 # wheels
l = 28.7
clr = 15 # clearance from the boundaries
extra = 22+clr
dt = 1

obstacle_space = []

def create_graph():
	print("Creating graph") 
	graph = {}
	for i in range(int(1110 / res)):
		for j in range(int(1010 / res)):
			graph[(i,j)] = {'visited':False, 'g':np.inf, 'valid':True, 'parent': (0, 0), 'neighbour':[0.0,0.0,0.0,0.0,0.0,0.0]}

			C1 = (i - round(390/res))**2 + (j - round(960/res))**2 - ((40.5/res) + extra)**2
			if C1 <= 0:
				graph[(i,j)]['valid'] = False
				obstacle_space.append((i, j))

			C2 = (i - round(438 / res)) ** 2 + (j - round(736 / res)) ** 2 - ((40.5 / res) + extra) ** 2
			if C2 <= 0:
				graph[(i,j)]['valid'] = False
				obstacle_space.append((i, j))

			C3 = (i - round(390 / res)) ** 2 + (j - round(45 / res)) ** 2 - ((40.5 / res) + extra) ** 2
			if C3 <= 0:
				graph[(i,j)]['valid'] = False
				obstacle_space.append((i, j))

			C4 = (i - round(438 / res)) ** 2 + (j - round(274 / res)) ** 2 - ((40.5 / res) + extra) ** 2
			if C4 <= 0:
				graph[(i,j)]['valid'] = False
				obstacle_space.append((i, j))

	# ########### RECTANGLE 1 ##############
			f1 = -j + (0/res) - extra
			f2 = j - (35/res) - extra
			f3 = -i + (685/res) - extra
			f4 = i - (1110/res) - extra

			if f1 <= 0 and f2 <= 0 and f3 <= 0 and f4 <= 0:
				graph[(i,j)]['valid'] = False
				obstacle_space.append((i, j))

	# ########### RECTANGLE 2 ##############

			f1 = -j + (35 / res) - extra
			f2 = j - (111 / res) - extra
			f3 = -i + (927 / res) - extra
			f4 = i - (1110 / res) - extra

			if f1 <= 0 and f2 <= 0 and f3 <= 0 and f4 <= 0:
				graph[(i,j)]['valid'] = False
				obstacle_space.append((i, j))

	# ########### RECTANGLE 3 ##############

			f1 = -j + (35 / res) - extra
			f2 = j - (93 / res) - extra
			f3 = -i + (779 / res) - extra
			f4 = i - (896 / res) - extra

			if f1 <= 0 and f2 <= 0 and f3 <= 0 and f4 <= 0:
				graph[(i,j)]['valid'] = False
				obstacle_space.append((i, j))

	# ########### RECTANGLE 4 ##############

			f1 = -j + (35 / res) - extra
			f2 = j - (187 / res) - extra
			f3 = -i + (474 / res) - extra
			f4 = i - (748 / res) - extra

			if f1 <= 0 and f2 <= 0 and f3 <= 0 and f4 <= 0:
				graph[(i,j)]['valid'] = False
				obstacle_space.append((i, j))

	# # ########### RECTANGLE 5 ##############
	#         f1 = -j + (295.25/res) - extra
	#         f2 = j - (412.25/res) - extra
	#         f3 = -i + (1052/res) - extra
	#         f4 = i - (1110/res) - extra
	#
	#         if f1 <= 0 and f2 <= 0 and f3 <= 0 and f4 <= 0:
	#             obstacle_space.append((i, j))

	# ########### RECTANGLE 6 ##############
			f1 = -j + (919/res) - extra
			f2 = j - (1010/res) - extra
			f3 = -i + (983/res) - extra
			f4 = i - (1026/res) - extra

			if f1 <= 0 and f2 <= 0 and f3 <= 0 and f4 <= 0:
				graph[(i,j)]['valid'] = False
				obstacle_space.append((i, j))

	# ########### RECTANGLE 7 ##############
			f1 = -j + (827/res) - extra
			f2 = j - (1010/res) - extra
			f3 = -i + (832/res) - extra
			f4 = i - (918/res) - extra

			if f1 <= 0 and f2 <= 0 and f3 <= 0 and f4 <= 0:
				graph[(i,j)]['valid'] = False
				obstacle_space.append((i, j))

	# ########### RECTANGLE 8 ##############
			f1 = -j + (621/res) - extra
			f2 = j - (697/res) - extra
			f3 = -i + (744/res) - extra
			f4 = i - (1110/res) - extra

			if f1 <= 0 and f2 <= 0 and f3 <= 0 and f4 <= 0:
				graph[(i,j)]['valid'] = False
				obstacle_space.append((i, j))

	# ########### RECTANGLE 9 ##############
			f1 = -j + (449/res) - extra
			f2 = j - (566/res) - extra
			f3 = -i + (1052/res) - extra
			f4 = i - (1110/res) - extra

			if f1 <= 0 and f2 <= 0 and f3 <= 0 and f4 <= 0:
				graph[(i,j)]['valid'] = False
				obstacle_space.append((i, j))

	# ########### RECTANGLE 10 ##############
			f1 = -j + (363/res) - extra
			f2 = j - (449/res) - extra
			f3 = -i + (1019/res) - extra
			f4 = i - (1110/res) - extra

			if f1 <= 0 and f2 <= 0 and f3 <= 0 and f4 <= 0:
				graph[(i,j)]['valid'] = False
				obstacle_space.append((i, j))

	# ########### RECTANGLE 11 ##############
			f1 = -j + (178.75/res) - extra
			f2 = j - (295.75/res) - extra
			f3 = -i + (1052/res) - extra
			f4 = i - (1110/res) - extra

			if f1 <= 0 and f2 <= 0 and f3 <= 0 and f4 <= 0:
				graph[(i,j)]['valid'] = False
				obstacle_space.append((i, j))

	# ########### RECTANGLE 12 ##############
			f1 = -j + (315/res) - extra
			f2 = j - (498/res) - extra
			f3 = -i + (438/res) - extra
			f4 = i - (529/res) - extra

			if f1 <= 0 and f2 <= 0 and f3 <= 0 and f4 <= 0:
				graph[(i,j)]['valid'] = False
				obstacle_space.append((i, j))

	# ########### RECTANGLE 13 ##############
			f1 = -j + (265/res) - extra
			f2 = j - (341/res) - extra
			f3 = -i + (529/res) - extra
			f4 = i - (712/res) - extra

			if f1 <= 0 and f2 <= 0 and f3 <= 0 and f4 <= 0:
				graph[(i,j)]['valid'] = False
				obstacle_space.append((i, j))

	# ########### RECTANGLE 14 ##############
			f1 = -j + (267/res) - extra
			f2 = j - (384/res) - extra
			f3 = -i + (784.5/res) - extra
			f4 = i - (936.5/res) - extra

			if f1 <= 0 and f2 <= 0 and f3 <= 0 and f4 <= 0:
				graph[(i,j)]['valid'] = False
				obstacle_space.append((i, j))

	# ########### TABLE SQUARE ##############
			f1 = -j + (751/res) - extra
			f2 = j - (910/res) - extra
			f3 = -i + (149.5/res) - extra
			f4 = i - (318.885/res) - extra

			if f1 <= 0 and f2 <= 0 and f3 <= 0 and f4 <= 0:
				graph[(i,j)]['valid'] = False
				obstacle_space.append((i, j))

	# ########### TABLE CIRCLE LEFT ##############
			TCL = (i - round(149.5/res))**2 + (j - round(830.5/res))**2 - ((79.5/res) + extra)**2
			if TCL <= 0:
				graph[(i,j)]['valid'] = False
				obstacle_space.append((i, j))

	# ########### TABLE CIRCLE RIGHT ##############
			TCR = (i - round(318.885 / res)) ** 2 + (j - round(830.5 / res)) ** 2 - ((79.5 / res) + extra) ** 2
			if TCR <= 0:
				graph[(i,j)]['valid'] = False
				obstacle_space.append((i, j))
	# ########### WALLS ##############
	# for i in range(int(1110 / res)):
	# 	graph[(i,0)]['valid'] = False
	# 	graph[(i,1009)]['valid'] = False
	# 	obstacle_space.append((i, 0))
	# 	obstacle_space.append((i, 1009 / res))

	# for j in range(int(1010 / res)):
	# 	graph[(0,j)]['valid'] = False
	# 	graph[(1109,j)]['valid'] = False
	# 	obstacle_space.append((0, j))
	# 	obstacle_space.append((1109 / res, j))	
	
	return graph

def calculate_distance(goal, current):
	d = math.sqrt(((goal[0]-current[0])*(goal[0]-current[0]))+((goal[1]-current[1])*(goal[1]-current[1])))
	return d

def goal_reached(current, goal):
	if (int(current[0])-goal[0])**2 + (int(current[1])-goal[1])**2 < (5)**2:
		return True
	else:
		return False

def find_neighbour(current,ul,ur):
	
	theta = current[2]
	x = current[0]
	y = current[1]
	# for i in range(100):
	d_theta = (r/l)*(ur-ul)*dt
	dx = (r/2)*(ul+ur)*(math.cos(theta + d_theta))*dt
	dy = (r/2)*(ul+ur)*(math.sin(theta + d_theta))*dt

	x = x + dx
	y = y + dy
	theta = theta + d_theta
	
	if x < 0:
		x =0
	if y < 0:
		y = 0
	if x >= 1110:
		x = 1109
	if y >= 1010:
		y = 1009
	x = round(x)
	y = round(y)
	# dx = round(dx)
	# dy = round(dy)
	# d_theta = theta - current[2]
	# dx = x - current[0]
	# dy = x - current[1]
	# neighbour_position =  (x, y, theta)
	# neighbour_velocity =  (dx/dt, dy/dt, d_theta/dt)
	params = [x, y, theta, dx/dt, dy/dt, d_theta/dt]
	key = (x,y)

	return params, key

def astar(graph, source, goal, rpm):
	theta = 0
	(rpm1, rpm2) = rpm
	rpm1 = 2*math.pi*rpm1/60
	rpm2 = 2*math.pi*rpm2/60
	row = []
	(goal_x, goal_y) = goal
	graph[source]['visited'] = True
	graph[source]['neighbour'] = [source[0], source[1], 0.0, 0.0, 0.0, 0.0]
	num_nodes_visited = 1
	graph[source]['g'] = 0
	queue = []
	current = (source[0], source[1], theta)
	queue_distance = calculate_distance(goal, current)+graph[source]['g']
	heappush(queue, (queue_distance, current))
	action_space = [(0, rpm1),(rpm1, 0),(rpm1, rpm1),(0,rpm2), (rpm2,0), (rpm2,rpm2),(rpm1,rpm2), (rpm2,rpm1)]

	print("Exploring the nodes...")
	while (len(queue) != 0):
		current = heappop(queue)[1]
		crt = list(current)
		if current[0] >= 1110:
			current[0] = 1109
		if current[1] >= 1010:
			current[1] = 1009
		current = tuple(crt)	
		if goal_reached(current,goal) ==  True:
			print("Goal reached")
			# if row:
			# 	x,y = zip(*row)
			# 	plt.plot(x,y,'y.')
			# 	plt.pause(0.01)
			break
		for i,j in action_space:
			# print(i,j)
			params, key = find_neighbour(current,i,j)
			neighbour = (params[0], params[1], params[2])
			# print(params)
			# neighbour = (abs(current[0]+i), abs(current[1]+j))
			lst = list(key)
			if lst[0] >=1110:
				lst[0] = 1109
			if lst[1] >=1010:
				lst[1] = 1009
			key = tuple(lst)
			if graph[(key)]['valid'] == True:
				distance = calculate_distance(current, neighbour)
				if graph[key]['visited'] == False:
					graph[key]['visited'] = True
					# row.append([neighbour[0], neighbour[1]])
					# print(params)
					# x,y = zip(*row)
					# if (num_nodes_visited) % 1000 == 0:
					# 	plt.plot(x,y,'y.')
					# 	del row[:]
					# 	# row.clear()
					# 	plt.pause(0.01)
						
					num_nodes_visited += 1							
					graph[key]['parent'] = (current[0], current[1])
					graph[key]['neighbour'] = params								
					graph[key]['g'] = graph[graph[key]['parent']]['g'] + distance
					queue_distance = calculate_distance(goal, neighbour)+graph[key]['g']
					heappush(queue, (queue_distance, neighbour))
																
	path = [(current[0], current[1])]
	# path_x = source[0]
	# path_y = source[1]
	parent = (current[0], current[1])
	while parent != source:
		parent = graph[path[len(path)-1]]['parent']
		# print(parent)
		# print(graph[parent]['neighbour'])
		# path_x += graph[parent]['neighbour'][3]
		# path_y += graph[parent]['neighbour'][4]
		# print(path_x, path_y)
		path.append(parent)

	min_distance = (graph[(goal_x,goal_y)]['g'])	
	print("Total Number of Nodes Visited:", num_nodes_visited)  	
	return min_distance, path, graph

def simulate(path_list, graph):
	print("Simulation started")
	rospy.init_node('Motion_command',anonymous=True)
	vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	# pos_pub = rospy.Publisher('/odom', Odometry, queue_size=1000)

	vel_msg = Twist()
	
	for p in path_list:
		node = graph[p]['neighbour']
		v = math.sqrt(node[3]**2 + node[4]**2)
		w = node[5]
		r = rospy.Rate(1)
		vel_msg.linear.x = v/100
		vel_msg.linear.y = 0
		vel_msg.linear.z = 0
		vel_msg.angular.x = 0
		vel_msg.angular.y = 0
		vel_msg.angular.z = w
		t0 = rospy.Time.now().to_sec()
		while not rospy.is_shutdown():
			t1 = rospy.Time.now().to_sec()
			elapsed = t1 - t0
			print("elapsed: ", elapsed)
			if elapsed >= 1	:
				break
			vel_pub.publish(vel_msg)
			print("published velocity: ",vel_msg.linear.x)
			r.sleep()
		vel_msg.linear.x = 0.0
		vel_msg.angular.z = 0.0
		vel_pub.publish(vel_msg)
	# rospy.spin()

if __name__ == "__main__":
	
	x1,y1 = raw_input("Enter start point, with a space between x and y coordinates of start: ").split()
	x1 = int(x1)
	y1 = int(y1)
	start = (x1,y1)
	if x1 >= 1110 or x1 < 0 or y1 >= 1010 or y1 <0:
		print("Invalid start state, exiting")
		sys.exit(0)
	p,q = raw_input("Enter goal point, with a space between x and y coordinates of goal: ").split()
	p = int(p)
	q = int(q)
	goal = (p,q)
	if p >= 1110 or p < 0 or q >= 1010 or q <0:
		print("Invalid Goal state, exiting")
		sys.exit(0)

	rpm1, rpm2 = raw_input("Enter the two RPMS for the wheels: ").split()
	rpm = (int(rpm1), int(rpm2))

	g = create_graph()

	points = [x for x in g.keys() if not (g[x]['valid'])]

	x = [i[0] for i in points]
	y = [i[1] for i in points]
	for i in points:
		if x1 == i[0] and y1 == i[1]:
			print("Start point inside obstacle, exiting")
			sys.exit(0)
		if p == i[0] and q == i[1]:
			print("Goal point inside obstacle, exiting")
			sys.exit(0)
	
	# plt.xlim(right=1110)
	# plt.ylim(top=1010)	
	# plt.plot(x,y, 'k.')
	# plt.plot(x1,y1,'xr')
	# plt.plot(p,q,'xg') 
	min_distance, path, final_graph = astar(g, start, goal, rpm)
	path = path[::-1]

	# print(path)
	# x = [i[0] for i in path]
	# y = [i[1] for i in path]
	# plt.plot(x,y, 'g-')
	# plt.show()
	try:
	   #Testing our function
	   simulate(path, final_graph)
	except rospy.ROSInterruptException: pass
	
	# simulate(path, final_graph)
	
	#print("Minimum Distance from start to goal:", min_distance)
	
	




import math
import json
import random

from src.Obstacle import Obstacle
from src.Robot import Robot
from src.ObstaclesDeteccion import ObstaclesDeteccion
from src.RRT import RRT


class Enviroment(object):
	"""docstring for Enviroment"""
	def __init__(self,width,height,tr):

		self.tr=tr
		self.width = width
		self.height =height

		self.delta = 150 #delta inicial, despues se ajusta
		self.ratio= 100
		self.total_angle_options = 4
		self.steps=1
		self.length_sample= 200

		self.robot = Robot(50,70,100,140,math.pi/2,tr,self.width,self.height)
		with open("./src/obstacles/Anaqueles.json") as json_file:
			temp = json.load(json_file)
		self.obstacles =[Obstacle(obs,100,140,self.tr) for obs in temp]
		self.create_free_points(n=1000) #ajustable
		self.deteccionObstacles = []
		for i in range(self.tr.position_plane_total[0]):
			temp = []
			for j in range(self.tr.position_plane_total[0]):
				temp.append(ObstaclesDeteccion(self.tr))
			self.deteccionObstacles.append(temp[:])

		self.RRT = RRT(random.sample(self.free_points,self.length_sample),self.total_angle_options,self.ratio,self.steps,self.delta,self.tr) #ajustable

		self.RRT.run(self.obstacles,self.robot.x,self.robot.y)
		self.path = self.RRT.best_path(0,len(self.RRT.coords)-1)
		self.reference_path = 0
		self.total_path = len(self.path)

		self.actual_path = []
		
		if self.path[self.reference_path][2] == False:
			self.actual_path=self.RRT.paths[self.path[self.reference_path][1]]
		else:
			
			self.actual_path=reversed(self.RRT.paths[self.path[self.reference_path][1]])
		
		self.reference_actual_dubins=0
		self.total_actual_dubins=len(self.RRT.paths[self.path[self.reference_path][1]])
		self.robot.actual_node =self.path[self.reference_path][0]
		
	def random_point(self):
		return random.uniform(0,self.width),random.uniform(0,self.height)


	def create_free_points(self, n=100,add = False):
		#Crea puntos aleatorios sin validar aun
		if add == False:
			self.free_points = []
		for _ in range(n):
			x_temp,y_temp = self.random_point()
			var = False
			for obs in self.obstacles:
				var=obs.containsThePoint(x_temp,y_temp)
				if var:
					break
			if var==False:
				self.free_points.append((x_temp,y_temp))

	#def path_is_free(self,path):

	def update_obstacles_actual_coords(self):
		if self.tr.position_plane_ant[1]!=self.tr.position_plane[1] or self.tr.position_plane_ant[0]!=self.tr.position_plane[0]:
			for obstacle in self.obstacles:
				obstacle.update_actual_coords()


	def plot_left(self, window ,extended = False):
		from pygame.draw import polygon
		from shapely.geometry import Polygon

		
		if self.reference_actual_dubins!=self.total_actual_dubins:
			self.robot.rotation_angles(self.actual_path[self.reference_actual_dubins][2])
			self.robot.move_to(self.actual_path[self.reference_actual_dubins][0],self.actual_path[self.reference_actual_dubins][1])
			self.reference_actual_dubins+=1
		else:
			self.reference_path +=1
			if self.reference_path != self.total_path:
				self.reference_actual_dubins=0
				self.total_actual_dubins=len(self.RRT.paths[self.path[self.reference_path][1]])
				self.robot.actual_node = self.path[self.reference_path][0]
				if self.path[self.reference_path][2]==False:
					self.actual_path=self.RRT.paths[self.path[self.reference_path][1]]
				else:
					self.actual_path=self.RRT.paths[self.path[self.reference_path][1]].reverse()
			else:
				self.path = self.RRT.best_path(self.robot.actual_node,random.randint(0,len(self.RRT.coords)))
				self.reference_path = self.robot.actual_node
				self.reference_actual_dubins=0
				self.total_path = len(self.path)
				self.actual_path = []
				self.total_actual_dubins=len(self.RRT.paths[self.path[self.reference_path][1]])
			
				if self.path[self.reference_path][2] == False:
					self.actual_path=self.RRT.paths[self.path[self.reference_path][1]]
				else:
					
					self.actual_path=reversed(self.RRT.paths[self.path[self.reference_path][1]])
		

		for obstacle in self.obstacles:
			temp = []

			if extended:
				p1 = Polygon(self.tr.get_actual_rectangle())
				p2 = Polygon(obstacle.extendedCorners)
				if p1.intersects(p2):
					polygon(window,"blue",obstacle.extendedCornersPlot)
			p1 = Polygon(self.tr.get_actual_rectangle())
			p2 = Polygon(obstacle.corners)
			if p1.intersects(p2):
				polygon(window,"green",obstacle.cornersPlot)
		
		#robot
		self.robot.plot_left(window)

	def plot_right(self, window):
		from pygame.draw import circle

		if self.robot.stop_scan == False:
			temp1,temp2 = self.robot.rotation_scan(self.obstacles)
			self.deteccionObstacles[self.tr.position_plane[0]][self.tr.position_plane[1]].addLine(temp1)
			self.deteccionObstacles[self.tr.position_plane[0]][self.tr.position_plane[1]].addLineExtended(temp2)
		self.deteccionObstacles[self.tr.position_plane[0]][self.tr.position_plane[1]].plot(window)
		self.robot.plot_right(window)

		for c in self.RRT.coords_pĺot:
			circle(window,"white",c,2 )
		self.update_obstacles_actual_coords()
		self.RRT.coords_pĺot = self.tr.array_to_actual_coords(self.RRT.coords_pĺot_help)




			






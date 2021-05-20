

import math
import json
import random

from src.Obstacle import Obstacle
from src.Robot import Robot
from src.ObstaclesDeteccion import ObstaclesDeteccion

import dubins

class Enviroment(object):
	"""docstring for Enviroment"""
	def __init__(self,width,height,tr):

		self.tr=tr
		self.width = width
		self.height =height

		self.delta = 50 #delta inicial, despues se ajusta


		self.robot = Robot(50,70,100,140,math.pi/2,tr,self.width,self.height)
		with open("./src/obstacles/Anaqueles.json") as json_file:
			temp = json.load(json_file)
		self.obstacles =[Obstacle(obs,100,140,self.tr) for obs in temp]

		self.free_points = self.create_free_points()

		self.deteccionObstacles = []
		for i in range(self.tr.position_plane_total[0]):
			temp = []
			for j in range(self.tr.position_plane_total[0]):
				temp.append(ObstaclesDeteccion(self.tr))
			self.deteccionObstacles.append(temp[:])




	def random_point(self):
		return random.uniform(0,self.width),random.uniform(0,self.height)


	def create_free_points(self, n=100,add = False):
		#Crea puntos aleatorios sin validar aun
		if add == False:
			self.free_points = []
		for _ in range(n):
			x_temp,y_temp = self.random_point()
			self.free_points.append((x_temp,y_temp))

	def maping(self):
		theta_next = 0 #prioridad




	#def path_is_free(self,path):

	def update_obstacles_actual_coords(self):
		if self.tr.position_plane_ant[1]!=self.tr.position_plane[1] or self.tr.position_plane_ant[0]!=self.tr.position_plane[0]:
			for obstacle in self.obstacles:
				obstacle.update_actual_coords()


	def plot_left(self, window ,extended = False):
		from pygame.draw import polygon
		from shapely.geometry import Polygon


		
		self.update_obstacles_actual_coords()

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

		if self.robot.stop_scan == False:
			temp1,temp2 = self.robot.rotation_scan(self.obstacles)
			self.deteccionObstacles[self.tr.position_plane[0]][self.tr.position_plane[1]].addLine(temp1)
			self.deteccionObstacles[self.tr.position_plane[0]][self.tr.position_plane[1]].addLineExtended(temp2)
		self.deteccionObstacles[self.tr.position_plane[0]][self.tr.position_plane[1]].plot(window)
		self.robot.plot_right(window)




			






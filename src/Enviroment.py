

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

		self.delta = 70 #delta inicial, despues se ajusta
		self.ratio= 20
		self.steps=1
		self.length_sample= 10000
		

		self.roots=[[50,70],[width-50,70],[width-50,height-70],[50,height-70]]
		


		self.robot = Robot(50,70,100,140,math.pi/2,tr,self.width,self.height)
		with open("./src/obstacles/Anaqueles.json") as json_file:
			temp = json.load(json_file)
		self.obstacles =[Obstacle(obs,100,140,self.tr) for obs in temp]

		self.create_free_points(n=self.length_sample) #ajustable

		if len(self.free_points)< int(0.5*self.length_sample):
			self.create_free_points(n=int(self.length_sample/2),add=True) #ajustable


		self.deteccionObstacles = []
		for i in range(self.tr.position_plane_total[0]):
			temp = []
			for j in range(self.tr.position_plane_total[0]):
				temp.append(ObstaclesDeteccion(self.tr))
			self.deteccionObstacles.append(temp[:])

		self.RRT = RRT(self.free_points,self.ratio,self.steps,self.delta,self.tr) #ajustable

		for c in self.roots:
			self.RRT.init_trees(c[0],c[1])

		
		self.RRT.run(self.obstacles)
		self.RRT.conectedTree(self.obstacles)
		self.RRT.update_coords()


		self.index_points_paint = 0
		self.index_lines_paint = 0

		
	def random_point(self):
		return random.uniform(50,self.width-50),random.uniform(70,self.height-70)


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

	
	def update_obstacles_actual_coords(self):
		if self.tr.position_plane_ant[1]!=self.tr.position_plane[1] or self.tr.position_plane_ant[0]!=self.tr.position_plane[0]:
			for obstacle in self.obstacles:
				obstacle.update_actual_coords()


	def plot_left(self, window ,extended = False):
		from pygame.draw import polygon
		from shapely.geometry import Polygon
	
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
		self.robot.plot_left(window,bg=False)

	def plot_right(self, window, Tree_steps=True):
		from pygame.draw import circle,line

		if Tree_steps:
			if self.index_points_paint != len(self.RRT.coords_pĺot):
				self.index_points_paint+=1
			if self.index_lines_paint != len(self.RRT.coords_pĺot_pairs):
				self.index_lines_paint+=1



			for c in range(self.index_points_paint):
				circle(window,"white",self.RRT.coords_pĺot[c],2)
			for c in range(self.index_lines_paint):
				line(window,"yellow",self.RRT.coords_pĺot_pairs[c-1][0],self.RRT.coords_pĺot_pairs[c-1][1])
		else:
			for c in range(len(self.RRT.coords_pĺot)):
				circle(window,"white",self.RRT.coords_pĺot[c],2 )
			for c in range(len(self.RRT.coords_pĺot_pairs)):
				line(window,"yellow",self.RRT.coords_pĺot_pairs[c-1][0],self.RRT.coords_pĺot_pairs[c-1][1])

		
		if self.robot.stop_scan == False:
			temp1,temp2 = self.robot.rotation_scan(self.obstacles)
			self.deteccionObstacles[self.tr.position_plane[0]][self.tr.position_plane[1]].addLine(temp1)
			self.deteccionObstacles[self.tr.position_plane[0]][self.tr.position_plane[1]].addLineExtended(temp2)
		self.deteccionObstacles[self.tr.position_plane[0]][self.tr.position_plane[1]].plot(window)
		self.robot.plot_right(window)


		self.update_obstacles_actual_coords()
		self.RRT.coords_pĺot = self.tr.array_to_actual_coords(self.RRT.coords_pĺot_help)
		self.RRT.coords_pĺot_pairs = self.tr.array_lines_to_actual_coords(self.RRT.coords_pĺot_help_pair)

	def change_cam(self,cam_auto):
		self.robot.cam_auto = cam_auto

	def change_plane(self,plane):
		if not self.robot.cam_auto:
			self.tr.position_plane_ant = self.tr.position_plane[:]
			self.tr.position_plane = plane





			






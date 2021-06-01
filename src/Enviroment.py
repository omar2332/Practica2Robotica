

import math
import json
import random
import tqdm

from src.Obstacle import Obstacle
from src.Robot import Robot
from src.ObstaclesDeteccion import ObstaclesDeteccion
from src.RRT import RRT
from src.space3D import space3D


class Enviroment(object):
	"""docstring for Enviroment"""
	def __init__(self,width,height,tr):


		self.tr=tr
		self.width = width
		self.height =height

		self.delta = 150 #delta inicial, despues se ajusta
		self.length_sample= 9000
		

		self.roots=[[50,70],[width-50,70],[width-50,height-70],[50,height-70]]


		#opciones
		self.animated_CE = False # Para la animacion del espacio de configuracion.
		self.animate_Tree = False

		#Van de la mano
		self.start_path = True
		self.animate_robot=True


		self.generate_CE = False	# Para generar el espacio de configuracion o solo cargarlo

		self.robot = Robot(50,70,100,140,math.pi/2,tr,self.width,self.height)
		self.obstacles=self.load_obstacles()
		self.points_free_obstacles_sample = []

		if self.generate_CE: 
			self.generate_configuration_space()
		else:
			self.load_configuration_space()
			self.load_Tree_Sample()


		#Analisis Obstacles
		if self.animated_CE:
			self.index_obstacle = 0
			self.total_obstacles = len(self.obstacles)
			self.index_points = 0
			self.index_total_points = len(self.obstacles[self.index_obstacle].coords_3D)
			self.index_point = 0
			self.total_actual_points = len(self.obstacles[self.index_obstacle].coords_3D[self.index_points])


		
		self.create_free_points(n=self.length_sample) #ajustable

		

		if len(self.free_points)< int(0.5*self.length_sample):
			self.create_free_points(n=int(self.length_sample/2),add=True) #ajustable

		random.shuffle(self.points_free_obstacles_sample)
		self.rrt = RRT(self.free_points,self.delta,self.tr)
		self.node_entrace =self.rrt.add_node_tree(50,70,math.pi/2)-1
		self.rrt.run(self.obstacles)

		self.node_obstacles_ref_start = self.rrt.reference-1

		self.rrt.append_Tree(self.points_free_obstacles_sample,self.obstacles)
		self.node_obstacles_ref_end = self.rrt.reference-1


		self.node_exit =self.rrt.add_node_tree(3430,1610,0)-1
		self.rrt.save_Tree_Coords()
		self.rrt.update_coords()
	

		if self.start_path:
			self.start_path_robot()

		#generacion del arbol
		if self.animate_Tree:
			self.index_node = 0
			self.total_nodes = len(self.rrt.coords_pĺot)
			self.index_edge = 0
			self.total_edges = len(self.rrt.coords_pĺot_pairs)

		#modificar
		self.deteccionObstacles = []
		for i in range(self.tr.position_plane_total[0]):
			temp = []
			for j in range(self.tr.position_plane_total[0]):
				temp.append(ObstaclesDeteccion(self.tr))
			self.deteccionObstacles.append(temp[:])

		self.space3D = space3D(self.obstacles,self.rrt.coords,self.rrt.pairs)
		self.space3D.plot_config_space_and_tree()


	def start_path_robot(self,returned = False):
		if not returned:
			node_end = random.randint(self.node_obstacles_ref_start,self.node_obstacles_ref_end)
		else:
			if self.robot.state:
				node_end = self.node_entrace
			else:
				node_end = self.node_exit

		path_nodes = self.rrt.best_path(self.robot.actual_node,node_end)
		pairs = []
		

		for i in range(len(path_nodes)-1):
			pairs.append( (path_nodes[i],path_nodes[i+1]))

		self.robot.start_path(pairs)
		self.robot.create_movement(self.rrt.coords)



		

	def animated_tree(self):
		if self.index_node < self.total_nodes:
			self.index_node+=1
		if self.index_node != 0:
			if self.index_edge< self.total_edges:
				self.index_edge+=1


	#revisar
	def animated_configuration_space(self):
		if self.index_obstacle<self.total_obstacles:
			if self.index_points <self.index_total_points-1:
				if self.index_point<self.total_actual_points-1:
					self.robot.move_robot_by_coords_3d(self.obstacles[self.index_obstacle].coords_3D[self.index_points][self.index_point])
					self.index_point+=1
				else:
					
					self.index_points+=1
					self.index_point=0
					self.total_actual_points = len(self.obstacles[self.index_obstacle].coords_3D[self.index_points])
					
			else:
				self.index_obstacle+=1
				self.index_points=0
				if self.index_obstacle<self.total_obstacles:
					self.index_total_points = len(self.obstacles[self.index_obstacle].coords_3D)
					
		else:
			self.animated_CE=False
			self.robot.reset()

	def animate_movement_robot(self):
		if self.robot.move_by_tree():
			if self.robot.next_pairs():

				self.robot.change_returned()
				if self.robot.returned:
					self.robot.change_state()
					self.start_path_robot()
				else:
					self.start_path_robot(returned=True)
					
			else:
				self.robot.create_movement(self.rrt.coords)




	def load_obstacles(self):
		with open("./src/obstacles/Anaqueles.json") as json_file:
			temp = json.load(json_file)
		obstacles =[Obstacle(obs,100,140,self.tr) for obs in temp]
		return obstacles

	def generate_configuration_space(self):
		print("Generando Espacio de Configuracion 3D, Por favor Espere")
		print("Para las vertices del robot")
		for obs in tqdm.tqdm(self.obstacles):
			for i in range(4):#4 lados del robot
				for point in obs.corners:
					data=self.robot.rotation_robot_by_edge(i,point[0],point[1],self.obstacles,obs)

					if len(data):
						self.points_free_obstacles_sample+= random.sample(data,int(len(data)/2))
						obs.coords_3D.append(data[:])

		print("Para las esquinas del robot")
		for obs in tqdm.tqdm(self.obstacles):
			for i in range(4):#4 esquinas del robot
				for points in obs.straight_division:
					for p in points:
						data = self.robot.rotation_robot_by_corner(i,p[0],p[1],self.obstacles,obs)
						if len(data)!=0:
							self.points_free_obstacles_sample+= random.sample(data,int(2*len(data)/3))
							obs.coords_3D.append(data[:])
						
			obs.save_coords_3D()
		self.save_Tree_Sample()
		self.robot.reset()

	def load_configuration_space(self):
		print("Cargando Espacio de Configuracion 3D, Por favor Espere")
		for obs in tqdm.tqdm(self.obstacles):
			obs.load_configuration_space()
		
	def random_point(self):
		return random.uniform(50,self.width-50),random.uniform(70,self.height-70),random.uniform(0,2*math.pi) 
	def create_free_points(self, n=100,add = False):
		#Crea puntos aleatorios sin validar aun
		if add == False:
			self.free_points = []
		for _ in range(n):
			x_temp,y_temp,angle = self.random_point()
			var = False
			for obs in self.obstacles:
				var=obs.containsThePoint(x_temp,y_temp)
				if var:
					break
			if var==False:
				self.free_points.append((x_temp,y_temp,angle))

	def update_obstacles_actual_coords(self):
		if self.tr.position_plane_ant[1]!=self.tr.position_plane[1] or self.tr.position_plane_ant[0]!=self.tr.position_plane[0]:
			for obstacle in self.obstacles:
				obstacle.update_actual_coords()

	def plot_left(self, window):
		from pygame.draw import polygon
		from shapely.geometry import Polygon


		if self.animate_robot:
			self.animate_movement_robot()


		for obstacle in self.obstacles:
			temp = []
			p1 = Polygon(self.tr.get_actual_rectangle())
			p2 = Polygon(obstacle.corners)
			if p1.intersects(p2):
				polygon(window,"green",obstacle.cornersPlot)
		
		if self.animated_CE:
			self.animated_configuration_space()
		self.robot.plot_left(window,bg=False)

	def plot_right(self, window):
		self.rrt.update_coords()
		from pygame.draw import circle,line

		if self.robot.stop_scan == False:
			temp1 = self.robot.rotation_scan(self.obstacles)
			self.deteccionObstacles[self.tr.position_plane[0]][self.tr.position_plane[1]].addLine(temp1)
		if not self.animate_Tree:
			for coords in self.rrt.coords_pĺot:
				circle(window,"blue",coords,2)
			for l in self.rrt.coords_pĺot_pairs:
				line(window,"yellow", l[0],l[1])
		else:
			for c in range(self.index_node):
				circle(window,"blue",self.rrt.coords_pĺot[c],2)
			for l in range(self.index_edge):
				line(window,"yellow", self.rrt.coords_pĺot_pairs[l][0],self.rrt.coords_pĺot_pairs[l][1])
			self.animated_tree()

		self.update_obstacles_actual_coords()

		self.deteccionObstacles[self.tr.position_plane[0]][self.tr.position_plane[1]].plot(window)
		self.robot.plot_right(window)
		
		
	def change_cam(self,cam_auto):
		self.robot.cam_auto = cam_auto

	def change_plane(self,plane):
		if not self.robot.cam_auto:
			self.tr.position_plane_ant = self.tr.position_plane[:]
			self.tr.position_plane = plane

	def save_Tree_Sample(self):
		import pickle
		outfile = open("./src/Tree/sample_obstacles",'wb')
		pickle.dump(self.points_free_obstacles_sample,outfile)
		outfile.close()
	def load_Tree_Sample(self):
		import pickle
		infile = open("./src/Tree/sample_obstacles",'rb')
		self.points_free_obstacles_sample=pickle.load(infile)
		infile.close()





			






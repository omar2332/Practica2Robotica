import networkx as nx


import math
import tqdm


class RRT(object):
	"""docstring for RRT"""
	def __init__(self,sample,delta,tr):

		
		self.sample = sample
		self.reference = 0
		

		self.Tree = nx.Graph()

		self.tr =tr
		self.delta = delta
		self.coords = []
		self.pairs = []

		#Margen
		self.width_env = self.tr.width_env-50
		self.height_env =self.tr.height_env-70


		#Points
		self.coords_pĺot=[]
		self.coords_pĺot_help= [] #descargar
		#Lines
		self.coords_pĺot_pairs=[]
		self.coords_pĺot_help_pair= [] #descargar

	def save_Tree_Coords(self):
		import pickle
		outfile = open("./src/Tree/coords",'wb')
		pickle.dump(self.coords,outfile)
		outfile = open("./src/Tree/pairs",'wb')
		pickle.dump(self.pairs,outfile)
		outfile.close()

	def set_new_sample(self,new_sample):
		self.sample = new_sample

	def reset(self,robot,delta):
		self.reference = 0
		self.Tree = nx.Graph()
		self.Tree.add_node(self.reference)
		self.reference +=1
		self.delta = delta
		self.coords = []
		self.pairs = []
		self.coords_pĺot=[]
		self.coords_pĺot_help= []
		
		self.coords_pĺot_pairs=[]
		self.coords_pĺot_help_pair= []

	def nearest_node(self,x,y,angle,minimun=False,distance=0):
		distances = [self.tr.distance3D(c[0],c[1],c[2],x,y,angle) for c in self.coords]

		if minimun:
			while min(distances) < distance:
				distances.remove(min(distances))
		return distances.index(min(distances))
	

	def intersectionLines(self,line1,line2,lines,two=True):
		var1 = False
		var2 = False
		for l in lines:
			if var1 and var2:
				break
			if line1[0]!=l[0] and line1[1]!=l[0] and line1[0]!=l[1] and line1[1]!=l[1]:
				var1 = self.tr.is_intersection_lines(line1,l)
				if var1:
					var1=True
			if two:
				if line2[0]!=l[0] and line2[1]!=l[0] and line2[0]!=l[1] and line2[1]!=l[1]:
					var2 = self.tr.is_intersection_lines(line2,l)
					if var2:
						var2=True

		return var1,var2


	def add_node_tree(self,x,y,angle):
		self.Tree.add_node(self.reference)
		if self.reference !=0:
			i = self.nearest_node(x,y,angle)
			distance = self.tr.distance3D(x,y,angle,self.coords[i][0],self.coords[i][1],self.coords[i][2])	
			self.coords_pĺot_help_pair.append([(self.coords[i][0],self.coords[i][1]),(x,y)])
			self.Tree.add_edge(i, self.reference, weight=distance)
			self.pairs.append((i, self.reference))

		self.coords.append((x,y,angle))
		self.coords_pĺot_help.append((x,y))
		self.reference +=1
		return self.reference


	def append_Tree(self,sample,obstacles):
		re_sample = []
		for coord in tqdm.tqdm(sample):
			var = True
			i = self.nearest_node(coord[0],coord[1],coord[2])
			distance = self.tr.distance3D(coord[0],coord[1],coord[2],self.coords[i][0],self.coords[i][1],self.coords[i][2])	
			if  math.fabs(coord[2]-self.coords[i][2])> math.pi/2:
				var = False


			if not self.lineIsFree(obstacles,coord[0],coord[1],self.coords[i][0],self.coords[i][1]) and distance<=self.delta and math.fabs(coord[2]-self.coords[i][2]) <= math.pi/2 and var:
				self.Tree.add_node(self.reference)
				self.coords.append((coord[0],coord[1],coord[2]))
				self.coords_pĺot_help.append((coord[0],coord[1]))
				self.coords_pĺot_help_pair.append([(self.coords[i][0],self.coords[i][1]),(coord[0],coord[1])])
				self.Tree.add_edge(i, self.reference, weight=distance)
				self.pairs.append((i, self.reference))
				self.reference +=1
			else:
				re_sample.append(coord)

		if len(re_sample)>=7500:
			self.append_Tree(re_sample,obstacles)

	def run(self,obstacles):
		

		for s in tqdm.tqdm(self.sample):
			var = False
			i = self.nearest_node(s[0],s[1],s[2])
			
			new_x,new_y,alpha = self.tr.deltalize_point_to_ref(self.coords[i][0],self.coords[i][1],self.coords[i][2],s[0],s[1],s[2],self.delta)


			#nearest_obstacle = None
			#distance_obstacle = 20000
			var = False

			for obs in obstacles:
				#distance_temp = obs.distance_point_to_poly(new_x,new_y)
				if obs.containsThePoint(new_x,new_y):
					var =True
					break

				#if distance_obstacle>distance_temp:
				#	distance_obstacle = distance_temp
				#	nearest_obstacle = obs

			"""
			var2 = False
			if distance_obstacle <= self.delta or var == True:
				for label in nearest_obstacle.coords_3D:
					for node in label:
						distance_temp = self.tr.distance3D(self.coords[i][0],self.coords[i][1],self.coords[i][2],node[0],node[1],node[2])
						if distance_temp <= self.delta and distance_temp >= self.delta/2:
							new_x,new_y,alpha = node[0],node[1],node[2]
							var2 =True
							break
					if var2:
						break
			"""

			
			if  alpha < 0 or alpha > 2*math.pi:
				alpha = math.atan2(math.sin(alpha), math.cos(alpha)) + math.pi

				if alpha < math.pi:
					alpha+=math.pi
				else:
					alpha-=math.pi


			
			if math.fabs(alpha-math.pi) < math.radians(5):
				alpha = math.pi
			if math.fabs(alpha-math.pi/2) < math.radians(5):
				alpha = math.pi/2
			if math.fabs(alpha-2*math.pi/3) < math.radians(5):
				alpha = 2*math.pi/3

			if math.fabs(alpha)< math.radians(5):
				alpha= 0

			if math.fabs(alpha-self.coords[i][2])>math.pi/2:
				var = True

			distance = self.tr.distance3D(new_x,new_y,alpha,self.coords[i][0],self.coords[i][1],self.coords[i][2])
			
			if not self.lineIsFree(obstacles,new_x,new_y,self.coords[i][0],self.coords[i][1],extended=True) and new_x>=50 and new_y>=70 and new_x<=self.width_env-50 and new_y<=self.height_env-70 and not var and math.fabs(distance-self.delta)<=5:
				self.Tree.add_node(self.reference)
				self.coords.append((new_x,new_y,alpha))
				self.coords_pĺot_help.append((new_x,new_y))
				self.coords_pĺot_help_pair.append([(self.coords[i][0],self.coords[i][1]),(new_x,new_y)])
				self.Tree.add_edge(i, self.reference, weight=distance)
				self.pairs.append((i, self.reference))
				self.reference +=1
		
		


	def lineIsFree(self,obstacles,x1,y1,x2,y2,extended=False):
		line = [(x1,y1),(x2,y2)]
		var = False
		for obs in obstacles:
			if obs.containsTheLine(line,extended=extended):
				var=True
				break
		return var

	
	def update_coords(self):
		self.coords_pĺot = self.tr.array_to_actual_coords(self.coords_pĺot_help)
		self.coords_pĺot_pairs = self.tr.array_lines_to_actual_coords(self.coords_pĺot_help_pair)
		



	def best_path(self,node_start,node_end):
		path = nx.dijkstra_path(self.Tree,node_start,node_end)
		print(path)
		return path
		






		
		
		
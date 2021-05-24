import networkx as nx
import dubins

import math
import tqdm


class RRT(object):
	"""docstring for RRT"""
	def __init__(self,sample,ratio ,steps,alpha,tr):

		
		self.ratio = ratio
		self.sample = sample
		self.steps = steps
		self.reference = 0
		self.Tree = nx.Graph()
		self.tr =tr
		self.alpha = alpha
		self.coords = []

		#Margen
		self.width_env = self.tr.width_env-50
		self.height_env =self.tr.height_env-70

		#Points
		self.coords_pĺot=[]
		self.coords_pĺot_help= []
		#Lines
		self.coords_pĺot_pairs=[]
		self.coords_pĺot_help_pair= []
		#nodes refs
		self.nodes_refs = []

	def set_new_sample(self,new_sample):
		self.sample = new_sample

	def reset(self,robot,alpha):
		self.reference = 0
		self.Tree = nx.Graph()
		self.Tree.add_node(self.reference)
		self.reference +=1
		self.alpha = alpha
		self.coords = []
		self.coords_pĺot=[]
		self.coords_pĺot_help= []
		#Lines
		self.coords_pĺot_pairs=[]
		self.coords_pĺot_help_pair= []

	def nearest_node(self,x,y):
		distances = [self.tr.distance(c[0],c[1],x,y) for c in self.coords]
		return distances.index(min(distances))
	
	def print_paths(self):
		for path in list(self.paths.keys()):
			print(path)
			print(self.paths[path])

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


	def init_trees(self,x,y):
		self.coords.append((x,y))
		self.coords_pĺot_help.append((x,y))
		self.Tree.add_node(self.reference)
		#ajustar
		self.nodes_refs.append([self.reference])


		self.reference +=1
		


	def run(self,obstacles):
		

		for s in tqdm.tqdm(self.sample):
			var = False
			i = self.nearest_node(s[0],s[1])
			
			new_x,new_y = self.tr.deltalize_point_to_ref(self.coords[i][0],self.coords[i][1],s[0],s[1],self.alpha)

			j = self.nearest_node(new_x,new_y)

			#if i!=j:
			#	i=j

			"""
			
			if i!=j:
				var1,var2 =self.intersectionLines([(self.coords[i][0],self.coords[i][1]),(new_x,new_y)],[(self.coords[j][0],self.coords[j][1]),(new_x,new_y)],self.coords_pĺot_help_pair)
				
				
				if not var1 and not var2:
					i=j
				if var1 and not var2:
					i=j
				if not var1 and var2:
					i=i
				if var1 and var2:
					var=True
			else:
				var1,_ =self.intersectionLines([(self.coords[i][0],self.coords[i][1]),(new_x,new_y)],[],self.coords_pĺot_help_pair,two=False)
				if var1:
					var=True

			"""
			#var,_ =self.intersectionLines([(self.coords[i][0],self.coords[i][1]),(new_x,new_y)],[],self.coords_pĺot_help_pair,two=False)

			distance = self.tr.distance(new_x,new_y,self.coords[i][0],self.coords[i][1])	
			if not self.lineIsFree(obstacles,new_x,new_y,self.coords[i][0],self.coords[i][1]) and new_x>=50 and new_y>=70 and new_x<=self.width_env and new_y<=self.height_env and var==False:
				self.Tree.add_node(self.reference)
				self.coords.append((new_x,new_y))
				self.coords_pĺot_help.append((new_x,new_y))
				self.coords_pĺot_help_pair.append([(self.coords[i][0],self.coords[i][1]),(new_x,new_y)])
				self.Tree.add_edge(i, self.reference, weight=distance)

				
				for t in range(len(self.nodes_refs)):
					for k in range(len(self.nodes_refs[t])):
						if i==self.nodes_refs[t][k]:
							self.nodes_refs[t].append(self.reference)
				self.reference +=1

		#print(self.nodes_refs)

	def lineIsFree(self,obstacles,x1,y1,x2,y2):
		line = [(x1,y1),(x2,y2)]
		var = False
		for obs in obstacles:
			if obs.containsTheLine(line):
				var=True
		return var


	def conectedTree(self,obstacles):
		from itertools import combinations
		temp =self.reference

		
		per = list(combinations(self.nodes_refs, 2))
		#print(per)

		for ref in tqdm.tqdm(per):
			for i in ref[0]:
				if self.Tree.degree(i) <= 1:	
					for j in ref[1]:
						if self.Tree.degree(i) <= 1:
							x1=self.coords[i][0]
							y1=self.coords[i][1]
							x2=self.coords[j][0]
							y2=self.coords[j][1]
							d =self.tr.distance(x1,y1,x2,y2)
							if d <= self.alpha and d >=self.alpha/2:
								var = False
								for obs in obstacles:
									if obs.containsTheLine([(x1,y1),(x2,y2)]):
										var=True
										break
								if not var:
									self.Tree.add_edge(i, j, weight=d)
									self.coords_pĺot_help_pair.append([(x1,y1),(x2,y2)])

	
	def update_coords(self):
		self.coords_pĺot = self.tr.array_to_actual_coords(self.coords_pĺot_help)
		self.coords_pĺot_pairs = self.tr.array_lines_to_actual_coords(self.coords_pĺot_help_pair)
		print(len(self.coords_pĺot))



	def best_path(self,node_start,node_end):
		
		path = nx.dijkstra_path(self.Tree,node_start,node_end)

		path_tuples = []
		for i in range(len(path)-1):

			try:
				temp = self.paths[str(path[i])+"-"+str(path[i+1]) ]
				path_tuples.append((path[i+1],str(path[i])+"-"+str(path[i+1]),False))
			except Exception as e:
				temp = self.paths[str(path[i+1])+"-"+str(path[i]) ]
				path_tuples.append((path[i+1],str(path[i+1])+"-"+str(path[i]),True))

		return path_tuples






		
		
		
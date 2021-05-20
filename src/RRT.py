import networkx as nx
import dubins

import math
import tqdm


class RRT(object):
	"""docstring for RRT"""
	def __init__(self,sample,total_angle_options,ratio ,steps,alpha,tr):

		self.total_angle_options = total_angle_options
		self.angle_options = [ (i*2*math.pi/total_angle_options) for i in range(self.total_angle_options) ]
		self.ratio = ratio
		self.sample = sample
		self.steps = steps
		self.reference = 0
		self.Tree = nx.Graph()
		self.Tree.add_node(self.reference)
		self.reference +=1
		self.tr =tr
		self.alpha = alpha
		self.coords = []
		self.coords_pĺot=[]
		self.coords_pĺot_help= []
		self.paths= {}

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
		self.paths= {}

	def nearest_node(self,x,y):
		distances = [self.tr.distance(c[0],c[1],x,y) for c in self.coords]
		return distances.index(min(distances))

	def best_path_two_nodes(self,coords1,x,y,obstacles):#Necesito optimizarlo
		temp = []
		distances = []
		angle_options = []

		for a in self.angle_options:
			path = dubins.shortest_path(coords1,(x,y,a), self.ratio)
			configurations, dis = path.sample_many(self.steps)
			is_option_path = True
			for obs in obstacles:
				if obs.array_points_contains(configurations): #falta agregar que no se salga del marco xd
					is_option_path = False
					break
			if is_option_path:
				temp.append(configurations[:])
				angle_options.append(a)
				distances.append(dis[-1])
		try:
			k = distances.index(min(distances))
		except Exception as e:
			k = -1
		if k!=-1:
			return temp[k],angle_options[k],distances[k]
		else:
			return [],-1,0
	
	def print_paths(self):
		for path in list(self.paths.keys()):
			print(path)
			print(self.paths[path])

	def run(self,obstacles,x,y,angle=math.pi/2,first=True):
		ant= len(self.coords)
		if first:
			self.coords.append((x,y,angle))
			self.coords_pĺot_help.append((x,y))
		for s in tqdm.tqdm(self.sample):
			
			i = self.nearest_node(s[0],s[1])
			new_x,new_y = self.tr.deltalize_point_to_ref(self.coords[i][0],self.coords[i][1],s[0],s[1],self.alpha)
			#print("actual",s[0],s[1])
			#print("mas cercano",self.coords[i][0],self.coords[i][1])
			#print("estandarizado",new_x,new_y)


			path,best_angle,distance = self.best_path_two_nodes(self.coords[i],new_x,new_y,obstacles)
				
			if len(path)!= 0:
				self.Tree.add_node(self.reference)
				self.coords.append((new_x,new_y,best_angle))
				self.coords_pĺot_help.append((new_x,new_y))
				self.Tree.add_edge(i, self.reference, weight=distance)
				self.paths[str(i) +"-"+str(self.reference)]= path[:]
				self.reference +=1
					
		self.coords_pĺot = self.tr.array_to_actual_coords(self.coords_pĺot_help)
		#if math.fabs(len(self.coords) - len(self.sample)-ant)< 5 #total de nodos agregados, aqui queria hacer un nuevo resample y con recursividad

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






		
		
		
class ObstaclesDeteccion(object):
	"""docstring for obstacles_deteccion"""
	def __init__(self,tr):
		self.tr = tr
		self.points_obstacles = []
		self.points_extended_obstacles = []



	def addLine(self,l):
		if len(l)!=0:
			self.points_obstacles.append(self.tr.array_to_actual_coords(l))

	def addLineExtended(self,l):
		if len(l)!=0:
			self.points_extended_obstacles.append(self.tr.array_to_actual_coords(l))

	def plot(self,win,normals = False):
		from pygame.draw import line
		if normals:
			for l in self.points_obstacles:
				line(win,"green",l[0],l[1] )
		for l in self.points_extended_obstacles:
			line(win,"blue",l[0],l[1] )

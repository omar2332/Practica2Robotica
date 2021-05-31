class ObstaclesDeteccion(object):
	"""docstring for obstacles_deteccion"""
	def __init__(self,tr):
		self.tr = tr
		self.points_obstacles = []
		self.stop = False



	
	def addLine(self,l):
		if len(l)!=0 and not self.stop:
			self.points_obstacles.append(self.tr.array_to_actual_coords(l))


	def plot(self,win):
		from pygame.draw import line
		
		for l in self.points_obstacles:
			try:
				line(win,"green",l[0],l[1] )
			except Exception as e:
				pass
			
			

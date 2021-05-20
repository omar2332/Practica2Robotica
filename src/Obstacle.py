from shapely.geometry import Polygon, Point
import json

class Obstacle(object):
	"""docstring for Obstacle"""
	def __init__(self,data,robotWidth,robotHeight,tr):
		self.name= data['name']
		self.corners = data['corners']
		print(self.corners)
		self.cornersPoly= Polygon(self.corners)
		self.tr=tr
		self.robotWidth = robotWidth
		self.robotHeight = robotHeight
		self.minimun_side = min(self.robotWidth/2,self.robotHeight/2)
		self.extendedCorners = self.generateExtendedCorners()
		self.extendedCornersPoly = Polygon(self.extendedCorners)
		self.cornersPlot = self.tr.array_to_actual_coords(self.corners)
		self.extendedCornersPlot = self.tr.array_to_actual_coords(self.extendedCorners)

	def generateExtendedCorners(self):
		extendedCorners = []
		center = list(self.cornersPoly.centroid.coords)[0]
		for coords in self.corners:
			vector = ( coords[0] - center[0] , coords[1]-center[1] )
			if vector[0]<0 and vector[1]<0:
				extendedCorners.append((coords[0] - self.minimun_side, coords[1]-self.minimun_side))
			if vector[0]>0 and vector[1]>0:
				extendedCorners.append((coords[0] + self.minimun_side, coords[1]+self.minimun_side))
			if vector[0]<0 and vector[1]>0:
				extendedCorners.append((coords[0] - self.minimun_side, coords[1]+self.minimun_side))
			if vector[0]>0 and vector[1]<0:
				extendedCorners.append((coords[0] + self.minimun_side, coords[1]-self.minimun_side))
		return extendedCorners

	def containsThePoint(self,x,y):
		point = Point((x,y))
		return self.extendedCornersPoly.contains(point)

	def containsTheLine(self,line):
		from shapely.geometry import LineString
		l1 = LineString( line)
		return self.extendedCornersPoly.intersects(l1)

	def intersectionTheLine(self,line):
		from shapely.geometry import LineString
		l1 = LineString( line)
		return list(self.cornersPoly.intersection(l1).coords),list(self.extendedCornersPoly.intersection(l1).coords)


	def update_actual_coords(self):
		self.cornersPlot = self.tr.array_to_actual_coords(self.corners)
		self.extendedCornersPlot = self.tr.array_to_actual_coords(self.extendedCorners)


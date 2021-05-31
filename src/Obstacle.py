from shapely.geometry import Polygon, Point
import json


class Obstacle(object):
	
	def __init__(self,data,robotWidth,robotHeight,tr):
		self.name= data['name']
		self.corners = data['corners']
		self.cornersPoly= Polygon(self.corners)

		self.tr=tr
		self.robotWidth = robotWidth
		self.robotHeight = robotHeight


		self.minimun_side = min(self.robotWidth/2,self.robotHeight/2)
		self.maximun_side = max(self.robotWidth/2,self.robotHeight/2)
		self.cornersPlot = self.tr.array_to_actual_coords(self.corners)
		self.straight_division=[]
		
		for i in range(len(self.corners)-1):
			d = self.tr.distance(self.corners[i][0],self.corners[i][1],self.corners[i+1][0],self.corners[i+1][1]  )
			num = int(d/self.maximun_side)
			if num ==0:
				num=2
			self.straight_division.append(self.tr.points_in_interval(self.corners[i],self.corners[i+1],num))

		d = self.tr.distance(self.corners[-1][0],self.corners[-1][1],self.corners[0][0],self.corners[0][1])
		num = int(d/self.maximun_side)
		
		if num ==0:
			num=2
		self.straight_division.append(self.tr.points_in_interval(self.corners[-1],self.corners[0],num))	
		
		self.extendedCorners = self.generateExtendedCorners()
		self.extendedCornersPoly = Polygon(self.extendedCorners)



		self.coords_3D = []


	def save_coords_3D(self):
		import pickle
		outfile = open("./src/obstacles_4_degrees/"+self.name,'wb')
		pickle.dump(self.coords_3D,outfile)
		outfile.close()

	def containsThePoint(self,x,y):
		point = Point((x,y))
		return self.extendedCornersPoly.contains(point)

	def containsTheLine(self,line):
		from shapely.geometry import LineString
		l1 = LineString(line)
		return self.cornersPoly.intersects(l1)

	def intersectionTheLine(self,line):
		from shapely.geometry import LineString
		l1 = LineString( line)
		return list(self.cornersPoly.intersection(l1).coords)

	def update_actual_coords(self):
		self.cornersPlot = self.tr.array_to_actual_coords(self.corners)

	def array_points_contains(self,points):
		var = False
		for p in points:
			if self.containsThePoint(p[0],p[1]):
				var=True
				break
		return var

	def load_configuration_space(self):
		import pickle
		infile = open("./src/obstacles_4_degrees/"+self.name,'rb')
		self.coords_3D=pickle.load(infile)
		infile.close()

	def containsThePoly(self,poly):
		import shapely
		intersection = self.cornersPoly.intersection(poly)
		if type(intersection) == shapely.geometry.polygon.Polygon:
			temp = list(intersection.exterior.coords)
			if len(temp)!=0:
				return True
			else:
				return False
		else:
			return False

	def distance_point_to_poly(self,x,y):
		point = Point((x,y))
		return self.cornersPoly.exterior.distance(point)


	def generateExtendedCorners(self):
		extendedCorners = []
		center = list(self.cornersPoly.centroid.coords)[0]
		for coords in self.corners:
			vector = ( coords[0] - center[0] , coords[1]-center[1] )
			if vector[0]<0 and vector[1]<0:
				extendedCorners.append((coords[0] - self.maximun_side, coords[1]-self.maximun_side))
			if vector[0]>0 and vector[1]>0:
				extendedCorners.append((coords[0] + self.maximun_side, coords[1]+self.maximun_side))
			if vector[0]<0 and vector[1]>0:
				extendedCorners.append((coords[0] - self.maximun_side, coords[1]+self.maximun_side))
			if vector[0]>0 and vector[1]<0:
				extendedCorners.append((coords[0] + self.maximun_side, coords[1]-self.maximun_side))
		return extendedCorners
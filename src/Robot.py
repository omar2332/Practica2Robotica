from pygame import Rect
from shapely.geometry import Polygon, Point
import math
from src.Transformations import Transformations
class Robot(object):
	
	def __init__(self,x,y,width,height,angle,tr,width_env, height_env):
		"""
		x,y : Punto inicial del robot
		width,height : Medidas del robot
		angle: Angulo inicial del robot
		width_env, height_env: Variables del ambiente
		tr: Para todas las transformaciones necesarias

		"""
		self.tr= tr #objeto para transformaciones

		#variables de control
		self.x = x
		self.y = y
		self.angle = angle
		self.cam_auto = True 
		#######################
		self.width = width
		self.height = height

		######################################################################
		# x,y estan dentro del plano original de la bodega, y x_plot e y_ploy se encuentran dentro del plano de pygame
		# Esto es para que x,y se puedan trabajar dentro del algorimo si ser afectas o modificadas a la hora de pintar en pygame
		self.x_plot,self.y_plot = self.tr.point_to_actual_coord(self.x,self.y)

		#######################################################################

		self.width_env = width_env
		self.height_env = height_env

		self.reverse = False #para cuando necesite usar la reversa

		self.corners = [
			(self.x+self.width/2,self.y+self.height/2),
			(self.x-self.width/2,self.y+self.height/2),
			(self.x-self.width/2,self.y-self.height/2),
			(self.x+self.width/2,self.y-self.height/2)
		]


		self.distance_center_to_corners = self.tr.distance(self.corners[0][0],self.corners[0][1],self.x,self.y )
		self.corners_angles = self.calculate_initial_angles()
		self.angle_reference1 = math.pi/2 -self.corners_angles[0]
		self.angle_reference2 = self.corners_angles[1] - self.corners_angles[0]
		self.corners_plot = self.tr.array_to_actual_coords(self.corners)

		self.distance_scan = 100
		self.x_scan = self.x
		self.y_scan = self.x+self.distance_scan
		self.x_scan_plot,self.y_scan_plot = self.tr.point_to_actual_coord(self.x_scan,self.y_scan)

		self.theta_scan = self.angle
		self.speed_scan = 10 #numero de saltos entre angulos
		self.stop_scan = False

		self.actual_node = 0
		self.color = "red"




	def rotation_angles(self,theta):
		self.angle = theta
		self.corners_angles=self.configure_angles()
		for i in range(4):
			self.corners[i] = self.tr.point_angle_and_distance(self.x,self.y,self.corners_angles[i],self.distance_center_to_corners)

		self.corners_plot = self.tr.array_to_actual_coords(self.corners)

	def calculate_initial_angles(self):
		theta1 = math.acos( (self.width/2)/self.distance_center_to_corners)
		theta2 = self.tr.angle_two_vectors_same_origin(self.corners[0],self.corners[1],self.x,self.y) + theta1
		theta3 = theta1 + math.pi
		theta4 = theta2 + math.pi
		return [theta1,theta2,theta3,theta4]

	def configure_angles(self):
		theta1 = self.angle -self.angle_reference1
		theta2 =  self.angle_reference2+theta1
		theta3 = theta1 + math.pi
		theta4 = theta2 + math.pi
		return [theta1,theta2,theta3,theta4]

	def rotation_scan(self,obstacles):
		temp1=[]
		temp2=[]
		ref = self.speed_scan*2*math.pi/360

		if math.fabs(self.theta_scan - 2*math.pi)<=0.001:
			self.theta_scan = 0
		else:
			self.theta_scan+=ref
		self.x_scan,self.y_scan =self.tr.point_rotation_to_angles(self.theta_scan,self.x,self.y,self.distance_scan)
		
		self.x_scan_plot,self.y_scan_plot = self.tr.point_to_actual_coord(self.x_scan,self.y_scan)

		for obs in obstacles:
			if obs.containsTheLine([(self.x,self.y),(self.x_scan,self.y_scan)]):
				temp1,temp2 = obs.intersectionTheLine([(self.x,self.y),(self.x_scan,self.y_scan)])
		return temp1,temp2





	def plot_left(self,win,bg=False):
		from pygame.draw import line,circle,polygon
		if bg:
			polygon(win,self.color,self.corners_plot)
		else:
			for i in range(len(self.corners_plot)-1):
				line(win,self.color,(self.corners_plot[i][0],self.corners_plot[i][1]),(self.corners_plot[i+1][0],self.corners_plot[i+1][1]))
			line(win,self.color,(self.corners_plot[-1][0],self.corners_plot[-1][1]),(self.corners_plot[0][0],self.corners_plot[0][1]))
		
			x_temp1,x_temp2 = self.tr.inverse_plane(self.x_plot,self.y_plot)
			circle(win,self.color,(x_temp1,x_temp2),3 )

	def plot_right(self,win):
		from pygame.draw import circle,line
		if self.cam_auto:
			x_temp1,y_temp1 = self.tr.inverse_plane(self.x_plot,self.y_plot)
			x_temp2,y_temp2 = self.tr.inverse_plane(self.x_scan_plot,self.y_scan_plot)


			line(win,self.color,(x_temp1,y_temp1),(x_temp2,y_temp2))
			circle(win,self.color,(x_temp1,y_temp1),3)



	def move_to(self,x,y):
		if x<self.width_env and y<self.height_env and x>=0 and y>=0:
			self.x=x
			self.y=y

			#Auto
			if self.cam_auto:
				self.x_plot,self.y_plot = self.tr.point_to_actual_coord(self.x,self.y)
				for i in range(self.tr.position_plane_total[0]):
					for j in range(self.tr.position_plane_total[1]):
						point = Point((x,y))
						poly = Polygon(self.tr.intervals_rectangles_corners[i][j])
						if poly.contains(point):
							self.tr.position_plane_ant = self.tr.position_plane[:]
							self.tr.position_plane = (i,j)
							break

	def colision_point(obstacle):
		point = Point((self.x,self.y))
		poly = Polygon(obstacle)
		a= poly.contains(point)
		if a:
			print("Colision")
			return False
		else:
			print("Libre")
			return True

	def colision_polygons(obstacle):
		from shapely.geometry import Polygon
		p1 = Polygon([(0,0), (1,1), (1,0)])
		p2 = Polygon(obstacle)
		print(p1.intersects(p2))
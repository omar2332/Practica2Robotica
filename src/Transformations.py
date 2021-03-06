import math
class Transformations(object):
	def __init__(self,width,height,position_plane,position_plane_total,width_env , height_env):
		self.width=width
		self.height=height

		self.position_plane=position_plane
		self.position_plane_ant=self.position_plane[:]

		self.position_plane_total=position_plane_total
		self.width_env = width_env
		self.height_env = height_env
		
		self.intervals = []
		self.intervals_rectangles_corners = []


		self.half_width = self.width/2

		for i in range(self.position_plane_total[0]):
			temp = []
			for j in range(self.position_plane_total[1]):
				temp.append( (i*self.half_width, j*self.height) )
			self.intervals.append(temp[:])
		for i in range(self.position_plane_total[0]):
			temp = []
			for j in range(self.position_plane_total[1]):	
					temp.append([
					  self.intervals[i][j],
					  (self.intervals[i][j][0]+self.half_width,self.intervals[i][j][1]),
					  (self.intervals[i][j][0]+self.half_width,self.intervals[i][j][1]+self.height),
					  (self.intervals[i][j][0],self.intervals[i][j][1]+self.height)])

			self.intervals_rectangles_corners.append(temp[:])


	def reset(self):
		self.position_plane=(0,0)
		self.position_plane_ant=self.position_plane[:]

	def is_intersection_lines(self,line1,line2):
		from shapely.geometry import LineString
		l1 = LineString(line1)
		l2 = LineString(line2)

		return l1.intersects(l2)
	
	def points_in_interval(self,P1,P2,n):

		x1=P1[0]
		y1=P1[1]
		x2=P2[0]-P1[0]
		y2=P2[1]-P1[1]

		t_ref = 1/n
		points = []
		for i in range(n):
			points.append((x1+i*t_ref*x2,y1+i*t_ref*y2 ))

		return points[:]

	def point_angle_and_distance(self,x,y,theta,distance):
		x_temp = x+math.cos(theta)*distance
		y_temp = y+math.sin(theta)*distance
		return (x_temp,y_temp)

	def angle_two_vectors_same_origin(self,p1,p2,x,y):
		x_temp1 = p1[0]-x
		y_temp1 = p1[1]-y
		x_temp2 = p2[0]-x
		y_temp2 = p2[1]-y
		distance1 = self.distance(p1[0],p1[1],x,y)
		distance2 = self.distance(p2[0],p2[1],x,y)
		temp = self.product_point(x_temp1,y_temp1,x_temp2,y_temp2)
		theta = math.acos( temp/(distance1*distance2) )
		return theta

	def point_rotation_to_angles(self,theta,x_center,y_center,distance):
		x_temp = x_center+math.cos(theta)*distance		
		y_temp = y_center+math.sin(theta)*distance
		if x_temp - math.floor(x_temp) < 0.5:
			x_temp = math.floor(x_temp)
		else:
			x_temp = math.ceil(x_temp)
		if y_temp - math.floor(y_temp) < 0.5:
			y_temp = math.floor(y_temp)
		else:
			y_temp = math.ceil(y_temp)
		return x_temp,y_temp

	def product_point(self,x1,y1,x2,y2):
		return x1*x2 + y1*y2

	def distance(self,x1,y1,x2,y2):
		return math.sqrt(((x1-x2)**2) +((y1-y2)**2))

	def distance3D(self,x1,y1,z1,x2,y2,z2):
		return math.sqrt(((x1-x2)**2) +((y1-y2)**2) +((z1-z2)**2))

	# Faltan validaciones
	def inverse_plane(self,x,y):
		return x, self.height-y-1
	def inverse_plane_array(self,array):
		temp= []
		for data in array:
			x,y =self.inverse_plane(data[0],data[1])
			temp.append( (x,y)  )
		return temp

	def inverse_plane_array_lines(self,lines):
		temp= []
		for l in lines:
			temp2=[]
			for data in l:
				x,y =self.inverse_plane(data[0],data[1])
				temp2.append( (x,y)  )
			temp.append(temp2[:])
		return temp


	def p1_camera_traslations_horizontal(self,x,y):
		return x-self.position_plane[0]*self.half_width,y

	def p1_camera_traslations_vertical(self,x,y):
		return x,y-self.position_plane[1]*self.height

	def position_plane_change_horizontal(self, right=True):
		if right:
			self.position_plane[0]+=1
		else:
			self.position_plane[0]-=1
		self.position_plane_ant=self.position_plane[:]

	def position_plane_change_vertical(self, top= True):
		if top:
			self.position_plane[1]+=1
		else:
			position_plane[1]-=1
		self.position_plane_ant=self.position_plane[:]

	def get_actual_rectangle(self):
		return self.intervals_rectangles_corners[self.position_plane[0]][self.position_plane[1]]

	def array_to_actual_coords(self, array):
		new_array = []
		for coords in array:
			x_new,y_new = self.p1_camera_traslations_horizontal(coords[0],coords[1])
			x_new,y_new = self.p1_camera_traslations_vertical(x_new,y_new)
			new_array.append((x_new,y_new))

		return self.inverse_plane_array(new_array)

	def array_lines_to_actual_coords(self, array):
		new_array = []
		for l in array:
			temp=[]
			for coords in l:
				x_new,y_new = self.p1_camera_traslations_horizontal(coords[0],coords[1])
				x_new,y_new = self.p1_camera_traslations_vertical(x_new,y_new)
				temp.append((x_new,y_new))
			new_array.append(temp[:])

		return self.inverse_plane_array_lines(new_array)


	def point_to_actual_coord(self,x,y):
		x_new,y_new = self.p1_camera_traslations_horizontal(x,y)
		x_new,y_new = self.p1_camera_traslations_vertical(x_new,y_new)
		return x_new,y_new
	def coords_to_plane_right(self,x,y):
		return x+self.width/2, y
	def deltalize_point_to_ref(self,x_ref,y_ref,z_ref,x,y,z,delta):
		h = self.distance3D(x,y,z,x_ref,y_ref,z_ref)
		x_temp,y_temp,z_temp= x-x_ref , y-y_ref,z-z_ref
		p0,p1,p2 = x_temp*delta/h,y_temp*delta/h,z_temp*delta/h
		x_new,y_new,z_new = p0+x_ref,p1+y_ref,p2+z_ref
		return x_new,y_new,z_new

	def param_points(self,x1,y1,z1,x2,y2,z2,steps):
		h = self.distance3D(x1,y1,z1,x2,y2,z2)

		total_steps = int(h/steps)
		step = steps/h
		if total_steps < 1:
			total_steps = 1
			
		temp = []
		
		for i in range(total_steps+1):
			t = i*step
			x = x1 + t*(x2-x1)
			y = y1 + t*(y2-y1)
			z = z1 + t*(z2-z1)
			temp.append((x,y,z))
		return temp






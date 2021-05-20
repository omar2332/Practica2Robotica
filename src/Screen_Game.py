
import pygame
import math

from src.Transformations import Transformations
from src.Enviroment import Enviroment


import dubins



class Screen(object):
	def __init__(self,width,height,fps):

		#pygame
		pygame.init()
		clock = pygame.time.Clock()

		#variables generales
		self.width=width
		self.height=height


		#vars for optimizacion(ignore)

		half_width = self.width/2


		###############################



		self.fps =fps
		self.win = pygame.display.set_mode((self.width,self.height))

		self.width_env=3500
		self.height_env= 2000

		position_plane = (0,0)
		position_plane_total = ( int(self.width_env/(half_width)), int(self.height_env/self.height) )
		
		self.run = True

		#objetos
		tr = Transformations(self.width,self.height,position_plane,position_plane_total,self.width_env, self.height_env)
		self.env = Enviroment(self.width_env,self.height_env,tr)

		p1_camera = pygame.Rect(0,0,half_width,self.height)
		p2_camera = pygame.Rect(half_width,0,half_width,self.height )
		
		canvas = pygame.Surface((self.width, self.height))

		sub1 = canvas.subsurface(p1_camera)
		sub2 = canvas.subsurface(p2_camera)



		q0 = (50, 70, math.pi/2 )
		q1 = (1270, 500, math.pi/2)
		turning_radius = 50	
		step_size = 10

		path = dubins.shortest_path(q0, q1, turning_radius)
		configurations, _ = path.sample_many(step_size)
		i=0
		total = len(configurations)

		while self.run:
		
			clock.tick(self.fps)
			for event in pygame.event.get():
				if event.type == pygame.QUIT:
					self.run = False

			
			if i != total:

				self.env.robot.move_to(configurations[i][0],configurations[i][1])
				self.env.robot.rotation_angles(configurations[i][2])
				i+=1
			else:

				self.env.robot.move_to(self.env.robot.x,self.env.robot.y+1)
				self.env.robot.rotation_angles(math.pi/2)

			

			self.env.plot_left(sub1,extended=False)
			self.env.plot_right(sub2)
			pygame.draw.line( self.win,"white",(half_width,0),(half_width,self.height) )
			
			
			#print(tr.position_plane)


			self.win.blit(sub1, (0,0))
			self.win.blit(sub2, (half_width,0))
			
			pygame.display.flip()
			sub1.fill("black")
			sub2.fill("black")
			self.win.fill(("white"))
			

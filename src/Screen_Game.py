import pygame #libreria para el dibujado del programa
import math

from src.Transformations import Transformations
from src.Enviroment import Enviroment



class Screen(object):
	def __init__(self,width,height,fps):

		"""
		width y height son el tamaño de la pantalla que se muestra en pygame
		fps es el total de fotogramas en lso que se actualiza la pantalla por minuto
		"""

		#variables generales
		self.width=width
		self.height=height
		self.fps =fps
		self.run = True #controla el bucle de pygame
		self.cam_auto = True

		#vars for optimizacion(ignore)
		half_width = self.width/2
		###############################


		#este es el tamaño de la bodega
		self.width_env=3500
		self.height_env= 2000

		# se divide el plano del ambiente segun la mitad de la pantalla de pygame definida
		position_plane_total = ( int(self.width_env/(half_width)), int(self.height_env/self.height) ) 
		position_plane = (0,0) #Plano inicial
		plane_i=0
		plane_j=0
		
		
		############################################################################################################################
		#objetos
		############################################################################################################################
		#Variable muy importa, realiza distintas operaciones que se repiten de manera constante a travez de todas demas clases
		tr = Transformations(self.width,self.height,position_plane,position_plane_total,self.width_env, self.height_env)
		#Esta variable es el ambiente o la bodega
		self.env = Enviroment(self.width_env,self.height_env,tr)
		#############################################################################################################################
		#esta es la parte que divide la pantalla en 2 y facilita el proceso del pintado
		p1_camera = pygame.Rect(0,0,half_width,self.height)
		p2_camera = pygame.Rect(half_width,0,half_width,self.height )
		canvas = pygame.Surface((self.width, self.height))
		sub1 = canvas.subsurface(p1_camera)
		sub2 = canvas.subsurface(p2_camera)
		###############################################################################################################################

		#pygame
		pygame.init()
		clock = pygame.time.Clock() 
		#esta variable es la pantalla en si de pygame
		self.win = pygame.display.set_mode((self.width,self.height))
		
		while self.run:
			clock.tick(self.fps) #control de fps
						

			self.env.plot_left(sub1,extended=True) # Pinta el ambiente del lado izquierdo
			self.env.plot_right(sub2,Tree_steps=False) #Pinta el ambiente del lado derecho
			pygame.draw.line(self.win,"white",(half_width,0),(half_width,self.height) ) #linea que divide las dos pantallas
			

			#######################################################################################################
			#Esto es para actualizar la pantalla correctamente
			self.win.blit(sub1, (0,0))
			self.win.blit(sub2, (half_width,0))
			pygame.display.flip()
			sub1.fill("black")
			sub2.fill("black")
			self.win.fill(("white"))
			################################################################################################3
			#Esto es para que se cierre el bucle una vez cerrada la ventana de pygame
			for event in pygame.event.get():
				if event.type == pygame.QUIT:
					self.run = False
				if event.type == pygame.KEYDOWN:
					
					if event.key == pygame.K_SPACE:
						position_plane = (0,0) #Plano inicial
						plane_i=0
						plane_j=0
						self.cam_auto = not self.cam_auto
						if self.cam_auto:
							print("auto")
						else:
							print("manual")
						self.env.change_cam(self.cam_auto)
					if event.key == pygame.K_UP:
						if plane_j+1 <position_plane_total[1]:
							plane_j+=1
							position_plane = (plane_i,plane_j)
							self.env.change_plane(position_plane)
					if event.key == pygame.K_DOWN:
						if plane_j-1 >=0:
							plane_j-=1
							position_plane = (plane_i,plane_j)
							self.env.change_plane(position_plane)

					if event.key == pygame.K_RIGHT:
						if plane_i+1 <position_plane_total[0]:
							plane_i+=1
							position_plane = (plane_i,plane_j)
							self.env.change_plane(position_plane)
					if event.key == pygame.K_LEFT:
						if plane_i-1 >=0:
							plane_i-=1
							position_plane = (plane_i,plane_j)
							self.env.change_plane(position_plane)
						
			

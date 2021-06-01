import numpy as np
import math
from mpl_toolkits.mplot3d import Axes3D
import plotly.express as px
import chart_studio.plotly as py
import plotly.graph_objs as go

class space3D(object):
	"""docstring for space3D"""
	def __init__(self,obstacles,nodes_coords,pairs):
		self.nodes_coords = nodes_coords
		self.pairs = pairs
		self.config_space = self.formating_coords(obstacles)


	def formating_coords(self,obstacles):
		temp = []
		for obs in obstacles:
			for a in obs.coords_3D:
				for b in a:
					temp.append(b)
		np_array = np.array(temp)
		return np_array



	def plot_config_space(self):
		fig = go.Figure(data=[go.Scatter3d(
			x=self.config_space[:,0],
			y=self.config_space[:,1],
			z=self.config_space[:,2],
			mode='markers',
			marker=dict(
				size=1,
				opacity=0.8
			))])
		fig.update_layout(margin=dict(l=0, r=0, b=0, t=0))
		fig.show()

	def plot_config_space_and_tree(self):
		N = len(self.nodes_coords)
		Xn=[self.nodes_coords[k][0] for k in range(N)]# x-coordinates of nodes
		Yn=[self.nodes_coords[k][1] for k in range(N)]# y-coordinates
		Zn=[self.nodes_coords[k][2] for k in range(N)]# z-coordinates
		Xe=[]
		Ye=[]
		Ze=[]
		for e in self.pairs:
			Xe+=[self.nodes_coords[e[0]][0],self.nodes_coords[e[1]][0], None]# x-coordinates of edge ends
			Ye+=[self.nodes_coords[e[0]][1],self.nodes_coords[e[1]][1], None]
			Ze+=[self.nodes_coords[e[0]][2],self.nodes_coords[e[1]][2], None]

		trace1=go.Scatter3d(x=Xe,
			y=Ye,
			z=Ze,
			mode='lines',
			line=dict(color='rgb(125,125,125)', width=1),
			)

		trace2=go.Scatter3d(x=Xn,
			y=Yn,
			z=Zn,
			mode='markers',
			name='actors',
			marker=dict(symbol='circle',
				size=2,
				colorscale='Viridis',
				line=dict(color='rgb(50,50,50)', width=0.5)
				),)
		trace=go.Scatter3d(
			x=self.config_space[:,0],
			y=self.config_space[:,1],
			z=self.config_space[:,2],
			mode='markers',
			marker=dict(
				size=0.7,
				opacity=0.5
				)
			)
		data=[trace,trace1, trace2]
		fig=go.Figure(data=data)

		fig.show()


		
		
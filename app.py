from src import Screen_Game
def main():

	FPS = 30
	#para pantallas de 1080x1920

	WIDTH=875*2
	HEIGHT=1000

	#para pantallas menores a eso
	#WIDTH=875
	#HEIGHT=500

	game = Screen_Game.Screen(WIDTH,HEIGHT,FPS)



if __name__ == '__main__':
	main()
#! /usr/bin/env python

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class myclass:
	def __init__(self):
		print 'heloooooo'

	def test(self):
		fig = plt.figure()
		#ax = fig.add_subplot(111, projection='3d')
		ax = fig.gca(projection = '3d')

		ax.set_xlabel('X axis')
		ax.set_ylabel('Y axis')
		ax.set_zlabel('Z axis')
		ax.axis('equal')

		plt.ion()
		plt.show()

		raw_input('press enter to continue')
		plt.close(fig)
		plt.disconnect(fig)
		del fig
		del ax

def run():
	a = myclass()
	a.test()
	a.test()

if __name__ == "__main__":
    print "hello world"
    run()
import sys
from move_leg import move
from time import sleep

if __name__ == '__main__':
	if len(sys.argv) != 19:
		print 'Incorrect syntax, got {} args, expected {}'.format(len(sys.argv) - 1, 18)
		sys.exit(0)
	for i in range(1, len(sys.argv)):
		print 'Moving servo {}'.format(i-1)
		move(i-1, int(sys.argv[i]))
		raw_input()	

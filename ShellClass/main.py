import time
from Shell import Shell

# keyboard input to determine what to do
command = input("Pick either 1 (hypnotize) or 2 (camouflage): ")

shell = Shell()

while True:
	if command == '1':
		shell.hypnotize()
		
	elif command == '2':
		shell.camouflage((138, 43, 226), 1)
		
	command = input("Pick either 1 (hypnotize) or 2 (camouflage): ")
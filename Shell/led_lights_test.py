# libraries
import time
import board
import neopixel

# setting up the led
led = neopixel.NeoPixel(board.D21, 100, pixel_order=neopixel.GRB, brightness=0.75)

x = 0
wave_frequency = 0.1
blue = (0, 255, 255)
black = (0, 10, 10) #not really black but this is for the blue/black wave

home = (138, 43, 226)

row_led_count_back2front = [
	14,
	17,
	18,
	18,
	16,
	13
]

def _test_shell():
	led.fill(
		(0, 255, 255)
	)
	for i in range(100):
		led[i] = (255, 0, 0)
		time.sleep(1)
	

# keyboard input to determine what to do
command = input("Pick either 1 (hypnotize) or 2 (camouflage): ")

def hypnotize():
	start_index = 0
	for led_row_count in row_led_count_back2front:
		led.fill(blue)
		for led_index in range(start_index, start_index+led_row_count):
			led[led_index] = black
		time.sleep(wave_frequency)
		start_index += led_row_count
		
def camouflage():
	led.fill(home)
	time.sleep(10)


# set the color of the entire led strip
while True:
	#led.fill((98, 144, 220))
	#_test_shell()
	
	if command == '1':
		hypnotize()
		
	elif command == '2':
		R = int(input("Pick R value: "))
		G = int(input("Pick G value: "))
		B = int(input("Pick B value: "))
		home = (R, G, B)
		camouflage()
		
	#command = input("Pick either 1 (hypnotize) or 2 (camouflage): ")
	
	


time.sleep(1)



"""
# simple version of doing a wave
while True:
 while x < 16:
 	led[x] = (98, 144, 220)
 	led[x+1] = (0, 0, 0)
 	
 	x = x + 1
 	time.sleep(0.05)
 	
 while x > 3:
 	led[x] = (98, 144, 220)
 	led[x-1] = (0, 0, 0)
 	
 	x = x - 1
 	time.sleep(0.05)
 	
 while x < 16:
 	led[x] = (98, 144, 220)
 	led[x+1] = (0, 0, 0)
 	
 	x = x + 1
 	time.sleep(0.05)
 	
 while x > 3:
 	led[x] = (98, 144, 220)
 	led[x-3] = (0, 0, 0)
 	
 	x = x - 1
 	time.sleep(0.05)
"""

"""
x = 0
is_towards_end = True

while True:
	led.fill((0, 0, 0))
	led[x] = (0, 0, 0)
	time.sleep(0.05)
	
	if(is_towards_end):
		x += 1
		if(x == 16):
			is_towards_end = False
	else:
		x -= 1
		if(x == -1):
			is_towards_end = True
"""		
	
	
time.sleep(3)

# turn off all the leds
led.fill((0, 0, 0))
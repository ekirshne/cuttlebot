import time
import board
import neopixel

class Shell():
    #Shell class constructor
    def __init__(self):
        # setting up the led
        self.led = neopixel.NeoPixel(board.D21, 100, pixel_order=neopixel.GRB, brightness=0.75) #GRB actually means RGB, this is an issue on the led seller side
        self.row_led_count_back2front = [14, 17, 18, 18, 16, 13]

        self.wave_frequency = 0.1
        self.hypnotize_main_color = (255, 165, 0)   #orange
        self.hypnotize_secondary_color = (5, 10, 10)   #black

        self.camouflage_color = (138, 43, 226)
        self.blanching_color = (200, 200, 200)

        self.hypnosis_row_index = 0

    def start_hypnosis(self):
        self.led.fill(self.hypnotize_main_color)

    def next_hypnosis_step(self):
        start_index = 0
        for i in range(len(self.row_led_count_back2front)):
            #filling the shell with the main color unless off color row seen
            if(i%3 == self.hypnosis_row_index%3):
                for led_index in range(start_index, start_index + self.row_led_count_back2front[i]):
                    self.led[led_index] = self.hypnotize_secondary_color
            else:
                for led_index in range(start_index, start_index + self.row_led_count_back2front[i]):
                    self.led[led_index] = self.hypnotize_main_color
            start_index += self.row_led_count_back2front[i]

        self.hypnosis_row_index = (self.hypnosis_row_index + 1) % len(self.row_led_count_back2front)

    #hypnotize feature
    def hypnotize(self):
        start_index = 0
        for led_row_count in self.row_led_count_back2front:
            #filling the shell with the main color
            self.led.fill(self.hypnotize_main_color)

            #changing one row of led lights to the secondary color to create the hypnotize wave effect
            for led_index in range(start_index, start_index + led_row_count):
                self.led[led_index] = self.hypnotize_secondary_color
            time.sleep(self.wave_frequency)
            start_index += led_row_count
    
    #camouflage feature
    def camouflage(self, color, mode):
        #filling the shell with only one color
        if mode == 1:
            #filling the shell with the camouflage color
            self.camouflage_color = color
            self.led.fill(self.camouflage_color)
        elif mode == 2:
            #filling the shell with a pattern
            print("still need to work on this")
            #assuming color input is an array of six
            """
            row_count = 1
            for led_index in range(start_index, start_index + led_row_count):
                self.led[led_index] = color[row_count - 1]
                row_count += 1
            """
        
        time.sleep(0.1)

    #blanching feature
    def blanching(self):
        #blanching/flashing white
        self.led.fill(self.blanching_color)
        #changes back to camouflage color
        #self.camouflage(color, 1)

    #turns off the lights on the shell
    def turn_off_shell(self):
        self.led.fill((0, 0, 0))

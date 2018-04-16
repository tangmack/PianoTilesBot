
import threading
import time
import pygame

from pygame.locals import *


def keyboardListener(number):
    pygame.init()

    #must generate screen
    screen = pygame.display.set_mode((100,100))

    print "hello"
    while True:
        events = pygame.event.get()
        for event in events:
            if(event.type == KEYUP) or (event.type == KEYDOWN):
                print "key pressed"




threads = []

k = threading.Thread\
            (target = keyboardListener, args=(1,))
##            threads.append(a)
k.start() # note this can still cause more than one threads named "a"

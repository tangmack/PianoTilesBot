import RPi.GPIO as GPIO
import time

LedPin = 11    # pin11
LedPin2 = 12

LedPin3 = 13
LedPin4 = 15

def setup():
  GPIO.setmode(GPIO.BOARD)       # Numbers GPIOs by physical location
  GPIO.setup(LedPin, GPIO.OUT)   # Set LedPin's mode is output
  GPIO.output(LedPin, GPIO.HIGH) # Set LedPin high(+3.3V) to turn on led

  GPIO.setup(LedPin2, GPIO.OUT)   # Set LedPin's mode is output
  GPIO.output(LedPin2, GPIO.HIGH) # Set LedPin high(+3.3V) to turn on led

  GPIO.setup(LedPin3, GPIO.OUT)   # Set LedPin's mode is output
  GPIO.output(LedPin3, GPIO.HIGH) # Set LedPin high(+3.3V) to turn on led

  GPIO.setup(LedPin4, GPIO.OUT)   # Set LedPin's mode is output
  GPIO.output(LedPin4, GPIO.HIGH) # Set LedPin high(+3.3V) to turn on led

def blink():
  while True:
    GPIO.output(LedPin, GPIO.HIGH)  # led on
##    GPIO.output(LedPin2, GPIO.LOW)  # led on
##    GPIO.output(LedPin3, GPIO.LOW)  # led on
##    GPIO.output(LedPin4, GPIO.LOW)  # led on
    time.sleep(.05)
    
    GPIO.output(LedPin, GPIO.LOW) # led off
##    GPIO.output(LedPin2, GPIO.HIGH) # led off
##    GPIO.output(LedPin3, GPIO.HIGH)  # led on
##    GPIO.output(LedPin4, GPIO.HIGH)  # led on
    time.sleep(.05)

def destroy():
  GPIO.output(LedPin, GPIO.LOW)   # led off
  GPIO.cleanup()                  # Release resource

if __name__ == '__main__':     # Program start from here
  setup()
  try:
    blink()
  except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, the child program destroy() will be  executed.
    destroy()

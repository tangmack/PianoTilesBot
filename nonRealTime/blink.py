import RPi.GPIO as GPIO
import time

LedPin = 11    # pin11
LedPin2 = 12    # pin11
LedPin3 = 15    # pin11
LedPin4 = 13    # pin11

def setup():
  GPIO.setmode(GPIO.BOARD)       # Numbers GPIOs by physical location
  GPIO.setup(LedPin, GPIO.OUT)   # Set LedPin's mode is output
  GPIO.output(LedPin, GPIO.LOW) # Set LedPin high(+3.3V) to turn on led

  GPIO.setup(LedPin2, GPIO.OUT)   # Set LedPin's mode is output
  GPIO.output(LedPin2, GPIO.LOW) # Set LedPin high(+3.3V) to turn on led

  GPIO.setup(LedPin3, GPIO.OUT)   # Set LedPin's mode is output
  GPIO.output(LedPin3, GPIO.LOW) # Set LedPin high(+3.3V) to turn on led

  GPIO.setup(LedPin4, GPIO.OUT)   # Set LedPin's mode is output
  GPIO.output(LedPin4, GPIO.LOW) # Set LedPin high(+3.3V) to turn on led

def blink(pinNumber):
##  while True:
    for i in range(0,50):
        GPIO.output(pinNumber, GPIO.HIGH)  # led on
        time.sleep(.019)
        GPIO.output(pinNumber, GPIO.LOW) # led off
        time.sleep(.019)

def destroy():
  GPIO.output(LedPin, GPIO.LOW)   # led off
  GPIO.output(LedPin2, GPIO.LOW)   # led off
  GPIO.output(LedPin3, GPIO.LOW)   # led off
  GPIO.output(LedPin4, GPIO.LOW)   # led off
  GPIO.cleanup()                  # Release resource

if __name__ == '__main__':     # Program start from here
  setup()
  try:
    blink(LedPin)
    blink(LedPin2)
    blink(LedPin3)
    blink(LedPin4)
  except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, the child program destroy() will be  executed.
    destroy()
    
  finally:
    GPIO.cleanup()

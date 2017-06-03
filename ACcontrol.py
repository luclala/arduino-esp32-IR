#AC control script
#L-A Boulanger
#2017/06/03

#!/usr/bin/python3

import datetime
import time
import RPi.GPIO as GPIO
import sys

OUTPUT = 17

now = datetime.datetime.now()

GPIO.output(OUTPUT, FALSE)

sys.argv
if len(sys.argv) >1:
     starttime = sys.argv[1]
if len(sys.argv) >2:
     stoptime = sys.argv[2]

if now.hour == starttime:
	GPIO.output(OUTPUT, TRUE)
	time.sleep(0.2)
	GPIO.output(OUTPUT, FALSE)

if now.hour == stoptime:
	GPIO.output(OUTPUT, TRUE)
	time.sleep(0.2)
	GPIO.output(OUTPUT, FALSE)

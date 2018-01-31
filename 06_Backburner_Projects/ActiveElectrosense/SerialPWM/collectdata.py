#!/usr/bin/python
# Plot data from the PIC32 in python
# requries pyserial, matplotlib, and numpy
import rospy

import serial
import matplotlib.pyplot as plt
import numpy as np

#from hdt_nri_description.msg import ADC

#rospy.init_node('test_node', anonymous=True)

port = '/dev/ttyUSB0' # the name of the serial port

#global pub
#pub = rospy.Publisher('adc_chatter', myADC, queue_size=10)

with serial.Serial(port,230400,rtscts=1) as ser:
  ser.write("\n".encode())  #tell the pic to send data. encode converts to a byte array
  line = ser.readline()
  #nsamples = 50000 #int(line)
  x = []#np.arange(0,nsamples) # x is [1,2,3,... nsamples]
  y = []#np.zeros(nsamples)# x is 1 x nsamples an array of zeros and will store the data
  i = 0

  while(1): # read each sample
    line = ser.readline()   # read a line from the serial port
    if(int(line) >= 2000):
    	print int(line)
    	break
    y.append(3.3*int(line)/1023)       # parse the line (in this case it is just one integer)
 #   pub.publish(3.3*int(line)/1023)
    x.append(i)
    i += 1

plt.plot(x,y)
plt.show()
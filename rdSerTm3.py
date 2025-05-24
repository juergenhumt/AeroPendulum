# Copyright 2025 Juergen Humt
# 
# This file is part of AeroPendulum.
# 
#
#     AeroPendulum, is free  software: you can redistribute it and/or modify
#     it under the terms of the GNU General Public License as published by the 
#     Free Software Foundation, either version 3 of the License or any later 
#     version.
# 
#     AeroPendulum is distributed in the hope that it will be useful,
#     but WITHOUT ANY WARRANTY; without even the implied warranty of
#     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#     GNU General Public License for more details.
# 
#     You should have received a copy of the GNU General Public License along 
#     with AeroPendulum.  If not, see <http://www.gnu.org/licenses/>.
#
#     version v0.1.0
#

# read data from the serial port which are transmitted from the aruino
# and store the data on file
#
import serial
import time
import csv
import os
from timeit import default_timer as timer


ser = serial.Serial('/dev/ttyACM0')
ser.flushInput()

k=0;
m=0;


outF = open("test_data.csv","w")
tStrt = timer()

while True:
    ser_bytes = ser.readline()

    print(ser_bytes)
    lst3= ser_bytes.decode().split(',')  
    if k < 1:
      # print('lst3 ' + lst3[0][0:6] + '   ' + lst3[1][0:8] + '   ' + lst3[2][0:8])
      # bR = lst3[2][0:4]=='9999'
      # print('lst31' + lst3[0][0:6] + '   ' + lst3[1][0:4] + '   ' + str(bR))
      print('Program Started!')

    tStop=False
    if len(ser_bytes) > 1:
      if lst3[0][0:4]=='9999':
        tStop=True
        jSt = 0
      if lst3[2][0:4]=='9999':
        tStop=True
        jSt = 3

    if False:
      if lst3[2][0:2]=='99':
        tStop=True
        jSt = 4
      if lst3[1][0:2]=='99':
        tStop=True
        jSt = 2
      if lst3[1][0:4]=='9999':
        tStop=True
        jSt = 1        
      if lst3[1][0:4]=='7777':
        tStop=True
        jSt = 1
      if lst3[1][0:2]=='77':
        tStop=True
        jSt = 2
      if lst3[2][0:4]=='7777':
        tStop=True
        jSt = 3
      if lst3[2][0:2]=='77':
        tStop=True
        jSt = 4
    if lst3[0][0:4]=='7777':
       tStop=True
       jSt = 0
    if lst3[0][0:3]=='77':
       tStop=True
       jSt = 0

    if not tStop:
       ser_str = str(ser_bytes,'utf-8')
       outF.write(ser_str)

    if (k > 30) and tStop:
      tEnd = timer()        
      cmd = "aplay ~/Data/MorseCode/txt2morse-master/test/r.wav"
      returned_value = os.system(cmd)  # returns the exit code in unix
      print('returned value:', returned_value)
      print('dltT ' + str(tEnd-tStrt) + '  jSt ' + str(jSt))
      print('lst3 ' + lst3[0][0:6] + ',   ' + lst3[1][0:len(lst3[1])] + ',  ' + lst3[2][0:8])        
      
      break

    k= k+1



    if k > 3760:
      tEnd = timer()
      cmd = "aplay ~/Data/MorseCode/txt2morse-master/test/r.wav"
      returned_value = os.system(cmd)  # returns the exit code in unix
      print('Max Count Reached!')
      print('Time dltT ',tEnd-tStrt)
      break

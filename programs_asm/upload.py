#
# Uploads binary executable to CMK Computer 2
#   Usage: python3 send.py path/to/file.txt
#
# Create text file with program bytes:
# python3 assembler.py hello.asm

import sys, time
import serial

ser = serial.Serial('/dev/ttyUSB0', 9600, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE)
input("Press enter to start...")

# executable bytes
program = ''

try:
    # read file
    with open(sys.argv[1]) as f:
        program = [int(c, 16) for c in f.read().split(", ")]

except:
    print('Usage: python3 send.py path/to/file.txt');



# write to serial port
with open('/dev/ttyUSB0', 'wb') as f:
  print('Uploading program to CMK Computer 2');
  ser.write(program)
    
    
    
    
 #!/bin/sh
# Bind the USBCAN device
slcand -o -c -f -s8 /dev/ttyUSB0 slcan0
sleep 2
ifconfig slcan0 up 

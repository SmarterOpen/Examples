from network import LoRa
import socket
import time
import ubinascii
import pycom

# Use the following commands to get the device EUI:
# from network import LoRa
# import binascii
# lora = LoRa(mode=LoRa.LORAWAN)
# print(binascii.hexlify(lora.mac()).upper().decode('utf-8'))

# Be sure to update device firmware, or things in the LoRa library won't be found

# Initialise LoRa in LORAWAN mode.
# Please pick the region that matches where you are using the device:
# Asia = LoRa.AS923
# Australia = LoRa.AU915
# Europe = LoRa.EU868
# United States = LoRa.US915
lora = LoRa(mode=LoRa.LORAWAN, region=LoRa.US915)

# turn off heartbeat so we can control the on board LED
pycom.heartbeat(False)

# create an OTAA authentication parameters
app_eui = ubinascii.unhexlify('INSERT YOUR APP EUI')
app_key = ubinascii.unhexlify('INSERT YOUR APP KEY')

# join a network using OTAA (Over the Air Activation)
lora.join(activation=LoRa.OTAA, auth=(app_eui, app_key), timeout=0)

# wait until the module has joined the network
pycom.rgbled(0x550000)  # Red
while not lora.has_joined():
    time.sleep(2.5)
    print('Not yet joined...')

print('Joined!')
pycom.rgbled(0x005500)  # Green

# create a LoRa socket
s = socket.socket(socket.AF_LORA, socket.SOCK_RAW)

# set the LoRaWAN data rate
s.setsockopt(socket.SOL_LORA, socket.SO_DR, 3)

# make the socket blocking
# (waits for the data to be sent and for the 2 receive windows to expire)
s.setblocking(True)

# send some data
s.send(bytes([0x01, 0x02, 0x03]))

# make the socket non-blocking
# (because if there's no data received it will block forever...)
s.setblocking(False)

# get any data received (if any...)
data = s.recv(64)
print(data)

while True:

    pycom.rgbled(0x005500)
    
    # send some data
    s.send(bytes([0x01, 0x02, 0x03]))

    # make the socket non-blocking
    # (because if there's no data received it will block forever...)
    s.setblocking(False)

    # get any data received (if any...)
    data = s.recv(64)
    print(data)

    time.sleep(45)
    pycom.rgbled(0x000055)
    time.sleep(15)
    
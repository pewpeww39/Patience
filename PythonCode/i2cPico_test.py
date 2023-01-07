from smbus2 import SMBus, i2c_msg

with SMBus(1) as bus:
    # Read 64 bytes from address 80
    msg = i2c_msg.read(80, 64)
    bus.i2c_rdwr(msg)

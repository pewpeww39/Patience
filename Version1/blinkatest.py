import board
import digitalio
import busio

print("Hello blinka!")

pin  = digitalio.DigitalInOut(board.D4)
print ("Digital IO ok")

i2c = busio.I2C(board.SCL, board.SDA)
print("I2C ok")

spi = busio.SPI(board.SCLK, board.MOSI, board.MISO)
print("SPI ok!")

print("done!")

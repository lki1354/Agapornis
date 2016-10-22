from machine import I2C
# configure the I2C bus
i2c = I2C(0, I2C.MASTER, baudrate=100000,pins=('GP13', 'GP12'))
dev = i2c.scan() # returns list of slave addresses
print(dev)

import sensor, image, time, pyb
from machine import Pin, I2C
import vl53l1x

# 1. Power the headers
Pin("PG12", Pin.OUT).value(1)
time.sleep_ms(100)

# 2. Initialize ToF Sensor (Onboard is usually on internal I2C bus)
# Note: On Nicla, the internal bus for the ToF is often Bus 2 or a specific I2C instance
tof_i2c = I2C(2)
distance_sensor = vl53l1x.VL53L1X(tof_i2c)

# 3. Initialize I2C 1 as Slave for Arduino communication
i2c_slave = pyb.I2C(1, pyb.I2C.SLAVE, addr=0x12)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
clock = time.clock()

# We now use 5 bytes: [X, Y, W, H, Distance]
data = bytearray([0, 0, 0, 0, 0])

while (True):
    clock.tick()
    img = sensor.snapshot()
    blobs = img.find_blobs([(80, 100, -20, 20, -20, 20)], pixels_threshold=200)

    # Get Distance from ToF (in mm)
    # We divide by 4 or 10 if need to fit a large distance into a single byte (0-255)
    # If surface is always within 255mm, can send it raw.
    current_dist = distance_sensor.read()

    if blobs:
        b = max(blobs, key=lambda x: x.pixels())
        img.draw_rectangle(b.rect())
        img.draw_cross(b.cx(), b.cy())
        data[0] = b.cx() // 2
        data[1] = b.cy() // 2
        data[2] = b.w() // 2
        data[3] = b.h() // 2

    # Pack distance into the 5th byte (clipping at 255mm for simplicity)
    data[4] = min(current_dist // 2, 255)

    try:
        i2c_slave.send(data, timeout=10)
    except:
        pass


# import sensor, image, time, pyb
# from machine import Pin

# # 1. Enable header power
# Pin("PG12", Pin.OUT).value(1)
# time.sleep_ms(100)

# # 2. Initialize I2C 1 as
# i2c = pyb.I2C(1, pyb.I2C.SLAVE, addr=0x12)

# sensor.reset()
# sensor.set_pixformat(sensor.RGB565)
# sensor.set_framesize(sensor.QVGA)
# sensor.skip_frames(time=2000)
# clock = time.clock()

# thresholds = [(80, 100, -20, 20, -20, 20)]  # White surface thresholds

# while (True):
#     clock.tick()
#     img = sensor.snapshot()
#     blobs = img.find_blobs(thresholds, pixels_threshold=200, area_threshold=200)

#     if blobs:
#         b = max(blobs, key=lambda x: x.pixels())
#         img.draw_rectangle(b.rect())
#         img.draw_cross(b.cx(), b.cy())

#         # Scale 320x240 down to fit in 0-255 byte range
#         data = bytearray([b.cx()//2, b.cy()//2, b.w()//2, b.h()//2])

#         try:
#             # This pushes the data to the MKR 1010
#             i2c.send(data, timeout=10)
#         except:
#             pass

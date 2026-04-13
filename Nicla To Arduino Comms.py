import sensor, image, time, pyb
from machine import Pin

# 1. Enable header power
Pin("PG12", Pin.OUT).value(1)
time.sleep_ms(100)

# 2. Initialize I2C 1 as
i2c = pyb.I2C(1, pyb.I2C.SLAVE, addr=0x12)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
clock = time.clock()

thresholds = [(80, 100, -20, 20, -20, 20)]  # Your white surface thresholds

while (True):
    clock.tick()
    img = sensor.snapshot()
    blobs = img.find_blobs(thresholds, pixels_threshold=200, area_threshold=200)

    if blobs:
        b = max(blobs, key=lambda x: x.pixels())
        img.draw_rectangle(b.rect())
        img.draw_cross(b.cx(), b.cy())

        # Scale 320x240 down to fit in 0-255 byte range
        data = bytearray([b.cx()//2, b.cy()//2, b.w()//2, b.h()//2])

        try:
            # This pushes the data to the MKR 1010
            i2c.send(data, timeout=10)
        except:
            pass

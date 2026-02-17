from pymycobot import MyCobot280

mc = MyCobot280("/dev/ttyUSB0", 115200)
mc.get_angles()
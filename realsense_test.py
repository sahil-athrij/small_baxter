import pyrealsense2 as rs

cntx = rs.context()
device = cntx.devices[0]
serial_number = device.get_info(rs.camera_info.serial_number)
config = rs.config()
config.enable_device(serial_number)

print(serial_number)
#!/usr/bin/env python2.7
import rospy
import serial
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  # replace '/dev/ttyUSB0' with the appropriate serial port name

def uart():
	pub = rospy.Publisher('read_from_stm32',Float32MultiArray, queue_size=10)
	rospy.init_node('jetson', anonymous=True)
	rate = rospy.Rate(100) # 100hz
	while not rospy.is_shutdown():
		data = ser.readline().decode().strip()
		print(data)
        	# Extract data from string
		yaw = float(data[data.find("yaw:") + 4:data.find("Rv:")])
		Rv = float(data[data.find("Rv:") + 3:data.find("Lv:")])
		Lv = float(data[data.find("Lv:") + 3:])
		arr = Float32MultiArray()
		arr.data = [yaw,Rv,Lv]
		pub.publish(arr)
		print("Received: yaw: {}, Rv: {}, Lv: {}".format(yaw,Rv,Lv))
		rate.sleep()

if __name__ == '__main__':
	try:
		uart()
	except rospy.ROSInterruptException:
		pass


import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Int16MultiArray, Int8, UInt8
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

class CmdVelConverter(Node):

	def __init__(self):
		super().__init__('md100a_cmd_vel_converter')

		self.get_logger().info('start cmd_vel_converter')

		self.declare_parameter('pwm_left_max_db', 1545)
		self.declare_parameter('pwm_left_min_db', 1495)
		self.declare_parameter('pwm_right_max_db', 1555)
		self.declare_parameter('pwm_right_min_db', 1480)
		self.declare_parameter('vx_max', 2.0)
		self.declare_parameter('wz_max', 2.0)
		self.declare_parameter('show_log', False)

		self.add_on_set_parameters_callback(self.parameter_callback)

		self.pwm_left_max_db = self.get_parameter('pwm_left_max_db').get_parameter_value().integer_value
		self.pwm_right_max_db = self.get_parameter('pwm_right_max_db').get_parameter_value().integer_value
		self.pwm_left_min_db = self.get_parameter('pwm_left_min_db').get_parameter_value().integer_value
		self.pwm_right_min_db = self.get_parameter('pwm_right_min_db').get_parameter_value().integer_value
		self.vx_max = self.get_parameter('vx_max').get_parameter_value().double_value
		self.wz_max = self.get_parameter('wz_max').get_parameter_value().double_value
		self.show_log = self.get_parameter('show_log').get_parameter_value().bool_value

		self.get_logger().info("Using parameters as below")
		self.get_logger().info("pwm_left_max_db: {}".format(self.pwm_left_max_db))
		self.get_logger().info("pwm_left_min_db: {}".format(self.pwm_left_min_db))
		self.get_logger().info("pwm_right_max_db: {}".format(self.pwm_right_max_db))
		self.get_logger().info("pwm_right_min_db: {}".format(self.pwm_right_min_db))
		self.get_logger().info("vx_max: {}".format(self.vx_max))
		self.get_logger().info("wz_max: {}".format(self.wz_max))
		self.get_logger().info("show_log: {}".format(self.show_log))

		###################
		### Cart params ###
		###################
		self.pwm_max = 2000
		self.pwm_min = 1000
		self.pwm_mid = 1500
		self.prev_y = 0.0

		#############
		## Pub/Sub ##
		#############
		#### For relays the topic in ROS2
		self.cart_mode_sub = self.create_subscription(Int8, '/md100a/cart_mode', self.cart_mode_callback, 10)
		self.cart_mode_cmd_sub = self.create_subscription(UInt8, '/md100a/cart_mode_cmd', self.cart_mode_cmd_callback, 10)
		self.sbus_sub = self.create_subscription(Int16MultiArray, '/md100a/sbus_rc_ch', self.sbus_callback, 10)
		self.imu_sub = self.create_subscription(Imu, '/md100a/imu', self.imu_callback, 10)
		self.pwm_out_sub = self.create_subscription(Int16MultiArray, '/md100a/pwm_out', self.pwm_out_callback, 10)


		self.pwm_cmd_pub = self.create_publisher(Int16MultiArray, '/md100a/pwm_cmd',10)
		self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
		self.crawler_mode_pub = self.create_publisher(Int8, '/crawler/cart_mode', 10)

	#####################
	### ROS callbacks ###
	#####################
	def parameter_callback(self, params):
		for param in params:
			# print(param.name, param.type_)
			if (param.name == 'pwm_left_max_db') and (param.type_ == Parameter.Type.INTEGER):
				self.pwm_left_max_db = param.value
			elif (param.name == 'pwm_right_max_db') and (param.type_ == Parameter.Type.INTEGER):
				self.pwm_right_max_db = param.value
			elif (param.name == 'pwm_left_min_db') and (param.type_ == Parameter.Type.INTEGER):
				self.pwm_left_min_db = param.value
			elif (param.name == 'pwm_right_min_db') and (param.type_ == Parameter.Type.INTEGER):
				self.pwm_right_min_db = param.value
			elif (param.name == 'vx_max') and (param.type_ == Parameter.Type.DOUBLE):
				self.vx_max = param.value
			elif (param.name == 'wz_max') and (param.type_ == Parameter.Type.DOUBLE):
				self.wz_max = param.value
			elif (param.name == 'show_log') and (param.type_ == Parameter.Type.BOOL):
				self.show_log = param.value

		self.get_logger().info("Updated parameter")

		return SetParametersResult(successful=True)

	def cart_mode_callback(self, msg):
		# print("cart_mode", msg.data)
		# pass
		cart_mode_msg = Int8()
		cart_mode_msg.data = msg.data

		self.crawler_mode_pub.publish(cart_mode_msg)

	def cart_mode_cmd_callback(self, msg):
		print("cart_mode_cmd", msg.data)

	def sbus_callback(self, msg):
		pass

	def imu_callback(self, msg):
		pass

	def pwm_out_callback(self, msg):
		pass

	def cmd_vel_callback(self, msg):
		
		if msg.linear.x > self.vx_max:
			vx = self.vx_max
		elif msg.linear.x < -self.vx_max:
			vx = -self.vx_max
		else:
			vx = msg.linear.x

		if msg.angular.z > self.wz_max:
			wz = self.wz_max
		elif msg.angular.z < -self.wz_max:
			wz = -self.wz_max
		else:
			wz = msg.angular.z

		if ((abs(vx) > 0.0) and (abs(wz) > 0.0)):
			y_percent = self.map(vx, -self.vx_max, self.vx_max, -100.0, 100.0)
			# x_percent = self.map(wz, -self.wz_max, self.wz_max, y_percent, -y_percent)
			if vx >= 0.0:
				x_percent = self.map(wz, -self.wz_max, self.wz_max, 100.0, -100.0)
			else:
				x_percent = self.map(wz, -self.wz_max, self.wz_max, 100.0, -100.0)
		else:
			y_percent = self.map(vx, -self.vx_max, self.vx_max, -100.0, 100.0)
			x_percent = self.map(wz, -self.wz_max, self.wz_max, 100.0, -100.0)
			

		left_200_per, right_200_per = self.xy_mixing(x_percent, y_percent)
		left_pwm, right_pwm = self.wheels_percent_to_wheels_pwm(left_200_per, right_200_per)

		print(int(left_pwm), int(right_pwm))

		pwm_cmd_msg = Int16MultiArray()
		pwm_cmd_msg.data = [int(left_pwm), int(right_pwm)]
		self.pwm_cmd_pub.publish(pwm_cmd_msg)

	####################
	### Cart control ###
	####################
	def map(self, val, in_min, in_max, out_min, out_max):
		m = (out_max - out_min)/(in_max - in_min)
		out = m*(val - in_min) + out_min
		return out

	def xy_mixing(self, x, y):
		## x, y must be in the range of -100 to 100

		left = y+x
		right = y-x

		diff = abs(x) - abs(y)

		if (left < 0.0):
			left = left - abs(diff)
		else:
			left = left + abs(diff)

		if (right < 0.0):
			right = right - abs(diff)
		else:
			right = right + abs(diff)

		if (self.prev_y < 0.0):
			swap = left
			left = right
			right = swap
		
		self.prev_y = y

		## left and right are in -200 to 200 ranges

		return left, right


	def wheels_percent_to_wheels_pwm(self, left_per, right_per):

		if left_per > 0.0:
			left_pwm = self.map(left_per, 0.0, 200.0, self.pwm_left_max_db, self.pwm_max)
		elif left_per < 0.0:
			left_pwm = self.map(left_per, -200.0, 0.0, self.pwm_min, self.pwm_left_min_db)
		else:
			left_pwm = self.pwm_mid

		if right_per > 0.0:
			right_pwm = self.map(right_per, 0.0, 200.0, self.pwm_right_max_db, self.pwm_max)
		elif right_per < 0.0:
			right_pwm = self.map(right_per, -200.0, 0.0, self.pwm_min, self.pwm_right_min_db)
		else:
			right_pwm = self.pwm_mid

		return left_pwm, right_pwm

def main(args=None):

	rclpy.init(args=None)
	node = CmdVelConverter()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()

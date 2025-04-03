#!/home/drinkalotofwater/anaconda3/bin/python

import rclpy
import time 
from rclpy.node import Node
from std_msgs.msg import Float64
from rclpy.parameter import Parameter
from rclpy.node import SetParametersResult
from numpy import pi
from pid_controller_msgs.srv import SetReference
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class pidController:
    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0, reference=0.0):
        self.Kp = Kp 
        self.Ki = Ki 
        self.Kd = Kd 
        self.reference = reference 
        self.voltage = 0    
        self.P = 0
        self.I = 0
        self.D = 0
        self.error = 0
        self.lastTime = time.time()
        self.lastError = 0
        
    def update(self, measuredValue):
        currentTime = time.time()
        deltaT = currentTime - self.lastTime
        maxVoltage = 10.0
        maxIntegral = 10.0
        
        self.error = self.reference - measuredValue
        self.P = self.Kp * self.error
        deltaError = self.error - self.lastError
        self.D = self.Kd * (deltaError/deltaT if deltaT > 0 else 0)
        self.I += self.error * deltaT
        self.I = max(min(self.I, maxIntegral), -maxIntegral)

        self.voltage = -(self.P + self.Ki * self.I + self.D)
        
        
        self.voltage = max(min(self.voltage, maxVoltage), -maxVoltage)
        self.lastError = self.error
        self.lastTime = currentTime
        return self.voltage

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller_node')
        self.I = 0.0 # integral anti-windup
        self.declare_parameter('p', 1.0)
        self.declare_parameter('i', 0.0)
        self.declare_parameter('d', 0.0)
        self.declare_parameter('reference', 1.2)

        self.p = self.get_parameter('p').get_parameter_value().double_value
        self.i = self.get_parameter('i').get_parameter_value().double_value
        self.d = self.get_parameter('d').get_parameter_value().double_value
        self.reference =self.get_parameter('reference').get_parameter_value().double_value
        
        self.pid = pidController(self.p, self.i, self.d, self.reference)
        
        self.publisher = self.create_publisher(Float64MultiArray, '/velocity_controller/command', 10)
        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        
        self.add_on_set_parameters_callback(self.parameter_callback)
        self.get_logger().info('PID Controller Node has been started.')
        self.serv = self.create_service(SetReference, 'set_reference', self.set_reference_callback)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'p' and param.value >= 0.0:
                self.p = param.value 
                self.pid.Kp = self.p
                self.get_logger().info(f'Updated P: {self.p}')
            elif param.name == 'i' and param.value >= 0.0:
                self.i = param.value
                self.pid.Ki = self.i
                self.get_logger().info(f'Updated I: {self.i}')
            elif param.name == 'd' and param.value >= 0.0:
                self.d = param.value
                self.pid.Kd = self.d
                self.get_logger().info(f'Updated D: {self.d}')
        return SetParametersResult(successful=True)
    
    def set_reference_callback(self, request, response):
        if -pi <= request.request <= pi:
            self.pid.reference = request.request
            response.success = True
        else:
            response.success = False
        return response
        
    def joint_state_callback(self, msg):
        index = msg.name.index('qube_angle')
        position = msg.position[index]
        updated_velocity = self.pid.update(position)
        vel_msg = Float64MultiArray()
        vel_msg.data = [updated_velocity]
        self.publisher.publish(vel_msg)
        self.get_logger().info(
        f"[PID] Target: {self.pid.reference:.2f} | Position: {position:.2f} | Commanded Vel: {updated_velocity:.2f}"
    )	    

    

def main():
    rclpy.init()
    node = PIDControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()




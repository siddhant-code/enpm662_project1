import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist,PoseStamped
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class AutomaticControlNode(Node):
    def __init__(self): 
        super().__init__('automatic_control_node')
        self.target_reached = False
        self.target_x = 10
        self.target_y = 10
        self.kp = 100 * math.pi/180
        self.steer_angle = 0.0
        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        self.measure_odom= self.create_subscription(PoseStamped,"/odom",self.update_pose,qos_profile=10)
        self.measure_velocity = self.create_subscription(Twist,"/velocity",self.update_vel,qos_profile=10)

        
    def euler_distance(self,point1,point2):
        return math.sqrt((point1[1]-point2[1])**2 + (point1[0]-point2[0])**2)
    
    def target_direction(self,position,target):
        return math.atan2(target[1]-position[1],target[0]-position[0])
        
    
    def update_vel(self,msg):
        self.x_vel = msg.linear.x
        self.y_vel = msg.linear.y
        self.z_vel = msg.linear.z
        wheel_velocities = Float64MultiArray()
        condition_to_move = self.euler_distance([self.x,self.y],[self.target_x,self.target_y]) > 1.0
        if not condition_to_move:
            wheel_velocities.data = [0.0]
            self.target_reached = True
        else:
            wheel_velocities.data = [-10.0]
        self.wheel_velocities_pub.publish(wheel_velocities)

        
    def update_pose(self,msg):
        if self.target_reached:
            return
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z
        joint_positions = Float64MultiArray()
        self.roll_x , self.pitch_y , self.yaw_z = self.euler_from_quaternion(msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w)
        error = self.target_direction([self.x,self.y],[self.target_x,self.target_y]) - self.yaw_z

        print("Error:",error)
        change = self.kp * error        
        self.steer_angle = change
        print("Calculated angle:",self.steer_angle)
        if self.steer_angle>1.0:
            self.steer_angle=1.0
        if self.steer_angle<-1.0:
            self.steer_angle=-1.0
           
        final_angle = self.steer_angle
        
        joint_positions.data = [final_angle,final_angle]
        self.joint_position_pub.publish(joint_positions)
        #self.get_logger().info(str(msg.orientation))
 
    def euler_from_quaternion(self,x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians




def main(args=None):
    rclpy.init(args=args)
    node = AutomaticControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

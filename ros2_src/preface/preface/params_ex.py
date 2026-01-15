import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor, ParameterType, FloatingPointRange

#edited the params example to use the rcply Parameter import and set parameters seperately instead
#of using get_parameter_or to set default values

class DifferentialDriveNode(Node):
    def __init__(self, left_wheel_speed, right_wheel_speed):
        super().__init__('differential_drive_node')
        self.left_wheel_speed = left_wheel_speed
        self.right_wheel_speed = right_wheel_speed

        # Declaring parameters

        self.declare_parameter('wheel_radius', 0.1, descriptor=ParameterDescriptor(
            name='wheel_radius',
            type=ParameterType.PARAMETER_DOUBLE,
            floating_point_range = [FloatingPointRange(from_value=0.0, to_value=1.0, step=0.05)],
            description='Wheel radius in meters'
        ))

        self.declare_parameter('baseline_distance', 0.5, descriptor=ParameterDescriptor(
            name='baseline_distance',
            type=ParameterType.PARAMETER_DOUBLE,
            floating_point_range = [FloatingPointRange(from_value=0.0, to_value=2.0, step=0.1)],
            description='Baseline distance in meters'
        ))

        self.declare_parameter('max_linear_velocity', 1.0, descriptor=ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE))

        self.declare_parameter('max_angular_velocity', 2.0, descriptor=ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE))
        
        #setting parameter default values
        #self.set_parameters([
        #    Parameter('wheel_radius', Parameter.Type.DOUBLE, 0.1),
        #    Parameter('baseline_distance', Parameter.Type.DOUBLE, 0.5),
        #    Parameter('max_linear_velocity', Parameter.Type.DOUBLE, 1.0),
        #    Parameter('max_angular_velocity', Parameter.Type.DOUBLE, 2.0)
        #])

        #retrieving parameter values from the parameters that where just set
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.baseline_distance = self.get_parameter('baseline_distance').value
        self.max_linear_velocity = self.get_parameter('max_linear_velocity').value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value


        # Log the parameters for verification
        self.get_logger().info(f"Wheel radius: {self.wheel_radius}")
        self.get_logger().info(f"Baseline distance: {self.baseline_distance}")
        self.get_logger().info(f"Max linear velocity: {self.max_linear_velocity}")
        self.get_logger().info(f"Max angular velocity: {self.max_angular_velocity}")

        # Calculate the velocities with the initial parameters
        self.get_logger().info("Calculating initial velocities...")
        self.calculate_velocities()

        # Add a parameter change callback
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        """This function is called every time a parameter is changed."""
        for param in params:
            if hasattr(self, param.name):
                setattr(self, param.name, param.value)
                self.get_logger().info(f"Updated {param.name} to {param.value}")
            else:
                reason = f"Parameter {param.name} is not an attribute of this class"
                self.get_logger().warn(reason)
                return SetParametersResult(successful=False, reason=reason)
            
        # Use the new parameters to calculate the velocities
        self.get_logger().info("Calculating updated velocities...")
        self.calculate_velocities()
        
        # Return success message
        return SetParametersResult(successful=True)

    def calculate_velocities(self):
        # Calculate forward and angular velocities from wheel speeds
        linear_velocity = self.wheel_radius * (self.left_wheel_speed + self.right_wheel_speed) / 2.0
        angular_velocity = self.wheel_radius * (self.right_wheel_speed - self.left_wheel_speed) / self.baseline_distance

        # Limit velocities to the maximum values
        linear_velocity = max(-self.max_linear_velocity, min(linear_velocity, self.max_linear_velocity))
        angular_velocity = max(-self.max_angular_velocity, min(angular_velocity, self.max_angular_velocity))

        # Log the calculated velocities
        self.get_logger().info(f"Calculated Linear Velocity: {linear_velocity}")
        self.get_logger().info(f"Calculated Angular Velocity: {angular_velocity}")

def main(args=None):
    rclpy.init(args=args)

    # Example wheel speeds for testing
    left_wheel_speed = 2.0  # radians per second
    right_wheel_speed = 2.5  # radians per second
    node = DifferentialDriveNode(left_wheel_speed, right_wheel_speed)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
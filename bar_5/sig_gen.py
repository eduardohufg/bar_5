#!/usr/bin/env python3
import rclpy, time
from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np

# Class Definition
class MotorSetPoint(Node):
    def __init__(self):
        super().__init__("cinco_barras")
        self.get_logger().info("Motor Set Point node has started")

        self.sampling_time = 0.01
        
        # Create a publisher and timer for the signal
        self.create_timer(self.sampling_time, self.generator_callback)
        self.pub_signal_1 = self.create_publisher(Float32, 'set_point1', 10)
        self.pub_signal_2 = self.create_publisher(Float32, 'set_point2', 10)

        # Initialize message and variables
        self.msg_1 = Float32()
        self.msg_2 = Float32()
        self.gain = 1
        self.t0 = time.time()

    def generator_callback(self):
        # Calculate the elapsed time
        t = time.time() - self.t0
        
        self.msg_1.data, self.msg_2.data = self.equation(t)

        # Publish the signal
        self.pub_signal_1.publish(self.msg_1)
        self.pub_signal_2.publish(self.msg_2)
    
    def equation(self,t):
        num1 = 25 * (299 + 150 * np.cos(5 * t) + 70 * np.sin(5 * t))
        num2 = -((2 * (3 + np.cos(5 * t)) * (7 * np.cos(5 * t) - 15 * np.sin(5 * t)) * 
                (237 + 75 * np.cos(5 * t) + 35 * np.sin(5 * t))) /
                (299 + 150 * np.cos(5 * t) + 70 * np.sin(5 * t))**2)
        num3 = -((15 * np.sqrt(1 - ((62 + 75 * np.cos(5 * t) + 35 * np.sin(5 * t))**2 /
                                    (225 * (299 + 150 * np.cos(5 * t) + 70 * np.sin(5 * t))))) *
                (45 + 100 * np.cos(5 * t) + 15 * np.cos(10 * t) + 
                    42 * np.sin(5 * t) + 7 * np.sin(10 * t))) /
                (299 + 150 * np.cos(5 * t) + 70 * np.sin(5 * t))**(3/2))
        den1 = 8 * np.pi**2 * (3 + np.cos(5 * t))
        den2 = np.sqrt(60006 + 24450 * np.cos(5 * t) - 2200 * np.cos(10 * t) + 
                    11410 * np.sin(5 * t) - 2625 * np.sin(10 * t))
        ecuacion1 = (num1 * (num2 + num3)) / (den1 * den2)

        # Segunda ecuación
        num4 = 3 * (157 + 75 * np.cos(5 * t) - 40 * np.sin(5 * t))
        num5 = -((25 * (3 + np.cos(5 * t)) * (489 + 150 * np.cos(5 * t) - 
                    80 * np.sin(5 * t)) * (8 * np.cos(5 * t) + 15 * np.sin(5 * t))) /
                (24 * (157 + 75 * np.cos(5 * t) - 40 * np.sin(5 * t))**2))
        num6 = -((125 * np.sqrt(1 - ((139 + 150 * np.cos(5 * t) - 80 * np.sin(5 * t))**2 /
                                    (1800 * (157 + 75 * np.cos(5 * t) - 40 * np.sin(5 * t))))) *
                (45 + 100 * np.cos(5 * t) + 15 * np.cos(10 * t) - 
                    48 * np.sin(5 * t) - 8 * np.sin(10 * t))) /
                (2 * (314 + 150 * np.cos(5 * t) - 80 * np.sin(5 * t))**(3/2)))
        den3 = np.pi**2 * (3 + np.cos(5 * t))
        den4 = np.sqrt(248829 + 93300 * np.cos(5 * t) - 8050 * np.cos(10 * t) - 
                    49760 * np.sin(5 * t) + 12000 * np.sin(10 * t))
        ecuacion2 = (num4 * (num5 + num6)) / (den3 * den4)

        return ecuacion1*(np.pi), ecuacion2 *(np.pi)


# Main Function
def main(args=None):
    rclpy.init(args=args)
    nodeh = MotorSetPoint()
    try: rclpy.spin(nodeh)
    except Exception as error: print(error)
    except KeyboardInterrupt: print("Terminated by user")
    finally:
        nodeh.destroy_node()
        rclpy.try_shutdown()

# Execute Node
if __name__ == "__main__":
    main()
import unittest
import time
from test_support.movement import MovementManager

class RobotTests(unittest.TestCase):
    def setUp(self):
        self.movement_manager = MovementManager()
    
    def test_drive_straight_slow(self):
        self.movement_manager.set_max_linear_velocity(0.25)

        # move forward 1 meter then backward
        self.movement_manager.move_straight(2)
        time.sleep(1)
        self.movement_manager.move_straight(-2)
        time.sleep(1)

    def test_drive_straight_fast(self):
        self.movement_manager.set_max_linear_velocity(1)

        # move forward 1 meter then backward
        self.movement_manager.move_straight(2)
        time.sleep(1)
        self.movement_manager.move_straight(-2)
        time.sleep(1)
        
    def test_rotate_clockwise_fast(self):
        self.movement_manager.set_max_angular_velocity(1.24)
        self.movement_manager.turn(6.28)
        time.sleep(1)

    def test_rotate_anticlockwise_fast(self):
        self.movement_manager.set_max_angular_velocity(1.24)
        self.movement_manager.turn(-6.28)
        time.sleep(1)
        
    def test_rotate_clockwise_slow(self):
        self.movement_manager.set_max_angular_velocity(.628)
        self.movement_manager.turn(6.28)
        time.sleep(1)   
        
    def test_rotate_anticlockwise_slow(self):
        self.movement_manager.set_max_angular_velocity(.628)
        self.movement_manager.turn(-6.28)
        time.sleep(1)
        
    def test_square_clockwise(self):
        pass
    def test_square_anticlockwise(self):
        pass
        




if __name__ == '__main__':
    unittest.main()
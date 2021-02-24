import unittest
import time
from test_support.movement import MovementManager
import sys

def user_accepts(question):
    
    val = None
    while val not in ['Y', 'y', 'N', 'n']:
        val = raw_input(question + '( Y/n )  ')
    
    if val in ['N', 'n']:
        return False
    else:
        return True

def print_test_case_name(function_name):
    banner = '\nRunning test case %s ' % function_name
    print banner

class RobotTests(unittest.TestCase):
    def setUp(self):
        self.movement_manager = MovementManager()
    
    def test_drive_straight_slow(self):
        print_test_case_name(sys._getframe().f_code.co_name)
        self.movement_manager.set_max_linear_velocity(0.25)

        # move forward 1 meter then backward
        self.movement_manager.move_straight(2)
        time.sleep(1)
        self.movement_manager.move_straight(-2)
        time.sleep(1)

        if not user_accepts('did the robot move forward and backward equal distance of 2 meters and the same speed?'):
            raise ValueError('SlowSpeedStraightTestError')

    def test_drive_straight_moderate(self):
        print_test_case_name(sys._getframe().f_code.co_name)
        self.movement_manager.set_max_linear_velocity(0.5)

        # move forward 1 meter then backward
        self.movement_manager.move_straight(2)
        time.sleep(1)
        self.movement_manager.move_straight(-2)
        time.sleep(1)

        if not user_accepts('did the robot move forward and backward equal distance of 2 meters and the same speed?'):
            raise ValueError('ModerateSpeedStraightTestError')

    def test_drive_straight_fast(self):
        print_test_case_name(sys._getframe().f_code.co_name)
        self.movement_manager.set_max_linear_velocity(0.75)

        # move forward 1 meter then backward
        self.movement_manager.move_straight(2)

        time.sleep(1)
        self.movement_manager.move_straight(-2)
        time.sleep(1)

        if not user_accepts('did the robot move forward and backward equal distance of 2 meters and the same speed?'):
            raise ValueError('FastSpeedStraightTestError')
        
    def test_rotate_clockwise_fast(self):
        print_test_case_name(sys._getframe().f_code.co_name)
        self.movement_manager.set_max_angular_velocity(.75)
        self.movement_manager.turn(-6.28)
        time.sleep(1)

        if not user_accepts('did the robot move 360 degrees (+/-) 45?'):
            raise ValueError('FastClockwiseTurnError')

    def test_rotate_anticlockwise_fast(self):
        print_test_case_name(sys._getframe().f_code.co_name)
        self.movement_manager.set_max_angular_velocity(0.75)
        self.movement_manager.turn(6.28)
        time.sleep(1)

        if not user_accepts('did the robot move 360 degrees (+/-) 45?'):
            raise ValueError('FastAntiClockwiseTurnError')
        
    def test_rotate_clockwise_slow(self):
        print_test_case_name(sys._getframe().f_code.co_name)
        self.movement_manager.set_max_angular_velocity(.628)
        self.movement_manager.turn(-6.28)
        time.sleep(1)   

        if not user_accepts('did the robot move 360 degrees (+/-) 45?'):
            raise ValueError('SlowClockwiseTurnError')
        
    def test_rotate_anticlockwise_slow(self):
        print_test_case_name(sys._getframe().f_code.co_name)
        self.movement_manager.set_max_angular_velocity(.628)
        self.movement_manager.turn(6.28)
        time.sleep(1)

        if not user_accepts('did the robot move 360 degrees (+/-) 45?'):
            raise ValueError('SlowAntiClockwiseTurnError')
        
    def test_square_clockwise(self):
        print_test_case_name(sys._getframe().f_code.co_name)
        self.movement_manager.set_max_angular_velocity(1.24)
        self.movement_manager.set_max_linear_velocity(0.25)

        for _ in range(4):
            self.movement_manager.move_straight(1)
            time.sleep(0.5)
            self.movement_manager.turn(-0.25 * 6.28)
            time.sleep(0.5)

        if not user_accepts('did the robot drive in a shape resembling a square?'):
            raise ValueError('ClockwiseSquareError')
        
    def test_square_anticlockwise(self):
        print_test_case_name(sys._getframe().f_code.co_name)
        self.movement_manager.set_max_angular_velocity(1.24)
        self.movement_manager.set_max_linear_velocity(0.25)

        for _ in range(4):
            self.movement_manager.move_straight(1)
            time.sleep(0.5)
            self.movement_manager.turn(0.25 * 6.28)
            time.sleep(0.5)

        if not user_accepts('did the robot drive in a shape resembling a square?'):
            raise ValueError('AntiClockwiseSquareError')

if __name__ == '__main__':
    
    if not user_accepts('is the robot ready to be moved? (clear by 2 meters all directions?) '):
        print 'exiting'
        exit()

    if not user_accepts('is closed loop control on in the launch file? '):
        print 'exiting'
        exit()

    if not user_accepts('is the drive type set appropriately (4wd / 2wd / flippers) in the launch file? '):
        print 'exiting'
        exit()

    sn = raw_input('enter the device serial number:  ')

    unittest.main()
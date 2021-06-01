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

class TestBase(unittest.TestCase):
    def setUp(self):
        self.movement_manager = MovementManager()
    def runTest(self):
        raise ValueError('test not implemented')

class StraightFast(TestBase):
    def runTest(self):
        self.movement_manager.set_max_linear_velocity(0.75)

        # move forward 1 meter then backward
        self.movement_manager.move_straight(2)
        time.sleep(1)
        self.movement_manager.move_straight(-2)
        time.sleep(1)

        if not user_accepts('did the robot move forward and backward equal distance of 2 meters and the same speed?'):
            raise ValueError('FastSpeedStraightTestError')

class StraightModerate(TestBase):
    def runTest(self):
        self.movement_manager.set_max_linear_velocity(0.5)

        # move forward 1 meter then backward
        self.movement_manager.move_straight(2)
        time.sleep(1)
        self.movement_manager.move_straight(-2)
        time.sleep(1)

        if not user_accepts('did the robot move forward and backward equal distance of 2 meters and the same speed?'):
            raise ValueError('ModerateSpeedStraightTestError')

class StraightSlow(TestBase):
    def runTest(self):
        self.movement_manager.set_max_linear_velocity(0.25)

        # move forward 1 meter then backward
        self.movement_manager.move_straight(2)
        time.sleep(1)
        self.movement_manager.move_straight(-2)
        time.sleep(1)

        if not user_accepts('did the robot move forward and backward equal distance of 2 meters and the same speed?'):
            raise ValueError('SlowSpeedStraightTestError')

class RotateClockwiseFast(TestBase):
    def runTest(self):
        self.movement_manager.set_max_angular_velocity(.75)
        self.movement_manager.turn(-6.28)
        time.sleep(1)

        if not user_accepts('did the robot move 360 degrees (+/-) 45?'):
            raise ValueError('FastClockwiseTurnError')

class RotateClockwiseSlow(TestBase):
    def runTest(self):
        self.movement_manager.set_max_angular_velocity(.628)
        self.movement_manager.turn(-6.28)
        time.sleep(1)   

        if not user_accepts('did the robot move 360 degrees (+/-) 45?'):
            raise ValueError('SlowClockwiseTurnError')

class RotateAnticlockwiseFast(TestBase):
    def runTest(self):
        self.movement_manager.set_max_angular_velocity(0.75)
        self.movement_manager.turn(6.28)
        time.sleep(1)

        if not user_accepts('did the robot move 360 degrees (+/-) 45?'):
            raise ValueError('FastAntiClockwiseTurnError')

class RotateAnticlockwiseSlow(TestBase):
    def runTest(self):
        self.movement_manager.set_max_angular_velocity(.628)
        self.movement_manager.turn(6.28)
        time.sleep(1)

        if not user_accepts('did the robot move 360 degrees (+/-) 45?'):
            raise ValueError('SlowAntiClockwiseTurnError')

class SquareClockwise(TestBase):
    def runTest(self):
        self.movement_manager.set_max_angular_velocity(1.24)
        self.movement_manager.set_max_linear_velocity(0.25)

        for _ in range(4):
            self.movement_manager.move_straight(1)
            time.sleep(0.5)
            self.movement_manager.turn(-0.25 * 6.28)
            time.sleep(0.5)

        if not user_accepts('did the robot drive in a shape resembling a square?'):
            raise ValueError('ClockwiseSquareError')

class SquareAnticlockwise(TestBase):
    def runTest(self):
        self.movement_manager.set_max_angular_velocity(1.24)
        self.movement_manager.set_max_linear_velocity(0.25)

        for _ in range(4):
            self.movement_manager.move_straight(1)
            time.sleep(0.5)
            self.movement_manager.turn(0.25 * 6.28)
            time.sleep(0.5)

        if not user_accepts('did the robot drive in a shape resembling a square?'):
            raise ValueError('AntiClockwiseSquareError')

def get_test_suite():
    test_suite = unittest.TestSuite()
    test_suite.addTest(unittest.makeSuite(StraightFast))
    test_suite.addTest(unittest.makeSuite(StraightModerate))
    test_suite.addTest(unittest.makeSuite(StraightSlow))
    test_suite.addTest(unittest.makeSuite(StraightSlow))
    test_suite.addTest(unittest.makeSuite(RotateClockwiseFast))
    test_suite.addTest(unittest.makeSuite(RotateClockwiseSlow))
    test_suite.addTest(unittest.makeSuite(RotateAnticlockwiseFast))
    test_suite.addTest(unittest.makeSuite(SquareClockwise))
    test_suite.addTest(unittest.makeSuite(SquareAnticlockwise))
    

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
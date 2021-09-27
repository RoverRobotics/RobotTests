import logging
import unittest
import time
from test_support.movement import MovementManager
import sys
import os



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
        self.logger = logging.getLogger("testlog")
        self.logger.setLevel(logging.DEBUG)
        with open(os.path.dirname(os.path.realpath(__file__)) + "/unittest.log", mode="a+") as f:
            pass
        fh = logging.FileHandler(os.path.dirname(os.path.realpath(__file__)) + "/unittest.log")
        fh.setLevel(logging.DEBUG)
        self.logger.addHandler(fh)
        

    def runTest(self):
        raise ValueError('test not implemented')

class StraightFast(TestBase):
    def runTest(self):
        if not user_accepts('Next test: Fast drive straight and back. Are you ready?'):
            exit()

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
        if not user_accepts('Next test: Medium drive straight and back. Are you ready?'):
            exit()

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
        if not user_accepts('Next test: Slow drive straight and back. Are you ready?'):
            exit()

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
        if not user_accepts('Next test: Fast Right turn. Are you ready?'):
            exit()

        self.movement_manager.set_max_angular_velocity(.75)
        self.movement_manager.turn(-6.28)
        time.sleep(1)

        if not user_accepts('did the robot move 360 degrees (+/-) 45?'):
            raise ValueError('FastClockwiseTurnError')

class RotateClockwiseSlow(TestBase):
    def runTest(self):
        if not user_accepts('Next test: Slow Right turn. Are you ready?'):
            exit()

        self.movement_manager.set_max_angular_velocity(.628)
        self.movement_manager.turn(-6.28)
        time.sleep(1)   

        if not user_accepts('did the robot move 360 degrees (+/-) 45?'):
            raise ValueError('SlowClockwiseTurnError')

class RotateAnticlockwiseFast(TestBase):
    def runTest(self):
        if not user_accepts('Next test: Fast Left turn. Are you ready?'):
            exit()

        self.movement_manager.set_max_angular_velocity(0.75)
        self.movement_manager.turn(6.28)
        time.sleep(1)

        if not user_accepts('did the robot move 360 degrees (+/-) 45?'):
            raise ValueError('FastAntiClockwiseTurnError')

class RotateAnticlockwiseSlow(TestBase):
    def runTest(self):
        if not user_accepts('Next test: Slow Left turn. Are you ready?'):
            exit()

        self.movement_manager.set_max_angular_velocity(.628)
        self.movement_manager.turn(6.28)
        time.sleep(1)

        if not user_accepts('did the robot move 360 degrees (+/-) 45?'):
            raise ValueError('SlowAntiClockwiseTurnError')

class SquareClockwise(TestBase):
    def runTest(self):
        if not user_accepts('Next test: Clockwise Box. Are you ready?'):
            exit()

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
        if not user_accepts('Next test: Counter clockwise Box. Are you ready?'):
            exit()

        self.movement_manager.set_max_angular_velocity(1.24)
        self.movement_manager.set_max_linear_velocity(0.25)

        for _ in range(4):
            self.movement_manager.move_straight(1)
            time.sleep(0.5)
            self.movement_manager.turn(0.25 * 6.28)
            time.sleep(0.5)

        if not user_accepts('did the robot drive in a shape resembling a square?'):
            raise ValueError('AntiClockwiseSquareError')

class MinTurnSpeed(TestBase):
    def runTest(self):
        if not user_accepts('Next test: Min turn speed: Are you ready?'):
            exit()

        for speed in [0.3, 0.4, 0.5]:
            self.movement_manager.set_max_angular_velocity(speed)
            self.movement_manager.turn(0.5 * 6.28)
            if user_accepts('Was the robot able to turn without stalling?'):
                self.logger.debug('Min turn speed: %f' % speed)
                return

        self.logger.error('Robot has unacceptable minimum turn speed')
        raise ValueError('Robot has unacceptable minimum turn speed')

def get_test_suite():
    test_suite = unittest.TestSuite()
    test_suite.addTest(unittest.makeSuite(StraightFast))
    test_suite.addTest(unittest.makeSuite(StraightModerate))
    test_suite.addTest(unittest.makeSuite(StraightSlow))
    test_suite.addTest(unittest.makeSuite(RotateClockwiseFast))
    test_suite.addTest(unittest.makeSuite(RotateClockwiseSlow))
    test_suite.addTest(unittest.makeSuite(RotateAnticlockwiseFast))
    test_suite.addTest(unittest.makeSuite(RotateAnticlockwiseSlow))
    test_suite.addTest(unittest.makeSuite(SquareClockwise))
    test_suite.addTest(unittest.makeSuite(SquareAnticlockwise))
    test_suite.addTest(unittest.makeSuite(MinTurnSpeed))

    return test_suite
    
def system_ready_for_test():
    if not user_accepts('is the robot ready to be moved? (clear by 2 meters all directions?) '):
        print 'exiting'
        exit()

    return True

if __name__ == '__main__':
    runner = unittest.TextTestRunner()
    runner.run(get_test_suite())

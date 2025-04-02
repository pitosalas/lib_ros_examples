#!/usr/bin/env python3

import rospy
import bru_utils as bu


class BaseNode:

    def __init__(self):
        self.hertz: int = 15
        self.shutdown_requested: bool = False

    def wait_for_simulator(self):
        # Wait for the simulator to be ready. If simulator is not ready, then time will be stuck at zero
        while rospy.Time.now().to_sec() == 0:
            self.rate.sleep()

    def set_hertz(self, value):
        self.hertz = value
        self.rate = rospy.Rate(self.hertz)


    def get_hertz(self):
        return self.hertz
        
    def stop(self):
        pass

    def shutdown_hook(self):
        self.shutdown_requested = True
        bu.info("Shutdown Requested")
        self.stop()

    def loop(self):
        pass

    def initial_setup(self):
        pass

    def run(self):
        try:
            self.rate = rospy.Rate(self.hertz)
            self.initial_setup()
            while not rospy.is_shutdown() and not self.shutdown_requested:
                self.loop()
                self.rate.sleep()
        except rospy.exceptions.ROSInterruptException:
            bu.info("exiting...")

if __name__ == '__main__':
    rn = BaseNode()
    rospy.on_shutdown(rn.shutdown_hook)
    rn.run()

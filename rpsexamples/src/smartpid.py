import rospy
class PID:
    def __init__(self, Kp, Td, Ti, dt):
        self.Kp = Kp
        self.Td = Td
        self.Ti = Ti
        self.curr_error = 0
        self.prev_error = 0
        self.sum_error = 0
        self.prev_error_deriv = 0
        self.curr_error_deriv = 0
        self.control = 0
        self.dt = dt
        
    def update_params(self):
    	self.Kp = rospy.get_param("smartpid/Kp")
    	self.Td = rospy.get_param("smartpid/Td")
    	self.Ti = rospy.get_param("smartpid/Ti")
        self.dt = rospy.get_param("smartpid/dt")
        
    def update_control(self, current_error, reset_prev=False):
        self.update_params()
        self.prev_error = self.curr_error
        self.curr_error = current_error
        
        #Calculating the integral error
        self.sum_error = self.sum_error + self.curr_error*self.dt

        #Calculating the derivative error
        self.curr_error_deriv = (self.curr_error - self.prev_error) / self.dt

        #Calculating the PID Control
        self.control = self.Kp * self.curr_error + self.Ti * self.sum_error + self.Td * self.curr_error_deriv
        print "pid (%.1f,%.1f,%.1f) curr_err: %2f, control: %.2f" % (self.Kp, self.Ti, self.Td, self.curr_error, self.control)

    def get_control(self):
        return self.control
        
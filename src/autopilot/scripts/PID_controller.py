#!/usr/bin/env python3
import rospy
from ros_msgs.srv import PIDService
import matplotlib.pyplot as plt

class PIDController:
    def __init__(self,Kp:float = 0, Ki:float = 0, Kd:float = 0,Ku:float = None,Tu:float = None, Imax = 0 , alpha:float = 0.15, sampleRate:int = 50):
        """PID Controller Class
        Args:
            Ku (float, optional): For Ziegler-Nichols method. Default is None.
            Tu (float, optional): For Ziegler-Nichols method. Default is None.
            sampleRate (int, optional): should be in hz format. Default is 50.
            
        Description:
            For loop frequency use this:
                sampleRate = 50 # Adjust it for your needs
                rate = rospy.Rate(sampleRate) # Set a publish rate of 50 Hz
            In the loop:
                rate.sleep() # Make sure the publish rate maintains at 50 Hz
        """        
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        
        self.prevKp = Kp
        self.prevKi = Ki
        self.prevKd = Kd
        
        self.PTerm = 0
        self.ITerm = 0
        self.DTerm = 0
        
        self.Imax = Imax
        
        self.error = 0
        self.prevError = 0
        self.errorLog = []
        self.time_values = []
        
        self.alpha = alpha
        
        self.sampleRate = sampleRate
        self.sampleTime = 1/sampleRate
        
        if Ku is not None or Tu is not None:
            self.setTunings(Ku,Tu)
        
    def setTunings(self,Ku,Tu):
        self.Kp = 0.6 * Ku
        self.Ki = Tu/2
        self.Kd = Tu/8
        
    def updateGains(self,altitude):
        self.Kp = self.updateGain(self.Kp, altitude)
        self.Ki = self.updateGain(self.Ki, altitude)
        self.Kd = self.updateGain(self.Kd, altitude)
        
    def updateGain(self,K, altitude):
        return self.alpha * (K + (self.error-self.prevError)) + (self.alpha - 1) * altitude
        

    def get_pid(self,error):
        """Calculates PID value for given reference feedback
        Args:
            error (float): error between reference and feedback
        Math:
            u(t) = Kp.e(t) + Ki.Integral(e(t)) + Kd.(de(t)/dt) 

        Returns:
            float: PID output
        """
        self.getPTerm(error)
        self.getITerm(error,self.sampleTime)
        self.getDTerm(error,self.sampleTime)
        
        self.errorLog.append(error)
        self.time_values.append(rospy.get_time())
        self.prevError = 0
        
        return self.PTerm + self.ITerm + self.DTerm
    
    def get_pi(self,error):
        """Calculates PID value for given reference feedback
        Args:
            error (float): error between reference and feedback
        Math:
            u(t) = Kp.e(t) + Ki.Integral(e(t)) 

        Returns:
            float: PID output
        """
        self.getPTerm(error)
        self.getITerm(error,self.sampleTime)
                
        self.prevError = error
        
        return self.PTerm + self.ITerm
    
    def getPTerm(self,error):
        self.PTerm = self.Kp * error
        
    def getITerm(self,error,sampleTime):
        self.ITerm += self.Ki * (error + self.prevError) * sampleTime / 2
        if self.ITerm > self.Imax:
            self.ITerm = self.Imax
        elif self.ITerm < -self.Imax:
            self.ITerm = -self.Imax
        
    def getDTerm(self,error,sampleTime):
        self.DTerm = self.Kd * (error - self.prevError) / sampleTime
        
    def setKp(self,Kp):
        self.Kp = Kp
    
    def setKi(self,Ki):
        self.Ki = Ki
        
    def setKd(self,Kd):
        self.Kd = Kd
        
    def setSampleRate(self,sampleRate):
        self.sampleRate = sampleRate
        self.sampleTime = 1/sampleRate

        
class PIDNode:
    def __init_(self):
        rospy.init_node("PID_controller")
        rospy.loginfo("PID controller initialized")
        self.pid_x = PIDController(Kp=1,Ki=0,Kd=0)
        self.pid_y = PIDController(Kp=1,Ki=0,Kd=0)
        self.pid_yaw = PIDController(Kp=1,Ki=0,Kd=0)

        self.pid_service = rospy.Service("pid_service",PIDService,self.handle_pid_service)
        rospy.loginfo("PID service initialized")
    
    def handle_pid_service(self, req: PIDService):
        if req.request_type == "get":
            return self.pid_x.get_pid(req.x), self.pid_y.get_pid(req.y), self.pid_yaw.get_pid(req.yaw)
        elif req.request_type == "plot":
            # Plot the error values over time
            plt.plot(self.pid_x.time_values, self.pid_x.errorLog, label='X Error')
            plt.plot(self.pid_y.time_values, self.pid_y.errorLog, label='Y Error')
            plt.plot(self.pid_yaw.time_values, self.pid_yaw.errorLog, label='Yaw Error')

            plt.xlabel('Time')
            plt.ylabel('Error')
            plt.title('PID Error Over Time')
            plt.legend()
            plt.grid(True)
            try:
                plt.savefig('~/figures/pid_error_figure.fig')
            except:
                rospy.loginfo("Couldn't save the figure")
            plt.show()
        
def main():
    node = PIDNode()
    rospy.spin()
        
if __name__ == "__main__":
    main()
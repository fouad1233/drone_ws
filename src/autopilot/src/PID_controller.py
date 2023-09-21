#!/usr/bin/env python3
import rospy

class PIDController:
    def __init__(self,Kp:float = 0, Ki:float = 0, Kd:float = 0,Ku:float = None,Tu:float = None, alpha:float = 0.15, sampleRate:int = 50):
        """PID Controller Class

        Args:
            Ku (float, optional): For Ziegler-Nichols method. Defaults to None.
            Tu (float, optional): For Ziegler-Nichols method. Defaults to None.
            sampleRate (int, optional): should be in hz format. Defaults to 50.
            
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
        
        self.prevPTerm = 0
        self.prevITerm = 0
        self.prevDTerm = 0
        
        self.error = 0
        self.prevError = 0
        
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
        
        self.prevError = error
        
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
        self.prevPTerm = self.PTerm
        
    def getITerm(self,error,sampleTime):
        self.ITerm = self.prevITerm + self.Ki * (error + self.prevError) * sampleTime / 2
        self.prevITerm = self.ITerm
        
    def getDTerm(self,error,sampleTime):
        self.DTerm = self.Kd * (error - self.prevError) / sampleTime
        self.prevDTerm = self.DTerm
        
    def setKp(self,Kp):
        self.Kp = Kp
    
    def setKi(self,Ki):
        self.Ki = Ki
        
    def setKd(self,Kd):
        self.Kd = Kd
        
    def setSampleRate(self,sampleRate):
        self.sampleRate = sampleRate
        self.sampleTime = 1/sampleRate
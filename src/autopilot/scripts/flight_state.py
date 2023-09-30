import rospy
from std_msgs.msg import String
from ros_msgs.srv import flight_state

class FlightState:
    def __init__(self):
        rospy.init_node('flight_state_handler')
        self.pub = rospy.Publisher('/flight_state', String, queue_size=10)
        self.service = rospy.Service('/flight_state', flight_state, self.handle_flight_state)
        
        self.states =  ["TAKINGOFF",
                        "PENDING",
                        "SEARCHING",
                        "LANDING",
                        "LANDED"]
        self.state = "PENDING"
        
        rospy.loginfo(f"Flight state status: {self.state}")
        self.pub.publish(self.state)
    
    def handle_flight_state(self,req):
        if req.RequestType == "get":
            return self.state
        elif req.RequestType == "set":
            self.set_flight_state(req)
    
    def set_flight_state(self,req):
        if req.state in self.states:
            self.pub.publish(req.state)
            self.state = req.state
            rospy.loginfo("Flight state changed to: " + req.state)
            return True

        else:
            rospy.logerr("Invalid flight state: " + req.state)
            return False
        
def main():
    node = FlightState()
    rospy.spin()
    
if __name__ == "__main__":
    main()
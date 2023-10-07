import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from field_scan import FieldScan


class RosNode:
    def __init__(self):
        rospy.init_node('flight_state_handler')
        self.pub = rospy.Publisher('/flight_state', String, queue_size=10)
        
class StateMachine:
    def __init__(self):
        self.srv = rospy.Service("/startMain", Trigger, self.server_callback)
        self.states = {}
        self.current_state = None
        self.is_running = False
        
    def server_callback(self, req):
        self.is_running = True
        return True, "Success"

    def add_state(self, state_name, state):
        self.states[state_name] = state

    def set_initial_state(self, state_name):
        if state_name in self.states:
            self.current_state = self.states[state_name]
        else:
            raise ValueError("State does not exist")

    def transition_to(self, state_name):
        if state_name in self.states:
            self.current_state = self.states[state_name]
        elif state_name == "EndState":
            self.is_running = False
            self.set_initial_state("ArmState")
        else:
            raise ValueError("State does not exist")

    def execute(self):
        if self.current_state:
            self.current_state.execute()


class ArmState:
    def execute(self):
        print("ARM state")
        
    def next(self):
        return "TakeoffState"

class TakeoffState:
    def execute(self):
        print("PENDING state")
    
    def next(self):
        return "SearchState"

class SearchState:
    def execute(self):
        print("HOVER state")
        
    def next(self):
        return "LandState"

class LandState:
    def execute(self):
        print("ONGROUND state")
        
    def next(self):
        return "RTLState"

class RTLState:
    def execute(self):
        print("LANDED state")
        
    def next(self):
        return "EndState"
    
def main():
    node = RosNode()
    state_machine = StateMachine()
    state_machine.add_state("ArmState", ArmState())
    state_machine.add_state("TakeoffState", TakeoffState())
    state_machine.add_state("SearchState", SearchState())
    state_machine.add_state("LandState", LandState())
    state_machine.add_state("RTLState", RTLState())
    state_machine.set_initial_state("ArmState")
    
    while not rospy.is_shutdown():
        if not state_machine.is_running:
            pass
        state_machine.execute()
        next_state = state_machine.current_state.next()
        state_machine.transition_to(next_state)
    
if __name__ == "__main__":
    main()
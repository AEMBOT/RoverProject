from control.controllers.Xbox import Xbox

class SimulatedXbox(Xbox):

    # Analog / Joystick Inputs

    def get_joystick_left_x(self):
        return  self.controller.get_axis(0)

    def get_joystick_left_y(self):
        return -self.controller.get_axis(1)
    
    def get_joystick_right_x(self):
        return self.controller.get_axis(2)
    
    def get_joystick_right_y(self):
        return -self.controller.get_axis(3)

    def get_right_trigger(self):
         return super().remap(self.controller.get_axis(4), (-1, 1), (0, 1)) 
    
    def get_left_trigger(self):
        return super().remap(self.controller.get_axis(5), (-1, 1), (0, 1)) 

 
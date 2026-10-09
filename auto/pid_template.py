import numpy as np


def make_car(desired_v:float=20.0, dt:float=0.1) -> dict:
    """ 
    Generates a dictionary that holds all the car's values. Keeps track of state varaibles.
    """
    car_state_dictionary : dict[str, float] = {
        "v" : 0, #velocity of your car 
        "a" : 0, #acceleration of your car
        "t" : 0, #time of your car
        "x" : 0, #position of your car
        "dt" : dt, #time step of your car, how much the time changes every time you update/step
        "desired_v" : desired_v, #desired velocity of your car, the velocity you want to maintain
        "step" : 0,
    
        #hint: use these variables in the integral and derivative portion of your PID control (steps 5 and 6 )
        "error_prev" : None,
        "net_integral" : 0.0
    }
    return car_state_dictionary

def update(car: dict, throttle_perc: float, mass: float = 1000, max_throttle_force: float = 5000, friction: float = 2.0) -> None:
        """
        Updates the car's state variables based on the throttle percentage.
        Use this function after finding throttle percentage to update the car's state variables.

        Inputs:
        car: dictionary containing the car's state variables
        throttle_perc: float, throttle percentage (-1 to 1)

        Outputs:
        None, but updates the car's state variables
        """
        force = throttle_perc * max_throttle_force
        car["a"] = (force / mass) - friction
        car["v"] += car["a"] * car["dt"]
        car["x"] += car["v"] * car["dt"]
        car["t"] += car["dt"]
        car["step"] += 1


def calculate_desired_acceleration(car: dict, K_P: float, K_I: float = 0.0, K_D: float = 0.0) -> tuple[float, float]:
        #input: car["v"], car["desired_v"] (floats)
        #output: desired acceleration and error tuple(float, float)

        controller_error = car["desired_v"] - car["v"]
        # Taking the difference of desired and current velocities to calculate error
        
        prop_gain = K_P * controller_error 
        # Proportional gain

        # From steps 5 & 6 below
        car["net_integral"] += controller_error * car["dt"]
        car["net_integral"] = np.clip(car["net_integral"], -20.0, 20.0) # Extension #1: PID Integral Windup Prevention
        int_gain = K_I * car["net_integral"]
        # Integral gain

        
        if car["error_prev"] is None:
               delta_error = 0
        else:
               delta_error = (controller_error - car["error_prev"])

        dev_gain = K_D * (delta_error / car["dt"])
        # Derivative gain

        desired_acceleration = (prop_gain + int_gain + dev_gain, controller_error)
        # Tuple representation for desired_acceleration and error
        return desired_acceleration

def acceleration_to_throttle_percentage(acceleration_desired: float, mass: float = 1000, max_throttle_force: float = 5000) -> float:
        #input: desired_acceleration(float)
        #output: throttle percentage (float, -1 to 1)
        
        max_acceleration = max_throttle_force / mass
        # F = ma -> A = F/m 

        throttle_percentage = acceleration_desired / max_acceleration
        # Calculating throttle percentage
        
        return float(np.clip(throttle_percentage, -1.0, 1.0))
        # Returning as a float


        
        




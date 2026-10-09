import matplotlib.pyplot as plt
from pid_template import make_car
from pid_template import update
from pid_template import calculate_desired_acceleration
from pid_template import acceleration_to_throttle_percentage

K_P = 0.4
K_I = 0.2
K_D = 0.2
 
STEPS = 550
 
car = make_car(desired_v=20.0, dt=0.1)

#WRITE CODE HERE

velocities = []
errors = []
times = []


for step in range(STEPS):
    
    if 350 < step < 550:  # Extension #2: Friction increase after 350 steps (35 seconds)
        curr_friction = 4.0
    else:
        curr_friction = 2.0
    
    des_acceleration, error = calculate_desired_acceleration(car, K_P, K_I, K_D) 
    # Extracting the values from the tuple


    #           Calling the acceleration to throttle function
    update(car, acceleration_to_throttle_percentage(des_acceleration), friction=curr_friction)
    # Updating car, which takes in the throttle perc
    
    velocities.append(car["v"])
    errors.append(error)
    times.append(car["t"])
    # Adding the velocities and errors to their respective lists


fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(8, 5))
ax1.plot(times, errors)  # Graph for Error vs Time
ax1.set_xlabel("Time(s)")
ax1.set_ylabel("Error(m/s)")
ax1.set_title("Error vs Time")
ax1.axhline(y = 0, color = "y", linestyle = "--")
ax1.axvline(x = 35, color = "r", linestyle = "--")
# Formatting for graph



ax2.plot(times, velocities)  # Graph for Velocity vs Time
ax2.set_xlabel("Time(s)")
ax2.set_ylabel("Velocity(m/s)")
ax2.set_title("Velocity vs Time")
ax2.axhline(y = 20, color = "y", linestyle = "--")
ax2.axvline(x = 35, color = "r", linestyle = "--")
# Also formatting for graph

plt.tight_layout()
plt.show()




    



    
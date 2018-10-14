import numpy as np
import matplotlib.pyplot as plt
import random

def predict_pos(pos, vel, dt, K):
    """
    Predicts future positions in K steps, assuming speed remains constant.
    Args:
      pos: current position of the car.
      vel: current velocity of the car.
      dt: time step.
      K: Number of steps for prediction horizon

    Returns:
      The result of veh_pos_predict, where stores the future positions in K steps based on information of pos and vel.
    """
    veh_pos_predict = []    # Initialize a list
    for step in range(K):
        veh_pos_predict.append(pos + vel*step*dt)  # Store the predicted position in K steps in the list
    return veh_pos_predict
def predict_acc(acc, pos, vel, dt, K, da_list, priorVeh_pos):
    """
    Predicts future accelerations for ego car in K steps, considering the position of the prior car.
    Args:
      acc: current acceleration of the ego car.
      pos: current position of the ego car.
      vel: current velocity of the ego car.
      dt: time step.
      K: Number of steps for prediction horizon
      da_list: A list of delta acceleration subjected to constraints
      priorVeh_pos: The position of the prior car

    Returns:
      The result of acc_list, where stores the eligible accelerations in K steps.
      The result of inMargin, a boolean indicates that if any of these accelerations will cause potential collision
    """
    inMargin = False    # Initialize inMargin as False
    acc_list = np.add(acc, da_list).tolist() # Add current acceleration with delta accelerations to get candidate accelerations
    for acc in acc_list:

        # Calculate predicted position with respect to each candidate acceleration in K steps
        vel = vel + acc*dt
        egoCar_pos_Predict = predict_pos(pos, vel, dt, K)

        # Check from the last position to the first in the list, if a position is found that it's in the margin region,
        # then remove the candidate acceleration as it will cause potential collisions
        for step in range(K-1, -1, -1):
            if egoCar_pos_Predict[step] > -margin and 0 > priorVeh_pos[step]:
                acc_list = [acc_list for acc_list in acc_list if acc_list < acc]
                break
    if len(acc_list) == 0:
        inMargin = True
    return acc_list, inMargin
def calculate_costlist(acc_list, K, vel_target, vel, Cv, Ca):
    """
    Calculates a list of cost with a given acceleration list.
    Args:
      acc_list: A list of accelerations calculated in predict_acc function.
      K: Number of steps for prediction horizon.
      vel_target: The desired velocity of ego car, here is set as maximum velocity.
      vel: current velocity of the ego car.
      Cv: Factor for velocity term in cost function
      Ca: Factor for penalizing acceleration term in cost function

    Returns:
      The result of costlist, where stores calculated costs according acceleration list
    """
    costlist = []   # Initialize a cost candidate list
    for acc in acc_list:
        cost = []
        for step in range(K):
            """
            Calculate the total cost for each candidate acceleration in K steps
            A quadratic cost function is selected
            Formula for the cost function : Cv(vel_target - vel_next)^2 + Ca(acc^2)
            where the first term is the efficiency cost (gap between the next speed and the target speed)
            and the second term penalizes the control signal
            """
            vel_next = vel + acc*step*dt
            cost.append(Cv*(vel_target - vel_next)**2 + Ca*acc**2)
        costlist.append(sum(cost))
    return costlist
def update_egocar(acc, vel, pos, dt, max_a, min_a, max_v, min_v):
    """
    Updates the states of the ego car.
    Args:
      acc: current acceleration of the ego car.
      pos: current position of the ego car.
      vel: current velocity of the ego car.
      dt: time step.
      max_a: Maximum acceleration constraint
      min_a: Minimum acceleration constraint
      max_v: Maximun velocity constraint
      min_v: Minimum velocity constraint

    Returns:
      The result of updated pos, vel, acc of the ego car
    """
    # Update pos, vel and acc subjected to max_a, min_a, max_v and min_v
    if acc > max_a:
        acc = max_a
    elif acc < min_a:
        acc = min_a
    pos += vel * dt
    vel += acc * dt
    if vel > max_v:
        vel = max_v
        acc = 0
    elif vel < min_v:
        vel = min_v
        acc = 0
    return pos, vel, acc
def update_priorcar(pos, vel, dt):
    """
    Updates the states of the prior car.
    Args:
      pos: current position of the prior car.
      vel: current velocity of the prior car.
      dt: time step.

    Returns:
      The result of updated pos, vel, acc of the prior car
    """
    # Update the prior car, assuming constant speed
    pos += vel * dt
    return pos,vel
def setPlot(lane_length, margin, max_v, min_v, max_a, min_a):
    fig = plt.figure(figsize=(10, 7))
    fig.suptitle('Episode ' + str(simTime+1))
    ax1 = fig.add_subplot(2, 2, 2, aspect='equal')
    ax2 = fig.add_subplot(2, 2, 3)
    ax3 = fig.add_subplot(2, 2, 4)
    ax4 = fig.add_subplot(2, 2, 1, aspect='equal')
    ax1.set_xlim((-lane_length, lane_length))
    ax1.set_ylim((-lane_length, lane_length))
    ax1.set_ylabel("Prior car position (m)")
    ax1.set_xlabel("Ego car position (m)")
    ax1.add_patch(plt.Rectangle((-margin, -lane_length), lane_length + margin, lane_length, color='k', alpha=0.3))
    ax1.add_patch(plt.Rectangle((0, -lane_length), lane_length, lane_length, color='r', alpha=0.3))
    ax1.text(lane_length/2 - 15 , -lane_length/2 -8, "Obstacle \n Region")
    ax1.text(-margin, -lane_length / 2 + 5 , "Margin", rotation = 90)
    ax1.set_title('Obstacle region')
    ax2.set_title('Velocity diagram')
    ax2.set_ylim((min_v * 1.1, max_v * 1.1))
    ax2.set_ylabel("Velocity (m/s)")
    ax2.set_xlabel("Simulation step")
    ax3.set_title('Acceleration of Ego car')
    ax3.set_ylim((min_a * 1.1, max_a * 1.1))
    ax3.set_ylabel("Acceleration (m/s^2)")
    ax3.set_xlabel("Simulation step")
    return fig, ax1, ax2, ax3, ax4
def Plot(ax1, ax2, ax3, ax4, egoCar_pos, priorCar_pos,egoCar_vel, priorCar_vel, egoCar_acc, step, lane_length):
    ax1.scatter(egoCar_pos, priorCar_pos, s=1)
    ax2.scatter(step, egoCar_vel, s=1, c='b', label='Ego Car speed')
    ax2.scatter(step, priorCar_vel, s=1, c='r', label='Prior Car speed')
    if step == 1:
        ax2.legend(loc="upper right")
    ax3.scatter(step, egoCar_acc, s=1, c='b')
    ax4.cla()
    ax4.set_title('Birds-eye view of the intersection')
    ax4.set_xlim((-lane_length, lane_length))
    ax4.set_ylim((-lane_length, lane_length))
    ax4.set_ylabel("Y(m)")
    ax4.set_xlabel("X(m)")
    ax4.scatter(egoCar_pos, 0)
    ax4.text(egoCar_pos, 0.5, "Ego Car")
    ax4.scatter(0, priorCar_pos)
    ax4.text(0.5, priorCar_pos, "Prior Car")

    plt.subplots_adjust(hspace=0.35)
    plt.pause(dt)

K = 50  # Number of steps for prediction horizon
dt = 0.1 # Time step in seconds
vel_target = 20.0   # Take max velocity as the target velocity
margin = 5.0  # Margin as the safety distance before the intersection
Cv = 1.0  # Factor for velocity term in cost function
Ca = 2.0 # Factor for acceleration term in cost function
da_list = [-0.2, -0.1, 0.0, 0.1, 0.2]   # List of delta acceleration(resolution 0.1 m/s^2)
max_a, min_a, max_v, min_v = 2.0, -2.0, 20.0, 0.0   # Constraints setup
lane_length = 50.0    # length of each lane in meters
crash = 0   # Number of crashes
timeout = 0 # Number of timeouts
episode = 10    # Number of episodes for simulation
timeout_step = 150 # Timeout threshold
Goal = 30.0   # Goal for the ego car
toGoal_dis = 0.0 # Initialize the distance from start point to goal

for simTime in range(episode):
    # Setup the plot
    fig, ax1, ax2, ax3, ax4 = setPlot(lane_length, margin, max_v, min_v, max_a, min_a)

    # Initialize positions, velocities and accelerations for the ego car and the prior car
    egoCar_pos, priorCar_pos = random.randrange(-lane_length,-margin), random.randrange(-lane_length,0)
    egoCar_vel, priorCar_vel = random.randrange(5,15), random.randrange(5,20)
    egoCar_acc, priorCar_acc = 0.0, 0.0
    toGoal_dis = Goal - egoCar_pos  # Distance from start point to goal
    acc_accum = 0.0  # Initialize accumulated acceleration


    # Initialize first acceleration with initial egoCar_acc, initialize step
    acc = egoCar_acc
    step = 0

    while egoCar_pos < Goal and step < timeout_step:

        # Predict the state of the prior car in K steps
        priorCar_acc += random.uniform(-0.2, 0.2)   # Add some noise to the priorCar_acc
        priorCar_vel += priorCar_acc*dt # Update priorCar_vel
        if priorCar_vel < 0:
            priorCar_vel = 0    # Prior car cannot go backwards, otherwise it will end up in endless loop
        priorCar_pos_predict = predict_pos(priorCar_pos, priorCar_vel, dt, K)    # Predict the priorCar_pos in K steps

        # Select candidate accelerations in K steps, subjected to the constraints
        egoCar_acclist, inMargin= predict_acc(egoCar_acc, egoCar_pos, egoCar_vel, dt, K, da_list, priorCar_pos_predict)

        if inMargin:
            # If all candidate accelerations will cause ego car go into/beyond margin zone, take the min delta acceleratoin
            acc -= 0.2
        else:
            # Otherwise calculate the costs according to acceleration list
            costlist = calculate_costlist(egoCar_acclist, K, vel_target, egoCar_vel, Cv, Ca)
            # Afterwards choose the acceleration with the min cost
            acc = egoCar_acclist[np.argmin(costlist)]
        acc_accum += abs(acc)    # Accumulate acceleration

        # Update egoCar_pos, egoCar_vel and egoCar_acc according to the chosen acceleration, subjecting to max_a, min_a, max_v and min_v
        egoCar_pos, egoCar_vel, egoCar_acc = update_egocar(acc, egoCar_vel, egoCar_pos, dt, max_a, min_a, max_v, min_v)

        # Update priorCar_pos and priorCar_vel
        priorCar_pos, priorCar_vel = update_priorcar(priorCar_pos, priorCar_vel, dt)
        # If ego car enters the obstacle region, mark as a crash case
        if egoCar_pos > 0 > priorCar_pos:
            crash += 1
            break

        step += 1
        if step > (timeout_step - 1) :
            # Timeout case
            timeout += 1

        # Plot the results
        Plot(ax1, ax2, ax3, ax4, egoCar_pos, priorCar_pos, egoCar_vel, priorCar_vel, egoCar_acc, step, lane_length)
    evaluate_efficiency = toGoal_dis / step # Evaluate the efficiency in meter/step
    evaluate_comfort = acc_accum / step # Evaluate the comfort with average acceleration
    print("\nEpisode : " + str(simTime + 1))
    print("Simulation runs for " +str(step) + " steps.")
    print("Efficiency evaluation : " +str(evaluate_efficiency) + " step(s) can reach 1 meter.")
    print("Efficiency comfort : Average acceleration is " + str(evaluate_comfort) + " m/s^2 in " + str(step) + " step(s)")
    plt.close()
print("Crashed " + str(crash) +" times in " + str(episode) + " episodes.")
print("Timeouted " + str(timeout) +" times in " + str(episode) + " episodes.")

import numpy as np

# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!
    if len(Rover.goal_angles) > 0:
        if Rover.vel < Rover.max_vel:
            # Set throttle value to throttle setting
            Rover.throttle = Rover.throttle_set
        else: # Else coast
            Rover.throttle = 0
        Rover.brake = 0
        # Set steering to average angle clipped to the range +/- 15
        Rover.steer = np.mean(Rover.goal_angles * 180/np.pi)
            
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None and (len(Rover.goal_angles) == 0) and not(Rover.near_sample):

        #If rover is trying to move but is stuck, go backwards then continue
        if (Rover.throttle > 0) and (Rover.vel < 0.3) and (Rover.forward_time > 3) and not(Rover.picking_up):
            Rover.throttle = 0
            Rover.last_pos = Rover.pos
            Rover.mode = 'get_unstuck'

        if (Rover.steer == 15) or (Rover.steer == -15):
            if(Rover.steer == Rover.past_steer):
                Rover.steer_time = Rover.steer_time + 0.03
            else:
                Rover.steer_time = 0
            if(Rover.steer_time > Rover.max_steer_time):
                Rover.mode = 'steer_anew'
                Rover.to_steer = Rover.steer

        # Check for Rover.mode status
        if Rover.mode == 'forward':
            Rover.forward_time  = Rover.forward_time + 0.03
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.turning = True
                    Rover.steer = 0
                    Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    # Rover.steer = 15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                    Rover.mode = 'turning'
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
        
        #If rover is stuck, maneuver out of conondrum
        elif Rover.mode == 'get_unstuck':
            if (Rover.stuck_time < Rover.max_stuck_time):
                Rover.throttle = -0.2
                Rover.steer = 15
                Rover.stuck_time  = Rover.stuck_time + 0.03
                Rover.mode = 'get_unstuck'
            else:
                #In case rover is stuck in different situation
                if(np.abs(Rover.pos[0] - Rover.last_pos[0]) < 0.3 and np.abs(Rover.pos[1] - Rover.last_pos[1]) < 0.3):
                    Rover.mode = 'maneuver'
                    Rover.forward_time = 0
                    Rover.stuck_time = 0
                #else rover was freed
                else:
                    Rover.brake = Rover.brake_set
                    Rover.throttle = Rover.throttle_set
                    Rover.forward_time = 0
                    Rover.mode = 'forward'
                    Rover.stuck_time = 0

        elif Rover.mode == 'maneuver':
            if(Rover.maneuver_time < 2):
                Rover.throttle = 0
                Rover.steer = 15
                Rover.maneuver_time = Rover.maneuver_time + 0.03
            else:
                Rover.mode = 'forward'
                Rover.maneuver_time = 0

        elif Rover.mode == 'turning':
            if(Rover.duration_steer <2):
                Rover.steer = 15
                Rover.duration_steer = Rover.duration_steer +0.03
            else:
                Rover.mode = 'forward'
                # Rover.throttle = Rover.throttle_set
                Rover.duration_steer = 0
                Rover.forward_time = 0
                Rover.steer_time = 0

        #If rover is continuously going in a circle, steer anew
        elif Rover.mode == 'steer_anew':
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
            if(Rover.duration_steer < 2):
                if(Rover.to_steer == 15):
                    Rover.steer = -15
                else:
                    Rover.steer = 15
                Rover.duration_steer = Rover.duration_steer +0.03
            else:
                Rover.mode = 'forward'
                Rover.throttle = Rover.throttle_set
                Rover.duration_steer = 0
                Rover.steer_time = 0

        #If samples have been located, head to start position
        if Rover.samples_collected == 6:
            if(np.abs(Rover.pos[0] - Rover.start_pos[0]) < 6 and np.abs(Rover.pos[1] - Rover.start_pos[1]) < 6):
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.finished = True

    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and not Rover.picking_up:
        Rover.brake = Rover.brake_set
        Rover.mode = 'stop'
        Rover.steer = 0
        Rover.send_pickup = True

    if Rover.picking_up:
        Rover.throttle = 0
        Rover.steer = 0
        Rover.samples_collected += 1
        Rover.brake = Rover.brake_set
        Rover.forward_time = 0
        Rover.mode = 'forward'
    
    Rover.past_steer = Rover.steer
    return Rover


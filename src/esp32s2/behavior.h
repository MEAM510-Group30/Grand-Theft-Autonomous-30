/*
    Discription:
    
        In this file, we define the functions for the behavior tree.
        Like a tree, there are different branches or modes: 
            fully-automatic (competition), wall-following, car-pushing, trophy-moving, fully-manual, and do nothing.
        Each mode has its own logic, like what to do first, what to do next, what if stucked or failed, etc.
        
        At the begining of each loop, we'll call a mode dection function to decide which mode to use in this loop, then we'll call the corresponding function to do the job. 
        This detection function should read a global variable that can be changed by html website at any time, and also the logic itself as there might be failure or stucked situations. 
        
        When in fully-auto mode, there should also be a evaluation function that is called at the begining of each loop, after the detection function.
        This function evaluates the friendly and enemy team's scores, and decide whether to grab the real/fake trophy, to push the police car, or to do nothing.

        When in wall-following mode, the robot will first choose a wall acoording to the current position, move close to the wall, turn align with the wall, then follow the wall until obstacles in front of the robot are within a small distance. 
        Position PID is very likely to be needed in this mode.

        When in car-pushing mode, the robot will first get the police car position provided by the staff. 
        Position PID is very likely to be needed in this mode. 
        The robot need to approach the car from the friendly side, and head towards the center of the target location in the enemy's side.
        When the robot is close and well-aligned to the car, it will open the gripper to the maximum position, then move forward to push the car.
        Every loop we'll check the position of the police car. 
        If the car derivatived largely from the center line, the robot will stop pushing, move backward, recalculate the direction, realign, and push again.
        When the car is pushed to the target, the robot will move back to the friendly side, and choose do nothing mode before the next task is assigned.

        When in trophy-moving mode, the robot will first detect the trophy's rough direction using all photosensors.
        If none of the sensors detect a desired signal, the robot will turn around in the same position until a desiredw signal is picked up by any of the sensors.
        Then it will turn to that direction, move forward under the guidance of the two photosensors mounted on the front servo, and uses the ToF to estimate the position of the trophy.
        When the trophy is close enough, the robot will grab the trophy, move to the target location, and release the trophy.
        Trophy loss is likely to happen in this mode, so we need to check the trophy using the photosensors and the ToF sensors every loop.
        If the distance between the trophy and the robot is too large, or the trophy signal is lost for several loops, the robot will return to look for the trophy again.
        Target positions of the real/fake trophy are internally defined according to the rules.

        When in fully-manual mode, the robot will do nothing but wait for the user to control it using the html website, just like Lab-4.
*/ 


#ifndef BEHAVIOR_H
#define BEHAVIOR_H





#endif
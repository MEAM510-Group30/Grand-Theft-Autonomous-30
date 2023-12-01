/*
    Authors:

        @jbwenjoy: Furina de Fontaine

        @YuruiWang821: Yurui

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

enum Mode
{
    AUTO,
    WALL,
    PUSH,
    TROPHY,
    MANUAL,
    NOTHING
};

class Behavior
{
private:
    // Private methods for each behavior mode
    void fullyAutomatic()
    {
        // Logic for fully-automatic mode
    }

    void wallFollowing()
    {
        // Logic for wall-following mode
        bool flag = false;
        int TOFValue_Front = getTOFValue_Front();    //return the front TOF value
        int TOFValue_left = getTOFValue_left();    //return the left TOF value
        int dist_to_turn = 15;  //the distance to turn at the corner
        Actions wall_follow_action;

        if(TOFValue_Front <= dist_to_turn && !flag){
            flag = true;
        }
        if(atDesAngel() && flag){  //define () to return a flag when finish turning
            flag = false;
        }
        if(flag){
            wall_follow_action.turnLeft90degree();
        }
        if(!flag){
            wall_follow_action.followWall();
        }
    }

    void carPushing()
    {
        // Logic for car-pushing mode
    }

    void trophyMoving()
    {
        // Logic for trophy-moving mode
    }

    void fullyManual()
    {
        // Logic for fully-manual mode
    }

    void doNothing()
    {
        // Logic for do-nothing mode
    }

public:
    // Here we need some variables to store the current position, the police car position,
    // the trophy position, the trophy signal, the friendly and enemy team's scores, etc.
    // These variables should be updated every loop, and can be changed by html website at any time.
    // So we also need some public functions to read and write these variables.
    Mode current_mode = NOTHING; // Initial mode

    // Constructor, copy constructor and destructor
    Behavior() {}
    Behavior(const Behavior &old) {}
    ~Behavior() {}

    // Method to detect mode
    void detectMode()
    {
        // Logic to determine the mode based on global variables or HTML input
    }

    // Main behavior tree execution method
    void runBehaviorTree()
    {
        while (true)
        {
            detectMode(); // Detect current mode

            // Execute corresponding mode logic
            if (current_mode == AUTO)
            {
                fullyAutomatic();
            }
            else if (current_mode == WALL)
            {
                wallFollowing();
            }
            else if (current_mode == PUSH)
            {
                carPushing();
            }
            else if (current_mode == TROPHY)
            {
                trophyMoving();
            }
            else if (current_mode == MANUAL)
            {
                fullyManual();
            }
            else
            {
                doNothing();
            }

            // Implement any necessary delays or loop control
        }
    }
};

// Below shows how to use the behavior tree
// void loop() {
//     BehaviorTree behavior_tree;
//     behavior_tree.runBehaviorTree(); // Run the behavior tree
// }

#endif
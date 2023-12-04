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

#include "actions.h"

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
    bool atDesiredOrientation(float desired_theta) // alsolute angle from vive, currently unable to deal with overshot
    {
        float threshold = 5.0; // degree, to be tuned
        return (abs(vive_theta - desired_theta) <= threshold);
    }

    bool atDesiredPosition(float desired_x, float desired_y) // alsolute position from vive, currently unable to deal with overshot
    {
        float threshold = 10.0; // mm, to be tuned
        return (abs(vive_x - desired_x) <= threshold) && (abs(vive_y - desired_y) <= threshold);
    }

    // Private methods for each behavior mode
    void fullyAutomatic()
    {
        // TODO: Logic for fully-automatic mode
        return;
    }

    void wallFollowing() // Logic for wall-following mode
    {
        uint8_t dist_to_turn_thres = 150; // distance threshold to turn at the corner, mm
        float previous_wall_theta;        // to record present theta when finish turning

        if (tof_front <= dist_to_turn_thres && !needTurnFlag) // if front distance is too small, turn right
        {
            needTurnFlag = true;
        }
        if (atDesiredOrientation(previous_wall_theta + 90) && needTurnFlag) // if finish turning, set needTurnFlag to false
        {
            needTurnFlag = false;
            previous_wall_theta = sensor.vive_theta;
        }
        if (needTurnFlag) // if need to turn, turn right at the same place
        {
            action.turnRightSamePlace();
        }
        if (!needTurnFlag) // if don't need to turn, follow the wall, and keep updating previous_wall_theta
        {
            action.followWall();
            previous_wall_theta = sensor.vive_theta;
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
        switch (html_manual_direction)
        {
        case 'f':
            action.moveForward(html_speed);
            break;
        case 'b':
            action.moveBackward(html_speed);
            break;
        case 'l':
            action.turnLeft(html_speed, html_turn_rate);
            break;
        case 'r':
            action.turnRight(html_speed, html_turn_rate);
            break;
        case 'o':
        default:
            action.stop();
            break;
        }
    }

    void doNothing()
    {
        // Logic for do-nothing mode
        return;
    }

public:
    // Here we need some variables to store ToF data, current vive position, police car position,
    // the trophy direction, the trophy signal freq, the friendly and enemy team's scores, etc.
    // These variables should be updated every loop, and can be changed by html website at any time.
    // So we also need some public functions to read and write these variables.

    Mode current_mode; // Initial mode
    Actions action;    // Action object

    bool needTurnFlag; // for wall following

    // sensors
    uint8_t tof_front;    // front TOF value, mm
    uint8_t tof_left;     // left TOF value, mm
    int vive_x;           // vive x coordinate, mm
    int vive_y;           // vive y coordinate, mm
    int vive_theta;       // vive heading, degree
    int police_x;         // police car x coordinate, mm
    int police_y;         // police car y coordinate, mm
    int trophy_direction; // trophy direction, degree

    // html
    char html_state;            // html state, 'a' for auto, 'w' for wall, 'p' for push, 't' for trophy, 'm' for manual, 'n' for nothing
    char html_manual_direction; // html manual direction, 'f' for forward, 'b' for backward, 'l' for left, 'r' for right, 'o' for stop
    bool html_jaw_open;         // html jaw, true for open, false for close
    int html_speed;             // html speed in duty cycle, 0-4095
    int html_turn_rate;         // html turn rate, 0-100

    // Constructor, copy constructor and destructor
    Behavior() : current_mode(NOTHING),
                 action(),
                 needTurnFlag(false),
                 tof_front(0), tof_left(0),
                 vive_x(0), vive_y(0), vive_theta(0),
                 police_x(0), police_y(0), trophy_direction(0),
                 html_state('n'), html_manual_direction('o'), html_jaw_open(false),
                 html_speed(0), html_turn_rate(0)
    {
    }
    Behavior(const Behavior &old) {}
    ~Behavior() {}

    void updateBehaviorClassHTMLVariables(char state, char manual_direction, bool jaw_open, int speed, int turn_rate)
    {
        // TODO: call this function in the main loop to update all HTML variables in this class
        html_state = state;
        html_manual_direction = manual_direction;
        html_jaw_open = jaw_open;
        html_speed = speed;
        html_turn_rate = turn_rate;
    }

    void updateBehaviorClassSensorsData(uint8_t front, uint8_t left, int x, int y, int theta, int police_x, int police_y, int trophy_direction)
    {
        // TODO: call this function in the main loop to update all sensors data in this class
        tof_front = front;
        tof_left = left;
        vive_x = x;
        vive_y = y;
        vive_theta = theta;
        police_x = police_x;
        police_y = police_y;
        trophy_direction = trophy_direction;
    }

    // Method to detect mode
    void detectMode()
    {
        // Logic to determine the mode based on global variables or HTML input
        switch (html_state)
        {
        case 'a':
            current_mode = AUTO;
            break;
        case 'w':
            current_mode = WALL;
            break;
        case 'p':
            current_mode = PUSH;
            break;
        case 't':
            current_mode = TROPHY;
            break;
        case 'm':
            current_mode = MANUAL;
            break;
        case 'n':
        default:
            current_mode = NOTHING;
            break;
        }
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
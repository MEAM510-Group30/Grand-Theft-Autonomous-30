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
    // Determine the mode based on global variables or HTML input
    void detectMode()
    {
        // Called at the begining of runBehaviorTree()
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

    bool compareOrientation(float desired_theta, float current_theta) // alsolute angle from vive, currently unable to deal with overshot
    {
        float threshold = 5.0; // degree, to be tuned
        return (abs(current_theta - desired_theta) < threshold);
    }

    bool comparePosition(float desired_x, float desired_y, float current_x, float current_y, float threshold = 10.0) // alsolute position from vive, currently unable to deal with overshot
    {
        return ((current_x - desired_x) * (current_x - desired_x) + (current_y - desired_y) * (current_y - desired_y) <= threshold * threshold);
    }

    // Private methods for each behavior mode
    void fullyAutomatic()
    {
        // TODO: Logic for fully-automatic mode
        return;
    }

    void wallFollowing() // Logic for wall-following mode
    {
        int dist_to_turn_thres = 350; // distance threshold to turn at the corner, mm
        float previous_wall_theta;        // to record present theta when finish turning

        if (tof_front <= dist_to_turn_thres && !needTurnFlag) // if front distance is too small, turn right
        {
            needTurnFlag = true;
            Serial.println("Wall.Need Turn");
        }
        if (compareOrientation(previous_wall_theta + 90, vive_theta) && needTurnFlag) // if finish turning, set needTurnFlag to false, and reset PID
        {
            needTurnFlag = false;
            previous_wall_theta = vive_theta;
            action.PID_wall.resetPID();
            Serial.println("Wall.Finish Turn");
        }
        if (needTurnFlag) // if need to turn, turn right at the same place
        {
            action.turnRightSamePlace();
            Serial.println("Wall.Right");
        }
        if (!needTurnFlag) // if don't need to turn, follow the wall, and keep updating previous_wall_theta
        {

            action.followWallForward();
            Serial.println("Wall.Forward");
            previous_wall_theta = vive_theta;
        }

        // End: I don't think we need the robot to terminate this mode automatically, so nothing to do here
    }

    void carPushing()
    {
        // Logic for car-pushing mode
        if (!carPushingInitFlag) // if not initialized, initialize
        {
            carPushingInitFlag = true;
            carPushingApproachCarFlag = false;
            carPushingApproachAlignFlag = false;
            // Push the police car 400 mm along x+, set the target position
            push_target_x = police_x + 400; // target x coordinate, mm
            push_target_y = police_y;       // target y coordinate, mm
            action.releaseTrophy();
            Serial.println("Push.Init");
        }

        if (comparePosition(push_target_x, push_target_y, police_x, police_y)) // if at the target position, stop pushing and do nothing
        {
            action.stop();
            // reset flags
            carPushingInitFlag = false;
            carPushingApproachCarFlag = false;
            carPushingApproachAlignFlag = false;

            current_mode = NOTHING;
            Serial.println("Push.Finished");
        }
        else // if not at the target position, push the car
        {
            // TODO: how to push the car
            // More specifically:
            //   how to approach the car: moveTo function
            //   how to check if the car derivates from the line: vector calculation
            //   how to recalculate the direction and realign: repeat the approach and align process

            // Approach the car
            if (!carPushingApproachCarFlag)
            {
                // generate target position
                approach_target_x = police_x - 200;
                approach_target_y = police_y;
                
                action.moveToPosition(approach_target_x, approach_target_y);
                Serial.println("Push.Approaching Car");
                if (comparePosition(approach_target_x, approach_target_y, police_x, police_y, 50.0))
                {
                    carPushingApproachCarFlag = true;
                    Serial.println("Push.Approached Car");
                }
            }
            // After approaching the car, align with the car and the target position
            else if (!carPushingApproachAlignFlag)
            {
                // generate orientation
                approach_target_theta = atan2(push_target_y - police_y, push_target_x - police_x) * 180 / PI;
                
                action.turnToHeading(approach_target_theta);
                Serial.println("Push.Aligning");
                if (compareOrientation(approach_target_theta, vive_theta))
                {
                    carPushingApproachAlignFlag = true;
                    Serial.println("Push.Aligned");
                }
                
            }
            else // if aligned, push the car
            {
                action.moveForward(); // use html_speed
                // action.moveForward(4000); // use 4000 as speed
                Serial.println("Push.Pushing");

                // calculate the distance between the car and the center line
                // if the car derivates from the line, repeat the approaching and aligning process
                float vec_cd[2] = {police_x - push_target_x, police_y - push_target_y}; // vector from the car to the target position
                float vec_rd[2] = {vive_x - push_target_x, vive_y - push_target_y}; // vector from the robot to the target position
                auto rd_dot_cd = vec_cd[0] * vec_rd[0] + vec_cd[1] * vec_rd[1]; // dot product of the two vectors
                auto scalar = rd_dot_cd / (vec_rd[0] * vec_rd[0] + vec_rd[1] * vec_rd[1]); // scalar of the projection of vec_cd to vec_rd
                float vec_cd_prj_to_rd[2] = {scalar * vec_rd[0], scalar * vec_rd[1]}; // projection of vec_cd to vec_rd
                float vec_c_dist[2] = {vec_cd[0] - vec_cd_prj_to_rd[0], vec_cd[1] - vec_cd_prj_to_rd[1]}; // vector from the car to the center line
                auto derivation = sqrt(vec_c_dist[0] * vec_c_dist[0] + vec_c_dist[1] * vec_c_dist[1]); // distance between the car and the center line
                if (derivation > 100)  // if the car derivates from the line 100mm
                {
                    carPushingApproachCarFlag = false;
                    carPushingApproachAlignFlag = false;
                    Serial.println("Push.Realigning");
                }
            }
        }
    }

    void trophyMoving(int trophy_freq = 550)
    {
        // Logic for trophy-moving mode 

        // Step I: find and move to trophy

        // Step II:

        // Step III:
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
        case 's':
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

    // Task variables
    bool needTurnFlag;       // for wall following
    bool carPushingInitFlag; // for car pushing
    bool carPushingApproachCarFlag; // for car pushing
    bool carPushingApproachAlignFlag; // for car pushing
    float push_target_x;       // for car pushing
    float push_target_y;       // for car pushing
    float approach_target_x;   // for car pushing
    float approach_target_y;   // for car pushing
    float approach_target_theta; // for car pushing

    // Sensors
    // Note that all coordinate and distance values here should be in mm
    uint8_t tof_front;    // front TOF value, mm
    uint8_t tof_left;     // left TOF value, mm
    float vive_x;           // vive x coordinate, mm
    float vive_y;           // vive y coordinate, mm
    float vive_theta;       // vive heading, degree
    float police_x;         // police car x coordinate, mm
    float police_y;         // police car y coordinate, mm
    float trophy_direction; // trophy direction, degree

    // HTML
    char html_state;            // html state, 'a' for auto, 'w' for wall, 'p' for push, 't' for trophy, 'm' for manual, 'n' for nothing
    char html_manual_direction; // html manual direction, 'f' for forward, 'b' for backward, 'l' for left, 'r' for right, 'o' for stop
    bool html_jaw_open;         // html jaw, true for open, false for close
    int html_speed;             // html speed in duty cycle, 0-4095
    int html_turn_rate;         // html turn rate, 0-100

    // Constructor, copy constructor and destructor
    Behavior() : current_mode(NOTHING),
                 action(),

                 needTurnFlag(false),
                 carPushingInitFlag(false), carPushingApproachCarFlag(false),
                 push_target_x(0), push_target_y(0),

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
        // Call this function in the main loop to update all HTML variables in this class
        html_state = state;
        html_manual_direction = manual_direction;
        html_jaw_open = jaw_open;
        html_speed = speed;
        html_turn_rate = turn_rate;
    }

    void updateBehaviorClassSensors(uint8_t front, uint8_t left, int x, int y, int theta, int police_x, int police_y, int trophy_direction)
    {
        // Call this function in the main loop to update all sensors data in this class
        tof_front = front;
        tof_left = left;
        vive_x = x;
        vive_y = y;
        vive_theta = theta;
        police_x = police_x;
        police_y = police_y;
        trophy_direction = trophy_direction;
    }

    // Main behavior tree execution method
    void runBehaviorTree()
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
            carPushingInitFlag = false;  // in case we manually terminated the car pushing mode last time
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
    }
};

// // Below shows how to use the behavior tree
// BehaviorTree behavior_tree;
// void loop() {
//     updateBehaviorClassHTMLVariables(...); // Update HTML variables in the behavior class
//     updateBehaviorClassSensorsData(...); // Update sensors data in the behavior class
//     behavior_tree.runBehaviorTree(); // Run the behavior tree
// }

#endif
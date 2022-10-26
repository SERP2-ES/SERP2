#include "arduino_bridge_node.h"


bool VelocitySetPoint(serp::VelocitySetPointRequest &req, serp::VelocitySetPointResponse &res)
{
    if(req.state) {
        robot_state = ManualControl;
        // Save requested velocities
        robot.motor_left_velocity = req.vel_motor_left;
        robot.motor_right_velocity = req.vel_motor_right;
        res.success = true;
    }
    else if(req.state == false) {
        // Exit FixedVelocity mode and stop the robot
        robot_state = Stopped;
        robot.motor_left_velocity = 0;
        robot.motor_right_velocity = 0;
        res.success = true;
    }
    else {
        res.success = false;
    }
    return res.success;
}

bool sendBatteryLevel(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {
    res.success = true;
    res.message = std::to_string(robotget.battery_level);
    return true;
}

void getInfo(const serp::RobotInfo &msg)
{
    robotget.battery_level=msg.battery_level;
    robotget.vel_linear=msg.vel_linear;
    robotget.vel_angular=msg.vel_angular;
}

void cb_robot_state(const std_msgs::StringConstPtr &state) {
    if(state->data == "Stopped" || state->data == "ReadingProgrammingSheet") robot_state = Stopped;
    else if(state->data == "ManualControl") robot_state = ManualControl;
    else if(state->data == "Executing") robot_state = Executing;
}

void cb_matrix(const serp::Matrix &msg)
{
    for (int i=0; i < 100; i++)
    {
        for (int j=0 ; j < 100 ; j++)
        {
            matrixValores[i][j]=msg.matrix_values[100*i+j];
            ligacoes[i][j]=msg.matrix_links[100*i+j];
        }
    }
}

int main(int argc, char** argv)
{
    // Initialize robot state
    robot_state = Stopped;
    float velocidades[2]={0};
    float velocidadesblocos[2]={0};


    ros::init(argc, argv, "arduino_bridge_node");
    ros::NodeHandle n_public;
    serp::Velocity vel_cmd;
    ros::ServiceServer service_velocity = n_public.advertiseService("velocity_setpoint", VelocitySetPoint);
    ros::ServiceServer service_battery = n_public.advertiseService("srv_battery_level", sendBatteryLevel);

    int8_t pvel_l;
    int8_t pvel_r;

    ros::Publisher send_velocities = n_public.advertise<serp::Velocity>("motors_vel", 2);
    ros::Subscriber receive_info = n_public.subscribe("hardware_info", 2, getInfo);

    // Subscriber for Robot State
    ros::Subscriber sub_robot_state = n_public.subscribe("robot_state", 2, cb_robot_state);

    // Subscriber for matrixes
    ros::Subscriber sub_matrixes = n_public.subscribe("send_matrix", 1, cb_matrix);


    while (ros::ok())
    {
        ros::Duration(0.1).sleep();

        //manual robot control mode
        if(robot_state == ManualControl)
        {
            pvel_l=vel_cmd.vel_motor_left;
            pvel_r=vel_cmd.vel_motor_right;
            vel_cmd.vel_motor_left=robot.motor_left_velocity;
            vel_cmd.vel_motor_right=robot.motor_right_velocity;
            //updates motor speeds only if they have changed
            if(pvel_l!=vel_cmd.vel_motor_left || pvel_r!=vel_cmd.vel_motor_right)
            {
                send_velocities.publish(vel_cmd);
            }
        }
        else if(robot_state == Stopped)
        {
            pvel_l=vel_cmd.vel_motor_left;
            pvel_r=vel_cmd.vel_motor_right;
            vel_cmd.vel_motor_left=0;
            vel_cmd.vel_motor_right=0;
            //updates motor speeds only if they have changed
            if(pvel_l!=vel_cmd.vel_motor_left || pvel_r!=vel_cmd.vel_motor_right)
            {
                send_velocities.publish(vel_cmd);
            }
        }
        else if(robot_state == Executing)
        {
            //processing cycle

            for (int j = 0; j < 100 ; j++) {
                std::cout << "row:" << j << " ";
                for (int i= 0; i < 100 ; i++) {
                    std::cout << matrixValores[j][i]<< " ";
                }
                std::cout << "\n";
            }


            //updates motor speeds according to code from sheet
            verificarBlocos(ligacoes, matrixValores, velocidades);
            robot.motor_left_velocity=velocidades[2];
            robot.motor_right_velocity=velocidades[1];
            pvel_l=vel_cmd.vel_motor_left;
            pvel_r=vel_cmd.vel_motor_right;
            vel_cmd.vel_motor_left=velocidades[2];
            vel_cmd.vel_motor_right=velocidades[1];
            //updates motor speeds only if they have changed
            if(pvel_l!=vel_cmd.vel_motor_left || pvel_r!=vel_cmd.vel_motor_right)
            {
                send_velocities.publish(vel_cmd);
            }
        }
        ros::spinOnce();
    }
    return 0;
}

/**
 * @file controller.cpp
 * @brief Controller file
 *
 */

#include <Sai2Model.h>
#include "Sai2Primitives.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>

using namespace std;
using namespace Eigen;

#include <signal.h>
bool runloop = false;
void sighandler(int){runloop = false;}

#define RAD(deg) ((double)(deg) * M_PI / 180.0)

#include "redis_keys.h"

// Location of URDF files specifying world and robot information
const string robot_file = "./resources/mmp_panda.urdf";
const string robot2_file = "./resources/kuka_iiwa.urdf";

unsigned long long controller_counter = 0;

const bool inertia_regularization = true;

enum State
{
    POSTURE = 0,
    HOOP_MOVE_BASE = 1,
    HOOP_MOVE_EE = 2,
    IDLE =     3,
    INITIALIZE =  4
};

int main() {

    // initial state
    int state = POSTURE;
    string controller_status = "1";

    // start redis client
    auto redis_client = RedisClient();
    redis_client.connect();

    // set up signal handler
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGINT, &sighandler);

    // load robots, read current state and update the model
    auto robot = new Sai2Model::Sai2Model(robot_file, false);
    robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
    robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
    robot->updateModel();
    
    auto robot2 = new Sai2Model::Sai2Model(robot2_file, false);
    robot2->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY_SHOOTER);
    robot2->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY_SHOOTER);
    robot2->updateModel();

    // prepare controller
    int dof = robot->dof();
    int dof2 = robot2->dof();
    VectorXd command_torques = VectorXd::Zero(dof);
    VectorXd command_torques2 = VectorXd::Zero(dof2);
    MatrixXd N_prec = MatrixXd::Identity(dof, dof);
    MatrixXd N_prec2 = MatrixXd::Identity(dof2, dof2); // for the second robot

    // pose task
    const string control_link = "link7";
    const Vector3d control_point = Vector3d(0, 0, 0.07);
    auto posori_task = new Sai2Primitives::PosOriTask(robot, control_link, control_point);
    posori_task->_use_interpolation_flag = true;
    posori_task->_use_velocity_saturation_flag = true;

    VectorXd posori_task_torques = VectorXd::Zero(dof);
    posori_task->_kp_pos = 400.0;
    posori_task->_kv_pos = 40.0;
    posori_task->_kp_ori = 400.0;
    posori_task->_kv_ori = 40.0;
    
    //second robot
    const string control_link2 = "link6";
    const Vector3d control_point2 = Vector3d(0, 0, 0.07);
    auto posori_task2 = new Sai2Primitives::PosOriTask(robot2, control_link2, control_point2);
    posori_task2->_use_interpolation_flag = true;
    posori_task2->_use_velocity_saturation_flag = true;

    VectorXd posori_task_torques2 = VectorXd::Zero(dof2);
    posori_task2->_kp_pos = 400.0;
    posori_task2->_kv_pos = 40.0;
    posori_task2->_kp_ori = 400.0;
    posori_task2->_kv_ori = 40.0;

    // joint task
    auto joint_task = new Sai2Primitives::JointTask(robot);
    joint_task->_use_interpolation_flag = true;
    joint_task->_use_velocity_saturation_flag = true;

    VectorXd joint_task_torques = VectorXd::Zero(dof);
    joint_task->_kp = 400.0;
    joint_task->_kv = 40.0;

    VectorXd q_init_desired(dof);
    q_init_desired <<  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;  ////!!!!!!!
    q_init_desired *= M_PI/180.0;
    q_init_desired(0) = -2.0;
    q_init_desired(1) = 0.0;

    joint_task->_desired_position = q_init_desired;
    
    //second robot
    auto joint_task2 = new Sai2Primitives::JointTask(robot2);
    joint_task2->_use_interpolation_flag = true;
    joint_task2->_use_velocity_saturation_flag = true;

    VectorXd joint_task_torques2 = VectorXd::Zero(dof2);
    joint_task2->_kp = 400.0;
    joint_task2->_kv = 40.0;

    VectorXd q_init_desired2(dof2);
    q_init_desired2 <<  0.0, 30.0, 0.0, -40.0, 0.0, 10.0, 0.0;
//    q_init_desired2 <<  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    q_init_desired2 *= M_PI/180.0;

    joint_task2->_desired_position = q_init_desired2;

    // containers
    Vector3d ee_pos;
    Matrix3d ee_rot;
    
    Vector3d ee_pos_shooter;
    Matrix3d ee_rot_shooter;

    // setup redis callback
    redis_client.createReadCallback(0);
    redis_client.createWriteCallback(0);

    // add to read callback
    redis_client.addEigenToReadCallback(0, JOINT_ANGLES_KEY, robot->_q);
    redis_client.addEigenToReadCallback(0, JOINT_VELOCITIES_KEY, robot->_dq);
    redis_client.addEigenToReadCallback(0, JOINT_ANGLES_KEY_SHOOTER, robot2->_q);
    redis_client.addEigenToReadCallback(0, JOINT_VELOCITIES_KEY_SHOOTER, robot2->_dq);

    // add to write callback
    redis_client.addStringToWriteCallback(0, CONTROLLER_RUNNING_KEY, controller_status);
    redis_client.addEigenToWriteCallback(0, JOINT_TORQUES_COMMANDED_KEY, command_torques);
    redis_client.addEigenToWriteCallback(0, JOINT_TORQUES_COMMANDED_KEY_SHOOTER, command_torques2);
    redis_client.addEigenToWriteCallback(0, SHOOTER_EE_POS, ee_pos_shooter);
    redis_client.addEigenToWriteCallback(0, HOOP_EE_POS, ee_pos);
    
    

    // create a timer
    LoopTimer timer;
    timer.initializeTimer();
    timer.setLoopFrequency(1000);
    double start_time = timer.elapsedTime(); //secs
    bool fTimerDidSleep = true;

    unsigned long long counter = 0;

    runloop = true;

    while (runloop) {
        // wait for next scheduled loop
        timer.waitForNextLoop();
        double time = timer.elapsedTime() - start_time;

        VectorXd q_curr_desired(dof);
        q_curr_desired = robot -> _q;

        // execute redis read callback
        redis_client.executeReadCallback(0);

        // update model
        robot->updateModel();
        robot2->updateModel();
        
        
        //for shooter
        N_prec2.setIdentity();
        joint_task2->updateTaskModel(N_prec2);

        // compute torques
        joint_task2->computeTorques(joint_task_torques2);
        command_torques2 = joint_task_torques2;
        
        //compute position
        robot2->position(ee_pos_shooter, control_link2, control_point2);

        if (state == POSTURE) {
            // update task model and set hierarchy
            N_prec.setIdentity();
            joint_task->updateTaskModel(N_prec);

            // compute torques
            joint_task->computeTorques(joint_task_torques);
            command_torques = joint_task_torques;
            

            if ( (robot->_q - q_init_desired).norm() < 0.15 && time > 3 ) {
                cout << "Posture To Motion" << endl;
                joint_task->reInitializeTask();
                posori_task->reInitializeTask();
                robot->position(ee_pos, control_link, control_point);

                posori_task->_desired_position = ee_pos - Vector3d(-0.1, -0.1, 0.1);
                posori_task->_desired_orientation = AngleAxisd(M_PI/6, Vector3d::UnitX()).toRotationMatrix() * posori_task->_desired_orientation;
                // posori_task->_desired_orientation = AngleAxisd(0.0000000000000001, Vector3d::UnitX()).toRotationMatrix() * posori_task->_desired_orientation;

                state = HOOP_MOVE_BASE;

            }
        } else if (state == HOOP_MOVE_BASE) {
            // update task model and set hierarchy
            N_prec.setIdentity();
            joint_task->updateTaskModel(N_prec);
            q_curr_desired = robot -> _q;
            q_curr_desired(0) = 1.0;  //move the base of the robot to x = 0.5
            q_curr_desired(1) = 0.0;    //move the base of the robot to y = 0.5
            joint_task -> _desired_position = q_curr_desired;
            // compute torques
            joint_task->computeTorques(joint_task_torques);
            command_torques = joint_task_torques;

            if ( (robot->_q - q_curr_desired).norm() < 0.05 ) {
                cout << "HOOP_BASE_ARRIVED" << endl;
                joint_task->reInitializeTask();
                posori_task->reInitializeTask();
                robot->position(ee_pos, control_link, control_point);
                joint_task -> _use_velocity_saturation_flag = true;
                joint_task -> _saturation_velocity(0) = 0.0;
                joint_task -> _saturation_velocity(1) = 0.0;
                command_torques = 0 * joint_task_torques;
                //posori_task->_desired_position = ee_pos - Vector3d(-0.1, -0.1, 0.1);
                //posori_task->_desired_orientation = AngleAxisd(M_PI/6, Vector3d::UnitX()).toRotationMatrix() * posori_task->_desired_orientation;
                //posori_task->_desired_orientation = AngleAxisd(0.0000000000000001, Vector3d::UnitX()).toRotationMatrix() * posori_task->_desired_orientation;
                state = HOOP_MOVE_EE;
            }
        }
        else if (state == HOOP_MOVE_EE) {
                // update task model and set hierarchy
                N_prec.setIdentity();
                posori_task->updateTaskModel(N_prec);
                N_prec = posori_task->_N;
                joint_task->updateTaskModel(N_prec);

                joint_task -> _use_velocity_saturation_flag = true;
                joint_task -> _saturation_velocity(0) = 0.0;
                joint_task -> _saturation_velocity(1) = 0.0;


                posori_task->_desired_orientation = AngleAxisd(M_PI/3, Vector3d::UnitX()).toRotationMatrix() * posori_task->_desired_orientation;
                joint_task -> _desired_position = q_curr_desired;
                // compute torques
                posori_task->computeTorques(posori_task_torques);
                joint_task->computeTorques(joint_task_torques);
                command_torques = posori_task_torques + joint_task_torques;

                if ( (robot->_q - q_curr_desired).norm() < 0.15 ) {
                    cout << "HOOP_EE_ARRIVED" << endl;
                    joint_task->reInitializeTask();
                    posori_task->reInitializeTask();
                    robot->position(ee_pos, control_link, control_point);
                    posori_task->_desired_position = ee_pos - Vector3d(-0.1, -0.1, 0.1);
                    posori_task->_desired_orientation = AngleAxisd(M_PI/6, Vector3d::UnitX()).toRotationMatrix() * posori_task->_desired_orientation;
                    // posori_task->_desired_orientation = AngleAxisd(0.0000000000000001, Vector3d::UnitX()).toRotationMatrix() * posori_task->_desired_orientation;

                    state = IDLE;
                }
        } else if (state == IDLE) {
            // update task model and set hierarchy
            N_prec.setIdentity();
            joint_task->updateTaskModel(N_prec);
            // joint_task -> _use_velocity_saturation_flag = true;
            // joint_task -> _saturation_velocity(0) = 0.0;
            // joint_task -> _saturation_velocity(1) = 0.0;

            // compute torques
            joint_task->computeTorques(joint_task_torques);
            command_torques =  joint_task_torques;
        }

        // execute redis write callback
        redis_client.executeWriteCallback(0);

        counter++;
    }

    double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

    redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, 0 * command_torques);  // back to floating

    return 0;
}

/**
 * @file controller.cpp
 * @brief Controller file
 *
 */

#include "Sai2Model.h"
#include "Sai2Primitives.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "redis_keys.h"

#include <iostream>
#include <string>
#include <signal.h>

using namespace std;
using namespace Eigen;

bool runloop = false;
void sighandler(int sig)
{runloop = false;}

#define RAD(deg) ((double)(deg) * M_PI / 180.0)

// Location of URDF files specifying world and robot information
const string robot_file = "./resources/mmp_panda.urdf";
const string robot2_file = "./resources/kuka_iiwa.urdf";

unsigned long long controller_counter = 0;

const bool inertia_regularization = true;

// define three robot states
enum State
{
	INITIALIZE = 0,
	HOOP_MOVE = 1,
	IDLE = 2,
};

int main() {

	// set initial state
	int state = IDLE;
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
	VectorXd initial_q = robot->_q;
	robot->updateModel();

	auto robot2 = new Sai2Model::Sai2Model(robot2_file, false);
	robot2->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY_SHOOTER);
	robot2->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY_SHOOTER);
	VectorXd initial_q2 = robot2->_q;
	robot2->updateModel();

	// prepare controller
	int dof = robot->dof();
	int dof2 = robot2->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	VectorXd command_torques2 = VectorXd::Zero(dof2);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);
	MatrixXd N_prec2 = MatrixXd::Identity(dof2, dof2); // for the second robot

	// panda pose task
	const string control_link = "link7";
	const Vector3d control_point = Vector3d(0, 0, 0.07);
	auto posori_task = new Sai2Primitives::PosOriTask(robot, control_link, control_point);

	posori_task->_use_interpolation_flag = true;
	posori_task->_use_velocity_saturation_flag = false;
	posori_task->_otg->setMaxLinearVelocity(1000);

	VectorXd posori_task_torques = VectorXd::Zero(dof);
	posori_task->_kp_pos = 400.0;
	posori_task->_kv_pos = 40.0;
	posori_task->_kp_ori = 400.0;
	posori_task->_kv_ori = 40.0;

	//kuka pose task
	const string control_link2 = "link6";
	const Vector3d control_point2 = Vector3d(0, 0, 0.07);
	auto posori_task2 = new Sai2Primitives::PosOriTask(robot2, control_link2, control_point2);

	posori_task2->_use_interpolation_flag = true;
	posori_task2->_use_velocity_saturation_flag = true;
	posori_task2->_otg->setMaxLinearVelocity(10);

	VectorXd posori_task_torques2 = VectorXd::Zero(dof2);
	posori_task2->_kp_pos = 400.0;
	posori_task2->_kv_pos = 40.0;
	posori_task2->_kp_ori = 400.0;
	posori_task2->_kv_ori = 40.0;

	// set the current EE posiiton as the desired EE position
	Vector3d x_desired = Vector3d::Zero(3);
	robot->position(x_desired, control_link, control_point);
	Vector3d ee_pos = Vector3d::Zero(3);
	Vector3d x_init_desired = x_desired;
	Vector3d x_curr_desired = x_init_desired;

	// partial joint task to control the mobile base
	vector<int> base_joint_selection{0, 1, 2};
	auto base_task = new Sai2Primitives::PartialJointTask(robot, base_joint_selection);
	base_task->_use_interpolation_flag = false;  // turn off if trajectory following; else turn on
	base_task->_use_velocity_saturation_flag = true;
	base_task->_saturation_velocity << 0.7, 0.7, 0;  // adjust based on speed

	VectorXd base_task_torques = VectorXd::Zero(dof);
	base_task->_kp = 300;
	base_task->_kv = 20;

	VectorXd base_pose_desired = initial_q.head(3);
	VectorXd base_pose_init_desired = base_pose_desired;
	VectorXd base_pose_curr_desired = base_pose_init_desired;

	// joint (posture) task
	vector<int> arm_joint_selection{3, 4, 5, 6, 7, 8, 9};
	auto arm_joint_task = new Sai2Primitives::PartialJointTask(robot, arm_joint_selection);
	arm_joint_task->_use_interpolation_flag = false;

	VectorXd arm_joint_task_torques = VectorXd::Zero(dof);
	arm_joint_task->_kp = 100;
	arm_joint_task->_kv = 40;

	// set the desired posture
	VectorXd q_init_desired = initial_q.tail(7);
	//VectorXd q_curr_desired = q_init_desired;
	q_init_desired <<  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	q_init_desired *= M_PI/180.0;
	arm_joint_task->_desired_position = q_init_desired;

	//second robot joint task
	auto joint_task2 = new Sai2Primitives::JointTask(robot2);
	joint_task2->_use_interpolation_flag = true;
	joint_task2->_use_velocity_saturation_flag = true;

	VectorXd joint_task_torques2 = VectorXd::Zero(dof2);
	joint_task2->_kp = 400.0;
	joint_task2->_kv = 40.0;


	std::string mode;
	//mode = redis_client.get(SHOOTER_MODE);
	VectorXd q_init_desired2(dof2); //initial joint space for kuka
	mode = "straight";
	if (mode == "straight") {
	q_init_desired2 <<  0.0, 40.0, 0.0, -40.0, 0.0, 10.0, 0.0;
	}

	else if (mode == "low_arc") {
	q_init_desired2 <<  0.0, 30.0, 0.0, -40.0, 0.0, 20.0, 0.0;
	}

	else if (mode == "high_arc") {
	q_init_desired2 <<  0.0, 30.0, 0.0, -40.0, 0.0, 30.0, 0.0;
	} // three shooting modes.
	q_init_desired2 *= M_PI/180.0;
	joint_task2->_desired_position = q_init_desired2;


	// containers
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
	std::string game_state;
	double start_time;
	bool fTimerDidSleep;
	game_state = redis_client.get(GAME_STATE);
	runloop = false; // timer in waiting state
	unsigned long long counter = 0;

	if (game_state == "0") {

        timer.initializeTimer();
        timer.setLoopFrequency(1000);
        start_time = timer.elapsedTime(); //secs
        fTimerDidSleep = true;
        runloop = true;

        while (runloop) {

            // wait for next scheduled loop
            timer.waitForNextLoop();
            double time = timer.elapsedTime() - start_time;

            //read robot state from redis
            robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
            robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);

            // execute redis read callback
            redis_client.executeReadCallback(0);

            // update model
            robot->updateModel();
            robot2->updateModel();

            //shooter complete the shooting motion
            N_prec2.setIdentity();
            joint_task2->updateTaskModel(N_prec2);
            joint_task2->computeTorques(joint_task_torques2);
            command_torques2 = joint_task_torques2;
            robot2->position(ee_pos_shooter, control_link2, control_point2);

            if (state == INITIALIZE) {

                // set controller inputs
                x_init_desired(0) = 0;
                x_init_desired(1) = 0;
                x_init_desired(2) = 1; // initial position
                base_pose_init_desired(0) = 0;
                base_pose_init_desired(1) = 0;
                x_init_desired(0) += base_pose_init_desired(0); //ee_x
                x_init_desired(1) += base_pose_init_desired(1); //ee_y

                posori_task->_desired_position = x_init_desired;
                base_task->_desired_position = base_pose_init_desired;
                arm_joint_task->_desired_position = q_init_desired;

                N_prec.setIdentity();
                posori_task->updateTaskModel(N_prec);
                N_prec = posori_task->_N;
                base_task->updateTaskModel(N_prec);
                N_prec = base_task->_N;
                arm_joint_task->updateTaskModel(N_prec);

                base_task->computeTorques(base_task_torques);
                arm_joint_task->computeTorques(arm_joint_task_torques);
                posori_task->computeTorques(posori_task_torques);

                command_torques = base_task_torques + arm_joint_task_torques + posori_task_torques;
                robot->position(ee_pos, control_link, control_point);

                if ( (ee_pos - x_init_desired).norm() < 0.015) {
                    cout << "Robot Initialized" << endl;
                    base_task->reInitializeTask();
                    arm_joint_task->reInitializeTask();
                    posori_task->reInitializeTask();
                    robot->position(ee_pos, control_link, control_point);
                    redis_client.setEigenMatrixJSON(HOOP_EE_POS, ee_pos);
                    state = IDLE;
                }
            }
            else if (state == IDLE) {

                x_desired(0) = 0;
                x_desired(1) = 0;
                x_desired(2) = 0.9;
                base_pose_init_desired(0) = 0;
                base_pose_init_desired(1) = 0;
                x_desired(0) += base_pose_init_desired(0); //ee_x
                x_desired(1) += base_pose_init_desired(1); //ee_y

                posori_task->_desired_position = x_desired;
                base_task->_desired_position = base_pose_init_desired;
                arm_joint_task->_desired_position = q_init_desired;

                // update task model and set hierarchy
                N_prec.setIdentity();
                posori_task->updateTaskModel(N_prec);
                N_prec = posori_task->_N;
                base_task->updateTaskModel(N_prec);
                N_prec = base_task->_N;
                arm_joint_task->updateTaskModel(N_prec);

                // compute torques
                posori_task->computeTorques(posori_task_torques);
                base_task->computeTorques(base_task_torques);
                arm_joint_task->computeTorques(arm_joint_task_torques);

                command_torques = posori_task_torques + base_task_torques + arm_joint_task_torques;
                robot->position(ee_pos, control_link, control_point);

                if ( (ee_pos - x_desired).norm() < 0.015) {
                    cout << "Ready for Next Mission" << endl;
                    base_task->reInitializeTask();
                    arm_joint_task->reInitializeTask();
                    posori_task->reInitializeTask();
                    robot->position(ee_pos, control_link, control_point);
                    redis_client.setEigenMatrixJSON(HOOP_EE_POS, ee_pos);
                    state = HOOP_MOVE;
                }
            }
            else if (state == HOOP_MOVE) {

                x_desired(0) = -2;
                x_desired(1) = -4;
                x_desired(2) = 1;
                base_pose_init_desired(0) = x_desired(0);
                base_pose_init_desired(1) = x_desired(1);

                posori_task->_desired_position = x_desired;
                base_task->_desired_position = base_pose_init_desired;
                arm_joint_task->_desired_position = q_init_desired;

                // update task model and set hierarchy
                N_prec.setIdentity();
                posori_task->updateTaskModel(N_prec);
                N_prec = posori_task->_N;
                base_task->updateTaskModel(N_prec);
                N_prec = base_task->_N;
                arm_joint_task->updateTaskModel(N_prec);

                // compute torques
                posori_task->computeTorques(posori_task_torques);
                base_task->computeTorques(base_task_torques);
                arm_joint_task->computeTorques(arm_joint_task_torques);

                command_torques = posori_task_torques + base_task_torques + arm_joint_task_torques;
                robot->position(ee_pos, control_link, control_point);

                if ( (ee_pos - x_desired).norm() < 0.015) {
                    cout << "Goal Reached" << endl;
                    base_task->reInitializeTask();
                    arm_joint_task->reInitializeTask();
                    posori_task->reInitializeTask();
                    robot->position(ee_pos, control_link, control_point);
                    redis_client.setEigenMatrixJSON(HOOP_EE_POS, ee_pos);
                    state = INITIALIZE;
                }
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
}

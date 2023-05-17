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

unsigned long long controller_counter = 0;

const bool inertia_regularization = true;

enum State
{
	INITIALIZE = 0,
	HOOP_MOVE = 1,
};

int main() {

	// initial state
	int state = INITIALIZE;
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

	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// pose task
	const string control_link = "link7";
	const Vector3d control_point = Vector3d(0, 0, 0.07);
	auto posori_task = new Sai2Primitives::PosOriTask(robot, control_link, control_point);

	posori_task->_use_interpolation_flag = true;
	posori_task->_use_velocity_saturation_flag = true;
	posori_task->_otg->setMaxLinearVelocity(0.8);

	VectorXd posori_task_torques = VectorXd::Zero(dof);
	posori_task->_kp_pos = 400.0;
	posori_task->_kv_pos = 40.0;
	posori_task->_kp_ori = 400.0;
	posori_task->_kv_ori = 40.0;

	// set the current EE posiiton as the desired EE position
	Vector3d x_desired = Vector3d::Zero(3);
	Vector3d ee_pos = Vector3d::Zero(3);
	robot->position(x_desired, control_link, control_point);
	Vector3d x_init_desired = x_desired;
	Vector3d x_curr_desired = x_init_desired;

	// partial joint task to control the mobile base
	vector<int> base_joint_selection{0, 1, 2};
	auto base_task = new Sai2Primitives::PartialJointTask(robot, base_joint_selection);
	base_task->_use_interpolation_flag = false;  // turn off if trajectory following; else turn on
	base_task->_use_velocity_saturation_flag = true;
	base_task->_saturation_velocity << 0.2, 0.2, 0.3;  // adjust based on speed

	VectorXd base_task_torques = VectorXd::Zero(dof);
	base_task->_kp = 200;
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
	arm_joint_task->_kv = 20;

	// set the desired posture
	VectorXd q_init_desired = initial_q.tail(7);
	VectorXd q_curr_desired = q_init_desired;
	q_init_desired <<  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	q_init_desired *= M_PI/180.0;
	arm_joint_task->_desired_position = q_init_desired;

	// setup redis callback
	redis_client.createReadCallback(0);
	redis_client.createWriteCallback(0);

	// add to read callback
	redis_client.addEigenToReadCallback(0, JOINT_ANGLES_KEY, robot->_q);
	redis_client.addEigenToReadCallback(0, JOINT_VELOCITIES_KEY, robot->_dq);

	// add to write callback
	redis_client.addStringToWriteCallback(0, CONTROLLER_RUNNING_KEY, controller_status);
	redis_client.addEigenToWriteCallback(0, JOINT_TORQUES_COMMANDED_KEY, command_torques);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000);
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	unsigned long long counter = 0;

	runloop = true;


	base_pose_init_desired(0)=0.5;
	base_pose_init_desired(1)=0.5;
	x_init_desired(0) += base_pose_init_desired(0); //ee_x
	x_init_desired(1) += base_pose_init_desired(1);//e_y
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

		if (state == INITIALIZE) {
			// set controller inputs
			x_init_desired(0) = 2;
			x_init_desired(1) = 2;
			x_init_desired(2) = x_desired(2) - .5;

			posori_task->_desired_position = x_init_desired;
			base_task->_desired_position = base_pose_init_desired;
			arm_joint_task->_desired_position = q_init_desired;

			// update task model and set hierarchy
			/*
				arm-driven motion hieararchy: arm -> base -> arm nullspace
			*/
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			base_task->updateTaskModel(N_prec);
			N_prec = base_task->_N;
			arm_joint_task->updateTaskModel(N_prec);

			// /*
			// 	base-driven motion hieararchy: base -> arm -> arm nullspace
			// */
			// N_prec.setIdentity();
			// base_task->updateTaskModel(N_prec);
			// N_prec = base_task->_N;
		  // posori_task->updateTaskModel(N_prec);
		  // N_prec = posori_task->_N;
		  // arm_joint_task->updateTaskModel(N_prec);

			// compute torques
			base_task->computeTorques(base_task_torques);
			arm_joint_task->computeTorques(arm_joint_task_torques);
			posori_task->computeTorques(posori_task_torques);

			command_torques = base_task_torques + arm_joint_task_torques + posori_task_torques;
			robot->position(ee_pos, control_link, control_point);
			if ( (ee_pos - x_init_desired).norm() < 0.5 && time > 3 ) {
				cout << "Robot Initialized" << endl;
				base_task->reInitializeTask();
				arm_joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				robot->position(ee_pos, control_link, control_point);
        redis_client.setEigenMatrixJSON(HOOP_EE_POS, ee_pos);

				// command_torques = 0 * (base_task_torques + arm_joint_task_torques + posori_task_torques);
				state = HOOP_MOVE;

			}
		}
		else if (state == HOOP_MOVE) {

			x_curr_desired(0) = -0.3;
			x_curr_desired(1) = 0.5;

			posori_task->_desired_position = x_curr_desired;
			base_task->_desired_position = base_pose_init_desired;
			arm_joint_task->_desired_position = q_init_desired;

			// update task model and set hierarchy
			/*
				arm-driven motion hieararchy: arm -> base -> arm nullspace
			*/
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

			if ( (ee_pos - x_curr_desired).norm() < 0.15 && time > 3 ) {
				cout << "Robot Reached" << endl;
				base_task->reInitializeTask();
				arm_joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				robot->position(ee_pos, control_link, control_point);
        redis_client.setEigenMatrixJSON(HOOP_EE_POS, ee_pos);

				// command_torques = 0 * (base_task_torques + arm_joint_task_torques + posori_task_torques);
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

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
int sleep_counter = 0;

bool runloop = false;
void sighandler(int sig){runloop = false;}
//bool fSimulationLoopDone = false;
//bool fControllerLoopDone = false;

// helper function
double sat(double x);

// function for converting string to bool
bool string_to_bool(const std::string& x);

// function for converting bool to string
inline const char * const bool_to_string(bool b);

#define RAD(deg) ((double)(deg) * M_PI / 180.0)

// Location of URDF files specifying world and robot information
const string robot_file = "./resources/mmp_panda.urdf";
const string robot2_file = "./resources/kuka_iiwa.urdf";

unsigned long long controller_counter = 0;

VectorXd q_init_desired2(7); //initial joint space for kuka
// get shooting angle from redis
std::string shooting_angle;

const bool inertia_regularization = true;

// define three robot states
enum HOOP_STATE
{
	HOOP_INITIALIZE = 0,
	HOOP_MOVE = 1,
	HOOP_IDLE = 2,
	HOOP_POSTURE = 3
};

enum SHOOTER_STATE
{
    SHOOTER_IDLE = 0,
    SHOOTER_SHOOT = 1,
    SHOOTER_RESET = 2,
    SHOOTER_SET = 3
};

int main() {

	// set initial state
	int hoop_state = HOOP_POSTURE;
	int shooter_state = SHOOTER_IDLE;
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
    redis_client.set(SHOOTER_READY_KEY, "1");

	// panda pose task
	const string control_link = "link7";
	const Vector3d control_point = Vector3d(0, 0, 0.07);
	auto posori_task = new Sai2Primitives::PosOriTask(robot, control_link, control_point);

	posori_task->_use_interpolation_flag = true;
	posori_task->_use_velocity_saturation_flag = false;
	posori_task->_otg->setMaxLinearVelocity(1000);
	posori_task->_linear_saturation_velocity = 2;

	VectorXd posori_task_torques = VectorXd::Zero(dof);
	posori_task->_kp_pos = 400.0;
	posori_task->_kv_pos = 40.0;
	posori_task->_kp_ori = 400.0;
	posori_task->_kv_ori = 40.0;

	//kuka pose task
	const string control_link2 = "link6";
	const Vector3d control_point2 = Vector3d(0, 0, 0.2);
	MatrixXd Jv = MatrixXd::Zero(3,dof2);
//	robot2->Jv(Jv, control_link2, control_point2);
	auto posori_task2 = new Sai2Primitives::PosOriTask(robot2, control_link2, control_point2);

	posori_task2->_use_interpolation_flag = false;
	posori_task2->_use_velocity_saturation_flag = false;
	posori_task2->_otg->setMaxLinearVelocity(1000);

    std::string shooter_power = "0";
	VectorXd posori_task_torques2 = VectorXd::Zero(dof2);
//	posori_task2->_kp_pos = 400.0 * (power + 1);
//	posori_task2->_kv_pos = 40.0;
//	posori_task2->_kp_ori =  400.0 * (power + 1);
//	posori_task2->_kv_ori = 40.0;

	// set the current EE posiiton as the desired EE position
	Vector3d x_desired = Vector3d::Zero(3);
	robot->position(x_desired, control_link, control_point);
	Vector3d ee_pos = Vector3d::Zero(3);
	Vector3d x_init_desired = x_desired;
	Vector3d x_curr_desired = x_init_desired;

	// partial joint task to control the mobile base
	auto joint_task = new Sai2Primitives::JointTask(robot);
	joint_task -> _use_interpolation_flag = true;
	joint_task -> _use_velocity_saturation_flag = true;

	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task -> _kp = 400.0;
	joint_task -> _kv = 40.0;

	vector<int> base_joint_selection{0, 1, 2};
	auto base_task = new Sai2Primitives::PartialJointTask(robot, base_joint_selection);
	base_task->_use_interpolation_flag = false;  // turn off if trajectory following; else turn on
	base_task->_use_velocity_saturation_flag = false;
	base_task->_saturation_velocity << 10.3, 10.3, 0.2;  // adjust based on speed

	VectorXd base_task_torques = VectorXd::Zero(dof);
	base_task->_kp = 400;
	base_task->_kv = 40;

	VectorXd base_pose_desired = initial_q.head(3);
	VectorXd base_pose_init_desired = base_pose_desired;
	VectorXd base_pose_curr_desired = base_pose_init_desired;

	// joint (posture) task
	vector<int> arm_joint_selection{3, 4, 5, 6, 7, 8, 9};
	auto arm_joint_task = new Sai2Primitives::PartialJointTask(robot, arm_joint_selection);
	arm_joint_task->_use_interpolation_flag = true;

	VectorXd arm_joint_task_torques = VectorXd::Zero(dof);
	arm_joint_task->_kp = 100;
	arm_joint_task->_kv = 25;

	// set the desired posture
	VectorXd q_init_desired = initial_q.tail(7);
	q_init_desired << -45.0, -10.0, -10.0, -100.0, 5.0, 95.0, -50.0;
	q_init_desired *= M_PI/180.0;

	VectorXd _q_init_desired = VectorXd::Zero(dof);
	_q_init_desired << 0.0, 0.0, 0.0, -45.0, -0.0, 0.0, -90.0, 0.0, 90.0, -45.0;
	_q_init_desired *= M_PI/180.0;
	joint_task -> _desired_position = _q_init_desired;

	//second robot joint task
	auto joint_task2 = new Sai2Primitives::JointTask(robot2);
	joint_task2->_use_interpolation_flag = false;
	joint_task2->_use_velocity_saturation_flag = false;

	VectorXd joint_task_torques2 = VectorXd::Zero(dof2);
//	joint_task2->_kp = 800.0;
//	joint_task2->_kv = 40.0;

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
	redis_client.addStringToReadCallback(0, SHOOTER_POWER, shooter_power);

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

	if (game_state == "1") {

        timer.initializeTimer();
        timer.setLoopFrequency(1000);
        start_time = timer.elapsedTime(); //secs
        fTimerDidSleep = true;
        runloop = true;

        std::string pre_mode;
        pre_mode = "";

        std::string reset;
        reset = redis_client.get(RESET_KEY);

        while (runloop) {


            // read simulation state
//            fSimulationLoopDone = string_to_bool(redis_client.get(SIMULATION_LOOP_DONE_KEY));

            // wait for next scheduled loop
            timer.waitForNextLoop();
            double time = timer.elapsedTime() - start_time;

            if (true) {
                // get shooter mode from redis
                std::string mode;
                mode = redis_client.get(SHOOTER_MODE);
//                if (mode != pre_mode) {
//                     // cout << mode << endl;  //cout the mode only once
//                     pre_mode = mode;
//                 }

                shooting_angle = redis_client.get(SHOOTING_ANGLE);
                int angle = stof(shooting_angle); //transfer to integer

                //read robot state from redis
                robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
                robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);

                // execute redis read callback
                redis_client.executeReadCallback(0);

                // update model
                robot->updateModel();
                robot2->updateModel();
                robot->position(ee_pos, control_link, control_point);
                base_pose_init_desired(0) = ee_pos(0) - 0.1; // make the base follow the arm
                base_pose_init_desired(1) = ee_pos(1) - 0.1; // offset between the base and the arm

                //get future position of the basketball
    //            std::string future_position;
    //            future_position = redis_client.get(FUTURE_POS);
    //            cout << future_position << endl;
    //            x_desired(0) = future_position(0);
    //            x_desired(1) = future_position(1);
    //            x_desired(2) = future_position(2);

                if (hoop_state == HOOP_POSTURE) {

                    N_prec.setIdentity();
                    joint_task -> updateTaskModel(N_prec);
                    joint_task -> computeTorques(joint_task_torques);
                    command_torques = joint_task_torques;

                    if ((robot->_q - _q_init_desired).norm() < 0.05){
                        cout << "hoop_idle!"<< endl;
                        joint_task ->reInitializeTask();
                        posori_task ->reInitializeTask();
                        // system("pause");
                        hoop_state = HOOP_IDLE;
                    }
                }
                else if (hoop_state == HOOP_INITIALIZE) {

                    //initialize timer before each task
                    timer.initializeTimer();

                    // set controller inputs
                    x_init_desired(0) = 0;
                    x_init_desired(1) = 0;
                    x_init_desired(2) = 0.9; // initial position

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

                    if ( (ee_pos - x_init_desired).norm() < 0.015) {

//                        cout << "Robot Initialized" << endl;
                        base_task->reInitializeTask();
                        arm_joint_task->reInitializeTask();
                        posori_task->reInitializeTask();
                        redis_client.setEigenMatrixJSON(HOOP_EE_POS, ee_pos);

                        // reset the basketball
                        if (redis_client.get(SHOOTER_READY_KEY) == "1") {
                            cout << "hoop_idle" << '\n';
                            redis_client.set(RESET_KEY, "1");
                            hoop_state = HOOP_IDLE;
                            //controller_status = "0";
                        }
                    }
                }
                else if (hoop_state == HOOP_IDLE) {

                    // setting idle position for hoop
                    x_desired(0) = 0;
                    x_desired(1) = 0;
                    x_desired(2) = 0.9;

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
                }
                else if (hoop_state == HOOP_MOVE) {

                    // desired position for hoop
                    x_desired(0) = 2;
                    x_desired(1) = 3;
                    x_desired(2) = 0.6; // todo: need to change this to the predicted position

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

                    if ( (ee_pos - x_desired).norm() < 0.015) {
//                        cout << "Goal Reached" << endl;
                        base_task->reInitializeTask();
                        arm_joint_task->reInitializeTask();
                        posori_task->reInitializeTask();
                        redis_client.setEigenMatrixJSON(HOOP_EE_POS, ee_pos);
                        if (sleep_counter < 10) {
//                            if (sleep_counter == 0) cout << "oh\n" << endl;
                            sleep_counter++;
                            }
                        else {
                            cout << "hoop initialize" << endl;
                            hoop_state = HOOP_INITIALIZE;
                            sleep_counter = 0;
//                            cout << "my\ngod\n" << endl;
                        }
                    }

                }

                if (shooter_state == SHOOTER_IDLE){

                    // set the shooter to initial position
//                    Vector3d force_sensed = Vector3d::Zero(3);
//                    force_sensed = redis_client.getEigenMatrixJSON(EE_FORCE_KEY);
//                    VectorXd torque_compensate = VectorXd::Zero(dof2);
//                    robot2->Jv(Jv, control_link2, control_point2);
//                    torque_compensate = Jv.transpose() * force_sensed;
                    q_init_desired2 *= 0;
                    q_init_desired2(0) = angle;
                    q_init_desired2 *= M_PI/180.0;
                    joint_task2->_desired_position = q_init_desired2;
                    N_prec2.setIdentity();
                    joint_task2->updateTaskModel(N_prec2);
                    joint_task2->computeTorques(joint_task_torques2);
                    command_torques2 = joint_task_torques2;
                    //command_torques2 = joint_task_torques2 - torque_compensate;
                    robot2->position(ee_pos_shooter, control_link2, control_point2);

                    // detect if the ball is reseted in the hand
                    if (((robot2->_q - q_init_desired2).norm() < 0.05) && (redis_client.get(BALL_READY_KEY) == "1")){
                        cout << "shooter set"<< endl;
                        joint_task2 ->reInitializeTask();
                        posori_task2 ->reInitializeTask();
                        shooter_state = SHOOTER_SET;
                        redis_client.set(SHOOTER_READY_KEY, "0");
                    }
                }
                else if (shooter_state == SHOOTER_SET){

                    // set the shooting angle for shooter
                    VectorXd q_init_desired_2 = VectorXd::Zero(dof2);
                    q_init_desired_2(0) = angle; //getting the shooting angle
                    q_init_desired_2 *= M_PI/180.0;
                    if ((hoop_state != HOOP_IDLE) || ((robot2 -> _q - q_init_desired_2).norm() > 0.05)){

                        // turn the first angle
                        joint_task2->_desired_position = q_init_desired_2;
                        N_prec2.setIdentity();
                        joint_task2->updateTaskModel(N_prec2);
                        joint_task2->computeTorques(joint_task_torques2);
                        command_torques2 = joint_task_torques2;
                        robot2->position(ee_pos_shooter, control_link2, control_point2);

                    }
                    else {
                        float power = stof(shooter_power);
//                        cout << power << endl;
                        joint_task2->_kp = 50.0 * (power + 1);
                        joint_task2->_kv = 15.0;

                         // set shooting gesture
                        if (mode == "straight") {
                            q_init_desired2 << angle, 30.0, 0.0, -30.0, 0.0, 10.0, 0.0;
                            q_init_desired2 *= M_PI/180.0;
                        }

                        else if (mode == "low_arc") {
                            q_init_desired2 << angle, 25.0, 0.0, -25.0, 0.0, 20.0, 0.0;
                            q_init_desired2 *= M_PI/180.0;
                        }

                        else if (mode == "high_arc") {
                            q_init_desired2 << angle, 20.0, 0.0, -20.0, 0.0, 30.0, 0.0;
                            q_init_desired2 *= M_PI/180.0;
                        } // three shooting modes

                        if (sleep_counter < 10) {
//                            if (sleep_counter == 0) cout << "oh\n" << endl;
                            sleep_counter++;
                            }
                        else {
                            cout << "shooter shoot"<< endl;
                            shooter_state = SHOOTER_SHOOT;
                            sleep_counter = 0;
//                            cout << "my\ngod\n" << endl;
                        }
                    }
                }
                else if (shooter_state == SHOOTER_SHOOT){
//                    cout << joint_task2->_kp << endl;
                    joint_task2->_desired_position = q_init_desired2;
                    N_prec2.setIdentity();
                    joint_task2->updateTaskModel(N_prec2);
                    joint_task2->computeTorques(joint_task_torques2);
                    command_torques2 = joint_task_torques2;
                    robot2->position(ee_pos_shooter, control_link2, control_point2);

                    if ((robot2 -> _q - q_init_desired2).norm() < 0.015){
                        cout << "shooter reset"<< endl;
                        shooter_state = SHOOTER_RESET;
                        redis_client.set(PREDICTION_READY_KEY, "1");
                        cout << "hoop move" << endl;
                        hoop_state = HOOP_MOVE;
                    }
                }
                else if (shooter_state == SHOOTER_RESET){

                    // shooter back to initial position
                    q_init_desired2 *= 0;
                    q_init_desired2(0) = angle;
                    q_init_desired2 *= M_PI/180.0;
                    joint_task2->_desired_position = q_init_desired2;
                    N_prec2.setIdentity();
                    joint_task2->updateTaskModel(N_prec2);
                    joint_task2->computeTorques(joint_task_torques2);
                    command_torques2 = joint_task_torques2;
                    robot2->position(ee_pos_shooter, control_link2, control_point2);

                    if ((robot2 -> _q - q_init_desired2).norm() < 0.015){
                        cout << "shooter idle"<< endl;
                        shooter_state = SHOOTER_IDLE;
                        redis_client.set(SHOOTER_READY_KEY, "1");
                    }
                }

            // execute redis write callback
            redis_client.executeWriteCallback(0);

            counter++;
	        }
//		fControllerLoopDone = true;
//        redis_client.set(CONTROLLER_LOOP_DONE_KEY, bool_to_string(fControllerLoopDone));
        }

	// controller loop is turned off
//    fControllerLoopDone = false;
//    redis_client.set(CONTROLLER_LOOP_DONE_KEY, bool_to_string(fControllerLoopDone));

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

    redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, 0 * command_torques);  // back to floating

	return 0;
    }
}


double sat(double x) {
    if (abs(x) <= 1.0) {
        return x;
    }
    else {
        return signbit(x);
    }
}

//------------------------------------------------------------------------------

bool string_to_bool(const std::string& x) {
    assert(x == "false" || x == "true");
    return x == "true";
}

//------------------------------------------------------------------------------

inline const char * const bool_to_string(bool b)
{
    return b ? "true" : "false";
}
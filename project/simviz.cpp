/**
 * @file simviz.cpp
 * @brief Simulation an visualization of panda robot
 * 
 */

#include <GL/glew.h>
#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "Sai2Simulation.h"
#include <dynamics3d.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include <GLFW/glfw3.h>  // must be loaded after loading opengl/glew
#include "uiforce/UIForceWidget.h"  // used for right-click drag interaction in window
#include <random>  // used for white-noise generation
#include "force_sensor/ForceSensorSim.h"  // references src folder in sai2-common directory
#include "force_sensor/ForceSensorDisplay.h"
#include <signal.h>
#include <iostream>
#include <random>

// flags for simulation and controller states
bool fSimulationRunning = false;
void sighandler(int){fSimulationRunning = false;}

#include "redis_keys.h"
#include "../include/object.h"

using namespace std;
using namespace Eigen;

// specify urdf and robots 
const string world_file = "./resources/world.urdf";
const string robot_file = "./resources/mmp_panda.urdf";
const string robot_name = "mmp_panda";
const string robot2_file = "./resources/kuka_iiwa.urdf";
const string robot2_name = "kuka_iiwa";    //urdf for the kuka_iiwa
const string camera_name = "camera_fixed";
const string base_link_name = "link0";
const string ee_link_name = "link7";
const string sensor_link_name = "link7";

// basketball
const string obj_file = "./resources/ball.urdf";
const string obj_name = "ball";

// ball's info
Vector3d curr_pos;
Vector3d object_future_pos;
Vector3d curr_lin_vel;
MatrixXd object_Jv = MatrixXd::Zero(3,6);
string object_ee_link = "link6";
Vector3d object_ee_point = Vector3d(0, 0, 0);
VectorXd object_acc(6);
Vector3d object_lin_acc;

// target
string target_name = "target";
Vector3d target_pos = Vector3d(0, 2, 0.1);
Quaterniond target_ori = Quaterniond(1, 0, 0, 0);
Vector3d target_dim = Vector3d(0.3, 0.3, 0.3);
Vector4d target_rgba = Vector4d(1, 0.33, 0.33, 1);

// force sensor
ForceSensorSim* force_sensor;

// display widget for forces at end effector
ForceSensorDisplay* force_display;

// redis client 
RedisClient redis_client;
RedisClient redis_client_test;

// Create a random number generator engine
std::random_device rd;
std::mt19937 gen(rd());

// Generate a random real number between 0 and 2
std::uniform_real_distribution<double> x_dist(-1.5, 1.5);
std::uniform_real_distribution<double> y_dist(2, 4);

// score variables
string score_text = "0";
int score_value = 0;

/* Edited functions from collision demo */
// simulation thread
void simulation(Sai2Model::Sai2Model* robot, Sai2Model::Sai2Model* robot2, Sai2Model::Sai2Model* object,
                Simulation::Sai2Simulation* sim, UIForceWidget *ui_force_widget, Sai2Graphics::Sai2Graphics* graphics);

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback to print glew errors
bool glewInitialize();

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

// callback when a mouse button is pressed
void mouseClick(GLFWwindow* window, int button, int action, int mods);

// predict the future position for the obj
Vector3d posPrediction(Vector3d curr_pos, Vector3d curr_lin_vel, Vector3d object_lin_acc,
                       double curr_time, Sai2Model::Sai2Model* object, Simulation::Sai2Simulation* sim);

// set dynamic object velocity - added by WWY
void setObjectVel(const std::string& object_name, const Eigen::Vector3d& lin_vel, const Eigen::Vector3d& ang_vel,
Simulation::Sai2Simulation* sim);

// flags for scene camera movement
bool fTransXp = false;
bool fTransXn = false;
bool fTransYp = false;
bool fTransYn = false;
bool fTransZp = false;
bool fTransZn = false;
bool fRotPanTilt = false;
bool fRobotLinkSelect = false;
bool fshowCameraPose = false;

int main() {
//    cout << randomNumber << endl;
	cout << "Loading URDF world model file: " << world_file << endl;

	// start redis client
	redis_client = RedisClient();
	redis_client.connect();

	redis_client_test = RedisClient();
	redis_client_test.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_file, true);
	Eigen::Vector3d camera_pos, camera_lookat, camera_vertical;
	graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
	graphics->_world->setBackgroundColor(73.0/255, 192.0/255, 182.0/255);  // set static blue background
//	graphics->showLinkFrame(true, robot_name, ee_link_name, 0.15);  // can add frames for different links
	graphics->getCamera(camera_name)->setClippingPlanes(0.1, 50);  // set the near and far clipping planes

	// load robots
	/* added from collision demo */
    Eigen::Vector3d robot_offset = Eigen::Vector3d(0.0, 0.0, 0.0);
    Eigen::Matrix3d R_world_robot = Eigen::Matrix3d::Identity();

    Eigen::Affine3d T_world_robot = Eigen::Affine3d::Identity();
    T_world_robot.translation() = robot_offset;
    T_world_robot.linear() = R_world_robot;

	auto robot = new Sai2Model::Sai2Model(robot_file, false, T_world_robot);
	robot->updateModel();
    robot->updateKinematics();
    
    // load second robot
    /* added from collision demo */
    Eigen::Vector3d robot2_offset = Eigen::Vector3d(0.0, 0.0, 0.0);
    Eigen::Matrix3d R_world_robot2 = Eigen::Matrix3d::Identity();

    Eigen::Affine3d T_world_robot2 = Eigen::Affine3d::Identity();
    T_world_robot2.translation() = robot2_offset;
    T_world_robot2.linear() = R_world_robot2;
    auto robot2 = new Sai2Model::Sai2Model(robot2_file, false, T_world_robot2);
    robot2->updateModel();
    robot2->updateKinematics();

    /* added from collision demo */
    // load robot objects
    Eigen::Vector3d object_offset = Eigen::Vector3d(0.0, 0.0, 0.0);
    Eigen::Matrix3d R_world_object = Eigen::Matrix3d::Identity();

    Eigen::Affine3d T_world_object = Eigen::Affine3d::Identity();
    T_world_object.translation() = object_offset;
    T_world_object.linear() = R_world_object;

    auto object = new Sai2Model::Sai2Model(obj_file, false, T_world_object);
    // object->_q(1) = 2.5;
    object->updateModel();

    // load target once todo: addbox
    addBox(graphics, target_name, target_pos, target_ori, target_dim, target_rgba);

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_file, false);
	sim->setJointPositions(robot_name, robot->_q);
	sim->setJointVelocities(robot_name, robot->_dq);
	sim->setJointPositions(robot2_name, robot2->_q);
	sim->setJointVelocities(robot2_name, robot2->_dq);
	sim->setJointPositions(obj_name, object->_q);

    // set co-efficient of restition to zero for force control
    sim->setCollisionRestitution(0.8);

    // set co-efficient of friction - this causes jitter
    sim->setCoeffFrictionStatic(0.5);
    sim->setCoeffFrictionDynamic(0.5);

    // initialize force sensor: needs Sai2Simulation sim interface type
    force_sensor = new ForceSensorSim(robot2_name, sensor_link_name, Eigen::Affine3d::Identity(), robot2);
    //force_display = new ForceSensorDisplay(force_sensor, graphics);

	/*------- Set up visualization -------*/
	// set up error callback
	glfwSetErrorCallback(glfwError);

	// initialize GLFW
	glfwInit();

	// retrieve resolution of computer display and position window accordingly
	GLFWmonitor* primary = glfwGetPrimaryMonitor();
	const GLFWvidmode* mode = glfwGetVideoMode(primary);

	// information about computer screen and GLUT display window
	int screenW = mode->width;
	int screenH = mode->height;
	int windowW = 0.5 * screenW;
	int windowH = 0.4 * screenH;
	int windowPosY = 0.6 * screenH;
	int windowPosX = 0.5 * screenW;

	// create window and make it current
	glfwWindowHint(GLFW_VISIBLE, 0);
	GLFWwindow* window = glfwCreateWindow(windowW, windowH, "HoopHero", NULL, NULL);
	glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	// set callbacks
	glfwSetKeyCallback(window, keySelect);
	glfwSetMouseButtonCallback(window, mouseClick);

	/* added from collision demo */
	// init click force widget
    auto ui_force_widget = new UIForceWidget(robot2_name, robot2, graphics);
    ui_force_widget->setEnable(false);

	// cache variables
	double last_cursorx, last_cursory;

	// init redis client values 
	redis_client.set(CONTROLLER_RUNNING_KEY, "0");
	redis_client.set(RESET_KEY, "0");
	redis_client.setEigenMatrixJSON(JOINT_ANGLES_KEY, robot->_q);
	redis_client.setEigenMatrixJSON(JOINT_VELOCITIES_KEY, robot->_dq);
    redis_client.setEigenMatrixJSON(JOINT_ANGLES_KEY_SHOOTER, robot2->_q);
    redis_client.setEigenMatrixJSON(JOINT_VELOCITIES_KEY_SHOOTER, robot2->_dq);
    redis_client.setEigenMatrixJSON(OBJ_JOINT_ANGLES_KEY, object->_q);
    redis_client.setEigenMatrixJSON(OBJ_JOINT_VELOCITIES_KEY, object->_dq);
    redis_client.set(SHOOTER_READY_KEY, "0");
    redis_client.set(BALL_READY_KEY, "0");
    redis_client.set(PREDICTION_READY_KEY, "0");
    redis_client.setEigenMatrixJSON(FUTURE_POS, object_future_pos);
    redis_client.set(SHOOTING_ANGLE, "0");
    redis_client.set(SCORE, "0");

	// start simulation thread
	thread sim_thread(simulation, robot, robot2, object, sim, ui_force_widget, graphics);

	// initialize glew
	glewInitialize();

	// while window is open:
	int count = 0;
	Vector3d start_pos = Vector3d(1, -1, 1);

	while (!glfwWindowShouldClose(window) && fSimulationRunning)
	{
		// add sphere for every nth count
		if (count % 120 == 0) {  // default refresh rate
            // get world gravity
            chai3d::cVector3d gravity_g = sim->_world->getGravity(); // gravity
            Vector3d gra_g(gravity_g.x(), gravity_g.y(), gravity_g.z());
            // cout << "gravity: " << gra_g.transpose() << "\n";
		}

		// get ball's info
		object->positionInWorld(curr_pos, object_ee_link, object_ee_point);
		object->Jv(object_Jv, object_ee_link, object_ee_point);
        curr_lin_vel = object_Jv * object->_dq;
        object->acceleration6dInWorld(object_acc, object_ee_link, object_ee_point);
        object_lin_acc(0) = object_acc(0);
        object_lin_acc(1) = object_acc(1);
        object_lin_acc(2) = object_acc(2);

		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot_name, robot);
        graphics->updateGraphics(robot2_name, robot2);
        graphics->updateGraphics(obj_name, object);

		//force_display->update();
		graphics->render(camera_name, width, height);

		// swap buffers
		glfwSwapBuffers(window);

		// wait until all GL commands are completed
		glFinish();

		// check for any OpenGL errors
		GLenum err;
		err = glGetError();
		assert(err == GL_NO_ERROR);

		// poll for events
		glfwPollEvents();

		// move scene camera as required
		// graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
		Eigen::Vector3d cam_depth_axis;
		cam_depth_axis = camera_lookat - camera_pos;
		cam_depth_axis.normalize();
		Eigen::Vector3d cam_up_axis;
		// cam_up_axis = camera_vertical;
		// cam_up_axis.normalize();
		cam_up_axis << 0.0, 0.0, 1.0; //TODO: there might be a better way to do this
		Eigen::Vector3d cam_roll_axis = (camera_lookat - camera_pos).cross(cam_up_axis);
		cam_roll_axis.normalize();
		Eigen::Vector3d cam_lookat_axis = camera_lookat;
		cam_lookat_axis.normalize();

		if (fTransXp) {
			camera_pos = camera_pos + 0.05*cam_roll_axis;
			camera_lookat = camera_lookat + 0.05*cam_roll_axis;
		}
		if (fTransXn) {
			camera_pos = camera_pos - 0.05*cam_roll_axis;
			camera_lookat = camera_lookat - 0.05*cam_roll_axis;
		}
		if (fTransYp) {
			// camera_pos = camera_pos + 0.05*cam_lookat_axis;
			camera_pos = camera_pos + 0.05*cam_up_axis;
			camera_lookat = camera_lookat + 0.05*cam_up_axis;
		}
		if (fTransYn) {
			// camera_pos = camera_pos - 0.05*cam_lookat_axis;
			camera_pos = camera_pos - 0.05*cam_up_axis;
			camera_lookat = camera_lookat - 0.05*cam_up_axis;
		}
		if (fTransZp) {
			camera_pos = camera_pos + 0.1*cam_depth_axis;
			camera_lookat = camera_lookat + 0.1*cam_depth_axis;
		}	    
		if (fTransZn) {
			camera_pos = camera_pos - 0.1*cam_depth_axis;
			camera_lookat = camera_lookat - 0.1*cam_depth_axis;
		}
		if (fshowCameraPose) {
            cout << endl;
            cout << "camera position : " << camera_pos.transpose() << endl;
            cout << "camera lookat : " << camera_lookat.transpose() << endl;
            cout << endl;
        }
		if (fRotPanTilt) {
			// get current cursor position
			double cursorx, cursory;
			glfwGetCursorPos(window, &cursorx, &cursory);
			//TODO: might need to re-scale from screen units to physical units
			double compass = 0.006*(cursorx - last_cursorx);
			double azimuth = 0.006*(cursory - last_cursory);
			double radius = (camera_pos - camera_lookat).norm();
			Eigen::Matrix3d m_tilt; m_tilt = Eigen::AngleAxisd(azimuth, -cam_roll_axis);
			camera_pos = camera_lookat + m_tilt*(camera_pos - camera_lookat);
			Eigen::Matrix3d m_pan; m_pan = Eigen::AngleAxisd(compass, -cam_up_axis);
			camera_pos = camera_lookat + m_pan*(camera_pos - camera_lookat);
		}
		graphics->setCameraPose(camera_name, camera_pos, cam_up_axis, camera_lookat);
		glfwGetCursorPos(window, &last_cursorx, &last_cursory);
		ui_force_widget->setEnable(fRobotLinkSelect);

		if (fRobotLinkSelect)
        {
            double cursorx, cursory;
            int wwidth_scr, wheight_scr;
            int wwidth_pix, wheight_pix;
            std::string ret_link_name;
            Eigen::Vector3d ret_pos;

            // get current cursor position
            glfwGetCursorPos(window, &cursorx, &cursory);

            glfwGetWindowSize(window, &wwidth_scr, &wheight_scr);
            glfwGetFramebufferSize(window, &wwidth_pix, &wheight_pix);

            int viewx = floor(cursorx / wwidth_scr * wwidth_pix);
            int viewy = floor(cursory / wheight_scr * wheight_pix);

            if (cursorx > 0 && cursory > 0)
            {
                ui_force_widget->setInteractionParams(camera_name, viewx, wheight_pix - viewy, wwidth_pix, wheight_pix);
                //TODO: this behavior might be wrong. this will allow the user to click elsewhere in the screen
                // then drag the mouse over a link to start applying a force to it.
            }
        }

		count++;
	}

	// wait for simulation to finish
	fSimulationRunning = false;
	sim_thread.join();

	// destroy context
	glfwDestroyWindow(window);

	// terminate
	glfwTerminate();

	return 0;
}

//------------------------------------------------------------------------------

void simulation(Sai2Model::Sai2Model* robot, Sai2Model::Sai2Model* robot2, Sai2Model::Sai2Model* object,
                Simulation::Sai2Simulation* sim, UIForceWidget *ui_force_widget, Sai2Graphics::Sai2Graphics* graphics)
{
	// prepare simulation
	int dof = robot->dof();
    int dof2 = robot2->dof();
    int dof_obj = object->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
    VectorXd command_torques2 = VectorXd::Zero(dof2);
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
    redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY_SHOOTER, command_torques2); //added for the second robot

	// init control variables
	VectorXd g = VectorXd::Zero(dof);
    VectorXd g2 = VectorXd::Zero(dof2);
    Vector3d graVec = Vector3d::Zero(3);
	string controller_status = "0";
	Vector3d ee_pos_shooter_inworld;
	Eigen::Vector3d ui_force;
    ui_force.setZero();
    Eigen::VectorXd ui_force_command_torques(dof2);
    ui_force_command_torques.setZero();

    // sensed forces and moments from sensor
    Eigen::Vector3d sensed_force;
    Eigen::Vector3d sensed_moment;

    // velocity damping for ui force drag
    double kvj = 4000;

    // init camera detection variables
    Vector3d camera_pos, obj_pos;
    Matrix3d camera_ori;
    bool detect;
    const std::string true_message = "Detected";
    const std::string false_message = "Not Detected";

    // setup white noise generator
    const double mean = 0.0;
    const double stddev = 0.001;  // tune based on your system
    std::default_random_engine generator;
    std::normal_distribution<double> dist(mean, stddev);

    unsigned long long counter = 0; // copied from collison demo

	//kuka ee_pos
	const string control_link2 = "link6";
	const Vector3d control_point2 = Vector3d(0, 0, 0.07);

	// setup redis callback
	redis_client.createReadCallback(0);
	redis_client.createWriteCallback(0);

	// add to read callback
	redis_client.addStringToReadCallback(0, CONTROLLER_RUNNING_KEY, controller_status);
	redis_client.addEigenToReadCallback(0, JOINT_TORQUES_COMMANDED_KEY, command_torques);
    redis_client.addEigenToReadCallback(0, JOINT_TORQUES_COMMANDED_KEY_SHOOTER, command_torques2);
    redis_client.addEigenToReadCallback(0, GRAVITY_KEY, graVec);


	// add to write callback
	redis_client.addEigenToWriteCallback(0, JOINT_ANGLES_KEY, robot->_q);
	redis_client.addEigenToWriteCallback(0, JOINT_VELOCITIES_KEY, robot->_dq);
    redis_client.addEigenToWriteCallback(0, JOINT_ANGLES_KEY_SHOOTER, robot2->_q);
    redis_client.addEigenToWriteCallback(0, JOINT_VELOCITIES_KEY_SHOOTER, robot2->_dq);
    redis_client.addEigenToWriteCallback(0, FUTURE_POS, object_future_pos);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000);
	double time_slowdown_factor = 2;  // adjust to higher value (i.e. 2) to slow down simulation by this factor relative to real time (for slower machines)
	bool fTimerDidSleep = true;
	double start_time = timer.elapsedTime() / time_slowdown_factor; // secs
	double last_time = start_time;

	// added by WWY
	double pred_start_time;
	int sim_pred_counter = 0;
	int reset_counter = 0;

	// start simulation 
	fSimulationRunning = true;	
	while (fSimulationRunning) {
		// execute redis read callback
		redis_client.executeReadCallback(0);

		// set world gravity
		// chai3d::cVector3d gravity_g = sim->_world->getGravity(); TODO: this may not work
        // Vector3d gra_g(gravity_g.x(), gravity_g.y(), gravity_g.z());
		sim->_world->setGravity(graVec(0), graVec(1), graVec(2));
        VectorXd wind_torques(6);
        Vector3d wind_force(3);
        wind_force(0) = graVec(0) * 0.1;
        wind_force(1) = graVec(1) * 0.1;
        object->Jv(object_Jv, object_ee_link, object_ee_point);
        wind_torques = - object_Jv.transpose() * wind_force;
        sim->setJointTorques(obj_name, wind_torques);
//        cout << "gravity:" << "\t" << graVec.transpose() << endl;

        // read the gravity
        chai3d::cVector3d gravity_g = sim->_world->getGravity(); // gravity
        Vector3d gra_g(gravity_g.x(), gravity_g.y(), gravity_g.z());
//        cout << gra_g.transpose() << endl;

        if (true) {
            // run simulation loop when (1) control loop is done or (2) there is user force input
            // apply gravity compensation
            robot->gravityVector(g);
            // apply gravity compensation for the second robot
            robot2->gravityVector(g2);

            // read arm torques from redis and apply to simulated robot
            if (controller_status == "1") {
                command_torques = redis_client.getEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY);
                command_torques2 = redis_client.getEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY_SHOOTER);
            }
            else {
                command_torques.setZero();
                command_torques2.setZero();
            }

            // get forces from interactive screen
            ui_force_widget->getUIForce(ui_force);
            ui_force_widget->getUIJointTorques(ui_force_command_torques);

            sim->setJointTorques(robot_name, command_torques + g);

            if (fRobotLinkSelect) {
                sim->setJointTorques(robot2_name, command_torques2 + ui_force_command_torques - robot2->_M * kvj * robot2->_dq + g2);
                }
            else
                sim->setJointTorques(robot2_name, command_torques2 + g2);

            // integrate forward
            double curr_time = timer.elapsedTime() / time_slowdown_factor;
            double loop_dt = curr_time - last_time;
    //        double loop_dt = 0.001;
            sim->integrate(loop_dt);

            // read joint positions, velocities, update model
            sim->getJointPositions(robot_name, robot->_q);
            sim->getJointVelocities(robot_name, robot->_dq);
            robot->updateModel();

            // read joint positions, velocities, and update model for the second robot
            sim->getJointPositions(robot2_name, robot2->_q);
            sim->getJointVelocities(robot2_name, robot2->_dq);
            robot2->updateModel();
            robot2->position(ee_pos_shooter_inworld, control_link2, control_point2);
            ee_pos_shooter_inworld(1) = ee_pos_shooter_inworld(1) - 1.5;

            // update object
            if (redis_client.get(RESET_KEY) == "1") {
                float target_distance;
                target_distance = sqrt(pow((object_future_pos(0) - target_pos(0)), 2) + pow(object_future_pos(1) - target_pos(1), 2));
                if (target_distance <= 0.4) {
                    score_value += 1;
                    score_text = to_string(score_value);
                    redis_client.set(SCORE, score_text);

                    // refresh target
//                target_pos(1) += count / 120;
                    target_pos(0) = x_dist(gen);
                    target_pos(1) = y_dist(gen);
                    graphics->updateObjectGraphics(target_name, target_pos, target_ori);
                }
                VectorXd reset_ball_pos = object->_q;
                VectorXd reset_ball_vel = VectorXd::Zero(dof_obj);
                reset_ball_pos(0) = 0.0;
                reset_ball_pos(1) = -2.3;
                reset_ball_pos(2) = 0.0;
                sim->setJointPositions(obj_name, reset_ball_pos);
                sim->setJointVelocities(obj_name, reset_ball_vel);
                redis_client.set(RESET_KEY, "0");
                redis_client.set(FALLING_KEY, "1");
            }
            sim->getJointPositions(obj_name, object->_q);
            sim->getJointVelocities(obj_name, object->_dq);
            object->updateModel();

            // update force sensor readings
            force_sensor->update(sim);
            force_sensor->getForce(sensed_force);  // refer to ForceSensorSim.h in sai2-common/src/force_sensor (can also get wrt global frame)
            force_sensor->getMoment(sensed_moment);
            if (sensed_force.norm() > 0.6) {
                redis_client.set(BALL_READY_KEY, "1");
                redis_client.set(FALLING_KEY, "0");
            }
            else redis_client.set(BALL_READY_KEY, "0");

            // calculate future pos
            if (redis_client.get(PREDICTION_READY_KEY) == "1") {
                object_future_pos = posPrediction(curr_pos, curr_lin_vel, object_lin_acc, curr_time, object, sim);
                string mesh_filename = "../../model/test_objects/meshes/visual/basketball.obj";
//                addSphere(graphics, "basketball", object_future_pos, Quaterniond(1, 0, 0, 0), 0.15, Vector4d(1, 1, 1, 1));
            }

            // query object position and ee pos/ori for camera detection
            object->positionInWorld(obj_pos, "link6");
            robot->positionInWorld(camera_pos, "link7");
            robot->rotationInWorld(camera_ori, "link7");  // local to world frame

            // publish all redis keys at once to reduce multiple redis calls that slow down simulation
            // shown explicitly here, but you can define a helper function to publish data
            redis_client.setEigenMatrixJSON(OBJ_JOINT_ANGLES_KEY, object->_q);
            redis_client.setEigenMatrixJSON(OBJ_JOINT_VELOCITIES_KEY, object->_dq);
            redis_client.setEigenMatrixJSON(CAMERA_POS_KEY, camera_pos);
            redis_client.setEigenMatrixJSON(CAMERA_ORI_KEY, camera_ori);
            redis_client.setEigenMatrixJSON(EE_FORCE_KEY, sensed_force);
            redis_client.setEigenMatrixJSON(EE_MOMENT_KEY, sensed_moment);

            // execute redis write callback
            redis_client.executeWriteCallback(0);

            // update last time
            last_time = curr_time;
            reset_counter++;
            ++counter;
		}
	}

	double end_time = timer.elapsedTime() / time_slowdown_factor;
	std::cout << "\n";
	std::cout << "Simulation Loop run time  : " << end_time << " seconds\n";
	std::cout << "Simulation Loop updates   : " << timer.elapsedCycles() << "\n";
	std::cout << "Simulation Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";
    std::cout << "Simulation Loop updates   : " << counter << "\n";
}


//------------------------------------------------------------------------------

void glfwError(int error, const char* description) {
	cerr << "GLFW Error: " << description << endl;
	exit(1);
}

//------------------------------------------------------------------------------

bool glewInitialize() {
	bool ret = false;
	#ifdef GLEW_VERSION
	if (glewInit() != GLEW_OK) {
		cout << "Failed to initialize GLEW library" << endl;
		cout << glewGetErrorString(ret) << endl;
		glfwTerminate();
	} else {
		ret = true;
	}
	#endif
	return ret;
}

//------------------------------------------------------------------------------

void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	bool set = (action != GLFW_RELEASE);
	switch(key) {
		case GLFW_KEY_ESCAPE:
			// exit application
			fSimulationRunning = false;
			glfwSetWindowShouldClose(window, GL_TRUE);
			break;
		case GLFW_KEY_D:
			fTransXp = set;
			break;
		case GLFW_KEY_A:
			fTransXn = set;
			break;
		case GLFW_KEY_UP:
			fTransYp = set;
			break;
		case GLFW_KEY_DOWN:
			fTransYn = set;
			break;
		case GLFW_KEY_W:
			fTransZp = set;
			break;
		case GLFW_KEY_S:
			fTransZn = set;
			break;
		default:
			break;
	}
}

//------------------------------------------------------------------------------

void mouseClick(GLFWwindow* window, int button, int action, int mods) {
	bool set = (action != GLFW_RELEASE);
	//TODO: mouse interaction with robot
	switch (button) {
		// left click pans and tilts
		case GLFW_MOUSE_BUTTON_LEFT:
			fRotPanTilt = set;
			// NOTE: the code below is recommended but doesn't work well
			// if (fRotPanTilt) {
			// 	// lock cursor
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
			// } else {
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
			// }
			break;
		// if right click: don't handle. this is for menu selection
		case GLFW_MOUSE_BUTTON_RIGHT:
			fRobotLinkSelect = set;
			break;
		// if middle click: don't handle. doesn't work well on laptops
		case GLFW_MOUSE_BUTTON_MIDDLE:
			break;
		default:
			break;
	}
}

//------------------------------------------------------------------------------

Vector3d posPrediction(Vector3d curr_pos, Vector3d curr_lin_vel, Vector3d object_lin_acc, double curr_time,
                        Sai2Model::Sai2Model* object, Simulation::Sai2Simulation* sim)
{
    // create the future position vector
    Vector3d object_future_pos;
    Vector3d object_future_vel;
    // read the gravity
    chai3d::cVector3d gravity_g = sim->_world->getGravity(); // gravity
    Vector3d gra_g(gravity_g.x(), gravity_g.y(), gravity_g.z());
    cout << "read gravity: " << gra_g.transpose() << endl;

    // calculate and print out
    double time_step = 0.001;
    double time_duration = 0.001;
    for (int time_counter = 0; time_counter < 1800; time_counter++){
        time_duration += time_counter * time_step;
        object_future_pos << curr_pos + curr_lin_vel * time_duration + 0.5 * gra_g * time_duration * time_duration;
        object_future_vel << curr_lin_vel + gra_g * time_duration;

        if ((object_future_vel(2) < 0) && object_future_pos(1) > 0.1 && (object_future_pos(2) > -0.34) && (object_future_pos(2) < 0.02)) {
            cout << object_future_pos.transpose() << endl;
            break;
        }
    }

    object_future_pos(2) = object_future_pos(2) + 1.06;

//    if (object_future_pos(2) <= 0.15) object_future_pos(2) = 0.15;
    cout << "current position: ";
    cout << curr_time << "s: " << '\t' << curr_pos.transpose() << endl;
    cout << "current velocity:";
    cout << curr_time << "s: " << '\t' << curr_lin_vel.transpose() << endl;
    cout << "future position" << endl;
    cout << time_duration << "s: " << '\t' << object_future_pos.transpose() << endl;
    redis_client.set(PREDICTION_READY_KEY, "0");
    redis_client.setEigenMatrixJSON(FUTURE_POS, object_future_pos);
    return object_future_pos;
}


// set object velocities
void setObjectVel(const std::string& object_name, const Eigen::Vector3d& lin_vel, const Eigen::Vector3d& ang_vel,
Simulation::Sai2Simulation* sim) {
    auto object = sim->_world->getBaseNode(object_name);
    assert(object);

    object->m_dynamicJoints[0]->setVel(lin_vel(0));
    object->m_dynamicJoints[1]->setVel(lin_vel(1));
    object->m_dynamicJoints[2]->setVel(lin_vel(2));

    chai3d::cVector3d ang_vel_chai(ang_vel(0), ang_vel(1), ang_vel(2));
    object->m_dynamicJoints[3]->setVelSpherical(ang_vel_chai);
}

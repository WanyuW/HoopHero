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
#include <signal.h>
#include <iostream>

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

// dynamic objects information
const vector<string> object_names = {"basketball"};
vector<Vector3d> object_pos;
vector<Vector3d> object_lin_vel;
vector<Quaterniond> object_ori;
vector<Vector3d> object_ang_vel;
const int n_objects = object_names.size();

// estimate future position for dynamic obj
Vector3d object_future_pos;
Vector3d object_future_lin_vel;
Quaterniond object_future_ori;
Vector3d object_future_ang_vel;
Vector3d curr_pos;
Vector3d curr_lin_vel;
Quaterniond curr_ori;
Vector3d curr_ang_vel;
int flag = 0;

// redis client 
RedisClient redis_client; 

// simulation thread
void simulation(Sai2Model::Sai2Model* robot, Sai2Model::Sai2Model* robot2, Simulation::Sai2Simulation* sim);

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback to print glew errors
bool glewInitialize();

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

// callback when a mouse button is pressed
void mouseClick(GLFWwindow* window, int button, int action, int mods);

// predict the future position for the obj
Vector3d posPrediction(Vector3d curr_pos, Vector3d curr_lin_vel, double time_duration, double curr_time, Simulation::Sai2Simulation* sim);

// flags for scene camera movement
bool fTransXp = false;
bool fTransXn = false;
bool fTransYp = false;
bool fTransYn = false;
bool fTransZp = false;
bool fTransZn = false;
bool fRotPanTilt = false;
bool fRobotLinkSelect = false;

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	// start redis client
	redis_client = RedisClient();
	redis_client.connect();

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
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
//    robot->_q = VectorXd::Zero(10);
//    robot->_dq = VectorXd::Zero(10);
	robot->updateModel();
    robot->updateKinematics();
    
    // load second robot
    auto robot2 = new Sai2Model::Sai2Model(robot2_file, false);
    robot2->updateModel();
    robot2->updateKinematics();

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_file, false);
	sim->setJointPositions(robot_name, robot->_q);
	sim->setJointVelocities(robot_name, robot->_dq);


	// fill in object information 
	for (int i = 0; i < n_objects; ++i) {
		Vector3d _object_pos, _object_lin_vel, _object_ang_vel;
		Quaterniond _object_ori;
		sim->getObjectPosition(object_names[i], _object_pos, _object_ori);
		sim->getObjectVelocity(object_names[i], _object_lin_vel, _object_ang_vel);
		object_pos.push_back(_object_pos);
		object_lin_vel.push_back(_object_lin_vel);
		object_ori.push_back(_object_ori);
		object_ang_vel.push_back(_object_ang_vel);
	}

    // set co-efficient of restition to zero for force control
    sim->setCollisionRestitution(1.0);

    // set co-efficient of friction
    sim->setCoeffFrictionStatic(0.0);
    sim->setCoeffFrictionDynamic(0.0);

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
	int windowW = 1.4 * screenH;
	int windowH = 0.8 * screenH;
	int windowPosY = (screenH - windowH) / 2;
	int windowPosX = windowPosY;

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

	// cache variables
	double last_cursorx, last_cursory;

	// init redis client values 
	redis_client.set(CONTROLLER_RUNNING_KEY, "0");  
	redis_client.setEigenMatrixJSON(JOINT_ANGLES_KEY, robot->_q); 
	redis_client.setEigenMatrixJSON(JOINT_VELOCITIES_KEY, robot->_dq);
    redis_client.setEigenMatrixJSON(JOINT_ANGLES_KEY_SHOOTER, robot2->_q);
    redis_client.setEigenMatrixJSON(JOINT_VELOCITIES_KEY_SHOOTER, robot2->_dq);

	// start simulation thread
	thread sim_thread(simulation, robot, robot2, sim);

	// initialize glew
	glewInitialize();

	// add obj file once 
//	string mesh_filename = "../../model/test_objects/meshes/visual/basketball.obj";
//	addMesh(graphics, mesh_filename, Vector3d(0.2, -0.2, 0), Quaterniond(1, 0, 0, 0), Vector3d(1, 1, 1));

	// while window is open:
	int count = 0;
	Vector3d start_pos = Vector3d(1, -1, 1);

	while (!glfwWindowShouldClose(window) && fSimulationRunning)
	{
		// add sphere for every nth count
		if (count % 60 == 0) {  // default refresh rate
//            string mesh_filename = "../../model/test_objects/meshes/visual/basketball.obj";
//            addMesh(graphics, mesh_filename, start_pos, Quaterniond(1, 0, 0, 0), Vector3d(1, 1, 1));
////			addSphere(graphics, "test", start_pos, Quaterniond(1, 0, 0, 0), 0.01, Vector4d(1, 1, 1, 1));
////			addBox(graphics, "test", start_pos + Vector3d(-2, 0, 0), Quaterniond(1, 0, 0, 0), Vector3d(0.05, 0.05, 0.05), Vector4d(1, 1, 1, 1));
//			start_pos(1) += 1e-1;
		}

		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot_name, robot);
        graphics->updateGraphics(robot2_name, robot2);
		for (int i = 0; i < n_objects; ++i) {
			graphics->updateObjectGraphics(object_names[i], object_pos[i], object_ori[i]);
		}
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

void simulation(Sai2Model::Sai2Model* robot, Sai2Model::Sai2Model* robot2, Simulation::Sai2Simulation* sim)
{
	// prepare simulation
	int dof = robot->dof();
    int dof2 = robot2->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
    VectorXd command_torques2 = VectorXd::Zero(dof2);
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
    redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY_SHOOTER, command_torques2); //added for the second robot
	VectorXd g = VectorXd::Zero(dof);
    VectorXd g2 = VectorXd::Zero(dof2);
	string controller_status = "0";
	double kv = 10;  // can be set to 0 if no damping is needed

	// setup redis callback
	redis_client.createReadCallback(0);
	redis_client.createWriteCallback(0);

	// add to read callback
	redis_client.addStringToReadCallback(0, CONTROLLER_RUNNING_KEY, controller_status);
	redis_client.addEigenToReadCallback(0, JOINT_TORQUES_COMMANDED_KEY, command_torques);
    redis_client.addEigenToReadCallback(0, JOINT_TORQUES_COMMANDED_KEY_SHOOTER, command_torques2);


	// add to write callback
	redis_client.addEigenToWriteCallback(0, JOINT_ANGLES_KEY, robot->_q);
	redis_client.addEigenToWriteCallback(0, JOINT_VELOCITIES_KEY, robot->_dq);
    redis_client.addEigenToWriteCallback(0, JOINT_ANGLES_KEY_SHOOTER, robot2->_q);
    redis_client.addEigenToWriteCallback(0, JOINT_VELOCITIES_KEY_SHOOTER, robot2->_dq);
	redis_client.addEigenToWriteCallback(0, BALL_POS, object_pos[0]);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000);
	bool fTimerDidSleep = true;
	double start_time = timer.elapsedTime();
	double last_time = start_time;
	double pred_start_time;
	int sim_pred_counter = 0;

	// start simulation 
	fSimulationRunning = true;	
	while (fSimulationRunning) {
		fTimerDidSleep = timer.waitForNextLoop();

		// execute redis read callback
		redis_client.executeReadCallback(0);

		// apply gravity compensation 
		robot->gravityVector(g);
        // apply gravity compensation for the second robot
        robot2->gravityVector(g2);

		// set joint torques
		if (controller_status == "1") {
			sim->setJointTorques(robot_name, command_torques + g);
            sim->setJointTorques(robot2_name, command_torques2 + g2);
		} else {
			sim->setJointTorques(robot_name, g - robot->_M * (kv * robot->_dq));
            sim->setJointTorques(robot2_name, g2 - robot2->_M * (kv * robot2->_dq));
		}


		// integrate forward
		double curr_time = timer.elapsedTime();
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

		// get dynamic object positions
		for (int i = 0; i < n_objects; ++i) {
			sim->getObjectPosition(object_names[i], object_pos[i], object_ori[i]);
			sim->getObjectVelocity(object_names[i], object_lin_vel[i], object_ang_vel[i]);
		}

        // calculate future pos
        if (flag == 0 && curr_time >= 1.59 && curr_time <= 1.6) {
            double time_duration = 0.1;
            object_future_pos = posPrediction(object_pos[0], object_lin_vel[0], time_duration, curr_time, sim);
            pred_start_time = curr_time;
        }

        if (flag == 1) {
            sim_pred_counter++;
            if (sim_pred_counter == 100) {
                cout << curr_time << '\t' << object_pos[0].transpose() << endl;
            }
        }


		// execute redis write callback
		redis_client.executeWriteCallback(0);		

		// update last time
		last_time = curr_time;
	}

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Simulation Loop run time  : " << end_time << " seconds\n";
	std::cout << "Simulation Loop updates   : " << timer.elapsedCycles() << "\n";
	std::cout << "Simulation Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

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

Vector3d posPrediction(Vector3d curr_pos, Vector3d curr_lin_vel, double time_duration, double curr_time, Simulation::Sai2Simulation* sim)
{
    // create the future position vector
    Vector3d object_future_pos;
    // read the gravity
    chai3d::cVector3d gravity_g = sim->_world->getGravity(); // gravity
    Vector3d gra_g(gravity_g.x(), gravity_g.y(), gravity_g.z());
    // calculate and print out
    object_future_pos << curr_pos + curr_lin_vel * time_duration + 0.5 * gra_g * time_duration * time_duration;
    cout << "current position" << endl;
    cout << curr_time << '\t' << curr_pos.transpose() << endl;
    cout << "future position" << endl;
    cout << curr_time + time_duration << '\t' << object_future_pos.transpose() << endl;
    flag = 1;
    redis_client.setEigenMatrixJSON(FUTURE_POS, object_future_pos);
    return object_future_pos;
}


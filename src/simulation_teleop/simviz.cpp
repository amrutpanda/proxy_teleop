// @ Author: Amrut Panda
// Special thanks to William chong from Stanford University.

#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "Sai2Simulation.h"
#include <dynamics3d.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "force_sensor/ForceSensorSim.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew

#include <iostream>
#include <string>
#include <random>
#include <queue>

#include <signal.h>
#include "redis_keys.h"
bool fSimulationRunning = false;
void sighandler(int){fSimulationRunning = false;}

using namespace std;
using namespace Eigen;
using namespace chai3d;

const string world_file = "../model/iiwa14/world.urdf";
const string robot_file = "../model/iiwa14/iiwa14.urdf";
// const string robot_file = "./resources/iiwa14.urdf";
// const string robot_name = "panda";
const string robot_name = "iiwa14";
const string camera_name = "camera";
// const string link_name = "end_effector"; //robot end-effector
const string link_name = "link7";

// redis keys:
// const string CONTROLLER_RUNNING_KEY = "sai2::dual_proxy::simviz::controller_running_key";
// string JOINT_ANGLES_KEY = "sai2::dual_proxy::simviz::sensors::q";
// string JOINT_VELOCITIES_KEY = "sai2::dual_proxy::simviz::sensors::dq";
// string ROBOT_COMMAND_TORQUES_KEY = "sai2::dual_proxy::simviz::actuators::tau_cmd";

// string ROBOT_SENSED_FORCE_KEY = "sai2::dual_proxy::simviz::sensors::sensed_force";

RedisClient redis_client;

// simulation function prototype
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim, ForceSensorSim* force_sensor);

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

// callback when a mouse button is pressed
void mouseClick(GLFWwindow* window, int button, int action, int mods);

// flags for scene camera movement
bool fTransXp = false;
bool fTransYp = false;
bool fTransXn = false;
bool fTransYn = false;
bool fTransZp = false;
bool fTransZn = false;
bool fshowCameraPose = false;
bool fRotPanTilt = false;
// flag for enabling/disabling remote task
bool fOnOffRemote = false;

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
	Vector3d camera_pos, camera_lookat, camera_vertical;
	graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);

	// load robots
	Affine3d T_workd_robot = Affine3d::Identity();
	T_workd_robot.translation() = Vector3d(0, 0, 0);
	auto robot = new Sai2Model::Sai2Model(robot_file, false, T_workd_robot);
	VectorXd q_init(7);
	// q_init << 0, 18, 0, -70, 0, 90, -90;  // changed last joint from 0 to -90
	// q_init << 0, 18, 0, -70, 0, 90, -71 * 0;
	// q_init *= M_PI/180.0;
	// q_init << 1.55686,-0.448781,-0.0481126,-2.12039,0.019258,1.66288,-0.0841794;
	q_init << 0, -0.036, 0, -1.57, 0, 1.57, 0.0;
	robot->_q = q_init;
	robot->_dq = VectorXd::Zero(7);
	robot->updateModel();

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_file, false);
	sim->setCollisionRestitution(0);
	sim->setCoeffFrictionStatic(0.0);

	// read joint positions, velocities, update model
	sim->setJointPositions(robot_name, robot->_q);
	sim->setJointVelocities(robot_name, robot->_dq);
	

	// Add force sensor to the end-effector
	Affine3d sensor_transform_in_link = Affine3d::Identity();
	const Vector3d sensor_pos_in_link = Eigen::Vector3d(0.0,0.0,0.0);
	const Vector3d pos_in_link = Vector3d(0.0,0.0,0.0815);
	sensor_transform_in_link.translation() = sensor_pos_in_link;
	auto force_sensor = new ForceSensorSim(robot_name, link_name, sensor_transform_in_link, robot);
	// force_sensor->enableFilter(0.01);

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
	int windowW = 0.8 * screenH;
	int windowH = 0.5 * screenH;
	int windowPosY = (screenH - windowH) / 2;
	int windowPosX = windowPosY;

	// create window and make it current
	glfwWindowHint(GLFW_VISIBLE, 0);
	GLFWwindow* window = glfwCreateWindow(windowW, windowH, "Kuka simulation", NULL, NULL);
	glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	// set callbacks
	glfwSetKeyCallback(window, keySelect);
	glfwSetMouseButtonCallback(window, mouseClick);

	// cache variables
	double last_cursorx, last_cursory;

	redis_client.set(CONTROLLER_RUNNING_KEY, "0"); 
	redis_client.setEigenMatrixJSON(JOINT_ANGLES_KEY, robot->_q); 
	redis_client.setEigenMatrixJSON(JOINT_VELOCITIES_KEY, robot->_dq); 
	redis_client.setEigenMatrixJSON(ROBOT_SENSED_FORCE_KEY, VectorXd::Zero(6));

	fSimulationRunning = true;
	thread sim_thread(simulation, robot, sim, force_sensor);

	// while window is open:
	while (!glfwWindowShouldClose(window) && fSimulationRunning)
	{
		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot_name, robot);
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
		graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
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
			camera_pos = camera_pos + 0.01*cam_roll_axis;
			camera_lookat = camera_lookat + 0.01*cam_roll_axis;
		}
		if (fTransXn) {
			camera_pos = camera_pos - 0.01*cam_roll_axis;
			camera_lookat = camera_lookat - 0.01*cam_roll_axis;
		}
		if (fTransYp) {
			// camera_pos = camera_pos + 0.05*cam_lookat_axis;
			camera_pos = camera_pos + 0.01*cam_up_axis;
			camera_lookat = camera_lookat + 0.01*cam_up_axis;
		}
		if (fTransYn) {
			// camera_pos = camera_pos - 0.05*cam_lookat_axis;
			camera_pos = camera_pos - 0.01*cam_up_axis;
			camera_lookat = camera_lookat - 0.01*cam_up_axis;
		}
		if (fTransZp) {
			camera_pos = camera_pos + 0.05*cam_depth_axis;
			camera_lookat = camera_lookat + 0.05*cam_depth_axis;
		}
		if (fTransZn) {
			camera_pos = camera_pos - 0.05*cam_depth_axis;
			camera_lookat = camera_lookat - 0.05*cam_depth_axis;
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
	}

	// stop simulation
	fSimulationRunning = false;
	sim_thread.join();

	// destroy context
	glfwDestroyWindow(window);

	// terminate
	glfwTerminate();

	return 0;
}



//------------------------------------------------------------------------------
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim, ForceSensorSim* force_sensor) {

	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	VectorXd gravity_torques = VectorXd::Zero(dof);
	redis_client.setEigenMatrixJSON(ROBOT_COMMAND_TORQUES_KEY, command_torques);

	// for testing.
	// command_torques << 1.0,0.0,0.0,0.0,0.0,0.0,0.0;

	// sensed force
	Vector3d sensed_force = Vector3d::Zero();
	Vector3d sensed_moment = Vector3d::Zero();
	VectorXd sensed_force_moment = VectorXd::Zero(6);

	// redis communication
	redis_client.createReadCallback(0);
	redis_client.addEigenToReadCallback(0, ROBOT_COMMAND_TORQUES_KEY, command_torques);

	redis_client.createWriteCallback(0);
	redis_client.addEigenToWriteCallback(0, JOINT_ANGLES_KEY, robot->_q);
	redis_client.addEigenToWriteCallback(0, JOINT_VELOCITIES_KEY, robot->_dq);
	redis_client.addEigenToWriteCallback(0, ROBOT_SENSED_FORCE_KEY, sensed_force_moment);


	// create a timer
	double sim_frequency = 1000.0;
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(sim_frequency);
	double last_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	unsigned long long simulation_counter = 0;

	while (fSimulationRunning) {
		fTimerDidSleep = timer.waitForNextLoop();

		// particle pos from controller
		redis_client.executeReadCallback(0);

		robot->gravityVector(gravity_torques);
		if (redis_client.get(CONTROLLER_RUNNING_KEY) == "0") {
			sim->setJointTorques(robot_name,command_torques + gravity_torques + (-1)* robot->_dq);
		}
		else {
			sim->setJointTorques(robot_name, command_torques + 1* gravity_torques + (-0.01)* robot->_dq);
		}

		// integrate forward
		sim->integrate(0.001);

		// read joint positions, velocities, update model
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);
		robot->updateKinematics();
		robot->updateModel();

		// // read end-effector task forces from the force sensor simulation
		force_sensor->update(sim);
		force_sensor->getForceLocalFrame(sensed_force);
		force_sensor->getMomentLocalFrame(sensed_moment);
		// May need modification here.
		sensed_force_moment << -sensed_force, -sensed_moment*0; // make sensed moment 0 at this time.
		
		redis_client.executeWriteCallback(0);

		simulation_counter++;
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

void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	bool set = (action != GLFW_RELEASE);
	switch(key) {
		case GLFW_KEY_ESCAPE:
			// exit application
			glfwSetWindowShouldClose(window,GL_TRUE);
			break;
		case GLFW_KEY_RIGHT:
			fTransXp = set;
			break;
		case GLFW_KEY_LEFT:
			fTransXn = set;
			break;
		case GLFW_KEY_UP:
			fTransYp = set;
			break;
		case GLFW_KEY_DOWN:
			fTransYn = set;
			break;
		case GLFW_KEY_A:
			fTransZp = set;
			break;
		case GLFW_KEY_Z:
			fTransZn = set;
			break;
		case GLFW_KEY_R:
			fOnOffRemote = set;
	    break;
		case GLFW_KEY_S:
			fshowCameraPose = set;
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
			//TODO: menu
			break;
		// if middle click: don't handle. doesn't work well on laptops
		case GLFW_MOUSE_BUTTON_MIDDLE:
			break;
		default:
			break;
	}
}
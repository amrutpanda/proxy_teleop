#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "force_sensor/ForceSensorSim.h"
#include "force_sensor/ForceSensorDisplay.h"

#include <iostream>
#include <fstream>
#include <string>

#define RAD(deg) ((double)(deg) * M_PI / 180.0)

// handle ctrl-c nicely
#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "../model/iiwa14/world.urdf";
const string robot_name = "iiwa";

// redis keys:
const string CONTROLLER_RUNNING_KEY = "sai2::dual_proxy::simviz::controller_running_key";
string JOINT_ANGLES_KEY = "sai2::dual_proxy::simviz::sensors::q";
string JOINT_VELOCITIES_KEY = "sai2::dual_proxy::simviz::sensors::dq";
string ROBOT_COMMAND_TORQUES_KEY = "sai2::dual_proxy::simviz::actuators::tau_cmd";

string ROBOT_SENSED_FORCE_KEY = "sai2::dual_proxy::simviz::sensors::sensed_force";

unsigned long long controller_counter = 0;

// helper function 
double sat(double x) {
	if (abs(x) <= 1.0) {
		return x;
	}
	else {
		return signbit(x);
	}
}


int main(int argc, char const *argv[])
{
    
	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	VectorXd initial_q = robot->_q;
	robot->updateModel();

	// prepare controller
	int dof = robot->dof();
	const string ee_link_name = "link7";
	const Vector3d pos_in_ee_link = Vector3d(0, 0, 0.0815);
	VectorXd command_torques = VectorXd::Zero(dof);

	// model quantities for operational space control
	MatrixXd Jv = MatrixXd::Zero(3,dof);
	MatrixXd Lambda = MatrixXd::Zero(3,3);
	// MatrixXd J_bar = MatrixXd::Zero(dof,3);
	MatrixXd N = MatrixXd::Zero(dof,dof);

	robot->Jv(Jv, ee_link_name, pos_in_ee_link);
	robot->taskInertiaMatrix(Lambda, Jv);
	// robot->dynConsistentInverseJacobian(J_bar, Jv);
	robot->nullspaceMatrix(N, Jv);

	// initialize force and moment on end-effector
	Eigen::Vector3d force, moment;
	// force = redis_client.getEigenMatrixJSON(EE_FORCE_KEY);
	// moment = redis_client.getEigenMatrixJSON(EE_MOMENT_KEY);

    string controller_number ;
    
    // check number of arguments.
    if (argc < 2){
        runtime_error("Need another argument.");
    }
    controller_number = argv[1];

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	redis_client.set(CONTROLLER_RUNNING_KEY, "1");

    while (runloop)
    {
        // wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
		robot->updateModel();
        if(controller_number == "1")
		{
			Vector3d x, x_d, dx, F;
			VectorXd g(dof), joint_task_torque(dof);

			double kp = 100;
			double kv = 20;
			double kpj = 50;
			double kvj = 14;

			// update Jv
			MatrixXd Jv = MatrixXd::Zero(3,7);
			robot->Jv(Jv,ee_link_name,pos_in_ee_link);
			// update Lambda
			MatrixXd Lambda = MatrixXd::Zero(3,3);
			robot->taskInertiaMatrix(Lambda,Jv);
			// update N
			MatrixXd N = MatrixXd::Zero(7,7);
			robot->nullspaceMatrix(N,Jv);
			// update x
			robot->position(x,ee_link_name,pos_in_ee_link);
			// update dx
			robot->linearVelocity(dx,ee_link_name,pos_in_ee_link);
			// update g
			robot->gravityVector(g);

			// set x_d
			x_d << 0.3 + 0.1*sin(M_PI*time), 0.1 + 0.1* cos(M_PI*time), 0.5 ;

			// calculate joint_task_torque
			VectorXd h(dof);
			robot->coriolisPlusGravity(h);
			joint_task_torque.setZero();
			joint_task_torque = robot->_M * robot->_ddq + h;

			// calculate F
			F.setZero();
			F =  Lambda*( - kp*(x - x_d) - kv*dx);

			// calculate command_torques
			command_torques.setZero();
			command_torques = Jv.transpose()*F + N.transpose()* ( - kpj*(robot->_q) - kvj*(robot->_dq) ) + g;

		}



        // **********************
		// WRITE YOUR CODE BEFORE
		// **********************
        

        command_torques.setZero();
		// send to redis
		redis_client.setEigenMatrixJSON(ROBOT_COMMAND_TORQUES_KEY, command_torques);

		controller_counter++;

    }

    command_torques.setZero();
	redis_client.setEigenMatrixJSON(ROBOT_COMMAND_TORQUES_KEY, command_torques);
	redis_client.set(CONTROLLER_RUNNING_KEY, "0");

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";
    
    return 0;
}


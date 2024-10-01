/* @ Amrut panda
   @ Haptic controller.
*/
#include <iostream>
#include <string>

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "tasks/JointTask.h"
#include "tasks/PosOriTask.h"
#include "redis_keys.h"
#include "timer/LoopTimer.h"
#include "perception/ForceSpaceParticleFilter.h"

#include <signal.h>

enum STATE {
    INIT = 0,
    CONTROL
};

bool runloop = false;
void sighandler(int){runloop = false;}
// particle filter thread declaration.
void particle_filter();

// redis client object.
RedisClient redis_client;

const string robot_file = "../model/iiwa14/iiwa14.urdf";

// particle filter parameters
const int n_particles = 1000;
MatrixXd particle_positions_to_redis = MatrixXd::Zero(3, n_particles);
int force_space_dimension = 0;
int prev_force_space_dimension = 0;
Matrix3d sigma_force = Matrix3d::Zero();
Matrix3d sigma_motion = Matrix3d::Identity();

Vector3d motion_control_pfilter;
Vector3d force_control_pfilter;
Vector3d measured_velocity_pfilter;
Vector3d measured_force_pfilter;

queue<Vector3d> pfilter_motion_control_buffer;
queue<Vector3d> pfilter_force_control_buffer;
queue<Vector3d> pfilter_sensed_force_buffer;
queue<Vector3d> pfilter_sensed_velocity_buffer;

const double control_loop_freq = 1000.0;
const double pfilter_freq = 50.0;
const double freq_ratio_filter_control = pfilter_freq / control_loop_freq;

// set control link name and control point for ee posori control.
const string link_name = "link7";
const Vector3d pos_in_link = Vector3d(0.0, 0.0, 0.0815);

// set force sensor frame transforms in ee frame.
Affine3d sensor_tranform_in_link = Affine3d::Identity();
const Vector3d sensor_pos_in_link = Vector3d(0.0, 0.0, 0.0);

int main()
{
    // start redis client local.
    redis_client = RedisClient();
    redis_client.connect();

    // setup signal handler
    signal(SIGABRT,&sighandler);
    signal(SIGTERM,&sighandler);
    signal(SIGINT, &sighandler);

    // load the robot.
    Affine3d T_world_robot = Affine3d::Identity();
    T_world_robot.translation() = Vector3d(0, 0, 0);
    auto robot = new Sai2Model::Sai2Model(robot_file,false,T_world_robot);

    // robot joint angles from redis.
    robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
    robot->_dq.setZero();
    robot->updateModel();

    int dof = robot->dof();
    VectorXd command_torques = VectorXd::Zero(dof);
    VectorXd coriolis = VectorXd::Zero(dof);
    MatrixXd N_prec = MatrixXd::Identity(dof,dof);
    // set the initial state
    int state = INIT;

    // joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);
	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->_use_interpolation_flag = true;
	joint_task->_use_velocity_saturation_flag = true;

	joint_task->_kp = 100.0;
	joint_task->_kv = 20.0;
	joint_task->_ki = 0.0;

    joint_task->_desired_position = robot->_q;
    // joint_task->_desired_position << 0, -0.236, 0, -1.57, 0, 1.57, 0.0;

    // posori_task
    auto posori_task = new Sai2Primitives::PosOriTask(robot,link_name,pos_in_link);
    // computed torque will be stored inside posori_task_torques matrix.
    VectorXd posori_task_torques = VectorXd::Zero(dof);
    Vector3d x_init = posori_task->_current_position;
    Matrix3d R_init = posori_task->_current_orientation;
    // intially set the robot proxy and haptic proxy.
    redis_client.setEigenMatrixJSON(ROBOT_PROXY_KEY,x_init);
    redis_client.setEigenMatrixJSON(ROBOT_PROXY_ROT_KEY,R_init);
    redis_client.setEigenMatrixJSON(HAPTIC_PROXY_KEY,x_init);

    // compute expected default rotation and position offset and send it to haptics.
    Matrix3d R_default = Matrix3d::Identity();
    Vector3d pos_default = Vector3d::Zero();
    robot->rotation(R_default,link_name);
    robot->position(pos_default,link_name,pos_in_link);
    redis_client.setEigenMatrixJSON(ROBOT_DEFAULT_POS_KEY,pos_default);
    redis_client.setEigenMatrixJSON(ROBOT_DEFAULT_ROT_KEY,R_default);

    // set posori task flags and gains.
    posori_task->_kp_pos = 100.0;
	posori_task->_kv_pos = 20.0;

	posori_task->_kp_ori = 50.0;
	posori_task->_kv_ori = 14.0;

    posori_task->_use_interpolation_flag = true;
    // setting up _otg parameters.
    posori_task->_otg->setMaxLinearVelocity(0.3);
    posori_task->_otg->setMaxLinearJerk(5.0);

	posori_task->_otg->setMaxAngularVelocity(M_PI/1.5);
	posori_task->_otg->setMaxAngularAcceleration(3*M_PI);
	posori_task->_otg->setMaxAngularJerk(15*M_PI);

    // force sensing.
    Matrix3d R_link_sensor = Matrix3d::Identity();
    sensor_tranform_in_link.translation() = sensor_pos_in_link;
    sensor_tranform_in_link.linear() = R_link_sensor;
    // set forcesensor frame in posori.
    posori_task->setForceSensorFrame(link_name,sensor_tranform_in_link);
    
    VectorXd sensed_force_moment_local_frame = VectorXd::Zero(6);
	VectorXd sensed_force_moment_world_frame = VectorXd::Zero(6);

    double tool_mass = 0;
	Vector3d tool_com = Vector3d::Zero();
    Vector3d init_force = Vector3d::Zero();
    // set first loop to be true.
    bool first_loop = true;

    // remove inertial forces from tool
	Vector3d tool_velocity = Vector3d::Zero();
	Vector3d prev_tool_velocity = Vector3d::Zero();
	Vector3d tool_acceleration = Vector3d::Zero();
	Vector3d tool_inertial_forces = Vector3d::Zero();

    // dual proxy parameters and variables
	double k_vir = 250.0;
	double max_force_diff = 0.1;
	double max_force = 10.0;

    // setup robot and haptic proxies.
    Vector3d robot_proxy = Vector3d::Zero();
	Matrix3d robot_proxy_rot = Matrix3d::Identity();
	Vector3d haptic_proxy = Vector3d::Zero();
	Vector3d prev_desired_force = Vector3d::Zero();

    // set haptic ready key.
    int haptic_ready = 0;
	redis_client.set(HAPTIC_DEVICE_READY_KEY, "0");


	// setup redis read and write keys to be updated with the callback
	redis_client.createReadCallback(0);
	redis_client.createWriteCallback(0);

    // objects to read from redis 
    redis_client.addEigenToReadCallback(0,JOINT_ANGLES_KEY,robot->_q);
    redis_client.addEigenToReadCallback(0,JOINT_VELOCITIES_KEY,robot->_dq);

    MatrixXd mass_from_robot = MatrixXd::Identity(dof,dof);
    VectorXd coriolis_from_robot = VectorXd::Zero(dof);

    redis_client.addEigenToReadCallback(0,ROBOT_PROXY_KEY,robot_proxy);
    redis_client.addEigenToReadCallback(0,ROBOT_PROXY_ROT_KEY,robot_proxy_rot);
    redis_client.addIntToReadCallback(0,HAPTIC_DEVICE_READY_KEY,haptic_ready);
    redis_client.addEigenToReadCallback(0,ROBOT_SENSED_FORCE_KEY,sensed_force_moment_local_frame);

    // objects to write to redis.
    redis_client.addEigenToWriteCallback(0,ROBOT_COMMAND_TORQUES_KEY,command_torques);
    redis_client.addEigenToWriteCallback(0,HAPTIC_PROXY_KEY,haptic_proxy);
    redis_client.addEigenToWriteCallback(0, SIGMA_FORCE_KEY, sigma_force);
    redis_client.addIntToWriteCallback(0,FORCE_SPACE_DIMENSION_KEY,force_space_dimension);

    // start particle filter thread
	runloop = true;
	redis_client.set(CONTROLLER_RUNNING_KEY,"1");
	thread particle_filter_thread(particle_filter);

    // create a timer
	unsigned long long controller_counter = 0;
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(control_loop_freq); //Compiler en mode release
	double current_time = 0;
	double prev_time = 0;
	// double dt = 0;
	bool fTimerDidSleep = true;
	double start_time = timer.elapsedTime(); //secs

    while (runloop)
    {
        // wait for next scheduled loop
		timer.waitForNextLoop();
		current_time = timer.elapsedTime() - start_time;

		// read haptic state and robot state
		redis_client.executeReadCallback(0);
        // update the robot model.
        robot->updateModel();
		robot->coriolisForce(coriolis);

        // for experimentation.
        Vector3d target_pos = Vector3d( 0.5, 0.3, 0.4);
        // robot_proxy = target_pos;

        N_prec.setIdentity(dof,dof);
		posori_task->updateTaskModel(N_prec);
		N_prec = posori_task->_N;
		joint_task->updateTaskModel(N_prec);
        // add ee weight to the sensed force. in case of simulation the tool mass is 0. so no forces or moments are present.
        Matrix3d R_world_sensor;
		robot->rotation(R_world_sensor, link_name);
		R_world_sensor = R_world_sensor * R_link_sensor;
		Vector3d p_tool_local_frame = tool_mass * R_world_sensor.transpose() * Vector3d(0,0,-9.81);
		sensed_force_moment_local_frame.head(3) += p_tool_local_frame;
		sensed_force_moment_local_frame.tail(3) += tool_com.cross(p_tool_local_frame);

        if (first_loop) {
			init_force = sensed_force_moment_local_frame.head(3);
			first_loop = false;
		}
		sensed_force_moment_local_frame.head(3) -= init_force;
        // update forces for posori task
		posori_task->updateSensedForceAndMoment(sensed_force_moment_local_frame.head(3), sensed_force_moment_local_frame.tail(3));
		sensed_force_moment_world_frame.head(3) = R_world_sensor * sensed_force_moment_local_frame.head(3);
		sensed_force_moment_world_frame.tail(3) = R_world_sensor * sensed_force_moment_local_frame.tail(3);

        if (state == INIT) {
            joint_task->updateTaskModel(MatrixXd::Identity(dof,dof));
            joint_task->computeTorques(joint_task_torques);
           
            command_torques = joint_task_torques + coriolis;
            if (haptic_ready && (joint_task->_desired_position - joint_task->_current_position).norm() < 0.2) {
                posori_task->reInitializeTask();
                joint_task->reInitializeTask();

                joint_task->_kp = 50.0;
				joint_task->_kv = 13.0;
				joint_task->_ki = 0.0;

                // change state to CONTROL.
                cout << "Switching to CONTROL mode \n" ;
                state = CONTROL;
            }
        }

        else if (state == CONTROL) {
            // dual proxy
            posori_task->_sigma_force = sigma_force;
            posori_task->_sigma_position = sigma_motion;

            Vector3d robot_position = posori_task->_current_position;
            Vector3d motion_proxy = robot_position + sigma_motion*(robot_proxy - robot_position);

            Vector3d desired_force = k_vir * sigma_force * (robot_proxy - robot_position);
            Vector3d desired_force_diff = desired_force - prev_desired_force;
			if (desired_force_diff.norm() > max_force_diff) {
				desired_force = prev_desired_force + desired_force_diff*max_force_diff/desired_force_diff.norm();
			}
			if(desired_force.norm() > max_force) {
				desired_force *= max_force/desired_force.norm();
			}

            //control the robot.
            posori_task->_desired_position = motion_proxy;
            posori_task->_desired_force = desired_force;
            posori_task->_desired_orientation = robot_proxy_rot;

            // cout << "desired position: \n" << motion_proxy << "\nrobot proxy: "<< robot_proxy << "\n";
            // cout << "current position: \n" << posori_task->_current_position << "\n";
            // cout << "desired force: \n" << desired_force << "\n";

            try {
                posori_task->computeTorques(posori_task_torques);
            }
            catch(const exception& e)
            {
                cout << "control cycle: " << controller_counter << endl;
				cout << "error in the torque computation of posori_task:" << endl;
				cerr << e.what() << endl;
				cout << "setting torques to zero for this control cycle" << endl;
				cout << endl;
            }

            joint_task->computeTorques(joint_task_torques);
            command_torques = posori_task_torques + joint_task_torques + coriolis;

            prev_desired_force = desired_force;
			prev_force_space_dimension = force_space_dimension;
        }
        // write control torques and dual proxy variables
		robot->position(haptic_proxy, link_name, pos_in_link);
		redis_client.executeWriteCallback(0);

        // particle filter
		pfilter_motion_control_buffer.push(sigma_motion * (robot_proxy - posori_task->_current_position) * freq_ratio_filter_control);
		pfilter_force_control_buffer.push(sigma_force * (robot_proxy - posori_task->_current_position) * freq_ratio_filter_control);
		// pfilter_motion_control_buffer.push(sigma_motion * posori_task->_Lambda_modified.block<3,3>(0,0) * posori_task->_linear_motion_control * freq_ratio_filter_control);
		// pfilter_force_control_buffer.push(sigma_force * posori_task->_linear_force_control * freq_ratio_filter_control);

		pfilter_sensed_velocity_buffer.push(posori_task->_current_velocity * freq_ratio_filter_control);
		pfilter_sensed_force_buffer.push(sensed_force_moment_world_frame.head(3) * freq_ratio_filter_control);

		motion_control_pfilter += pfilter_motion_control_buffer.back();
		force_control_pfilter += pfilter_force_control_buffer.back();
		measured_velocity_pfilter += pfilter_sensed_velocity_buffer.back();
		measured_force_pfilter += pfilter_sensed_force_buffer.back();

		if(pfilter_motion_control_buffer.size() > 1/freq_ratio_filter_control) {
			motion_control_pfilter -= pfilter_motion_control_buffer.front();
			force_control_pfilter -= pfilter_force_control_buffer.front();
			measured_velocity_pfilter -= pfilter_sensed_velocity_buffer.front();
			measured_force_pfilter -= pfilter_sensed_force_buffer.front();

			pfilter_motion_control_buffer.pop();
			pfilter_force_control_buffer.pop();
			pfilter_sensed_velocity_buffer.pop();
			pfilter_sensed_force_buffer.pop();			
		}

        controller_counter++;
    }
       
    //// Send zero force/torque to robot ////
	command_torques.setZero();
	redis_client.setEigenMatrixJSON(ROBOT_COMMAND_TORQUES_KEY, command_torques);
	redis_client.set(CONTROLLER_RUNNING_KEY,"0");


	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
	std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

    return 0;
}

void particle_filter() {
	// start redis client for particles
	auto redis_client_particles = RedisClient();
	redis_client_particles.connect();

	unsigned long long pf_counter = 0;

	// create particle filter
	auto pfilter = new Sai2Primitives::ForceSpaceParticleFilter(n_particles);

	pfilter->_mean_scatter = 0.0;
	pfilter->_std_scatter = 0.025;

	pfilter->_alpha_add = 0.3;
	pfilter->_alpha_remove = 0.05;

	pfilter->_F_low = 2.0;
	pfilter->_F_high = 6.0;
	pfilter->_v_low = 0.01;
	pfilter->_v_high = 0.07;

	pfilter->_F_low_add = 5.0;
	pfilter->_F_high_add = 10.0;
	pfilter->_v_low_add = 0.02;
	pfilter->_v_high_add = 0.1;

	Vector3d evals = Vector3d::Zero();
	Matrix3d evecs = Matrix3d::Identity();

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(pfilter_freq); //Compiler en mode release
	double current_time = 0;
	double prev_time = 0;
	// double dt = 0;
	bool fTimerDidSleep = true;
	double start_time = timer.elapsedTime(); //secs

	while(runloop) {
		timer.waitForNextLoop();

		pfilter->update(motion_control_pfilter, force_control_pfilter, measured_velocity_pfilter, measured_force_pfilter);
		sigma_force = pfilter->getSigmaForce(); // disable to only control motion
		sigma_motion = Matrix3d::Identity() - sigma_force;
        cout << "sigma force : \n" << sigma_force << "\n" << "sigma motion: \n" << sigma_motion << "\n";
		force_space_dimension = pfilter->_force_space_dimension;
		cout << "force_space dimension: \n" << force_space_dimension << "\n";
		cout << "+++++++++++++++++++++++++++++++++++++" << "\n";
		// for(int i=0 ; i<n_particles ; i++)
		// {
		// 	particle_positions_to_redis.col(i) = pfilter->_particles[i];
		// }
		// redis_client_particles.setEigenMatrixJSON(PARTICLE_POSITIONS_KEY, particle_positions_to_redis);

		pf_counter++;
	}

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Particle Filter Loop run time  : " << end_time << " seconds\n";
	std::cout << "Particle Filter Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Particle Filter Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";
}

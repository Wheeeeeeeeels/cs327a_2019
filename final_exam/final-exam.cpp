/*======================================================================================
 * final-exam-sol.cpp
 *
 * Implement whole body hierarchical control
 * on a 7-DOF panda robot with a mobile base
 * by completing the "FILL ME IN" section 
 *
 * 
 *
 * Elena Galbally, Spring 2019 *
 *======================================================================================*/

/* --------------------------------------------------------------------------------------
   Include Required Libraries and Files
-----------------------------------------------------------------------------------------*/
#include <iostream>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <math.h>

#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "Sai2Simulation.h"
#include <dynamics3d.h>
#include <chai3d.h>

#include "timer/LoopTimer.h"

#include <GLFW/glfw3.h> 

using namespace std;

const double time_slowdown_factor = 1;

const string world_fname = "resources/final_exam/world_panda.urdf";
const string robot_fname = "../resources/panda/panda_arm_base.urdf";
const string robot_name = "panda";
const string camera_name = "camera";
const string ee_link_name = "link7";

// Desired joint posture
Eigen::VectorXd q_des;

// Motion type selection
enum MOTION {
	MOTION1=0,
	MOTION2,
	MOTION3,
	N_MOTIONS
};
MOTION enum_motion;
void selectMotion(char* input);

/* ----------------------------------------------------------------------------------
	Simulation and Control Loop Setup
-------------------------------------------------------------------------------------*/

// simulation loop
bool fSimulationRunning = false;
void control(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim);
void simulation(Simulation::Sai2Simulation* sim);

// initialize window manager
GLFWwindow* glfwInitialize();

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);


/* =======================================================================================
   MAIN LOOP
========================================================================================== */
int main (int argc, char** argv) {

	// get motion type
	if (argc < 2) {
		cout << "Usage: ./final-exam <motion type: 1 or 2 or 3>" << endl;
		return 0;
	}
	selectMotion(argv[1]);

	cout << "Loading URDF world model file: " << world_fname << endl;

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_fname, false);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_fname, false);

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_fname, false);

    // set initial conditions
    q_des.setZero(robot->dof());
    q_des<<  0.0, //base joints
    		 0.0,
    		 0.0,
    		-0.00396337, // arm joints
  			-0.741172,
 			 0.00154581,
    		-2.4351,
  			-0.264621,
    		 1.69044,
  			 0.0294049;

	robot->_q = q_des;
	sim->setJointPositions(robot_name, robot->_q);
	robot->updateModel();
	Eigen::Affine3d ee_trans;

	// initialize GLFW window
	GLFWwindow* window = glfwInitialize();

    // set callbacks
	glfwSetKeyCallback(window, keySelect);

	// start the simulation thread first
    fSimulationRunning = true;
	thread sim_thread(simulation, sim);

	// next start the control thread
	thread ctrl_thread(control, robot, sim);
	
    // while window is open:
    while (!glfwWindowShouldClose(window)) {

		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot_name, robot);
		graphics->render(camera_name, width, height);
		glfwSwapBuffers(window);
		glFinish();

	    // poll for events
	    glfwPollEvents();
	}

	// stop simulation
	fSimulationRunning = false;
	sim_thread.join();
	ctrl_thread.join();

    // destroy context
    glfwDestroyWindow(window);

    // terminate
    glfwTerminate();

	return 0;
}

/* ----------------------------------------------------------------------------------
	Utility functions
-------------------------------------------------------------------------------------*/
void selectMotion(char* input) {
	switch (input[0]) {
		case '1':
			enum_motion = MOTION1;
			break;
		case '2':
			enum_motion = MOTION2;
			break;
		case '3':
			enum_motion = MOTION3;
			break;
		default:
			cout << "Usage: ./final-exam <motion type: 1 or 2 or 3>" << endl;
			exit(0);
	}
}

/* =======================================================================================
   CONTROL LOOP
========================================================================================== */
void control(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim) {
	
	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(200); //200Hz timer
	double last_time = timer.elapsedTime()/time_slowdown_factor; //secs

	// cache variables
	bool fTimerDidSleep = true;
	bool fTorqueUseDefaults = false; // set true when torques are overriden for the first time
	Eigen::VectorXd tau = Eigen::VectorXd::Zero(robot->dof());
	Eigen::VectorXd g(robot->dof());
	
	Eigen::Vector3d pos_in_link; //position of the end effector in link 7
	pos_in_link<< 0.0,0.0,0.107;
	
	Eigen::Vector3d ee_pos_des;
	Eigen::Vector3d ee_vel_des;
	Eigen::Vector3d ee_acc_des;
	ee_acc_des << 0.0, 0.0, 0.0;
	ee_vel_des << 0.0, 0.0, 0.0;

	// suggested gains 
	const double kpx = 20; //op. space
	const double kvx = 10;
	const double kpj = 40; //joint space
	const double kvj = 30;

	while (fSimulationRunning) { //automatically set to false when the simulation breaks
		fTimerDidSleep = timer.waitForNextLoop();

		// update time
		double curr_time = timer.elapsedTime()/time_slowdown_factor;
		double loop_dt = curr_time - last_time;

        // read joint position, velocity
        sim->getJointPositions(robot_name, robot->_q);
        sim->getJointVelocities(robot_name, robot->_dq);
        robot->updateModel();	

        /* ------------------------------------------------------------------------------------
			FILL ME IN: compute joint torques
		-------------------------------------------------------------------------------------*/
  		// Some hints as to how you might go about this:
  		//		- Update the desired ee position and desired base joint angles for each motion 
  		//		- Compute end-effector task torques, tau_ee
  		//		- Compute torques associated to base trajectory, tau_base
  		//		- Remember to compensate for gravity and add joint damping for stability

		switch (enum_motion) {

			case MOTION1: //base moves and ee fixed
				break;

			case MOTION2: //base still and ee makes circles 
				break;

			case MOTION3: //both make circles
				break;

			default:
				break; 
		}

		/* --------------------------------------------------------- */
		sim->setJointTorques(robot_name, tau);

		// update last time
		last_time = curr_time;
	}
}

/* =======================================================================================
   SIMULATION SETUP
   -----------------------
   * Simulation loop
   * Window initialization
   * Window error
   * Mouse click commands
========================================================================================== */
void simulation(Simulation::Sai2Simulation* sim) {
	fSimulationRunning = true;

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); //1000Hz timer
	
	double last_time = timer.elapsedTime()/time_slowdown_factor; //secs
	bool fTimerDidSleep = true;
	while (fSimulationRunning) {
		fTimerDidSleep = timer.waitForNextLoop();
		// integrate forward
		double curr_time = timer.elapsedTime()/time_slowdown_factor;
		double loop_dt = curr_time - last_time; 
		sim->integrate(loop_dt);
		// update last time
		last_time = curr_time;
	}
}

//------------------------------------------------------------------------------
GLFWwindow* glfwInitialize() { //--- Window initialization ---//

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
    GLFWwindow* window = glfwCreateWindow(windowW, windowH, "SAI2.0 - FINAL EXAM", NULL, NULL);
	glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
    glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	return window;
}

//------------------------------------------------------------------------------

void glfwError(int error, const char* description) { //--- Window error ---//
	cerr << "GLFW Error: " << description << endl;
	exit(1);
}

//------------------------------------------------------------------------------

void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods) //--- Mouse click commands ---//
{
 	// option ESC: exit
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
    {
        // exit application
         glfwSetWindowShouldClose(window, 1);
    }
}

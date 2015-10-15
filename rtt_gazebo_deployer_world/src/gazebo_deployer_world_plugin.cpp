/*
 * gazebo_deployer_world_plugin.cpp
 *
 *  Created on: Oct 12, 2015
 *      Author: dwigand
 */
// Boost
#include <boost/bind.hpp>

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

// Orocos
#include <rtt/deployment/ComponentLoader.hpp>
#include <ocl/DeploymentComponent.hpp>
#include <ocl/LoggingService.hpp>
#include <rtt/Logger.hpp>
//?
//#include <ocl/TaskBrowser.hpp>
//#include <rtt/transports/corba/corba.h>
//#include <rtt/transports/corba/TaskContextServer.hpp>

#include <rtt/scripting/Scripting.hpp>
#include <rtt/plugin/PluginLoader.hpp>

#include <rtt_ros/rtt_ros.h>
#include <rtt_rosclock/rtt_rosclock.h>

// RTT/ROS Simulation Clock Activity

#include "gazebo_deployer_world_plugin.h"

using namespace rtt_gazebo_deployer_world;
using namespace std;
using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(GazeboDeployerWorldPlugin);

boost::mutex GazeboDeployerWorldPlugin::deferred_load_mutex;

GazeboDeployerWorldPlugin::GazeboDeployerWorldPlugin() :
		gazebo::WorldPlugin(), onceDone(false) {
	cout << "Loading WORLD Plugin" << endl;
	/*// Setup TaskContext server if necessary
	 if(CORBA::is_nil(RTT::corba::TaskContextServer::orb)) {
	 // Initialize orb
	 RTT::corba::TaskContextServer::InitOrb(argc, argv);
	 // Propcess orb requests in a thread
	 RTT::corba::TaskContextServer::ThreadOrb();
	 }
	 */
}

GazeboDeployerWorldPlugin::~GazeboDeployerWorldPlugin() {
	// Disconnect from gazebo events
	for (std::vector<gazebo::event::ConnectionPtr>::iterator it =
			update_connections_.begin(); it != update_connections_.end();
			++it) {
		gazebo::event::Events::DisconnectWorldUpdateBegin(*it);
	}

	//TODO test only
	for (std::vector<RTTComponentPack>::iterator componentPackItr =
			all_components_.begin(); componentPackItr != all_components_.end();
			++componentPackItr) {
		componentPackItr->rttComponent->stop();
	}

}

// Overloaded Gazebo entry point
void GazeboDeployerWorldPlugin::Load(gazebo::physics::WorldPtr _parent,
		sdf::ElementPtr sdf) {
	RTT::Logger::Instance()->in("GazeboDeployerWorldPlugin::load");

	parent_model_ = _parent;

	deployer = new OCL::DeploymentComponent("gazebo");
	// is this necessary?
	deployer->import("rtt_rosnode");
	deployer->import("rtt_rosdeployment");
	static_cast<RTT::TaskContext*>(deployer)->loadService("rosdeployment");

	std::string model_component_package = "rtt_lwr_gazebo";
	std::string model_component_type = "LWRGazeboComponent";
	std::string model_component_name = "lwr_gazebo";
	std::string model_name = "kuka-lwr-arm";

	boost::thread(
			boost::bind(&GazeboDeployerWorldPlugin::deployRTTComponentWithModel,
					this, model_component_package, model_component_type,
					model_component_name, model_name));

}

gazebo::physics::ModelPtr GazeboDeployerWorldPlugin::pollForModel(
		std::string modelName, const int timeout) {
	int timer = 0;
	gazebo::physics::ModelPtr model;
	while (!(model = parent_model_->GetModel(modelName))) {
		gazebo::common::Time::MSleep(100);
		timer++;
		if (timer > timeout) {
			gzerr << "Could not find model: " << modelName
					<< ". Perhaps it is not yet loaded..." << std::endl;
			break;
		}
	}
	return model;
	//parent_model_->PrintEntityTree();
}

void GazeboDeployerWorldPlugin::deployRTTComponentWithModel(
		const std::string& model_component_package,
		const std::string& model_component_type,
		const std::string& model_component_name,
		const std::string& model_name) {
	boost::mutex::scoped_lock load_lock(deferred_load_mutex);

	gazebo::physics::ModelPtr specModel = pollForModel(model_name, 1000);
	if (!specModel) {
		gzerr << "Model " << model_name << " could not be found. Skipping."
				<< std::endl;
		return;
	} else {
		cout << "Found model " << model_name << "!" << endl;
	}

//RTT::corba::TaskContextServer* taskcontext_server = RTT::corba::TaskContextServer::Create(deployer);

// deploy component
	RTT::TaskContext* new_rtt_component = NULL;

// Import the package
	if (!rtt_ros::import(model_component_package)) {
		gzerr << "Could not import rtt_gazebo model component package: \""
				<< model_component_package << "\"" << std::endl;
		return;
	}

// Load the component
	if (!deployer->loadComponent(model_component_name, model_component_type)) {
		gzerr << "Could not load rtt_gazebo model component: \""
				<< model_component_type << "\"" << std::endl;
		return;
	}

// Get the model component from the deployer by name
	if (deployer->hasPeer(model_component_name)) {
		new_rtt_component = deployer->getPeer(model_component_name);
	} else {
		gzerr
				<< "SDF model plugin specified a special gazebo component to connect to the gazebo update, named \""
				<< model_component_name
				<< "\", but there is no peer by that name." << std::endl;
		return;
	}

// Make sure the component has the required interfaces
	if (new_rtt_component == NULL) {
		gzerr << "RTT model component was not properly created." << std::endl;
		return;
	}
	if (!new_rtt_component->provides()->hasService("gazebo")) {
		gzerr
				<< "RTT model component does not have required \"gazebo\" service."
				<< std::endl;
		return;
	}
	if (!new_rtt_component->provides("gazebo")->hasOperation("configure")) {
		gzerr
				<< "RTT model component does not have required \"gazebo.configure\" operation."
				<< std::endl;
		return;
	}
	if (!new_rtt_component->provides("gazebo")->hasOperation("update")) {
		gzerr
				<< "RTT model component does not have required \"gazebo.update\" operation."
				<< std::endl;
		return;
	}

// Configure the component with the parent model
	RTT::OperationCaller<bool(gazebo::physics::ModelPtr)> gazebo_configure =
			new_rtt_component->provides("gazebo")->getOperation("configure");

// Make sure the operation is ready
	if (!gazebo_configure.ready()) {
		gzerr
				<< "RTT model component's \"gazebo.configure\" operation could not be connected. Check its signature."
				<< std::endl;
		return;
	}

	if (!gazebo_configure(specModel)) {
		gzerr
				<< "RTT model component's \"gazebo.configure\" operation returned false."
				<< std::endl;
		return;
	}

	// Get gazebo update function
	GazeboUpdateCaller gazebo_update_caller = new_rtt_component->provides(
			"gazebo")->getOperation("update");

	if (!gazebo_update_caller.ready()) {
		gzerr
				<< "RTT model component's \"gazebo.update\" operation could not be connected. Check its signature."
				<< std::endl;
		return;
	}

	//deployed_rtt_components_.push_back(new_rtt_component);
	//gazebo_update_callers_.push_back(gazebo_update_caller);

	RTTComponentPack pack(gazebo_update_caller, specModel, new_rtt_component);
	all_components_.push_back(pack);

	std::string testOps =
			"setActivity(\"lwr_gazebo\",0.001,HighestPriority,ORO_SCHED_RT)\nlwr_gazebo.configure()\nlwr_gazebo.start()";
	loadOrocosScript(testOps);

	// ONLY tests TODO
	new_rtt_component->configure();
	//setActivity("lwr_gazebo",0.001,HighestPriority,ORO_SCHED_RT)
	new_rtt_component->setActivity(new RTT::Activity(ORO_SCHED_RT, 1, 0.001));
	new_rtt_component->start();

	if (!onceDone) {
		// GLOBAL! ONLY ONCE???? Listen to the update event. This event is broadcast every simulation iteration. TODO
		update_connections_.push_back(
				gazebo::event::Events::ConnectWorldUpdateEnd(
						boost::bind(&GazeboDeployerWorldPlugin::gazeboUpdate,
								this)));

		onceDone = true;
		cout << "Connect to Gazebo Update Loop" << endl;
	}
}

void GazeboDeployerWorldPlugin::deployRTTComponent(
		const std::string& model_component_package,
		const std::string& model_component_type,
		const std::string& model_component_name) {
	boost::mutex::scoped_lock load_lock(deferred_load_mutex);

	// deploy component
	RTT::TaskContext* new_rtt_component = NULL;

	// Import the package
	if (!rtt_ros::import(model_component_package)) {
		gzerr << "Could not import rtt_gazebo model component package: \""
				<< model_component_package << "\"" << std::endl;
		return;
	}

	// Load the component
	if (!deployer->loadComponent(model_component_name, model_component_type)) {
		gzerr << "Could not load rtt_gazebo model component: \""
				<< model_component_type << "\"" << std::endl;
		return;
	}

	// Get the model component from the deployer by name
	if (deployer->hasPeer(model_component_name)) {
		new_rtt_component = deployer->getPeer(model_component_name);
	} else {
		gzerr << "Deployed Component " << model_component_name
				<< " could not be found." << std::endl;
		return;
	}

	// perhaps we need a mutex lock? TODO
	deployed_rtt_components_.push_back(new_rtt_component);

	// TODO launch scripts here

	// ONLY tests TODO
	new_rtt_component->configure();
	//setActivity("lwr_gazebo",0.001,HighestPriority,ORO_SCHED_RT)
	new_rtt_component->setActivity(new RTT::Activity(ORO_SCHED_RT, 1, 0.001));
	new_rtt_component->start();
}

// Called by the world update start event
void GazeboDeployerWorldPlugin::gazeboUpdate() {
	RTT::Logger::Instance()->in("GazeboDeployerModelPlugin::gazeboUpdate");

	boost::mutex::scoped_try_lock lock(deferred_load_mutex);

	if (lock) {
		// Call orocos RTT model component gazebo.update() operations
		for (std::vector<RTTComponentPack>::iterator componentPackItr =
				all_components_.begin();
				componentPackItr != all_components_.end(); ++componentPackItr) {
			(componentPackItr->gazeboUpdateCaller)(componentPackItr->modelPtr);
		}
	}
}

void GazeboDeployerWorldPlugin::loadOrocosScriptFromFile(
		std::string oro_script_file) {
	gzlog << "Running orocos ops script from file: " << oro_script_file << "..."
			<< std::endl;
	if (!deployer->runScript(oro_script_file)) {
		gzerr << "Could not run ops script from file " << oro_script_file << "!"
				<< std::endl;
		// TODO perhaps throw an error?!
		return;
	}

// Don't know if I need this TODO

//	// Restore gravity modes
//	for (std::vector<std::pair<gazebo::physics::LinkPtr, bool> >::iterator it =
//			actual_gravity_modes_.begin(); it != actual_gravity_modes_.end();
//			++it) {
//		it->first->SetGravityMode(it->second);
//	}
}

void GazeboDeployerWorldPlugin::loadOrocosScript(std::string oro_script) {
	gzlog << "Running orocos ops script: ..." << std::endl << oro_script
			<< std::endl;
	if (!deployer->getProvider<RTT::Scripting>("scripting")->eval(oro_script)) {
		gzerr << "Could not run ops script!" << std::endl;
		return;
	}

// Don't know if I need this TODO

//	// Restore gravity modes
//	for (std::vector<std::pair<gazebo::physics::LinkPtr, bool> >::iterator it =
//			actual_gravity_modes_.begin(); it != actual_gravity_modes_.end();
//			++it) {
//		it->first->SetGravityMode(it->second);
//	}
}

void GazeboDeployerWorldPlugin::loadLuaScriptFromFile(
		std::string lua_script_file) {
	// Load lua scripting service
	if (!RTT::plugin::PluginLoader::Instance()->loadService("Lua", deployer)) {
		gzerr << "Could not load lua service." << std::endl;
		return;
	}

	RTT::OperationCaller<bool(std::string)> exec_file = deployer->provides(
			"Lua")->getOperation("exec_file");
	RTT::OperationCaller<bool(std::string)> exec_str =
			deployer->provides("Lua")->getOperation("exec_str");

	if (!exec_file.ready() || !exec_str.ready()) {
		gzerr << "Could not get lua operations." << std::endl;
		return;
	}

	// Load rttlib for first-class operation support
	exec_str("require(\"rttlib\")");

	gzlog << "Running orocos lua script file " << lua_script_file << "..."
			<< std::endl;
	if (!exec_file(lua_script_file)) {
		gzerr << "Could not run lua script file " << lua_script_file << "!"
				<< std::endl;
		return;
	}

// Don't know if I need this TODO

//	// Restore gravity modes
//	for (std::vector<std::pair<gazebo::physics::LinkPtr, bool> >::iterator it =
//			actual_gravity_modes_.begin(); it != actual_gravity_modes_.end();
//			++it) {
//		it->first->SetGravityMode(it->second);
//	}
}

void GazeboDeployerWorldPlugin::loadLuaScript(std::string lua_script) {
	// Load lua scripting service
	if (!RTT::plugin::PluginLoader::Instance()->loadService("Lua", deployer)) {
		gzerr << "Could not load lua service." << std::endl;
		return;
	}

	RTT::OperationCaller<bool(std::string)> exec_file = deployer->provides(
			"Lua")->getOperation("exec_file");
	RTT::OperationCaller<bool(std::string)> exec_str =
			deployer->provides("Lua")->getOperation("exec_str");

	if (!exec_file.ready() || !exec_str.ready()) {
		gzerr << "Could not get lua operations." << std::endl;
		return;
	}

	// Load rttlib for first-class operation support
	exec_str("require(\"rttlib\")");

	gzlog << "Running inline orocos lua script:" << std::endl << lua_script
			<< std::endl;
	if (!exec_str(lua_script)) {
		gzerr << "Could not run inline lua script!" << std::endl;
		return;
	}

// Don't know if I need this TODO

//	// Restore gravity modes
//	for (std::vector<std::pair<gazebo::physics::LinkPtr, bool> >::iterator it =
//			actual_gravity_modes_.begin(); it != actual_gravity_modes_.end();
//			++it) {
//		it->first->SetGravityMode(it->second);
//	}
}

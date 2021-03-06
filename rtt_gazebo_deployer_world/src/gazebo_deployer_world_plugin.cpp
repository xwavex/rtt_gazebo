/*
 * gazebo_deployer_world_plugin.cpp
 *
 *  Created on: Oct 12, 2015
 *      Author: dwigand
 */
// Boost
#include <boost/bind.hpp>
#include <boost/weak_ptr.hpp>

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

// Orocos
#include <rtt/deployment/ComponentLoader.hpp>
#include <ocl/DeploymentComponent.hpp>
#include <ocl/LoggingService.hpp>
#include <rtt/Logger.hpp>

#include <rtt/scripting/Scripting.hpp>
#include <rtt/plugin/PluginLoader.hpp>

#include "gazebo_deployer_world_plugin.h"

using namespace rtt_gazebo_deployer_world;
using namespace std;
using namespace gazebo;
using namespace rsb;
using namespace rsb::patterns;

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(GazeboDeployerWorldPlugin);

boost::mutex GazeboDeployerWorldPlugin::deferred_load_mutex;

GazeboDeployerWorldPlugin::GazeboDeployerWorldPlugin() :
		gazebo::WorldPlugin(), onceDone(false) {
	gzmsg << "Loading Gazebo RTT Deployer World Plugin" << endl;

	loader = RTT::ComponentLoader::Instance();

	urdfHelper = boost::shared_ptr<UrdfHelper>(new UrdfHelper());

	gzmsg << "initial loader->getComponentPath(): "
			<< loader->getComponentPath() << endl;

	addNewOrocosComponentPath("/opt/ros/indigo/lib/orocos"); // TODO remove this, find a proper way around!

	// in ros workspace
	//addNewOrocosComponentPath("/homes/dwigand/code/cogimon/rosws/gazebo_world/devel/lib/orocos"); // only for convenience TODO remove

	// outside of ros workspace
	addNewOrocosComponentPath(
			"/homes/dwigand/code/cogimon/rtt_lwr_gazebo/build/orocos"); // only for convenience TODO remove

	gzmsg << "New loader->getComponentPath(): " << loader->getComponentPath()
			<< endl;

	Factory& factory = getFactory();
	server = factory.createLocalServer("/GazeboDeployerWorldPlugin");
	server->registerMethod("deployRTTComponentWithModel",
			LocalServer::CallbackPtr(
					new LocalServer::FunctionCallback<
							rst::cogimon::ModelComponentConfig, bool>(
							boost::bind(
									&GazeboDeployerWorldPlugin::deployRTTComponentWithModel_cb,
									this, _1))));
	server->registerMethod("spawnModel",
			LocalServer::CallbackPtr(
					new LocalServer::FunctionCallback<string, void>(
							boost::bind(
									&GazeboDeployerWorldPlugin::spawnModel_cb,
									this, _1))));
	server->registerMethod("addOrocosComponentIncludePath",
			LocalServer::CallbackPtr(
					new LocalServer::FunctionCallback<string, void>(
							boost::bind(
									&GazeboDeployerWorldPlugin::addOrocosComponentIncludePath_cb,
									this, _1))));

	server->registerMethod("addOrocosPluginIncludePath",
			LocalServer::CallbackPtr(
					new LocalServer::FunctionCallback<string, void>(
							boost::bind(
									&GazeboDeployerWorldPlugin::addOrocosPluginIncludePath_cb,
									this, _1))));

	server->registerMethod("launchScriptFromFile",
			LocalServer::CallbackPtr(
					new LocalServer::FunctionCallback<string, void>(
							boost::bind(
									&GazeboDeployerWorldPlugin::launchScriptFromFile_cb,
									this, _1))));
}

void GazeboDeployerWorldPlugin::launchScriptFromFile_cb(
		boost::shared_ptr<string> scriptPath) {
	// TODO do we need a mutex lock before we run a script?
	gzmsg << "Execute script: " << scriptPath << endl;
	if (ends_with(scriptPath->c_str(), ".ops")) {
		loadOrocosScriptFromFile(scriptPath->c_str());
	} else if (ends_with(scriptPath->c_str(), ".lua")) {
		loadLuaScriptFromFile(scriptPath->c_str());
	} else {
		gzerr << "Script is neither .ops nor .lua. Check file-extensions."
				<< std::endl;
	}
}

void GazeboDeployerWorldPlugin::addOrocosPluginIncludePath_cb(
		boost::shared_ptr<string> path) {
	// /opt/ros/indigo/lib/orocos/gnulinux
	RTT::plugin::PluginLoader::Instance()->setPluginPath(
			"" + string(path->c_str()) + ":"
					+ RTT::plugin::PluginLoader::Instance()->getPluginPath());
	gzdbg << "PluginPath: "
			<< RTT::plugin::PluginLoader::Instance()->getPluginPath() << endl;

	if (!RTT::plugin::PluginLoader::Instance()->loadPlugins(path->c_str())) {
		gzwarn << "Could not load Plugins from: " << path->c_str() << endl;
	}

	vector<string> tmpPluginList =
			RTT::plugin::PluginLoader::Instance()->listPlugins();
	for (int i = 0; i < tmpPluginList.size(); i++) {
		gzdbg << "PluginList[" << i << "] = " << tmpPluginList[i] << endl;
	}

//	RTT::plugin::PluginLoader::Instance()->loadService("scripting", deployer);
//	RTT::plugin::PluginLoader::Instance()->loadService("Lua", deployer);
}

void GazeboDeployerWorldPlugin::addNewOrocosComponentPath(const string path) {
	string tmpPath = path;
	if (*tmpPath.rbegin() == '/') {
		tmpPath.erase(tmpPath.size() - 1);
	}
	if (isPathContained(tmpPath)) {
		gzwarn
				<< "Following path is already contained in orocos component path: "
				<< path << endl;
	} else {
		gzmsg << "Adding orocos component include path: " << path << endl;
		loader->setComponentPath(loader->getComponentPath() + path);
		gzlog << "Updated orocos component path: " << loader->getComponentPath()
				<< endl;
	}
}

bool GazeboDeployerWorldPlugin::isPathContained(const string path) {
	string currCompPath = loader->getComponentPath();
	std::istringstream ss(currCompPath);
	std::string token;

	while (std::getline(ss, token, ':')) {
		if (*token.rbegin() == '/') {
			token.erase(token.size() - 1);
		}
		if (token.compare(path) == 0) {
			return true;
		}
	}
	return false;
}

void GazeboDeployerWorldPlugin::addOrocosComponentIncludePath_cb(
		boost::shared_ptr<string> path) {
	addNewOrocosComponentPath(path->c_str());
}

void GazeboDeployerWorldPlugin::spawnModel_cb(
		boost::shared_ptr<string> modelFile) {
	// we'll definitely need MUTEX here TODO
	/** TODO
	 * 1. Replace Name, Plugin NS, Initial Pos and Orient
	 * 2. Pause Simulation
	 * 3. Publish Model
	 * 4. Wait for the Model to appear
	 * 5. Resume Simulation
	 */

	string model_xml;
	if (!urdfHelper->spawnURDFModel(modelFile->c_str(), model_xml)) {
		return;
	}

	TiXmlDocument gazebo_model_xml;

	// robot name
	string robotName = "kuka-lwr";
	// robot namespace
	string robotNS = "";
	// get initial pose of model
	gazebo::math::Vector3 initial_xyz(0.0, 0.0, 0.0);
	// get initial roll pitch yaw (fixed frame transform) (wxyz)
	gazebo::math::Quaternion initial_q(1, 0, 0, 0);
	// world reference frame "", world, map, /map
	string referenceFrame = "";

	if (!urdfHelper->spawnSDFModel(gazebo_model_xml, parent_model_, model_xml,
			robotName, robotNS, initial_xyz, initial_q, referenceFrame)) {
		return;
	}

	// Create a new transport node
	gazebo::transport::NodePtr node(new gazebo::transport::Node());
	// Initialize the node with the world name
	node->Init(parent_model_->GetName());

	// Create a publisher on the ~/factory topic
	gazebo::transport::PublisherPtr factoryPub = node->Advertise<msgs::Factory>(
			"~/factory");

	// push to factory iface
	std::ostringstream stream;
	stream << gazebo_model_xml;
	std::string gazebo_model_xml_string = stream.str();

	// publish to factory topic
	gazebo::msgs::Factory msg;
	gazebo::msgs::Init(msg, "spawn_model");
	msg.set_sdf(gazebo_model_xml_string);

	// FIXME: should use entity_info or add lock to World::receiveMutex
	// looking for Model to see if it exists already
	gazebo::msgs::Request *entity_info_msg = gazebo::msgs::CreateRequest(
			"entity_info", robotName);

	gazebo::transport::PublisherPtr requestPub = node->Advertise<
			gazebo::msgs::Request>("~/request");
	requestPub->Publish(*entity_info_msg, true);
	// todo: should wait for response response_sub_, check to see that if _msg->response == "nonexistant"

	gazebo::physics::ModelPtr model = parent_model_->GetModel(robotName);
	if (model) {
		gzerr << "SpawnModel: Failure - model name " << robotName.c_str()
				<< " already exist." << endl;
		return; // false
	}

	// Publish the factory message
	factoryPub->Publish(msg);

	/// FIXME: should change publish to direct invocation World::LoadModel() and/or
	///        change the poll for Model existence to common::Events based check.

	/// \brief poll and wait, verify that the model is spawned within Hardcoded 10 seconds

	gazebo::common::Time timeout(10.0);

	boost::shared_ptr<gazebo::common::Timer> modelDeployTimer(
			new gazebo::common::Timer());

	modelDeployTimer->Start();
	while (modelDeployTimer->GetRunning()) {
		if (modelDeployTimer->GetElapsed() > timeout) {
			gzerr
					<< "SpawnModel: Model pushed to spawn queue, but spawn service timed out waiting for model to appear in simulation under the name "
					<< robotName << endl;
			modelDeployTimer->Stop();
			return; // false
		}

		{
			//boost::recursive_mutex::scoped_lock lock(*world->GetMRMutex());
			if (parent_model_->GetModel(robotName)) {
				modelDeployTimer->Stop();
				break;
			}
		}
		usleep(2000);
	}

//	parent_model_->PrintEntityTree();
	return;// true
}

boost::shared_ptr<bool> GazeboDeployerWorldPlugin::deployRTTComponentWithModel_cb(
		boost::shared_ptr<rst::cogimon::ModelComponentConfig> deployerConfig) {

	std::cout << deployerConfig->component_name() << std::endl;

	boost::mutex::scoped_lock load_lock(deferred_load_mutex);

	gazebo::physics::ModelPtr specModel = pollForModel(
			deployerConfig->model_name(), 1000);
	if (!specModel) {
		gzerr << "Model " << deployerConfig->model_name()
				<< " could not be found. Skipping." << std::endl;
		return boost::shared_ptr<bool>(new bool(false));
	} else {
		cout << "Found model " << deployerConfig->model_name() << "!" << endl;
	}

	// deploy component
	RTT::TaskContext* new_rtt_component = NULL;

	boost::shared_ptr<RTT::ComponentLoader> loader =
			RTT::ComponentLoader::Instance();

	// import RTT package // i.e. export RTT_COMPONENT_PATH=/homes/dwigand/code/cogimon/rosws/gazebo_world/devel/lib/orocos:$RTT_COMPONENT_PATH
	if (loader->import(deployerConfig->component_package(),
			loader->getComponentPath())) {
		gzlog << "Importing " << deployerConfig->component_package()
				<< ": Success" << endl;
	} else {
		gzerr << "Could not import " << deployerConfig->component_package()
				<< " from " << loader->getComponentPath() << endl
				<< "Try to set RTT_COMPONENT_PATH." << endl;
	}

	// Load the component
	if (!deployer->loadComponent(deployerConfig->component_name(),
			deployerConfig->component_type())) {
		gzerr << "Could not load rtt_gazebo model component: \""
				<< deployerConfig->component_type() << "\"" << std::endl;
		return boost::shared_ptr<bool>(new bool(false));
	}

	// Get the model component from the deployer by name
	if (deployer->hasPeer(deployerConfig->component_name())) {
		new_rtt_component = deployer->getPeer(deployerConfig->component_name());
	} else {
		gzerr
				<< "SDF model plugin specified a special gazebo component to connect to the gazebo update, named \""
				<< deployerConfig->component_name()
				<< "\", but there is no peer by that name." << std::endl;
		return boost::shared_ptr<bool>(new bool(false));
	}

	// Make sure the component has the required interfaces
	if (new_rtt_component == NULL) {
		gzerr << "RTT model component was not properly created." << std::endl;
		return boost::shared_ptr<bool>(new bool(false));
	}
	if (!new_rtt_component->provides()->hasService("gazebo")) {
		gzerr
				<< "RTT model component does not have required \"gazebo\" service."
				<< std::endl;
		return boost::shared_ptr<bool>(new bool(false));
	}
	if (!new_rtt_component->provides("gazebo")->hasOperation("configure")) {
		gzerr
				<< "RTT model component does not have required \"gazebo.configure\" operation."
				<< std::endl;
		return boost::shared_ptr<bool>(new bool(false));
	}
	if (!new_rtt_component->provides("gazebo")->hasOperation("update")) {
		gzerr
				<< "RTT model component does not have required \"gazebo.update\" operation."
				<< std::endl;
		return boost::shared_ptr<bool>(new bool(false));
	}

	// Configure the component with the parent model
	RTT::OperationCaller<bool(gazebo::physics::ModelPtr)> gazebo_configure =
			new_rtt_component->provides("gazebo")->getOperation("configure");

	// Make sure the operation is ready
	if (!gazebo_configure.ready()) {
		gzerr
				<< "RTT model component's \"gazebo.configure\" operation could not be connected. Check its signature."
				<< std::endl;
		return boost::shared_ptr<bool>(new bool(false));
	}

	if (!gazebo_configure(specModel)) {
		gzerr
				<< "RTT model component's \"gazebo.configure\" operation returned false."
				<< std::endl;
		return boost::shared_ptr<bool>(new bool(false));
	}

	// Get gazebo update function
	GazeboUpdateCaller gazebo_update_caller = new_rtt_component->provides(
			"gazebo")->getOperation("update");

	if (!gazebo_update_caller.ready()) {
		gzerr
				<< "RTT model component's \"gazebo.update\" operation could not be connected. Check its signature."
				<< std::endl;
		return boost::shared_ptr<bool>(new bool(false));
	}

	RTTComponentPack pack(gazebo_update_caller, specModel, new_rtt_component);
	all_components_.push_back(pack);

	if (ends_with(deployerConfig->script(), ".ops")) {
		loadOrocosScriptFromFile(deployerConfig->script());
	} else if (ends_with(deployerConfig->script(), ".lua")) {
		loadLuaScriptFromFile(deployerConfig->script());
	} else {
		gzerr << "Script is neither .ops nor .lua. Check file-extensions."
				<< std::endl;
		return boost::shared_ptr<bool>(new bool(false));
	}

	if (!onceDone) {
		// GLOBAL! ONLY ONCE???? Listen to the update event. This event is broadcast every simulation iteration. TODO
		update_connections_.push_back(
				gazebo::event::Events::ConnectWorldUpdateEnd(
						boost::bind(&GazeboDeployerWorldPlugin::gazeboUpdate,
								this)));

		onceDone = true;
		cout << "Connect to Gazebo Update Loop" << endl;
	}
	return boost::shared_ptr<bool>(new bool(true));
}

GazeboDeployerWorldPlugin::~GazeboDeployerWorldPlugin() {
	// Disconnect from gazebo events
	for (std::vector<gazebo::event::ConnectionPtr>::iterator it =
			update_connections_.begin(); it != update_connections_.end();
			++it) {
		gazebo::event::Events::DisconnectWorldUpdateBegin(*it);
	}

	//deployer->getPeerList() // DO ot this way, do kill everything that is running in the deployer?

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
	RTT::plugin::PluginLoader::Instance()->loadPlugins(
			"/opt/ros/indigo/lib/orocos/gnulinux");

//	deployer->import("rtt_rosnode");
//	deployer->import("rtt_rosdeployment");
//	static_cast<RTT::TaskContext*>(deployer)->loadService("rosdeployment"); // LOAD rsbdeployment TODO
}

gazebo::physics::ModelPtr GazeboDeployerWorldPlugin::pollForModel(
		std::string modelName, const int timeout) {
	int timer = 0;
	gazebo::physics::ModelPtr model;
	while (!(model = parent_model_->GetModel(modelName))) {
		gazebo::common::Time::MSleep(10);
		timer++;
		if (timer > timeout) {
			gzerr << "Could not find model: " << modelName
					<< ". Perhaps it is not yet loaded..." << std::endl;
			break;
		}
	}
	return model;
}

// Called by the world update start event
void GazeboDeployerWorldPlugin::gazeboUpdate() {
	RTT::Logger::Instance()->in("GazeboDeployerModelPlugin::gazeboUpdate");
	boost::mutex::scoped_try_lock lock(deferred_load_mutex);

	// check if model exists, unload component if not. Not sure if its working properly. Can't test it!

	if (lock) {
		// Call orocos RTT model component gazebo.update() operations
		std::vector<RTTComponentPack>::iterator componentPackItr =
				all_components_.begin();

		while (componentPackItr != all_components_.end()) {
//			std::cout << componentPackItr->rttComponent->getName() << ", "
//					<< componentPackItr->modelPtr.use_count() << std::endl;
			if (componentPackItr->modelPtr.use_count() <= 0) {
				cout << "removing: "
						<< componentPackItr->rttComponent->getName()
						<< " due to use_count: "
						<< componentPackItr->modelPtr.use_count() << endl;
				//stops execution of updateHook() of this component
				componentPackItr->rttComponent->stop();
				componentPackItr->rttComponent->cleanup();
				//deployer-> // How to properly remove from deployer? TODO
				componentPackItr = all_components_.erase(componentPackItr);
			} else {
				(componentPackItr->gazeboUpdateCaller)(
						componentPackItr->modelPtr);
				componentPackItr++;
			}
		}
	}
}

void GazeboDeployerWorldPlugin::loadOrocosScriptFromFile(
		std::string oro_script_file) {
	gzlog << "Running orocos ops script from file: " << oro_script_file << "..."
			<< std::endl;

	std::ifstream ifs(oro_script_file.c_str());
	std::string oro_script((std::istreambuf_iterator<char>(ifs)),
			std::istreambuf_iterator<char>());

	if (!deployer->getProvider<RTT::Scripting>("scripting")->eval(oro_script)) { //if (!deployer->runScript(oro_script)) {
		gzerr << "Could not run ops script from file " << oro_script_file << "!"
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

bool GazeboDeployerWorldPlugin::ends_with(const std::string& str,
		const std::string& end) {
	size_t slen = str.size(), elen = end.size();
	if (slen < elen)
		return false;
	while (elen) {
		if (str[--slen] != end[--elen])
			return false;
	}
	return true;
}

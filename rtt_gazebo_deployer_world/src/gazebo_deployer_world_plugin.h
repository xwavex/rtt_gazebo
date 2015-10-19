/*
 * gazebo_deployer_world_plugin.h
 *
 *  Created on: Oct 12, 2015
 *      Author: dwigand
 */
#ifndef __RTT_GAZEBO_DEPLOYER_GAZEBO_DEPLOYER_WORLD_PLUGIN_H
#define __RTT_GAZEBO_DEPLOYER_GAZEBO_DEPLOYER_WORLD_PLUGIN_H

#include <boost/thread/mutex.hpp>

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

// Orocos
#include <rtt/deployment/ComponentLoader.hpp>
#include <ocl/DeploymentComponent.hpp>
#include <ocl/LoggingService.hpp>
#include <rtt/Logger.hpp>

//#include <rtt/scripting/Scripting.hpp>
//#include <rtt/transports/corba/corba.h>
//#include <rtt/transports/corba/TaskContextServer.hpp>

// Custom ROS msgs
#include <rtt_gazebo_msgs/DeployRTTWithModel.h>
#include <ros/ros.h>

#include "RTTComponentPack.h"

namespace rtt_gazebo_deployer_world {

class GazeboDeployerWorldPlugin: public gazebo::WorldPlugin {
public:

	// Constructor
	GazeboDeployerWorldPlugin();

	// Destructor
	virtual ~GazeboDeployerWorldPlugin();

	/*
	 * Init. Hook of the WorldPlugin
	 * sdf::ElementPtr sdf: Pointer the the SDF element of the plugin.
	 */
	void Load(gazebo::physics::WorldPtr _parent, sdf::ElementPtr sdf);


	/*
	 * Deploy random RTT Component
	 */
	void deployRTTComponent(const std::string& model_component_package,
			const std::string& model_component_type,
			const std::string& model_component_name);

	// Called by the world update start event
	void gazeboUpdate();

	/*
	 * RSB Hook to spawn a new deployerThread TODO
	 */

	gazebo::physics::ModelPtr pollForModel(std::string modelName,
			const int timeout);

	void loadOrocosScriptFromFile(std::string oro_script_file);
	void loadOrocosScript(std::string oro_script);
	void loadLuaScriptFromFile(std::string lua_script_file);
	void loadLuaScript(std::string lua_script);

private:
	/*
	 * [component name, component type, component package, (optional) model to attach] TODO
	 * Deploy new rtt component (Threaded)
	 *
	 * 1. If model param. != NULL -> poll until model is loaded (with timeout)
	 * 2. Load rtt component
	 * 3. deploy and configure rtt component
	 *
	 * X. Perhaps this needs to parse an entire Lua Deployment file... TODO TODO TODO
	 */
	// callback function for service
	bool deployRTTComponentWithModel_cb(
			rtt_gazebo_msgs::DeployRTTWithModel::Request& request,
			rtt_gazebo_msgs::DeployRTTWithModel::Response& response);

	// only once condition
	bool onceDone;

	ros::NodeHandle nh_;

	ros::ServiceServer deployWithModel_service_;

	// mutex
	static boost::mutex deferred_load_mutex;

	// Global pointer to the world
	gazebo::physics::WorldPtr parent_model_;
	// Global deployer
	OCL::DeploymentComponent* deployer;
	// Global deployer mutex TODO

	//! A Gazebo event connection to the world update
	std::vector<gazebo::event::ConnectionPtr> update_connections_;

	// Global rtt component list
	std::vector<RTT::TaskContext*> deployed_rtt_components_;

	//! Operation for polling the model component
	typedef RTT::OperationCaller<void(gazebo::physics::ModelPtr)> GazeboUpdateCaller;
	std::vector<GazeboUpdateCaller> gazebo_update_callers_;

	// RTT Component Pack
	std::vector<RTTComponentPack> all_components_;
};
}

#endif // ifndef __RTT_GAZEBO_DEPLOYER_GAZEBO_DEPLOYER_WORLD_PLUGIN_H

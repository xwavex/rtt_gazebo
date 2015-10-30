/*
 * UrdfHelper.h
 *
 *  Created on: Oct 30, 2015
 *      Author: dwigand
 */

#ifndef RTT_GAZEBO_RTT_GAZEBO_DEPLOYER_WORLD_SRC_URDFHELPER_H_
#define RTT_GAZEBO_RTT_GAZEBO_DEPLOYER_WORLD_SRC_URDFHELPER_H_

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <string>
#include <tinyxml.h>

namespace rtt_gazebo_deployer_world {
/**
 * This class acts just as some kind of struct to combine rtt component, associated model and gazebo caller.
 */
class UrdfHelper {
public:
	UrdfHelper();
	virtual ~UrdfHelper();
	bool isURDF(const std::string model_xml);
	bool isSDF(const std::string model_xml);
	bool spawnURDFModel(const std::string modelFile, std::string &model_xml_out);
	bool spawnSDFModel(TiXmlDocument &gazebo_model_xml,
			gazebo::physics::WorldPtr world_, const std::string model_xml_in,
			const std::string model_name, const std::string robot_namespace_,
			gazebo::math::Vector3 initial_xyz,
			gazebo::math::Quaternion initial_q,
			const std::string reference_frame);

private:
	void stripXmlDeclaration(std::string &model_xml);
	void updateSDFAttributes(TiXmlDocument &gazebo_model_xml,
			std::string model_name, gazebo::math::Vector3 initial_xyz,
			gazebo::math::Quaternion initial_q);
	gazebo::math::Pose parsePose(const std::string &str);
	void walkChildAddRobotNamespace(TiXmlNode* robot_xml,
			const std::string robot_namespace_);
	void updateURDFModelPose(TiXmlDocument &gazebo_model_xml,
			gazebo::math::Vector3 initial_xyz,
			gazebo::math::Quaternion initial_q);
	gazebo::math::Vector3 parseVector3(const std::string &str);
	void updateURDFName(TiXmlDocument &gazebo_model_xml,
			std::string model_name);

};
}

#endif /* RTT_GAZEBO_RTT_GAZEBO_DEPLOYER_WORLD_SRC_URDFHELPER_H_ */

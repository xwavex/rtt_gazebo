/*
 * UrdfHelper.cpp
 *
 *  Created on: Oct 30, 2015
 *      Author: dwigand
 */

#include "UrdfHelper.h"

using namespace std;

namespace rtt_gazebo_deployer_world {

UrdfHelper::UrdfHelper() {

}

UrdfHelper::~UrdfHelper() {

}

bool UrdfHelper::isURDF(const string model_xml) {
	TiXmlDocument doc_in;
	doc_in.Parse(model_xml.c_str());
	if (doc_in.FirstChild("robot"))
		return true;
	else
		return false;
}

bool UrdfHelper::isSDF(const string model_xml) {
	// FIXME: very crude check
	TiXmlDocument doc_in;
	doc_in.Parse(model_xml.c_str());
	if (doc_in.FirstChild("gazebo") || doc_in.FirstChild("sdf")) // sdf
		return true;
	else
		return false;
}

bool UrdfHelper::spawnURDFModel(const string modelFile, string &model_xml_out) {
	std::ifstream ifs(modelFile.c_str());
	std::string model_xml((std::istreambuf_iterator<char>(ifs)),
			std::istreambuf_iterator<char>());

	if (!isURDF(model_xml)) {
		gzerr << "SpawnModel: Failure - model format is not URDF." << endl;
		return false;
	}

	{
		std::string open_bracket("<?");
		std::string close_bracket("?>");
		size_t pos1 = model_xml.find(open_bracket, 0);
		size_t pos2 = model_xml.find(close_bracket, 0);
		if (pos1 != std::string::npos && pos2 != std::string::npos)
			model_xml.replace(pos1, pos2 - pos1 + 2, std::string(""));
	}

	{
		// this has to be done in preface and not here!
		std::string package_prefix("package://");
		size_t pos1 = model_xml.find(package_prefix, 0);
		while (pos1 != std::string::npos) {
			size_t pos2 = model_xml.find("/", pos1 + 10);
			if (pos2 == std::string::npos || pos1 >= pos2) {
				gzerr << "Malformed package name in file: " << modelFile
						<< endl;
				break;
			} else if (model_xml.at(pos2 - 1) == '.') {
				std::string package_name = model_xml.substr(pos1 + 10,
						pos2 - pos1 - 10);

				std::size_t foundLastSlash = modelFile.find_last_of("/");
				string package_path = modelFile.substr(0, foundLastSlash + 1);

				int offset = 1;
				if (model_xml.at(pos2 - 2) == '.')
					offset = 2;

				model_xml.replace(pos1, (pos2 - pos1 - offset), package_path);
			} else {
				model_xml.replace(pos1, 10, "");
			}

			pos1 = model_xml.find(package_prefix, 0);
		}
	}
	model_xml_out = model_xml;
	return true;
}

bool UrdfHelper::spawnSDFModel(TiXmlDocument &gazebo_model_xml,
		gazebo::physics::WorldPtr world_, const string model_xml_in,
		const string model_name, const string robot_namespace_,
		gazebo::math::Vector3 initial_xyz, gazebo::math::Quaternion initial_q,
		const string reference_frame) {

	// refernce frame for initial pose definition, modify initial pose if defined
	gazebo::physics::LinkPtr frame = boost::dynamic_pointer_cast<
			gazebo::physics::Link>(world_->GetEntity(reference_frame));
	if (frame) {
		// convert to relative pose
		gazebo::math::Pose frame_pose = frame->GetWorldPose();
		initial_xyz = frame_pose.rot.RotateVector(initial_xyz);
		initial_xyz += frame_pose.pos;
		initial_q *= frame_pose.rot;
	}

	/// @todo: map is really wrong, need to use tf here somehow
	else if (reference_frame == "" || reference_frame == "world"
			|| reference_frame == "map" || reference_frame == "/map") {
		gzdbg
				<< "SpawnModel: reference_frame is empty/world/map, using inertial frame"
				<< endl;
	} else {
		gzwarn
				<< "SpawnModel: reference reference_frame not found, did you forget to scope the link by model name?"
				<< endl;
		return false;
	}

	// incoming robot model string
	std::string model_xml = model_xml_in;

	// store resulting Gazebo Model XML to be sent to spawn queue
	// get incoming string containg either an URDF or a Gazebo Model XML
	// grab from parameter server if necessary convert to SDF if necessary
	stripXmlDeclaration(model_xml);

	// put string in TiXmlDocument for manipulation
	gazebo_model_xml.Parse(model_xml.c_str());

	// optional model manipulations: update initial pose && replace model name
	if (isSDF(model_xml)) {
		updateSDFAttributes(gazebo_model_xml, model_name, initial_xyz,
				initial_q);

		// Walk recursively through the entire SDF, locate plugin tags and
		// add robotNamespace as a child with the correct namespace
		if (!robot_namespace_.empty()) {
			// Get root element for SDF
			TiXmlNode* model_tixml = gazebo_model_xml.FirstChild("sdf");
			model_tixml =
					(!model_tixml) ?
							gazebo_model_xml.FirstChild("gazebo") : model_tixml;
			if (model_tixml) {
				walkChildAddRobotNamespace(model_tixml, robot_namespace_);
			} else {
				gzwarn << "Unable to add robot namespace to xml" << endl;
			}
		}
	} else if (isURDF(model_xml)) {
		updateURDFModelPose(gazebo_model_xml, initial_xyz, initial_q);
		updateURDFName(gazebo_model_xml, model_name);

		// Walk recursively through the entire URDF, locate plugin tags and
		// add robotNamespace as a child with the correct namespace
		if (!robot_namespace_.empty()) {
			// Get root element for URDF
			TiXmlNode* model_tixml = gazebo_model_xml.FirstChild("robot");
			if (model_tixml) {
				walkChildAddRobotNamespace(model_tixml, robot_namespace_);
			} else {
				gzwarn << "Unable to add robot namespace to xml" << endl;
			}
		}
	} else {
		gzerr
				<< "GazeboRosApiPlugin SpawnModel Failure: input xml format not recognized"
				<< endl;
		return false;
	}
	return true;
}

void UrdfHelper::stripXmlDeclaration(string &model_xml) {
	// incoming robot model string is a string containing a Gazebo Model XML
	/// STRIP DECLARATION <? ... xml version="1.0" ... ?> from model_xml
	/// @todo: does tinyxml have functionality for this?
	/// @todo: should gazebo take care of the declaration?
	std::string open_bracket("<?");
	std::string close_bracket("?>");
	size_t pos1 = model_xml.find(open_bracket, 0);
	size_t pos2 = model_xml.find(close_bracket, 0);
	if (pos1 != std::string::npos && pos2 != std::string::npos)
		model_xml.replace(pos1, pos2 - pos1 + 2, std::string(""));
}

void UrdfHelper::updateSDFAttributes(TiXmlDocument &gazebo_model_xml,
		std::string model_name, gazebo::math::Vector3 initial_xyz,
		gazebo::math::Quaternion initial_q) {
	// This function can handle both regular SDF files and <include> SDFs that are used with the
	// Gazebo Model Database

	TiXmlElement* pose_element; // This is used by both reguar and database SDFs

	// Check SDF for requires SDF element
	TiXmlElement* gazebo_tixml = gazebo_model_xml.FirstChildElement("sdf");
	if (!gazebo_tixml) {
		gzwarn
				<< "Could not find <sdf> element in sdf, so name and initial position cannot be applied"
				<< endl;
		return;
	}

	// Check SDF for optional model element. May not have one
	TiXmlElement* model_tixml = gazebo_tixml->FirstChildElement("model");
	if (model_tixml) {
		// Update model name
		if (model_tixml->Attribute("name") != NULL) {
			// removing old model name
			model_tixml->RemoveAttribute("name");
		}
		// replace with user specified name
		model_tixml->SetAttribute("name", model_name);
	} else {
		// Check SDF for world element
		TiXmlElement* world_tixml = gazebo_tixml->FirstChildElement("world");
		if (!world_tixml) {
			gzwarn
					<< "Could not find <model> or <world> element in sdf, so name and initial position cannot be applied"
					<< endl;
			return;
		}
		// If not <model> element, check SDF for required include element
		model_tixml = world_tixml->FirstChildElement("include");
		if (!model_tixml) {
			gzwarn
					<< "Could not find <include> element in sdf, so name and initial position cannot be applied"
					<< endl;
			return;
		}

		// Check for name element
		TiXmlElement* name_tixml = model_tixml->FirstChildElement("name");
		if (!name_tixml) {
			// Create the name element
			name_tixml = new TiXmlElement("name");
			model_tixml->LinkEndChild(name_tixml);
		}

		// Set the text within the name element
		TiXmlText* text = new TiXmlText(model_name);
		name_tixml->LinkEndChild(text);
	}

	// Check for the pose element
	pose_element = model_tixml->FirstChildElement("pose");
	gazebo::math::Pose model_pose;

	// Create the pose element if it doesn't exist
	// Remove it if it exists, since we are inserting a new one
	if (pose_element) {
		// save pose_element in math::Pose and remove child
		model_pose = this->parsePose(pose_element->GetText());
		model_tixml->RemoveChild(pose_element);
	}

	// Set and link the pose element after adding initial pose
	{
		// add pose_element Pose to initial pose
		gazebo::math::Pose new_model_pose = model_pose
				+ gazebo::math::Pose(initial_xyz, initial_q);

		// Create the string of 6 numbers
		std::ostringstream pose_stream;
		gazebo::math::Vector3 model_rpy = new_model_pose.rot.GetAsEuler(); // convert to Euler angles for Gazebo XML
		pose_stream << new_model_pose.pos.x << " " << new_model_pose.pos.y
				<< " " << new_model_pose.pos.z << " " << model_rpy.x << " "
				<< model_rpy.y << " " << model_rpy.z;

		// Add value to pose element
		TiXmlText* text = new TiXmlText(pose_stream.str());
		TiXmlElement* new_pose_element = new TiXmlElement("pose");
		new_pose_element->LinkEndChild(text);
		model_tixml->LinkEndChild(new_pose_element);
	}
}

gazebo::math::Pose UrdfHelper::parsePose(const string &str) {
	std::vector<std::string> pieces;
	std::vector<double> vals;

	boost::split(pieces, str, boost::is_any_of(" "));
	for (unsigned int i = 0; i < pieces.size(); ++i) {
		if (pieces[i] != "") {
			try {
				vals.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
			} catch (boost::bad_lexical_cast &e) {
				sdferr << "xml key [" << str << "][" << i << "] value ["
						<< pieces[i]
						<< "] is not a valid double from a 3-tuple\n";
				return gazebo::math::Pose();
			}
		}
	}

	if (vals.size() == 6)
		return gazebo::math::Pose(vals[0], vals[1], vals[2], vals[3], vals[4],
				vals[5]);
	else {
		gzerr << "Beware: failed to parse string " << str.c_str()
				<< " as gazebo::math::Pose, returning zeros." << endl;
		return gazebo::math::Pose();
	}
}

void UrdfHelper::walkChildAddRobotNamespace(TiXmlNode* robot_xml,
		const string robot_namespace_) {
	TiXmlNode* child = 0;
	child = robot_xml->IterateChildren(child);
	while (child != NULL) {
		if (child->ValueStr().find(std::string("plugin")) == 0) {
			if (child->FirstChildElement("robotNamespace") == NULL) {
				TiXmlElement* child_elem =
						child->ToElement()->FirstChildElement("robotNamespace");
				while (child_elem) {
					child->ToElement()->RemoveChild(child_elem);
					child_elem = child->ToElement()->FirstChildElement(
							"robotNamespace");
				}
				TiXmlElement* key = new TiXmlElement("robotNamespace");
				TiXmlText* val = new TiXmlText(robot_namespace_);
				key->LinkEndChild(val);
				child->ToElement()->LinkEndChild(key);
			}
		}
		walkChildAddRobotNamespace(child, robot_namespace_);
		child = robot_xml->IterateChildren(child);
	}
}

void UrdfHelper::updateURDFModelPose(TiXmlDocument &gazebo_model_xml,
		gazebo::math::Vector3 initial_xyz, gazebo::math::Quaternion initial_q) {
	TiXmlElement* model_tixml = (gazebo_model_xml.FirstChildElement("robot"));
	if (model_tixml) {
		// replace initial pose of robot
		// find first instance of xyz and rpy, replace with initial pose
		TiXmlElement* origin_key = model_tixml->FirstChildElement("origin");

		if (!origin_key) {
			origin_key = new TiXmlElement("origin");
			model_tixml->LinkEndChild(origin_key);
		}

		gazebo::math::Vector3 xyz;
		gazebo::math::Vector3 rpy;
		if (origin_key->Attribute("xyz")) {
			xyz = this->parseVector3(origin_key->Attribute("xyz"));
			origin_key->RemoveAttribute("xyz");
		}
		if (origin_key->Attribute("rpy")) {
			rpy = this->parseVector3(origin_key->Attribute("rpy"));
			origin_key->RemoveAttribute("rpy");
		}

		// add xyz, rpy to initial pose
		gazebo::math::Pose model_pose = gazebo::math::Pose(xyz, rpy)
				+ gazebo::math::Pose(initial_xyz, initial_q);

		std::ostringstream xyz_stream;
		xyz_stream << model_pose.pos.x << " " << model_pose.pos.y << " "
				<< model_pose.pos.z;

		std::ostringstream rpy_stream;
		gazebo::math::Vector3 model_rpy = model_pose.rot.GetAsEuler(); // convert to Euler angles for Gazebo XML
		rpy_stream << model_rpy.x << " " << model_rpy.y << " " << model_rpy.z;

		origin_key->SetAttribute("xyz", xyz_stream.str());
		origin_key->SetAttribute("rpy", rpy_stream.str());
	} else
		gzwarn
				<< "could not find <model> element in sdf, so name and initial position is not applied"
				<< endl;
}

gazebo::math::Vector3 UrdfHelper::parseVector3(const string &str) {
	std::vector<std::string> pieces;
	std::vector<double> vals;

	boost::split(pieces, str, boost::is_any_of(" "));
	for (unsigned int i = 0; i < pieces.size(); ++i) {
		if (pieces[i] != "") {
			try {
				vals.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
			} catch (boost::bad_lexical_cast &e) {
				sdferr << "xml key [" << str << "][" << i << "] value ["
						<< pieces[i]
						<< "] is not a valid double from a 3-tuple\n";
				return gazebo::math::Vector3();
			}
		}
	}

	if (vals.size() == 3)
		return gazebo::math::Vector3(vals[0], vals[1], vals[2]);
	else {
		gzerr << "Beware: failed to parse string " << str.c_str()
				<< " as gazebo::math::Vector3, returning zeros." << endl;
		return gazebo::math::Vector3();
	}
}

void UrdfHelper::updateURDFName(TiXmlDocument &gazebo_model_xml,
		std::string model_name) {
	TiXmlElement* model_tixml = gazebo_model_xml.FirstChildElement("robot");
	// replace model name if one is specified by the user
	if (model_tixml) {
		if (model_tixml->Attribute("name") != NULL) {
			// removing old model name
			model_tixml->RemoveAttribute("name");
		}
		// replace with user specified name
		model_tixml->SetAttribute("name", model_name);
	} else
		gzwarn << "could not find <robot> element in URDF, name not replaced"
				<< endl;
}

}

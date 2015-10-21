/*
 * RTTComponentPack.h
 *
 *  Created on: Oct 14, 2015
 *      Author: dwigand
 */

#ifndef RTT_GAZEBO_RTT_GAZEBO_DEPLOYER_WORLD_RTTCOMPONENTPACK_H_
#define RTT_GAZEBO_RTT_GAZEBO_DEPLOYER_WORLD_RTTCOMPONENTPACK_H_

// Gazebo
#include <gazebo/physics/physics.hh>
// rtt
#include <rtt/deployment/ComponentLoader.hpp>
#include <rtt/OperationCaller.hpp>

#include <boost/weak_ptr.hpp>

namespace rtt_gazebo_deployer_world {
/**
 * This class acts just as some kind of struct to combine rtt component, associated model and gazebo caller.
 */
class RTTComponentPack {
public:
	RTTComponentPack(
			RTT::OperationCaller<void(gazebo::physics::ModelPtr)> _gzUpdateCaller,
			gazebo::physics::ModelPtr _mptr, RTT::TaskContext* _component);
	virtual ~RTTComponentPack();
	gazebo::physics::ModelPtr modelPtr;
	RTT::OperationCaller<void(gazebo::physics::ModelPtr)> gazeboUpdateCaller;
	RTT::TaskContext* rttComponent;
};
}

#endif /* RTT_GAZEBO_RTT_GAZEBO_DEPLOYER_WORLD_RTTCOMPONENTPACK_H_ */

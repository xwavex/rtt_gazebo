/*
 * RTTComponentPack.cpp
 *
 *  Created on: Oct 14, 2015
 *      Author: dwigand
 */

#include "RTTComponentPack.h"

namespace rtt_gazebo_deployer_world {

RTTComponentPack::RTTComponentPack(
		RTT::OperationCaller<void(gazebo::physics::ModelPtr)> _gzUpdateCaller,
		gazebo::physics::ModelPtr _mptr, RTT::TaskContext* _component) {
	this->gazeboUpdateCaller = _gzUpdateCaller;
	this->modelPtr = _mptr;
	this->rttComponent = _component;
}

RTTComponentPack::~RTTComponentPack() {
}

}

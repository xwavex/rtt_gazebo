#include <rtt/RTT.hpp>
#include <rtt/internal/GlobalService.hpp>

#include "rtt_clock.h"
#include "rtt_clock_sim_clock_thread.h"

#include <rtt/os/StartStopManager.hpp>

namespace {
  boost::shared_ptr<rtt_clock::SimClockThread> sim_clock_thread;
}

void unloadROSClockService() {
  sim_clock_thread.reset();
}

void loadROSClockService(){
  RTT::Service::shared_ptr clock = RTT::internal::GlobalService::Instance()->provides("ros")->provides("clock");

  clock->doc("RTT service for realtime and non-realtime clock measurement.");

  // Create sim time thread
  sim_clock_thread = rtt_clock::SimClockThread::Instance();
  RTT::os::StartStopManager::Instance()->stopFunction(&unloadROSClockService);

  // Getting current time 
  clock->addOperation("host_now", &rtt_clock::host_now).doc(
      "Get a ros::Time structure based on the NTP-corrected RT time or the ROS simulation time.");
  clock->addOperation("host_wall_now", &rtt_clock::host_now).doc(
      "Get a ros::Time structure based on the NTP-corrected RT time or the ROS wall time.");
  clock->addOperation("rtt_now", &rtt_clock::rtt_now).doc(
      "Get a ros::Time structure based on the RTT time source.");
//  clock->addOperation("rtt_wall_now", &rtt_clock::rtt_wall_now).doc(
//      "Get a ros::Time structure based on the RTT wall clock time.");

//  // Getting time offset
//  clock->addOperation("host_offset_from_rtt", &rtt_clock::host_offset_from_rtt).doc(
//      "Get the difference between the Orocos wall clock and the NTP-corrected wall clock in seconds (host_wall - rtt_wall).");

  // Setting the source for the simulation clock
//  clock->addOperation("useROSClockTopic", &rtt_clock::use_ros_clock_topic).doc(
//      "Use the ROS /clock topic source for updating simulation time.");
  clock->addOperation("useManualClock", &rtt_clock::use_manual_clock).doc(
      "Use a manual source for simulation time by calling updateSimClock.");

  // Enabling/Disabling simulation clock
  clock->addOperation("enableSimClock", &rtt_clock::enable_sim).doc(
      "Enable simulation time based on the ROS /clock topic if the /use_sim_time parameter is set. This will override RTT::os::TimeService");
  clock->addOperation("disableSimClock", &rtt_clock::disable_sim).doc(
      "Disable simulation time based on the ROS /clock topic.");

  clock->addOperation("updateSimClock", &rtt_clock::update_sim_clock).doc(
      "Update the current simulation time and update all SimClockActivities as per their respective frequencies.").arg(
          "time","Current simulated time in seconds.");
}

using namespace RTT;
extern "C" {
  bool loadRTTPlugin(RTT::TaskContext* c){
    if (c != 0) return false;
    loadROSClockService();
    return true;
  }
  std::string getRTTPluginName (){
    return "clock";
  }
  std::string getRTTTargetName (){
    return OROCOS_TARGET_NAME;
  }
}

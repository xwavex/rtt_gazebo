
#include <cstdlib>

// Boost
#include <boost/bind.hpp>
#include <boost/thread/locks.hpp>

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

// Orocos
#include <rtt/deployment/ComponentLoader.hpp>
#include <ocl/DeploymentComponent.hpp>
#include <ocl/TaskBrowser.hpp>
#include <ocl/LoggingService.hpp>
#include <rtt/Logger.hpp>


#include <rtt/os/startstop.h>
#include <rtt/scripting/Scripting.hpp>
#include <rtt/transports/corba/corba.h>
#include <rtt/transports/corba/TaskContextServer.hpp>

// RTT/ROS Simulation Clock Activity
#include "rtt_clock.h"

//#ifdef RTT_GAZEBO_DEBUG
//#include <rtt_clock/prof.h>
//#include <rtt_clock/throttle.h>
//#endif

#include "rtt_system_plugin.h"

using namespace rtt_gazebo_system;

void RTTSystemPlugin::Load(int argc, char **argv)
{
  // Initialize RTT
  __os_init(argc, argv);

  RTT::Logger::log().setStdStream(std::cerr);
  RTT::Logger::log().mayLogStdOut(true);
  RTT::Logger::log().setLogLevel(RTT::Logger::Info);

  // Setup TaskContext server if necessary
  if(CORBA::is_nil(RTT::corba::TaskContextServer::orb)) {
	  std::cout << "Launching ORB!" << std::endl;
    // Initialize orb
    RTT::corba::TaskContextServer::InitOrb(argc, argv);
    // Propcess orb requests in a thread
    RTT::corba::TaskContextServer::ThreadOrb();
  }
}

void RTTSystemPlugin::Init()
{
  // Initialize and enable the simulation clock
  rtt_clock::use_manual_clock();
  rtt_clock::enable_sim();

  update_connection_ =
    gazebo::event::Events::ConnectWorldUpdateEnd(
        boost::bind(&RTTSystemPlugin::updateClock, this));

  // TODO: Create a worldupdateend connection
  
  simulate_clock_ = true;
}

RTTSystemPlugin::~RTTSystemPlugin()
{
  // Stop the Orb thread
  if(!CORBA::is_nil(RTT::corba::TaskContextServer::orb)) {
    RTT::corba::TaskContextServer::ShutdownOrb();
    RTT::corba::TaskContextServer::DestroyOrb();
    RTT::corba::TaskContextServer::CleanupServers();
  }
}

void RTTSystemPlugin::updateClock()
{
  // Wait for previous update thread
  if(update_thread_.joinable()) {
    update_thread_.join();
  }

  // Start update thread
  update_thread_ = boost::thread(
      boost::bind(&RTTSystemPlugin::updateClockLoop, this));
}

void RTTSystemPlugin::updateClockLoop()
{
  {

    // Get the simulation time
    gazebo::common::Time gz_time = gazebo::physics::get_world()->GetSimTime();

    // Update the clock from the simulation time and execute the SimClockActivities
    // NOTE: all orocos TaskContexts which use a SimClockActivity are updated within this call
//#ifdef RTT_GAZEBO_DEBUG
//    static rtt_clock::WallProf prof(5.0);
//    static rtt_clock::WallThrottle throttle(ros::Duration(1.0));
//
//    prof.tic();
//#endif

    const uint64_t one_E9 = 1000000000ll;

    rtt_clock::update_sim_clock(gz_time.sec * one_E9 + gz_time.nsec);

//#ifdef RTT_GAZEBO_DEBUG
//    prof.toc();
//    if(throttle.ready()) {
//      prof.analyze();
//      RTT::log(RTT::Debug) << prof.mean() << " +/- " << prof.stddev() <<" [s] ("<<prof.n()<<") for update_sim_clock()" << RTT::endlog();
//    }
//    static ros::Time last_update_time = rtt_clock::rtt_wall_now();
//#endif
  }
}

GZ_REGISTER_SYSTEM_PLUGIN(rtt_gazebo_system::RTTSystemPlugin)

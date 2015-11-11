#include <time.h>
#include <rtt/RTT.hpp>

#include "rtt_clock.h"
#include "rtt_clock_sim_clock_activity.h"
#include "rtt_clock_sim_clock_thread.h"

namespace rtt_clock {
boost::shared_ptr<rtt_clock::SimClockThread> sim_clock_thread;
}

const uint64_t rtt_clock::host_now() {
	const uint64_t one_E9 = 1000000000ll;

	if (SimClockThread::GetInstance()
			&& SimClockThread::GetInstance()->simTimeEnabled()) {
		return rtt_now();
	}

#ifdef __XENO__
	// Use Xenomai 2.6 feature to get the NTP-synched real-time clock
	timespec ts = {0,0};
	int ret = clock_gettime(CLOCK_HOST_REALTIME, &ts);
	if(ret) {
		RTT::log(RTT::Error) << "Could not query CLOCK_HOST_REALTIME (" << CLOCK_HOST_REALTIME <<"): "<< errno << RTT::endlog();
		return rtt_clock::rtt_now();
	}

	//return ros::Time(ts.tv_sec, ts.tv_nsec);
	return (ts.tv_sec * one_E9 + ts.tv_nsec);
#else

	timespec ts = { 0, 0 };
	int ret = clock_gettime(CLOCK_REALTIME, &ts);
	if (ret) {
		RTT::log(RTT::Error) << "Could not query CLOCK_REALTIME ("
				<< CLOCK_REALTIME << "): " << errno << RTT::endlog();
		return rtt_clock::rtt_now();
	}
	return (ts.tv_sec * one_E9 + ts.tv_nsec);
	//  return ros::Time::now();
#endif
}

//const ros::Time rtt_clock::host_wall_now()
//{
//	const uint64_t one_E9 = 1000000000ll;
//  #ifdef __XENO__
//    // Use Xenomai 2.6 feature to get the NTP-synched real-time clock
//    timespec ts = {0,0};
//    int ret = clock_gettime(CLOCK_HOST_REALTIME, &ts);
//    if(ret) {
//      RTT::log(RTT::Error) << "Could not query CLOCK_HOST_REALTIME (" << CLOCK_HOST_REALTIME <<"): "<< errno << RTT::endlog();
//      return rtt_clock::rtt_wall_now();
//    }
//
//    return ros::Time(ts.tv_sec, ts.tv_nsec);
//  #else
//    ros::WallTime now(ros::WallTime::now());
//    return ros::Time(now.sec, now.nsec);
//  #endif
//}

const uint64_t rtt_clock::rtt_now() {
	// count the zeros...   -987654321--
	const uint64_t one_E9 = 1000000000ULL;
	// NOTE: getNSecs returns wall time, getTicks returns offset time
#ifdef __XENO__
	uint64_t nsec64;
	if(SimClockThread::GetInstance() && SimClockThread::GetInstance()->simTimeEnabled())
	nsec64 = RTT::os::TimeService::ticks2nsecs(RTT::os::TimeService::Instance()->getTicks());
	else
	nsec64 = RTT::os::TimeService::Instance()->getNSecs();
#else
	uint64_t nsec64 = RTT::os::TimeService::ticks2nsecs(
			RTT::os::TimeService::Instance()->getTicks()); //RTT::os::TimeService::Instance()->getNSecs();
#endif
//  uint32_t sec32_part = nsec64 / one_E9;
//  uint32_t nsec32_part = nsec64 % one_E9;
	//RTT::log(RTT::Error) << "sec: " << sec32_part << " nsec: " << nsec32_part << RTT::endlog();
	return nsec64; //ros::Time(sec32_part, nsec32_part);
}

//const ros::Time rtt_clock::rtt_wall_now()
//{
//  const uint64_t one_E9 = 1000000000ll;
//  // NOTE: getNSecs returns wall time, getTicks returns offset time
//  uint64_t nsec64 = RTT::os::TimeService::Instance()->getNSecs();
//  uint64_t sec64_part = nsec64 / one_E9;
//  uint64_t nsec64_part = nsec64 % one_E9;
//  //RTT::log(RTT::Error) << "sec: " << sec32 << " nsec: " << nsec32 << RTT::endlog();
//  return ros::Time(sec64_part, nsec64_part);
//}

//const RTT::Seconds rtt_clock::host_offset_from_rtt()
//{
//  return (rtt_clock::host_wall_now() - rtt_clock::rtt_wall_now()).toSec();
//}

void rtt_clock::use_ros_clock_topic() {
	SimClockThread::Instance()->useROSClockTopic();
}

void rtt_clock::use_manual_clock() {
	SimClockThread::Instance()->useManualClock();
}

const bool rtt_clock::set_sim_clock_activity(RTT::TaskContext *t) {
	if (!t)
		return false;
	return t->setActivity(new SimClockActivity(t->getPeriod()));
}

const bool rtt_clock::enable_sim() {
	return SimClockThread::Instance()->start();
}

const bool rtt_clock::disable_sim() {
	return SimClockThread::Instance()->stop();
}

void rtt_clock::update_sim_clock(const uint64_t new_time) {
	SimClockThread::Instance()->updateClock(new_time);
}

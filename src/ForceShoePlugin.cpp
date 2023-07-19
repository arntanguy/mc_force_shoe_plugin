#include "ForceShoePlugin.h"

#include <mc_control/GlobalPluginMacros.h>
#include <bits/stdc++.h>
#include <chrono>


namespace mc_plugin
{

ForceShoePlugin::~ForceShoePlugin() = default;

void ForceShoePlugin::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
  auto & ctl = controller.controller();
  mc_rtc::log::info("ForceShoePlugin::init called with configuration:\n{}", config.dump(true, true));
  // loading config port and baudrate
  auto comPort = config("comPort", std::string{"/dev/ttyUSB0"});
  auto baudrate = config("baudrate", 921600);
  cmt3_.reset(new xsens::Cmt3);
  doHardwareConnect(baudrate, comPort);
  doMtSettings();
  UnloadedFS();

  ctl.datastore().make<sva::ForceVecd>("ForceShoePlugin::LFForce", sva::ForceVecd::Zero());
  ctl.datastore().make<sva::ForceVecd>("ForceShoePlugin::LBForce", sva::ForceVecd::Zero());
  ctl.datastore().make<sva::ForceVecd>("ForceShoePlugin::RFForce", sva::ForceVecd::Zero());
  ctl.datastore().make<sva::ForceVecd>("ForceShoePlugin::RBForce", sva::ForceVecd::Zero());
  
  ctl.datastore().make_call("ForceShoePlugin::GetLFForce", [&ctl, this]() { return ctl.datastore().get<sva::ForceVecd>("ForceShoePlugin::LFForce"); });  
  ctl.datastore().make_call("ForceShoePlugin::GetLBForce", [&ctl, this]() { return ctl.datastore().get<sva::ForceVecd>("ForceShoePlugin::LBForce"); });  
  ctl.datastore().make_call("ForceShoePlugin::GetRFForce", [&ctl, this]() { return ctl.datastore().get<sva::ForceVecd>("ForceShoePlugin::RFForce"); });  
  ctl.datastore().make_call("ForceShoePlugin::GetRBForce", [&ctl, this]() { return ctl.datastore().get<sva::ForceVecd>("ForceShoePlugin::RBForce"); });  
}

void ForceShoePlugin::reset(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("ForceShoePlugin::reset called");
}

void ForceShoePlugin::before(mc_control::MCGlobalController & controller) 
{
  auto & ctl = controller.controller();
  auto & LF = ctl.datastore().get<sva::ForceVecd>("ForceShoePlugin::LFForce");
  auto & LB = ctl.datastore().get<sva::ForceVecd>("ForceShoePlugin::LBForce");
  auto & RF = ctl.datastore().get<sva::ForceVecd>("ForceShoePlugin::RFForce");
  auto & RB = ctl.datastore().get<sva::ForceVecd>("ForceShoePlugin::RBForce");

	Packet* packet = new Packet((unsigned short)mtCount, cmt3_->isXm());

  // Time check: has to be multi threaded
  // auto start = std::chrono::high_resolution_clock::now();
  // // unsync the I/O of C and C++.
  // std::ios_base::sync_with_stdio(false);

  cmt3_->waitForDataMessage(packet);

  // auto end = std::chrono::high_resolution_clock::now();
  // double time_taken = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
  // time_taken *= 1e-9;
  // std::cout << "Time taken by waiting data is : " << std::fixed << time_taken << std::setprecision(9);
  // std::cout << " sec" << std::endl;


	//get sample count
	sdata = packet->getSampleCounter();

  for (int i = 0; i < 8; i++)
  {
    LBraw[i] = shortToVolts(packet->m_msg.getDataShort(packet->getInfoList(0).m_calAcc + 1 * CALIB_DATA_OFFSET + 0 * RAWFORCE_OFFSET + 2 * i));
    LFraw[i] = shortToVolts(packet->m_msg.getDataShort(packet->getInfoList(0).m_calAcc + 2 * CALIB_DATA_OFFSET + 1 * RAWFORCE_OFFSET + 2 * i));
    RBraw[i] = shortToVolts(packet->m_msg.getDataShort(packet->getInfoList(0).m_calAcc + 3 * CALIB_DATA_OFFSET + 2 * RAWFORCE_OFFSET + 2 * i));
    RFraw[i] = shortToVolts(packet->m_msg.getDataShort(packet->getInfoList(0).m_calAcc + 4 * CALIB_DATA_OFFSET + 3 * RAWFORCE_OFFSET + 2 * i));
  }
  computeAmpCalMat();		
	computeUDiff();
	computeForceVec();

  LB = sva::ForceVecd(Eigen::Vector3d{LBforcevec[3], LBforcevec[4], LBforcevec[5]}, Eigen::Vector3d{LBforcevec[0], LBforcevec[1], LBforcevec[2]});
  LF = sva::ForceVecd(Eigen::Vector3d{LFforcevec[3], LFforcevec[4], LFforcevec[5]}, Eigen::Vector3d{LFforcevec[0], LFforcevec[1], LFforcevec[2]});
  RB = sva::ForceVecd(Eigen::Vector3d{RBforcevec[3], RBforcevec[4], RBforcevec[5]}, Eigen::Vector3d{RBforcevec[0], RBforcevec[1], RBforcevec[2]});
  RF = sva::ForceVecd(Eigen::Vector3d{RFforcevec[3], RFforcevec[4], RFforcevec[5]}, Eigen::Vector3d{RFforcevec[0], RFforcevec[1], RFforcevec[2]});
  

  delete packet;

}

void ForceShoePlugin::after(mc_control::MCGlobalController & controller)  
{
  mc_rtc::log::info("ForceShoePlugin::after");
}

mc_control::GlobalPlugin::GlobalPluginConfiguration ForceShoePlugin::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = false;
  out.should_always_run = true;
  return out;
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("ForceShoePlugin", mc_plugin::ForceShoePlugin)

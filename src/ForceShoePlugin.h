/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/GlobalPlugin.h>
#include <SpaceVecAlg/SpaceVecAlg>
#include <cstring>

#include "cmt3.h"
#include "cmtscan.h"
#include "xsens_list.h"
using namespace xsens;

#define ampGain 4.7
#define CALIB_DATA_OFFSET 3 * 12 // 3*12 bytes
#define RAWFORCE_OFFSET 16 // 16 bytes

// this macro tests for an error and exits the program with a message if there was one
#define EXIT_ON_ERROR(res, comment)                                                                    \
  if(res != XRV_OK)                                                                                    \
  {                                                                                                    \
    mc_rtc::log::error_and_throw("Error {} occurred in " comment ": {}\n", res, xsensResultText(res)); \
    exit(1);                                                                                           \
  }

namespace mc_plugin
{

struct ForceShoePlugin : public mc_control::GlobalPlugin
{
  void init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config) override;

  void reset(mc_control::MCGlobalController & controller) override;

  void before(mc_control::MCGlobalController &) override;

  void after(mc_control::MCGlobalController & controller) override;

  mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override;

  ~ForceShoePlugin() override;

  //////////////////////////////////////////////////////////////////////////
  // doHardwareConnect
  //
  // Connects to the desired port at the desired baudrate and checks for
  void doHardwareConnect(uint32_t baudrate, std::string portName)
  {
    XsensResultValue res;
    List<CmtPortInfo> portInfo;

    xsens::cmtScanPorts(portInfo);

    CmtPortInfo current = {0, 0, 0, ""};
    current.m_baudrate = numericToRate(baudrate);
    sprintf(current.m_portName, portName.c_str());

    mc_rtc::log::info("Using COM port {} at {} baud", current.m_portName, current.m_baudrate);

    mc_rtc::log::info("Opening port...");

    // open the port which the device is connected to and connect at the device's baudrate.
    res = cmt3_->openPort(current.m_portName, current.m_baudrate);
    EXIT_ON_ERROR(res, "cmtOpenPort");

    mc_rtc::log::info("done");

    // set the measurement timeout to 100ms (default is 16ms)
    int timeOut = 100;
    res = cmt3_->setTimeoutMeasurement(timeOut);
    EXIT_ON_ERROR(res, "set measurement timeout");
    mc_rtc::log::info("Measurement timeout set to {} ms", timeOut);

    // get the Mt sensor count.
    mtCount = cmt3_->getMtCount();
    mtCount = mtCount;
    mc_rtc::log::info("MotionTracker count: {}", mtCount);

    // retrieve the device IDs
    mc_rtc::log::info("Retrieving MotionTrackers device ID(s)");
    for(unsigned int j = 0; j < mtCount; j++)
    {
      res = cmt3_->getDeviceId((unsigned char)(j + 1), deviceIds_[j]);
      EXIT_ON_ERROR(res, "getDeviceId");
      // long deviceIdVal = (long)deviceIds_[j];
      // mc_rtc::log::info("Device ID at busId {}: {}",j+1, deviceIdVal);
      // Done using a printf because device id is an unsigned int32 and mc rtc log does not seem to convert correctly
      printf("Device ID at busId %i: %08lx\n", j + 1, (long)deviceIds_[j]);
    }
  }

  //////////////////////////////////////////////////////////////////////////
  // doMTSettings
  //
  // Set user settings in MTi/MTx
  // Assumes initialized cmt3 class with open COM port
  void doMtSettings()
  {
    XsensResultValue res;

    // set sensor to config sate
    res = cmt3_->gotoConfig();
    EXIT_ON_ERROR(res, "gotoConfig");

    unsigned short sampleFreq;
    sampleFreq = cmt3_->getSampleFrequency();

    // set the device output mode for the device(s)
    CmtDeviceMode deviceMode(mode, settings, sampleFreq);
    for(unsigned int i = 0; i < mtCount; i++)
    {
      res = cmt3_->setDeviceMode(deviceMode, true, deviceIds_[i]);
      EXIT_ON_ERROR(res, "setDeviceMode");
    }

    // start receiving data
    res = cmt3_->gotoMeasurement();
    EXIT_ON_ERROR(res, "gotoMeasurement");
  }

  //////////////////////////////////////////////////////////////////////////////////////////
  // Convert the short raw value to the voltage in float.
  double shortToVolts(const uint16_t raw)
  {
    double U = double(raw);
    U *= 4.999924 / 65535;
    return U;
  }

  //////////////////////////////////////////////////////////////////////////////////////////
  // Convert a numeric baudrate in bps to correct hardware values
  uint32_t numericToRate(int numeric)
  {
    switch(numeric)
    {
      case 9600:
        return B9600;
      case 19200:
        return B19200;
      case 38400:
        return B38400;
      case 57600:
        return B57600;
      case 115200:
        return B115200;
      case 230400:
        return B230400;
      case 460800:
        return B460800;
      case 921600:
        return B921600;
      default:
        return 0;
    }
  }

  //////////////////////////////////////////////////////////////////////////////////////////
  // compute amplified calibration matrixes (raw/amplifier gain/excitation)
  void computeAmpCalMat()
  {
    for(int i = 0; i < 6; i++)
    {
      for(int j = 0; j < 6; j++)
      {
        ampCalMatLB[i][j] = rawCalMatLB[i][j] / ampGain / LBraw[6];
        ampCalMatLF[i][j] = rawCalMatLF[i][j] / ampGain / LFraw[6];
       
      }
    }
  }

  //////////////////////////////////////////////////////////////////////////////////////////
  // compute difference between current voltages and unloaded voltages
  void computeUDiff()
  {
    for(int i = 0; i < 6; i++)
    {
      LBdiff[i] = LBraw[i] - LBUnload[i];
      LFdiff[i] = LFraw[i] - LFUnload[i];
      
    }
  }

  //////////////////////////////////////////////////////////////////////////////////////////
  // process FT vectors from voltages (amplified calibration matrixes * diff voltage vectors)
  void computeForceVec()
  {
    // Reseting values
    for(int i = 0; i < 6; i++)
    {
      LBforcevec[i] = 0.;
      LFforcevec[i] = 0.;
    
    }

    for(int i = 0; i < 6; i++) // i: rows of cal mat
    {
      for(int j = 0; j < 6; j++) // j: column of voltage vect
      {
        LBforcevec[i] += ampCalMatLB[i][j] * LBdiff[j];
        LFforcevec[i] += ampCalMatLF[i][j] * LFdiff[j];
       
      }
    }
  }

  //////////////////////////////////////////////////////////////////////////////////////////
  // Thread to wait for message while not blocking controller
  void dataThread()
  {
    Mode prevMode = mode_;
    Eigen::Vector6d LBCalib, LFCalib = Eigen::Vector6d::Zero();
    while(th_running_)
    {
      auto res = cmt3_->waitForDataMessage(packet_.get());
      if(res != XRV_OK)
      {
        // FIXME Display a warning on read error?
        continue;
      }
      std::lock_guard<std::mutex> lock(mutex_);
      sdata_ = packet_->getSampleCounter();
      const auto & msg = packet_->m_msg;
      const auto & infoList = packet_->getInfoList(0);
      const auto & calAcc = infoList.m_calAcc;

      for(int i = 0; i < 8; i++)
      {

        LBraw[i] = shortToVolts(msg.getDataShort(calAcc + 1 * CALIB_DATA_OFFSET + 0 * RAWFORCE_OFFSET + 2 * i));
        LFraw[i] = shortToVolts(msg.getDataShort(calAcc + 2 * CALIB_DATA_OFFSET + 1 * RAWFORCE_OFFSET + 2 * i));
      }
      if(mode_ == Mode::Calibrate)
      {
        if(prevMode != mode_)
        {
          calibSamples_ = 0;
          LBCalib.setZero();
          LFCalib.setZero();         
        }
        for(int i = 0; i < 6; ++i)
        {
          LBCalib[i] += LBraw[i];
          LFCalib[i] += LFraw[i];
         
        }
        calibSamples_ += 1;
        if(calibSamples_ == 100)
        {
          mode_ = Mode::Acquire;
          LBUnload = LBCalib / 100;
          LFUnload = LFCalib / 100;
          auto calib = mc_rtc::ConfigurationFile(calibFile_);
          calib.add("LBUnload", LBUnload);
          calib.add("LFUnload", LFUnload);
          
          calib.save();
        }
      }
      prevMode = mode_;
    }
  }

private:
  bool liveMode_ = true; // by default, live reading
  std::string calibFile_ = "/tmp/force-shoe-calib.yaml";

  enum class Mode
  {
    Calibrate,
    Acquire
  };
  Mode mode_ = Mode::Calibrate;
  size_t calibSamples_ = 0;

  bool th_running_ = true;
  std::thread th_;
  std::mutex mutex_;
  std::shared_ptr<Packet> packet_;

  CmtOutputMode mode = CMT_OUTPUTMODE_CALIB | CMT_OUTPUTMODE_AUXILIARY;
  CmtOutputSettings settings = CMT_OUTPUTSETTINGS_AUXILIARYMODE_FORCE | CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT;
  unsigned long mtCount = 0;
  CmtDeviceId deviceIds_[256];
  std::shared_ptr<xsens::Cmt3> cmt3_;

  // sample counter
  unsigned short sdata_ = NULL;

  // Unloaded voltage measure of force shoes
  Eigen::Vector6d LFUnload, LBUnload = Eigen::Vector6d::Zero();

  // Difference between measured voltage and unloaded
  double LFdiff[6], LBdiff[6] = {0., 0., 0., 0., 0., 0.};

  // Raw data vectors (G0, G1, G2, G3, G4, G5, G6, ref)
  double LFraw[8], LBraw[8] = {0., 0., 0., 0., 0., 0., 0., 0.};

  // Processed F/T vector after calculations: result of ampCalMat[6][6]*diffVoltages[6]
  double LFforcevec[6], LBforcevec[6] = {0., 0., 0., 0., 0., 0.};

  // Amplified calibration matrixes: result of rawCalMat/amplifierGain/Excitation voltage
  double ampCalMatLF[6][6], ampCalMatLB[6][6];

  //This is the raw calibration matrix for the force sensor FT15249 (manip panda brace)
  double rawCalMatLF[6][6]={
       505.108, 87.11956, 1367.218428, -21930.738, -1652.28, 22496.79,
       -1108.37, 25287.9, 1064.192374, -12625.541, 623.9873, -12993.5,
       33947.17, 225.6711, 32492.42157, 231.449297, 33043.63, 163.8559,
       -6.08744, 174.8519, -530.014146, -91.41136, 542.79255, -85.3106,
       619.5521, 4.166905, -321.1294882, -149.553885, -303.852, -159.077,
       14.65041, -321.666, 18.89079645, -313.54886, 27.32414, -328.215};

  
  // This is the raw calibration matrix for the force sensor FT15248 (Left Back)
  double rawCalMatLB[6][6] = {
      41.8854593998548,  302.05858296048,   3529.08844076704,  -22746.922941752,  -1098.45056289325, 21839.4537665138,
      -3722.80476900553, 25806.0626270618,  1540.24941732271,  -13006.0497495542, 772.665809221095,  -12784.1210566955,
      31952.3390587929,  1517.32482708088,  33431.7185054805,  2190.93437446714,  32274.3603948901,  2006.34561142157,
      -23.1682511656024, 176.343773270153,  -519.706961556471, -123.266482600202, 526.218237676654,  -54.6080313458558,
      599.468254179071,  24.9602652720546,  -332.302205700307, 134.866380077964,  -319.584696900081, -169.351637118394,
      50.769384378332,   -318.022329213099, 45.2243488485171,  -327.60507049336,  14.3896157814584,  -322.550148070767};
  
};

} // namespace mc_plugin

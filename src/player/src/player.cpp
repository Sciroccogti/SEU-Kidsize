#include "dev/mcu.hpp"
#include <common/BodyAngles.h>
#include <common/HeadAngles.h>
#include <common/GetAngles.h>
#include <common/ImuData.h>
#include <common/common.hpp>
#include <seumath/math.hpp>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>
#include <bits/stdc++.h>
#include <ros/ros.h>

using namespace std;
using namespace common;
using namespace seumath;
using namespace Eigen;

void NormalizeIMU(ImuData &idata);
bool ResetImuService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

const int DXL_ZERO_POS = 2048;
const int DXL_MAX_POS = 4096;
const int DXL_MIN_POS = 0;

std::shared_ptr<Mcu> mcu;
float imu_init_yaw = 0.0;
std::atomic_bool imuReset(true);

bool dxl_connected = false;
uint32_t start_clock = 0;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "player");
  ros::NodeHandle node;
  ros::service::waitForService("/getangles");
  mcu = std::make_shared<Mcu>();
  mcu->open();
  start_clock = get_clock();

  ros::Publisher imuPublisher = node.advertise<ImuData>("/sensor/imu", 1);
  ros::Publisher headPublisher = node.advertise<HeadAngles>("/sensor/head", 1);
  ros::ServiceServer imuRstService = node.advertiseService("/imurst", ResetImuService);
  Mcu::McuPacket mcuPkt;
  ImuData imudata;

  uint32_t req_cnt = 0;
  uint8_t led_status = 0;
  while (ros::ok())
  {
    if (mcu->isOpen())
    {
      mcuPkt.type = 0;
      mcuPkt.len = 0;
      bool ok = mcu->readPacket(mcuPkt);
      if (mcuPkt.type == Mcu::REQ_DATA)
      {
        req_cnt++;
        imudata.stamp = get_clock();
        imudata.pitch = ((float)(int16_t)(mcuPkt.data[Mcu::REQ_DATA_IMU_OFFSET] +
                        (mcuPkt.data[Mcu::REQ_DATA_IMU_OFFSET + 1] << 8))) / 100.0;
        imudata.roll = ((float)(int16_t)(mcuPkt.data[Mcu::REQ_DATA_IMU_OFFSET + 2] +
                        (mcuPkt.data[Mcu::REQ_DATA_IMU_OFFSET + 3] << 8))) / 100.0;
        imudata.yaw = ((float)(int16_t)(mcuPkt.data[Mcu::REQ_DATA_IMU_OFFSET + 4] +
                        (mcuPkt.data[Mcu::REQ_DATA_IMU_OFFSET + 5] << 8))) / 10.0;
        NormalizeIMU(imudata);
        imuPublisher.publish(imudata);

        if (req_cnt % 50 == 0)
        {
          Mcu::McuPacket pkt;
          pkt.type = Mcu::LED_DATA;
          pkt.len = 2;
          pkt.data[0] = led_status;
          pkt.data[1] = led_status;
          led_status = 1 - led_status;
          mcu->writePacket(pkt);
        }
        if (mcuPkt.data[0] == 1)
        {
          if (!dxl_connected)
          {
            imu_init_yaw = imudata.yaw;
            dxl_connected = true;
          }
          GetAngles getsrv;
          getsrv.request.player = "real";
          ros::service::call("/getangles", getsrv);
          Mcu::McuPacket pkt;
          int i = 0;
          pkt.type = Mcu::MOTOR_DATA;
          pkt.len = getsrv.response.degs.size() * 5;
          for (int j=0; j<getsrv.response.degs.size(); j++)
          {
            uint32_t gpos = static_cast<uint32_t>(getsrv.response.degs[j] / 
                  (360.0f / (float)(DXL_MAX_POS - DXL_MIN_POS)) + DXL_ZERO_POS);
            pkt.data[i * 5 + 0] = static_cast<uint8_t>(j+getsrv.response.start_id);
            pkt.data[i * 5 + 1] = SEU_LOBYTE(SEU_LOWORD(gpos));
            pkt.data[i * 5 + 2] = SEU_HIBYTE(SEU_LOWORD(gpos));
            pkt.data[i * 5 + 3] = SEU_LOBYTE(SEU_HIWORD(gpos));
            pkt.data[i * 5 + 4] = SEU_HIBYTE(SEU_HIWORD(gpos));
            i++;
          }
          mcu->writePacket(pkt);
        }
        else
        {
          dxl_connected = false;
        }
      }
    }
    ros::spinOnce();
    // loop_rate.sleep();
  }
  mcu->close();
  return 0;
}

void NormalizeIMU(ImuData &idata)
{
  if(imuReset)
  {
    imuReset = false;
    imu_init_yaw = idata.yaw;
  }
  idata.yaw = normalizeDeg<float>(idata.yaw - imu_init_yaw);
  if (idata.pitch < -30.0)
    idata.fall = ImuData::FALL_BACKWARD;
  else if (idata.pitch > 40.0)
    idata.fall = ImuData::FALL_FORWARD;
  else if (idata.roll < -40.0)
    idata.fall = ImuData::FALL_RIGHT;
  else if (idata.roll > 40.0)
    idata.fall = ImuData::FALL_LEFT;
  else
    idata.fall = ImuData::FALL_NONE;
}

bool ResetImuService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  imuReset = true;
}
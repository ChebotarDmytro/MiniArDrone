#include <boost/shared_ptr.hpp>
#include <boost/atomic.hpp>
#include <ros/ros.h>
#include <stdexcept>


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>


#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>


extern "C"
{
#include "libARCommands/ARCommands.h"
#include "libARDiscovery/ARDiscovery.h"
#include "libARController/ARController.h"
#include "libARSAL/ARSAL.h"
}


class ArdroneDriver
{
private:
  boost::atomic<bool> is_connected_;

  ARDISCOVERY_Device_t* device_ptr_;

  eARCONTROLLER_ERROR error_;

public:
  ArdroneDriver();
 ~ArdroneDriver();

   inline bool IsConnected() const { return is_connected_; }
   void Connect();
   void Disconnect();
   void Cleanup();
};


int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "main");

    ArdroneDriver ar;
    uint16_t id = ARDISCOVERY_getProductID(ARDISCOVERY_PRODUCT_MINIDRONE);
    const char* name = ARDISCOVERY_getProductName(ARDISCOVERY_PRODUCT_MINIDRONE);
    ROS_INFO("ID [%d]", id);
    ROS_INFO("Name [%s]", name);
    ros::spin();


}

ArdroneDriver::ArdroneDriver()
  : device_ptr_(NULL),
    is_connected_(false)
{
   // ROS_INFO("HHHHH");
  Connect();
}

ArdroneDriver::~ArdroneDriver()
{
   if (device_ptr_) ARDISCOVERY_Device_Delete(&device_ptr_);
}

void ArdroneDriver::Connect()
{

  try
  {
    if(is_connected_) throw std::runtime_error("Already inited");

      // Error checking;
     // ARSAL_Sem_Init(&state_sem_, 0, 0); //Initializes a semaphore.

    eARDISCOVERY_ERROR error_discovery = ARDISCOVERY_OK;
    device_ptr_ = ARDISCOVERY_Device_New(&error_discovery);

    if (error_discovery != ARDISCOVERY_OK)
    {
      throw std::runtime_error("Discovery failed: " + std::string(ARDISCOVERY_Error_ToString(error_discovery)));
    }

       // Initialize the Discovery Device with a BLE device.
        error_discovery = ARDISCOVERY_Device_InitBLE (device_ptr_, ARDISCOVERY_PRODUCT_MINIDRONE,
                                                  (ARNETWORKAL_BLEDeviceManager_t)ARNETWORKAL_MANAGER_BLE_ID_MAX,
                                                  (ARNETWORKAL_BLEDevice_t)ARNETWORKAL_MANAGER_BLE_ID_MAX);
        ROS_INFO("InitBLE");

        if (error_discovery != ARDISCOVERY_OK)
        {
          throw std::runtime_error("Discovery failed: " + std::string(ARDISCOVERY_Error_ToString(error_discovery)));
        }

        ARDISCOVERY_Device_Delete(&device_ptr_);


        // This semaphore is touched inside the StateCallback
       // ARSAL_Sem_Wait(&state_sem_);


  }
  catch(const std::runtime_error& e)
  {
    Cleanup();
     throw e;
  }

   is_connected_ = true;
}


void ArdroneDriver::Cleanup()
{
  ARSAL_PRINT(ARSAL_PRINT_INFO, "MiniArDroneSDK", "MiniArDrone Cleanup()");
  is_connected_ = false;
}


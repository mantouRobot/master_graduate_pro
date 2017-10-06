/*******************************************************************************
* Description:
*    This example demonstrates how to use UpDown Counter function.
*
* Instructions for Running:
*    1  Set the 'deviceDescription' for opening the device.
*    2  Set the 'channel' as the channel for UpDown Counter.
*
* I/O Connections Overview:
*    Please refer to your hardware reference manual.
*
******************************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

#include "daq/compatibility.h"
#include "daq/bdaqctrl.h"
using namespace Automation::BDaq;
//-----------------------------------------------------------------------------------
// Configure the following three parameters before running the demo
//-----------------------------------------------------------------------------------
#define     deviceDescription L"PCI-1784,BID#0"
int32       channel = 0;
int32       startPort = 0;
int32       portCount = 1;

inline void waitAnyKey()
{
  do{SLEEP(1);} while(!kbhit());
}

int main(int argc, char* argv[])
{
  // 输出5V作为编码器电源
  ErrorCode ret = Success;
  // Step 1: Create a instantDoCtrl for DO function.
  InstantDoCtrl * instantDoCtrl = AdxInstantDoCtrlCreate();
  // Step 2: Select a device by device number or device description and specify the access mode.
  // in this example we use AccessWriteWithReset(default) mode so that we can
  // fully control the device, including configuring, sampling, etc.
  DeviceInformation devInfo(deviceDescription);
  ret = instantDoCtrl->setSelectedDevice(devInfo);
//  CHK_RESULT(ret);

  // Step 3: Write DO ports
  uint8  bufferForWriting[1] = {0xff};//the first element is used for start port
  ret = instantDoCtrl->Write(startPort,portCount,bufferForWriting );
//  CHK_RESULT(ret);
  std::cout << "DO output complete." << std::endl;

  // If something wrong in this execution, print the error code on screen for tracking.
  if(BioFailed(ret))
  {
    printf(" Some error occurred. And the last error code is Ox%X.\n", ret);
  }

// 读取编码器
  // Step 1: Create a 'UdCounterCtrl' for UpDown Counter function.
  UdCounterCtrl* udCounterCtrl = AdxUdCounterCtrlCreate();
  UdCounterCtrl* udCounterCtrl1 = AdxUdCounterCtrlCreate();

  do
  {
    // Step 2: Select a device by device number or device description and specify the access mode.
    // in this example we use AccessWriteWithReset(default) mode so that we can
    // fully control the device, including configuring, sampling, etc.
    ret = udCounterCtrl->setSelectedDevice(devInfo);
    ret = udCounterCtrl1->setSelectedDevice(devInfo);

    CHK_RESULT(ret);

    // Step 3: Set necessary parameters for counter operation,
    // Note: some of operation of this step is optional(you can do these settings via "Device Configuration" dialog).
    ret = udCounterCtrl->setChannel(channel);
    ret = udCounterCtrl1->setChannel(1);

    CHK_RESULT(ret);

    // Step 4: Set counting type for UpDown Counter
    ret = udCounterCtrl->setCountingType(AbPhaseX4);
    ret = udCounterCtrl1->setCountingType(AbPhaseX4);

    CHK_RESULT(ret);

    // Step 5: Start UpDown Counter
    ret= udCounterCtrl->setEnabled(true);
    ret= udCounterCtrl1->setEnabled(true);

    CHK_RESULT(ret);

    // Step 6: Read counting value: connect the input signal to channels you selected to get UpDown counter value.
    printf("UpDown counter is in progress...\nconnect the input signal to ");
    printf("any key will stop UpDown counter!\n\n");
    while ( !kbhit())
    {
      SLEEP(1);//get event UpDown count value per second
      printf("\n channel %u Current UpDown count: %d\n", channel,udCounterCtrl->getValue());
      printf("channel 1 Current UpDown count: %d\n", udCounterCtrl1->getValue());

    }

    // Step 7: stop UpDown Counter
    udCounterCtrl->setEnabled(false);
    udCounterCtrl1->setEnabled(false);


  }while(false);

  // Step 8: Close device and release any allocated resource.
  udCounterCtrl->Dispose();
  udCounterCtrl1->Dispose();

  // If something wrong in this execution, print the error code on screen for tracking.
  if(BioFailed(ret))
  {
    printf("\nSome error occurred. And the last error code is 0x%X.\n", ret);
    waitAnyKey();
  }


  return 0;
}

/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file usb_can_client.cpp
 * @brief the encapsulate call the api of usb can card according to can_client.h
 *interface
 **/

#include "modules/canbus/can_client/usb/usb_can_client.h"
 #include <stdio.h> 
namespace apollo {
namespace canbus {
namespace can {

using apollo::common::ErrorCode;

bool usbCanClient::Init(const CANCardParameter &parameter) {
  if (!parameter.has_channel_id()) {
    AERROR << "Init CAN failed: parameter does not have channel id. The "
              "parameter is "
           << parameter.DebugString();
    return false;
  }
  port_ = parameter.channel_id();
  return true;
}

ErrorCode usbCanClient::Start() {
  VCI_BOARD_INFO pInfo;//用来获取设备信息。
int count=0;//数据列表中，用来存储列表序号。
VCI_BOARD_INFO pInfo1 [50];
  if (is_started_) {
    return ErrorCode::OK;
  }
  int num=VCI_FindUsbDevice2(pInfo1);
	printf(">>USBCAN DEVICE NUM:");printf("%d", num);printf(" PCS");printf("\n");
  AINFO<<">>USBCAN DEVICE NUM:"<<num;
  int ret=VCI_OpenDevice(VCI_USBCAN2,0,0);
  if(ret!=1)//打开设备
	{
    AERROR<<"open usb_can error";
    printf("open usb_can error %d",ret);
    VCI_CloseDevice(VCI_USBCAN2,0);//关闭设备
		return ErrorCode::CAN_CLIENT_ERROR_BASE;
  }
  printf("open ok\n");
  VCI_INIT_CONFIG config;
	config.AccCode=0;
	config.AccMask=0xFFFFFFFF;
	config.Filter=1;//接收所有帧
	config.Timing0=0x00;/*波特率100 Kbps  0x04  0x1C*/
	config.Timing1=0x1C;
	config.Mode=0;//正常模式	
  printf("config ok\n");
  ret=VCI_InitCAN(VCI_USBCAN2,0,0,&config);
  if(ret!=1)
	{
		AERROR<<"Init CAN1 error";
    printf("Init CAN1 error %d",ret);
		VCI_CloseDevice(VCI_USBCAN2,0);
    return ErrorCode::CAN_CLIENT_ERROR_BASE;
	}
  printf("init can1 ok\n");
  ret=VCI_StartCAN(VCI_USBCAN2,0,0);
  printf("start can1 ok %d\n",ret);
	if(ret!=1)
	{
		AERROR<<"Start CAN1 error";
    printf("Start CAN1 error %d",ret);
		VCI_CloseDevice(VCI_USBCAN2,0);
    return ErrorCode::CAN_CLIENT_ERROR_BASE;

	}
  is_started_ = true;
  return ErrorCode::OK;
}

void usbCanClient::Stop() {
  if (is_started_) {
    is_started_ = false;
    VCI_CloseDevice(VCI_USBCAN2,0);//关闭设备
  }
}

// Synchronous transmission of CAN messages
ErrorCode usbCanClient::Send(const std::vector<CanFrame> &frames,
                             int32_t *const frame_num) {
  CHECK_NOTNULL(frame_num);
  CHECK_EQ(frames.size(), static_cast<size_t>(*frame_num));

  if (!is_started_) {
    AERROR << "usb can client has not been initiated! Please init first!";
    return ErrorCode::CAN_CLIENT_ERROR_SEND_FAILED;
  }
  VCI_CAN_OBJ send[1];
  for (size_t i = 0; i < frames.size() && i < MAX_CAN_SEND_FRAME_LEN; ++i) {
    send[0].ID=frames[i].id;
	  send[0].SendType=0;
	  send[0].RemoteFlag=0;
	  send[0].ExternFlag=0;
	  send[0].DataLen=frames[i].len;
    for(int j=0;j<send[0].DataLen;j++) {
				send[0].Data[j]=frames[i].data[j];
			}
  }
  if(VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == -1) {
    AERROR<<"send message failed";
    return ErrorCode::CAN_CLIENT_ERROR_SEND_FAILED;
  }
  return ErrorCode::OK;
}

// buf size must be 8 bytes, every time, we receive only one frame
ErrorCode usbCanClient::Receive(std::vector<CanFrame> *const frames,
                                int32_t *const frame_num) {
  if (!is_started_) {
    AERROR << "usb can client is not init! Please init first!";
    return ErrorCode::CAN_CLIENT_ERROR_RECV_FAILED;
  }

  if (*frame_num > MAX_CAN_RECV_FRAME_LEN || *frame_num < 0) {
    AERROR << "recv can frame num not in range[0, " << MAX_CAN_RECV_FRAME_LEN
           << "], frame_num:" << *frame_num;
    // TODO(Authors): check the difference of returning frame_num/error_code
    return ErrorCode::CAN_CLIENT_ERROR_FRAME_NUM;
  }
  VCI_CAN_OBJ rec[2100];
  int reclen=VCI_Receive(VCI_USBCAN2,0,0,rec,2100,100);
  *frame_num=reclen;
  if(reclen>=0)//调用接收函数，如果有数据，进行数据处理显示。
		{
			for(int j=0;j<reclen;j++)
			{
        CanFrame new_msg;
        new_msg.len=rec[j].DataLen;
        new_msg.id=rec[j].ID;
				for(int i = 0; i < rec[j].DataLen; i++)
				{
					new_msg.data[i]=rec[j].Data[i];
				}
        frames->push_back(new_msg);
			}
		}
  return ErrorCode::OK;
}

std::string usbCanClient::GetErrorString(const int32_t status) {
  return "0";
}

}  // namespace can
}  // namespace canbus
}  // namespace apollo

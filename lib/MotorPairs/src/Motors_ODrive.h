#pragma once

#include "MotorPairBase.h"

#include <ODriveCAN.h>

void onHeartbeat(Heartbeat_msg_t& msg, void* user_data);
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data);

class Motors_ODrive : public MotorPairBase {
public:
    struct ODriveUserData {
        inline ODriveUserData() : received_heartbeat(false), received_feedback(false) {}
        Heartbeat_msg_t             last_heartbeat;
        bool                        received_heartbeat;
        Get_Encoder_Estimates_msg_t last_feedback;
        bool                        received_feedback;
    };
public:
    explicit Motors_ODrive(float stepsPerRevolution);
    virtual void readEncoder() override;
public:
    enum { CAN_BAUDRATE = 250000 }; //!< Default ODrive baudrate is 250000
    enum { O_DRIVE_0_NODE_ID = 0, O_DRIVE_1_NODE_ID = 1 };
protected:
    void onHeartbeat(const Heartbeat_msg_t& msg, ODriveUserData* oDriveUserData);
    void onFeedback(const Get_Encoder_Estimates_msg_t& msg, ODriveUserData* oDriveUserData);
    friend void onHeartbeat(Heartbeat_msg_t& msg, void* user_data);
    friend void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data);
protected:
    // Keep some application-specific user data for every ODrive.
    ODriveUserData _oDrv0_user_data;
    ODriveUserData _oDrv1_user_data;
    Get_Encoder_Estimates_msg_t _feedback0;
    Get_Encoder_Estimates_msg_t _feedback1;
};

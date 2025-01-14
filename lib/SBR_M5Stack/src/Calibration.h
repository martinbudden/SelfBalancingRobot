#pragma once

class AHRS_Base;
class SV_Preferences;

enum calibrate_t {CALIBRATE_JUST_GYRO, CALIBRATE_ACC_AND_GYRO};
void calibrateGyro(AHRS_Base& ahrs, SV_Preferences& preferences, calibrate_t calibrationType);

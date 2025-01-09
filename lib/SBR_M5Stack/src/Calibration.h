#pragma once

class AHRS_Base;
class SBR_Preferences;

enum calibrate_t {CALIBRATE_JUST_GYRO, CALIBRATE_ACC_AND_GYRO};
void calibrateGyro(AHRS_Base& ahrs, SBR_Preferences& preferences, calibrate_t calibrationType);

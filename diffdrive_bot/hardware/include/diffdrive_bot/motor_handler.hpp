#ifndef MIRA__MOTOR_HANDLER_HPP
#define MIRA__MOTOR_HANDLER_HPP

#include <modbus/modbus.h>
#include <unistd.h>
#include <iostream>

const int ROTATION_SPEED_R = 0x0484;
const int FEEDBACK_SPEED_R = 0x00CE;
const int ACCEL_TIME_R = 0x0604;
const int DECEL_TIME_R = 0x0684;
const int TORQ_LT_R = 0x0704;
const int INPUT_CMD_R = 0x007C;
const int ANALOG_INPUT_SELECTION_R = 0x10E2;
const int APPLY_CONFIG_R = 0x018C;
const int OPERATION_INPUT_MODE_R = 0x1040;

class MotorDriverBLVD20KM {
public:
    ~MotorDriverBLVD20KM();

    void initialize(std::string serial_port, int baud, char parity, int data_bit, int stop_bit, int slave_id, int gear_ratio);
    void disconnect();
    void setSpeed(int value);
    double getSpeed();
    void brakeEngage();
    void brakeRelease();

private:
    bool is_initialized = false;
    int gear_ratio_;
    modbus_t *ctx_;

    int bigEndianToInt(uint16_t *data);
    void intToBigEndian(int value, uint16_t *data);
    void writeRegister(int addr, int value);
    int readRegister(int addr);
};

#endif // MIRA__MOTOR_HANDLER_HPP
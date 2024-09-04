#ifndef MIRA__DRIVE_HPP
#define MIRA__DRIVE_HPP

#include <string>
#include "diffdrive_bot/motor_handler.hpp"

class Drive : public MotorDriverBLVD20KM
{
public:
    std::string drive_name;
    std::string serial_port;
    int baud;
    int stop_bit;
    int data_bit;
    char parity;
    int slave_id;
    int gear_ratio;
    
    double joint_velocity;
    double cmd_velocity{0.0};
    double position{0.0};
};


#endif // MIRA__DRIVE_HPP
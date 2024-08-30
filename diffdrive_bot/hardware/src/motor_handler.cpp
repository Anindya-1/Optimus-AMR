#include "diffdrive_bot/motor_handler.hpp"
#include <string>

MotorDriverBLVD20KM::~MotorDriverBLVD20KM() {
    modbus_close(ctx_);
    modbus_free(ctx_);
}

void  MotorDriverBLVD20KM::initialize(std::string serial_port, int baud, char parity, int data_bit, int stop_bit, int slave_id, int gear_ratio){
    gear_ratio_ = gear_ratio;
    if(!is_initialized){

        std::cout << "Entered Initialized function" << std::endl;

        ctx_ = modbus_new_rtu(serial_port.c_str(), baud, parity, data_bit, stop_bit);

        if (ctx_ == NULL) {
            std::cerr << "Unable to allocate libmodbus context" << std::endl;
            exit(1);
        }

        if (modbus_connect(ctx_) == -1) {
            std::cerr << "Connection failed: " << modbus_strerror(errno) << std::endl;
            modbus_free(ctx_);
            exit(1);
        }

        if (modbus_set_slave(ctx_, slave_id) == -1) {
            std::cerr << "Invalid slave ID: " << modbus_strerror(errno) << std::endl;
            modbus_free(ctx_);
            exit(1);
        }

        std::cout << "Exited Initialized function" << std::endl;

        is_initialized = true;

        writeRegister(OPERATION_INPUT_MODE_R, 1);
        writeRegister(APPLY_CONFIG_R, 1);
        writeRegister(INPUT_CMD_R, 2);
        writeRegister(ROTATION_SPEED_R, 0);
        
    }
}

void MotorDriverBLVD20KM::disconnect() {
    is_initialized = false;
    writeRegister(ROTATION_SPEED_R, 0);
}

void MotorDriverBLVD20KM::setSpeed(int value) {
    // value = static_cast<int>(value / (0.00785/(double)gear_ratio_));
    if (value == 0) {
        writeRegister(ROTATION_SPEED_R, 0);
        writeRegister(INPUT_CMD_R, 10);                        
    } else if (value < 0) {
        if (value > -80) value = -80;
        if (value < -4000) value = -4000;
        writeRegister(ROTATION_SPEED_R, abs(value));
        writeRegister(INPUT_CMD_R, 58);                        
    } else {
        if (value < 80) value = 80;
        if (value > 4000) value = 4000;
        writeRegister(ROTATION_SPEED_R, abs(value));
        writeRegister(INPUT_CMD_R, 26);                        
    }
}

double MotorDriverBLVD20KM::getSpeed() {
    return readRegister(FEEDBACK_SPEED_R); // * (0.00785/(double)gear_ratio_);
}

void MotorDriverBLVD20KM::brakeEngage() {
    writeRegister(ROTATION_SPEED_R, 0);
    writeRegister(INPUT_CMD_R, 10);                             //00100000
}

void MotorDriverBLVD20KM::brakeRelease() {
    writeRegister(ROTATION_SPEED_R, 0);
    writeRegister(INPUT_CMD_R, 160);                            //101000007
}

int MotorDriverBLVD20KM::bigEndianToInt(uint16_t *data) {               //converts [upper_hex lower_hex] to int
    uint32_t combined = (data[0] << 16) | data[1];
    if (combined >= 0x80000000) combined -= 0x100000000;
    return combined;
}

void MotorDriverBLVD20KM::intToBigEndian(int value, uint16_t *data) {           // addresses overflow and negative numbers
    if (value < 0) value += 0x100000000;
    data[0] = (value >> 16) & 0xFFFF;
    data[1] = value & 0xFFFF;
}

void MotorDriverBLVD20KM::writeRegister(int addr, int value) {
    uint16_t data[2];
    intToBigEndian(value, data);
    if (modbus_write_registers(ctx_, addr, 2, data) == -1) {
        std::cerr << "Write failed: " << addr << " :" << modbus_strerror(errno) << std::endl;
    }
    sleep(1);
}

int MotorDriverBLVD20KM::readRegister(int addr) {
    uint16_t data[2];
    if (modbus_read_registers(ctx_, addr, 2, data) != 2) {
        std::cerr << "Read failed: " << addr << " :" << modbus_strerror(errno) << std::endl;
        return 0;
    }
    sleep(1);
    return bigEndianToInt(data);
}

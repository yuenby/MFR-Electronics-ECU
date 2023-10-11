#include "Bamocar_can.h"

Bamocar_data::Bamocar_data(uint16_t txID,uint16_t rxID) {
    _txID = txID;
    _rxID = rxID;
}

void Bamocar_data::setTxID(uint16_t txID){
    _txID = txID;
}

void Bamocar_data::setRxID(uint16_t rxID){
    _rxID = rxID;
}


bool Bamocar_data::_sendCAN(Bamocar_dataframe dataframe){
    can_message_t message = can_message_t();
    for(uint8_t i = 0; i < dataframe.length(); i++) {
    message.data[i] = dataframe.get(i);
    }

    message.identifier = _rxID;
    message.data_length_code = dataframe.length();
    if (message.identifier > 0x7FF) message.flags = CAN_MSG_FLAG_EXTD;

    return can_transmit(&message, pdMS_TO_TICKS(100)) == ESP_OK;
}

bool Bamocar_data::_requestData(uint8_t dataAddress, uint8_t interval) {
    return _sendCAN(Bamocar_dataframe(REG_REQUEST, dataAddress, interval));
}

// void Bamocar_data::_listenCAN() {
//     can_message_t message = can_message_t();

//     if (can_receive(&message, pdMS_TO_TICKS(100)) == ESP_OK)
//         parseMessage(message);
// }

bool Bamocar_data::parseMessage(can_message_t &msg) {
    if (msg.identifier == _txID) {
        int64_t receivedData = 0;
        if (msg.data_length_code == 4) {
            receivedData = _getData16Bit(msg);
        } else if (msg.data_length_code == 6) {
            receivedData = _getData32Bit(msg);
        } else return false;

        switch (msg.data[0]) {
            case REG_BATTERY_V:
                _data.BATTERY_V = receivedData;
                break;
            
            case REG_DC_BUS_V:
                _data.DC_BUS_V = receivedData;
                break;
            
            case REG_STATUS:
                _data.STATUS = receivedData;
                break;

            case REG_READY:
                _data.READY = receivedData;
                break;

            case REG_SPEED_ACTUAL:
                _data.SPEED_ACTUAL = receivedData;
                break;

            case REG_SPEED_RPMMAX:
                _data.SPEED_RPMMAX = receivedData;
                break;

            case REG_I_IST:
                _data.I_IST = receivedData;
                break;

            case REG_DEVICE_I:
                _data.DEVICE_I = receivedData;
                break;

            case REG_KERN_I_200PC:
                _data.KERN_I_200PC = receivedData;
                break;

            case REG_TORQUE_SETPOI:
                _data.TORQUE_SETPOI = receivedData;
                break;

            case REG_RAMP_ACC:
                _data.RAMP_ACC = receivedData;
                break;

            case REG_RAMP_DEC:
                _data.RAMP_DEC = receivedData;
                break;

            case REG_T_MOTOR:
                _data.T_MOTOR = receivedData;
                break;

            case REG_T_IGBT:
                _data.T_IGBT = receivedData;
                break;

            case REG_T_AIR:
                _data.T_AIR = receivedData;
                break;

            case REG_HARD_ENABLED:
                _data.HARD_ENABLED = receivedData;
                break;


            default:
                return false;
        }

        return true;
    }

    return false;
}

int16_t const Bamocar_data::_getData16Bit(can_message_t &msg) {
    int16_t returnValue;

    returnValue = msg.data[1];
    returnValue |= (msg.data[2] << 8);

    return returnValue;
}

int32_t const Bamocar_data::_getData32Bit(can_message_t &msg) {
    int16_t returnValue;

    returnValue = msg.data[1];
    returnValue |= (msg.data[2] << 8);
    returnValue |= (msg.data[3] << 16);
    returnValue |= (msg.data[4] << 24);

    return returnValue;
}

//----------------------------------------------------------------------------------------------

// Voltage

uint16_t Bamocar_data::getBattVoltage() {
    return _data.BATTERY_V;
}

bool Bamocar_data::requestBattVoltage(uint8_t interval) {
    bool success = true;
    if (!_requestData(REG_BATTERY_V, interval))
        success = false;

    return success;
}

uint16_t Bamocar_data::getBusVoltage() {
    return (_data.DC_BUS_V/31.58483 - 3);
}

bool Bamocar_data::requestBusVoltage(uint8_t interval) {
    bool success = true;
    if (!_requestData(REG_DC_BUS_V, interval))
        success = false;

    return success;
}

// Speed
float Bamocar_data::getSpeed() {
    return _data.SPEED_RPMMAX * (_data.SPEED_ACTUAL / 32767);
}

bool Bamocar_data::setSpeed(int16_t speed) {
    return _sendCAN(Bamocar_dataframe(REG_SPEED_CMD, speed));
}

bool Bamocar_data::requestSpeed(uint8_t interval) {
    bool success = true;
    if (!_requestData(REG_SPEED_ACTUAL, interval))
        success = false;

    if (!_requestData(REG_SPEED_RPMMAX, interval))
        success = false;

    return success;
}

// Accel
bool Bamocar_data::setAccel(int16_t accel) {
    return _sendCAN(Bamocar_dataframe(REG_RAMP_ACC, (int16_t)accel));
}

// Decel
bool Bamocar_data::setDecel(int16_t decel) {
    return _sendCAN(Bamocar_dataframe(REG_RAMP_DEC, (int16_t)decel));
}

// Torque
float Bamocar_data::getTorque() {
    return _data.TORQUE_SETPOI/0x5555;
}

bool Bamocar_data::setTorque(float torque) {
    if (torque > TORQUE_MAX_PERCENT)
        torque = TORQUE_MAX_PERCENT;

    int16_t torque16 = torque * 0x5555;
    return _sendCAN(Bamocar_dataframe(REG_TORQUE_SETPOI, (int16_t)torque16));
}

bool Bamocar_data::requestTorque(uint8_t interval) {
    return _requestData(REG_TORQUE_SETPOI, interval);
}

// Current
float Bamocar_data::getCurrent() {
    return ((2/10) * _data.DEVICE_I * (_data.I_IST / _data.KERN_I_200PC));
}

bool Bamocar_data::requestCurrent(uint8_t interval) {
    bool success = true;
    if (!_requestData(REG_I_IST, interval))
        success = false;

    if (!_requestData(REG_DEVICE_I, interval))
        success = false;

    if (!_requestData(REG_KERN_I_200PC, interval))
        success = false;

    return success;
}

// Temperatures
uint16_t Bamocar_data::getMotorTemp() {
    return _data.T_MOTOR;
}

bool Bamocar_data::requestMotorTemp(uint8_t interval) {
    return _requestData(REG_T_MOTOR, interval);
}

uint16_t Bamocar_data::getControllerTemp() {
    return _data.T_IGBT;
}

bool Bamocar_data::requestControllerTemp(uint8_t interval) {
    return _requestData(REG_T_IGBT, interval);
}

uint16_t Bamocar_data::getAirTemp() {
    return _data.T_AIR;
}

bool Bamocar_data::requestAirTemp(uint8_t interval) {
    return _requestData(REG_T_AIR, interval);
}

// Status
uint16_t Bamocar_data::getStatus() {
    return _data.STATUS;
}

bool Bamocar_data::requestStatus(uint8_t interval) {
    return _requestData(REG_STATUS, interval);
}

// Enable or disable controller
void Bamocar_data::setSoftEnable(bool enable) {
    uint8_t Bamocar_dataframe2 = 0;

    if (enable) {
        Bamocar_dataframe2 = 0x00;
    } else {
        Bamocar_dataframe2 = 0x04;
    }

    _sendCAN(Bamocar_dataframe(REG_ENABLE, Bamocar_dataframe2, 0x00));
}

bool Bamocar_data::getHardEnable() {
    return _data.HARD_ENABLED;
}

bool Bamocar_data::requestHardEnabled(uint8_t interval) {
    return _requestData(REG_HARD_ENABLED, interval);
}

#include <Arduino.h>
#include <driver/can.h>
#include <driver/gpio.h>
#include "Bamocar_reg.h"

#define CAN_TIMEOUT 0.01 // s (10ms)

#define TORQUE_MAX_PERCENT 1.00 // 100%

class Bamocar_dataframe {
    public:
        Bamocar_dataframe(uint8_t address, uint16_t m_data16) {
            _data[0] = address;
            _data[1] = m_data16 & 0xFF;
            _data[2] = m_data16 >> 8;
            _dlc = 3;
        }

        Bamocar_dataframe(uint8_t address, int16_t m_data16) {
            _data[0] = address;
            _data[1] = m_data16 & 0xFF;
            _data[2] = m_data16 >> 8;
            _dlc = 3;
        }

        Bamocar_dataframe(uint8_t address, uint32_t m_data32) {
            _data[0] = address;
            _data[1] = m_data32 & 0xFF;
            _data[2] = (m_data32 >> 8) & 0xFF;
            _data[3] = (m_data32 >> 16) & 0xFF;
            _data[4] = (m_data32 >> 24) & 0xFF;
            _dlc = 5;
        }

        Bamocar_dataframe(uint8_t address, int32_t m_data32) {
            _data[0] = address;
            _data[1] = m_data32 & 0xFF;
            _data[2] = (m_data32 >> 8) & 0xFF;
            _data[3] = (m_data32 >> 16) & 0xFF;
            _data[4] = (m_data32 >> 24) & 0xFF;
            _dlc = 5;
        }

        Bamocar_dataframe(uint8_t address, uint8_t req_address, uint8_t interval) {
            _data[0] = address;
            _data[1] = req_address;
            _data[2] = interval;
            _dlc = 3;
        }

        uint8_t get(uint8_t pos) {
            if(pos >= _dlc) return 0;
            return _data[pos];
        }

        uint8_t length() {
            return _dlc;
        }

    protected:
        uint8_t _data[5];
        uint8_t _dlc;
};

class Bamocar_data{

    
        public:
        Bamocar_data(uint16_t txID,uint16_t rxID);
        // Bamocar_data(gpio_num_t canRD, gpio_num_t canTD, int frequency = STD_BAUD_RATE)
        //     {

        //         can_general_config_t g_config = CAN_GENERAL_CONFIG_DEFAULT(canRD, canTD, CAN_MODE_NORMAL);
        //         can_timing_config_t t_config;
        //         switch (frequency)
        //         {
        //         case STD_BAUD_RATE:
        //             t_config = CAN_TIMING_CONFIG_100KBITS();
        //             break;
        //          case 250000:
        //             t_config = CAN_TIMING_CONFIG_250KBITS();
        //             break;
        //         case 500000:
        //              t_config = CAN_TIMING_CONFIG_500KBITS();
        //             break;
                
        //         default:
        //             break;
        //         }
        //         can_filter_config_t f_config = CAN_FILTER_CONFIG_ACCEPT_ALL();

        //     //???_can.attach(callback(this, &Bamocar::_listenCAN), CAN::RxIrq);

        //     can_driver_install(&g_config, &t_config, &f_config);

        //     _rxID = STD_RX_ID;
        //     _txID = STD_TX_ID;
        // }
        
        // Voltage
        uint16_t getBattVoltage();
        bool requestBattVoltage(uint8_t interval = INTVL_ONCE);
        uint16_t getBusVoltage();
        bool requestBusVoltage(uint8_t interval = INTVL_ONCE);

        // Speed
        float getSpeed();
        bool setSpeed(int16_t speed);
        bool requestSpeed(uint8_t interval = INTVL_ONCE);

        // Acceleration and Deceleration for Speed control
        bool setAccel(int16_t accel);
        bool setDecel(int16_t decel);

        // Torque
        float getTorque();
        bool setTorque(float torque);
        bool requestTorque(uint8_t interval = INTVL_ONCE);

        // Current (A)
        float getCurrent();
        bool requestCurrent(uint8_t interval = INTVL_ONCE);

        // Temperatures
        uint16_t getMotorTemp();
        uint16_t getControllerTemp();
        uint16_t getAirTemp();
        bool requestMotorTemp(uint8_t interval = INTVL_ONCE);
        bool requestControllerTemp(uint8_t interval = INTVL_ONCE);
        bool requestAirTemp(uint8_t interval = INTVL_ONCE);

        // Status
        uint16_t getStatus();
        bool requestStatus(uint8_t interval = INTVL_ONCE);

        // Enable
        void setSoftEnable(bool enable);
        bool getHardEnable();
        bool requestHardEnabled(uint8_t interval = INTVL_ONCE);

        // CAN IDs (of the Motor Controller)
        void setRxID(uint16_t rxID);
        void setTxID(uint16_t txID);

        bool parseMessage(can_message_t &msg);

    protected:

        uint16_t _rxID; //receive ID of bamocar
        uint16_t _txID; //transmit ID of bamocar
        
        struct _data {
            int16_t SPEED_ACTUAL = 0, SPEED_RPMMAX = 0, TORQUE_SETPOI = 0;
            uint16_t READY = 0,
                     I_IST = 0, DEVICE_I = 0, KERN_I_200PC = 0,
                     RAMP_ACC = 0, RAMP_DEC = 0,
                     T_MOTOR = 0, T_IGBT = 0, T_AIR = 0,
                     HARD_ENABLED = 0, BATTERY_V = 0, DC_BUS_V = 0;
            uint32_t STATUS = 0;
        } _data;

        bool _sendCAN(Bamocar_dataframe m_data);
        bool _requestData(uint8_t dataAddress, uint8_t interval = INTVL_ONCE);
        //void _listenCAN();

        int16_t const _getData16Bit(can_message_t &msg);
        int32_t const _getData32Bit(can_message_t &msg);

};
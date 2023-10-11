#ifndef bamocar_reg
#define bamocar_reg

//standard vals
#define STD_RX_ID        0x201    //The ID the MC will listen on
#define STD_TX_ID        0x181    //The ID the MC will answer with
#define STD_BAUD_RATE   100000    //Standard Baudrate (100kbps)


//Intervals for continuous msg transmission
#define INTVL_ONCE        0x00
#define INTVL_SUSPEND     0xFF
#define INTVL_100MS       0x64
#define INTVL_200MS       0xC8
#define INTVL_250MS       0xFA

#define REG_BATTERY_V     0x66
#define REG_DC_BUS_V      0xeb

#define REG_I_IST         0x20  //Current actual value
#define REG_DEVICE_I      0xc6  //Current device
#define REG_DEVICE_I_MAX_ 0xc4  //Limit for peak current
#define REG_DEVICE_I_CNT_ 0xc5  //Limit for continius current
#define REG_KERN_I_200PC  0xd9  //Current 200 PC

#define REG_SPEED_ACTUAL  0x30  //Speed actual value
#define REG_SPEED_CMD     0x31  //Digital Speed Set Point
#define REG_SPEED_RPMMAX  0xC8  //(SPEED_RPMMAX) Maximum rotation speed in turns per minute (Servo)


#define REG_STATUS        0x40  //Status
#define REG_READY         0xE2  //State of the device

#define REG_T_MOTOR       0x49  //motor temperature
#define REG_T_IGBT        0x4a  //power stage temperature
#define REG_T_AIR         0x4b  //air temperature

#define REG_TORQUE_SETPOI 0x90  // Digital Torque Set Point

#define REG_RAMP_ACC      0x35  //Ramp Acceleration command
#define REG_RAMP_DEC      0xED  //Ramp Deceleration command

#define REG_ENABLE        0x51  //Disable or Enable transmission
#define REG_REQUEST       0x3D  //Transmission request
#define REG_HARD_ENABLED  0xE8  //Hard Enabled State

#endif
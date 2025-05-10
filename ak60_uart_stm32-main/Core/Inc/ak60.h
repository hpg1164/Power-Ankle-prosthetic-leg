#include "usart.h"

typedef enum
{
    COMM_FW_VERSION = 0,
    COMM_JUMP_TO_BOOTLOADER,
    COMM_ERASE_NEW_APP,
    COMM_WRITE_NEW_APP_DATA,
    COMM_GET_VALUES,             // Get motor operating parameters
    COMM_SET_DUTY,               // Motor operates in duty cycle mode
    COMM_SET_CURRENT,            // Motor operates in current loop mode
    COMM_SET_CURRENT_BRAKE,      // Motor operates in current brake mode
    COMM_SET_RPM,                // Motor operates in speed loop mode
    COMM_SET_POS,                // Motor operates in position loop mode
    COMM_SET_HANDBRAKE,          // Motor operates in handbrake current loop mode
    COMM_SET_DETECT,             // Motor real-time feedback current position command
    COMM_ROTOR_POSITION = 22,    // Motor feedback current position
    COMM_GET_VALUES_SETUP = 50,  // Motor single or multiple parameter acquisition command
    COMM_SET_POS_SPD = 91,       // Motor operates in position-speed loop mode
    COMM_SET_POS_MULTI = 92,     // Set motor motion to single-turn mode
    COMM_SET_POS_SINGLE = 93,    // Set motor motion to multi-turn mode, range ±100 turns
    COMM_SET_POS_UNLIMITED = 94, // Reserved
    COMM_SET_POS_ORIGIN = 95,    // Set motor origin
} DATA_FRAME;

class AK60
{
public:
    DATA_FRAME mode;
    UART_HandleTypeDef uart;

    void send_command();
    void receive_data();
};
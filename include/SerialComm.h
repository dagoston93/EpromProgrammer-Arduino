/**
 * Define the communication protocol
 */
#define CONNECT_REQUEST       0xDA
#define DISCONNECT_REQUEST    0xFF
#define CONNECT_ACCEPT        0xAD
#define READ_REQUEST          0xAA
#define WRITE_REQUEST         0xBB
#define OK                    0xDD
#define DATA_SIZE_ERROR       0x01
#define INCORRECT_VCC         0x02
#define INCORRECT_VPP         0x03

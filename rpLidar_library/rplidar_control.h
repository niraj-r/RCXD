#ifndef RPLIDAR_
    #define RPLIDAR_


    #define BAUDRATE        B115200    // Device baudrate for UART communication
    #define PORT            "/dev/ttyUSB0"   // Serial Port to connect to. tty0x with x the UART port number. Replace by ttyUSB0 for USB

    // Standard period between two requests, in ms. Has to be at least 2ms after reboot, 1ms for the other requests.
    #define DEFAULT_TIMEOUT 2000

    // Commands without response
    #define CMD_STOP        0x25
    #define CMD_RESET       0x40
    // Commands with multiple response
    #define CMD_SCAN        0x20
    #define CMD_FORCE_SCAN  0x21
    #define CMD_EXPR_SCAN	0x82
    // Commands with single response
    #define CMD_INFO        0x50
    #define CMD_HEALTH      0x52    // 0x51 ?

    // Descriptors
    #define DES_REQUEST     0xA5    //request
    #define DES_RESPONSE    0x5A    //response

    // Send modes
    #define SINGLE_RES      0x00
    #define MULTIPLE_RES    0x01

    // Data types
    #define DATA_SCAN       0x81
    #define DATA_INFO       0x04
    #define DATA_HEALTH     0x06

    // Lidar device status
    #define STATUS_OK       0x00
    #define STATUS_WARNING  0x01
    #define STATUS_ERR      0x02

    // Response packet size
    #define SIZE_HEADER     7
    #define SIZE_SCAN       5
    #define SIZE_INFO       20
    #define SIZE_HEALTH     3


    typedef struct rplidar rplidar_t;


    /* connection functions */ 
    rplidar_t* rplidar_create(); /* create new rplidar connection */
    void rplidar_destroy( rplidar_t** rplidar ); /* close the connection with the rplidar */

    /* FUNCTIONS :
     *  rplidar_stop:       exit the current state and enter the idle state (e.g. stop scanning)
     *  rplidar_reset:      reset/reboot the rplidar
     *  rplidar_info:       get device info
     *  rplidar_health      get device health info
     *  rplidar_scan:       start scanning, continuous response of scan packets
     *  rplidar_force_scan: same as rplidar_scan, but force output without checking rotation speed TODO: implement (if necessary (?))
     */
    void rplidar_reset( rplidar_t* rplidar );
    void rplidar_stop( rplidar_t* rplidar );
    int rplidar_info( rplidar_t* rplidar );
    int rplidar_health( rplidar_t* rplidar );
    int rplidar_scan( rplidar_t* rplidar );

#endif //RPLIDAR_

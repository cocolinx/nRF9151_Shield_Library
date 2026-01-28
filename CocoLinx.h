#ifndef CATLINK_H
#define CATLINK_H

#include <Arduino.h>
#include <SoftwareSerial.h>

class CocoLinx {
public:
    typedef enum {
        SERIAL_HARDWARE,
        SERIAL_SOFTWARE
    } SerialType;

    typedef enum {
        CLOSED = 0,
        OPENED = 1,
        DISCONNECTED = 0,
        CONNECTED = 1,
    } NetStatus;

    typedef enum {
        ACK_OKAY = 0,
        ACK_ERR_UNKNOWN,    
    
        // # error return negative sign
        ACK_ERR_ARG, // args
        ACK_ERR_NO_RESPONSE, 
        ACK_ERR_CRC_ACK, 
        ACK_ERR_SERIAL,
        ACK_ERR_PARSE,

        // ack-return
        ACK_ERR_CRC_REQ = 16, // crc error
        ACK_ERR_CATEGORY, // category not supported
        ACK_ERR_CMD, // cmd not supported
        ACK_ERR_FAILED, // operation failed
        ACK_ERR_NOT_SUPPORT, // function not supported
        ACK_ERR_PARAMETER, // parameter check error
        ACK_ERR_NOT_FOUND, // ddns ip not found
        ACK_ERR_LTE_DISCONNECTED,
        ACK_ERR_NO_SOCK, // udp, tcp, mqtt not opened(connected)
        ACK_ERR_SENT, // 
        ACK_ERR_NOT_READY,
        ACK_ERR_HOST_TIMEOUT, // ping
        ACK_ERR_GNSS_NOT_FIXED,
        ACK_ERR_NULL = 255,
    } AckCode; 

    typedef enum {
        PLMN_NOT_SET = -1,
        PLMN_AUTO = 0,
        PLMN_SKT = 45005,
        PLMN_KT = 45008,
        PLMN_LGU = 45006,
    } LtePlmn;

    typedef struct
    {
        char imei[16]; // 15digits, string
        char iccid[24]; // 20digits, string
        char imsi[16]; // 
        uint32_t plmn;
        uint32_t cell_id;
        uint16_t area_code;
        uint16_t band;
        int32_t rsrp_dbm;
        char ip_address[16]; // "xxx.xxx.xxx.xxx"
    } LteInfo;
        
    typedef struct
    {
        int64_t elapsed_ms; // elapsed time from last fix.

        uint16_t fix_counter; // over flow, used to check if data is updated
        uint8_t satellites_visible; // 
        uint8_t satellites_used; // 
        uint16_t year;	/** 4-digit representation (Gregorian calendar). */
        uint8_t month; /** 1...12 */	
        uint8_t day; /** 1...31 */

        uint8_t hour; /** 0...23 */	
        uint8_t minute; /** 0...59 */

        uint8_t seconds; /** 0...59 */		        
        uint8_t rsv_internal[5];

        double latitude;
        double longitude;
        float accuracy; // meter, position accuracy
        float altitude;
        float speed; // m/s
        float heading; // degrees
    } GnssData;

    CocoLinx();

    /**
     * @brief Initialize the UART interface and resets the LTE shield.
     * @param type Serial interface type used to communicate with the shield.
     * @return True if success, false otherwise.
     */
    bool begin(SerialType type);
    void loop();

    /************************************************************************
     *                           System Functions
     ************************************************************************/

    /**
     * @brief Check shield communication status.
     * @return 0 on success, otherwise error code (see AckCode enum).
     */
    int32_t sysCheck();

    /**
     * @brief Reset shield.
     * @return True if success, false otherwise.
     */
    bool sysReset();

    /**
     * @brief Get shield version.
     * @param ver4 Pointer to a 4-byte buffer for the version.
     * @return 0 on success, otherwise error code (see AckCode enum).
     */
    int32_t sysGetVersion(uint8_t *ver4);

    /**
     * @brief Set shield LED status.
     * @param onoff True if turn on, false turn off.
     * @return 0 on success, otherwise error code (see AckCode enum).
     */ 
    int32_t sysSetLed(bool onoff);

    /**
     * @brief Get shield button count.
     * @param pCount Pointer to a buffer for the button count.
     * @return 0 on success, otherwise error code (see AckCode enum).
     */ 
    int32_t sysGetBtnCount(uint32_t *pCount);

    /**
     * @brief Factory reset for shield.
     * @return True if success, false otherwise.
     */   
    bool sysFactoryReset();

    /**
     * @brief Get system uptime. 
     * @param pRtc Pointer to a buffer for current uptime in milliseconds.
     * @return 0 on success, otherwise error code (see AckCode enum).
     */    
    int32_t sysGetRtc(int64_t *pRtc);

    /************************************************************************
     *                           RS-485 Functions
     ************************************************************************/

    /**
     * @brief Enable the RS-485 transceiver on the LTE shield.
     * @param baudrate Baudrate to set on RS-485 transceiver.
     * @return 0 on success, otherwise error code (see AckCode enum).
     */    
    int32_t rs485Enable(uint32_t baudrate);

    /**
     * @brief Disable the RS-485 transceiver on the LTE shield.
     * @return 0 on success, otherwise error code (see AckCode enum).
     */    
    int32_t rs485Disable();

    /**
     * @brief Sends data over the RS-485 interface.
     * @param data Pointer to a buffer for data to be sent.
     * @param size Number of bytes to send.
     * @return 0 on success, otherwise error code (see AckCode enum).
     */   
    int32_t rs485Send(const uint8_t *data, uint32_t size);

    /**
     * @brief Send character data over the RS-485 interface.
     * @param data Pointer to a buffer for data to be sent.
     * @param size Number of bytes to send.
     * @return 0 on success, otherwise error code (see AckCode enum).
     */   
    int32_t rs485Send(const char *data, uint32_t size);

    /**
     * @brief Receive data over the RS-485 interface.
     * @param data Pointer to a buffer for receive data.
     * @param size Number of bytes that reveice.
     * @return 0 on success, otherwise error code (see AckCode enum).
     */   
    int32_t rs485Recv(uint8_t *buffer, uint32_t bufferSize);

    /**
     * @brief Receive character data over the RS-485 interface.
     * @param data Pointer to a buffer for receive data.
     * @param size Number of bytes that reveice.
     * @return 0 on success, otherwise error code (see AckCode enum).
     */   
    int32_t rs485Recv(char *buffer, uint32_t bufferSize);

    /**
     * @brief Clear RS-485 RX ring buffer in shield.
     * @return 0 on success, otherwise error code (see AckCode enum).
     */   
    int32_t rs485ClearBuffer();
 
    /************************************************************************
     *                           GNSS Functions
     ************************************************************************/

    /**
     * @brief Start GNSS operation.
     * @return 0 on success, otherwise error code (see AckCode enum).
     */
    int32_t gnssStart();

    /**
     * @brief Stop GNSS operation.
     * @return 0 on success, otherwise error code (see AckCode enum).
     */
    int32_t gnssStop();

    /**
     * @brief Check if GNSS operation is running or not.
     * @return 1 if running, 0 if not running, otherwise error code (see AckCode enum).
     */
    int32_t gnssIsRunning();

    /**
     * @brief Read GNSS data.
     * @param pGnssData Pointer to a buffer for GNSS data.
     * @return 0 on success, otherwise error code (see AckCode enum).
     */
    int32_t gnssRead(GnssData *pGnssData);

    /************************
     * LTE Functions
     ************************/

    /**
     * @brief Connect LTE with PLMN selection.
     * @param timeout_ms Connection timeout in milliseconds (recommended 3 minutes).
     * @param plmn PLMN code. Set to 0 for automatic network selection.
     * @return 0 on success, otherwise error code (see AckCode enum).
     */
    int32_t lteConnect(uint32_t timeout_ms, int32_t plmn);

    /**
     * @brief Disconnect LTE.
     * @return 0 on success, otherwise error code (see AckCode enum).
     */
    int32_t lteDisconnect();

    /**
     * @brief Check if LTE is connected or not.
     * @return 1 if connected, 0 if not connected, otherwise error code (see AckCode enum).
     */
    int32_t lteIsConnected();

    /**
     * @brief Read LTE information data.
     * @param pInfo Pointer to a buffer for LTE information data.
     * @return 0 on success, otherwise error code (see AckCode enum).
     */
    int32_t lteReadInfo(LteInfo *pInfo);

    /**
     * @brief Get IP address from domain.
     * @param hostname Domain name to resolve IP address
     * @param hostIp Pointer to a 4-byte buffer for IP address.
     * @return 0 on success, otherwise error code (see AckCode enum).
     */
    int32_t ddnsResolve(const char* hostname, uint8_t* hostIp);

    /**
     * @brief Send ICMP ping requests 4 times to the specified IPv4 address.
     * @param dstIp Destination IPv4 address.
     * @param sent Pointer to a buffer for number of ping packets sent.
     * @param recv Pointer to a buffer for number of ping packets received.
     * @param rttAvg Pointer to a buffer for average RTT in ms.
     * @param rttMin Pointer to a buffer for min RTT in ms.
     * @param rttMax Pointer to a buffer for max RTT in ms.
     * @return 0 on success, otherwise error code (see AckCode enum).
     */
    int32_t ping(const uint8_t* dstIp, uint32_t *sent, uint32_t *recv, uint32_t *rttAvg, uint32_t *rttMin, uint32_t *rttMax);
    
    /**
     * @brief Get the current date time UTC.
     * @param unixMillis Pointer to a buffer for UTC.
     * @return 0 on success, otherwise error code (see AckCode enum).
     */
    int32_t datetimeUnixMillis(int64_t *unixMillis);

    /**
     * @brief Send a formatted AT command to the shield's modem and receive the response.
     * @param cmd AT command string to send.
     * @param respBuf Pointer to a buffer for response reveived from shield's modem.
     * @param respBufSize Number of respBuf buffer size.
     * @param pRetCode Pointer to a buffer for result code reveived from shield's modem.
     * @param timeoutMs Response timeout in ms.
     * @return 0 on success, otherwise error code (see AckCode enum).
     */
    int32_t modemAtCmd(char *cmd, char *respBuf, uint32_t respBufSize, int32_t *pRetCode, uint32_t timeoutMs);

    /************************************************************************
     *                           UDP Functions
     ************************************************************************/

    /**
     * @brief Open a UDP socket and connect to the specified remote endpoint.
     * @param dstIp Destination IPv4 address.
     * @param dstPort Destination UDP port number.
     * @return 0 on success, otherwise error code (see AckCode enum).
     */
    int32_t udpOpen(const uint8_t* dstIp, uint16_t dstPort);

    /**
     * @brief Close a UDP socket.
     * @return 0 on success, otherwise error code (see AckCode enum).
     */
    int32_t udpClose();

    /**
     * @brief Check if UDP socket is open or not.
     * @return 1 if opened, 0 if not opened, otherwise error code (see AckCode enum).
     */
    int32_t udpIsOpen();

    /**
     * @brief Send data to a connected peer.
     * @param data Pointer to a buffer for data to be sent.
     * @param size Number of data buffer size.
     * @return 0 on success, otherwise error code (see AckCode enum).
     */
    int32_t udpSend(const uint8_t* data, uint32_t size);

    /**
     * @brief Send character data to a connected peer.
     * @param data Pointer to a buffer for data to be sent.
     * @param size Number of data buffer size.
     * @return 0 on success, otherwise error code (see AckCode enum).
     */
    int32_t udpSend(const char* data, uint32_t size);

    /**
     * @brief Receive data from the connected peer.
     * @param buffer Pointer to a buffer for receive data.
     * @param bufferSize Number of bytes that reveice.
     * @return 0 on success, otherwise error code (see AckCode enum).
     */   
    int32_t udpRecv(uint8_t* buffer, uint32_t bufferSize);

    /**
     * @brief Receive character data from the connected peer.
     * @param buffer Pointer to a buffer for receive data.
     * @param bufferSize Number of bytes that reveice.
     * @return 0 on success, otherwise error code (see AckCode enum).
     */   
    int32_t udpRecv(char* buffer, uint32_t bufferSize);

    /**
     * @brief Clear UDP RX ring buffer in shield.
     * @return 0 on success, otherwise error code (see AckCode enum).
     */   
    int32_t udpClearBuffer();

    /************************************************************************
     *                           TCP Functions
     ************************************************************************/

    /**
     * @brief Open a TCP socket and connect to the specified remote endpoint.
     * @param dstIp Destination IPv4 address.
     * @param dstPort Destination TCP port number.
     * @return 0 on success, otherwise error code (see AckCode enum).
     */
    int32_t tcpConnect(const uint8_t* dstIp, uint16_t dstPort);

    /**
     * @brief Close a TCP socket and disconnect.
     * @return 0 on success, otherwise error code (see AckCode enum).
     */
    int32_t tcpDisconnect();

    /**
     * @brief Check if TCP socket is connected or not.
     * @return 1 if connected, 0 if not connected, otherwise error code (see AckCode enum).
     */
    int32_t tcpIsConnected();

    /**
     * @brief Send data to a connected peer.
     * @param data Pointer to a buffer for data to be sent.
     * @param size Number of data buffer size.
     * @return 0 on success, otherwise error code (see AckCode enum).
     */
    int32_t tcpSend(const uint8_t* data, uint32_t size);

    /**
     * @brief Send character data to a connected peer.
     * @param data Pointer to a buffer for data to be sent.
     * @param size Number of data buffer size.
     * @return 0 on success, otherwise error code (see AckCode enum).
     */
    int32_t tcpSend(const char *data, uint32_t size);

    /**
     * @brief Receive data from the connected peer.
     * @param buffer Pointer to a buffer for receive data.
     * @param bufferSize Number of bytes that reveice.
     * @return 0 on success, otherwise error code (see AckCode enum).
     */   
    int32_t tcpRecv(uint8_t* buffer, uint32_t bufferSize);

    /**
     * @brief Receive character data from the connected peer.
     * @param buffer Pointer to a buffer for receive data.
     * @param bufferSize Number of bytes that reveice.
     * @return 0 on success, otherwise error code (see AckCode enum).
     */   
    int32_t tcpRecv(char *buffer, uint32_t bufferSize);

    /**
     * @brief Clear TCP RX ring buffer in shield.
     * @return 0 on success, otherwise error code (see AckCode enum).
     */  
    int32_t tcpClearBuffer();

    /************************************************************************
     *                           MQTT Functions
     ************************************************************************/

    /**
     * @brief Connect to MQTT.
     * @param dstIp Destination IPv4 address.
     * @param dstPort Destination MQTT port number.
     * @param keepAliveSecs Keep-alive interval in seconds
     * @param cleanSeesion Clean session flag
     * @param clientid MQTT client ID
     * @param username MQTT username
     * @param password MQTT password
     * @return 0 on success, otherwise error code (see AckCode enum).
     */   
    int32_t mqttConnect(const uint8_t* dstIp, uint16_t dstPort, 
        uint16_t keepAliveSecs, bool cleanSeesion, 
        const char *clientid, const char *username, const char *password);
        
    /**
     * @brief Disconnect to MQTT.
     * @return 0 on success, otherwise error code (see AckCode enum).
     */  
    int32_t mqttDisconnect();

    /**
     * @brief Check if MQTT is connected or not.
     * @return 1 if connected, 0 if not connected, otherwise error code (see AckCode enum).
     */  
    int32_t mqttIsConnected();

    /**
     * @brief Publish a message to an MQTT topic.
     * @param topic MQTT topic to publish to
     * @param payload Message payload data
     * @param payloadSize Number of payload buffer size.
     * @param qosLevel MQTT QoS level (0, 1, or 2)
     * @param retain Retain flag
     * @return 0 on success, otherwise error code (see AckCode enum).
     */  
    int32_t mqttPublish(const char *topic, const uint8_t *payload, uint32_t payloadSize, uint8_t qosLevel, bool retain);

    /**
     * @brief Publish a message to an MQTT topic.
     * @param topic MQTT topic to publish to
     * @param payload Message payload character data
     * @param payloadSize Number of payload buffer size.
     * @param qosLevel MQTT QoS level (0, 1, or 2)
     * @param retain Retain flag
     * @return 0 on success, otherwise error code (see AckCode enum).
     */  
    int32_t mqttPublish(const char *topic, const char *payload, uint32_t payloadSize, uint8_t qosLevel, bool retain);

    /**
     * @brief Subscribe a MQTT topic.
     * @param topic MQTT topic to subscribe to
     * @param qosLevel MQTT QoS level (0, 1, or 2)
     * @return 0 on success, otherwise error code (see AckCode enum).
     */  
    int32_t mqttSubscribe(const char *topic, uint8_t qosLevel);

    /**
     * @brief Unsubscribe a MQTT topic.
     * @param topic MQTT topic to unsubscribe to
     * @return 0 on success, otherwise error code (see AckCode enum).
     */  
    int32_t mqttUnSubscribe(const char *topic);
    
    /**
     * @brief Receives an MQTT message from subscribed topic and copies the subscribed topic and payload (if available).
     * @param topicBuffer Pointer to a buffer for store the received topic name.
     * @param topicBufferSize Number of topicBuffer size in bytes.
     * @param payloadBuffer Pointer to a buffer for store the received payload.
     * @param payloadBufferSize Number of payloadBuffer size in bytes.
     * @param pRecvPayloadSize Actual size of the reveiced payload.
     * @return topic size, otherwise negative error code.
     */  
    int32_t mqttRecvMsg(char *topicBuffer, uint32_t topicBufferSize, uint8_t *payloadBuffer, uint32_t payloadBufferSize, int32_t *pRecvPayloadSize);

    /**
     * @brief Receives an MQTT message from subscribed topic and copies the subscribed topic and payload (if available).
     * @param topicBuffer Pointer to a buffer for store the received topic name.
     * @param topicBufferSize Number of topicBuffer size in bytes.
     * @param payloadBuffer Pointer to a character buffer for store the received payload.
     * @param payloadBufferSize Number of payloadBuffer size in bytes.
     * @param pRecvPayloadSize Actual size of the reveiced payload.
     * @return topic size, otherwise negative error code.
     */  
    int32_t mqttRecvMsg(char *topicBuffer, uint32_t topicBufferSize, char *payloadBuffer, uint32_t payloadBufferSize, int32_t *pRecvPayloadSize);

    /**
     * @brief Clear MQTT RX ring buffer in shield.
     * @return 0 on success, otherwise error code (see AckCode enum).
     */  
    int32_t mqttClearBuffer();

private:
    #define CATLINK_DATA_SIZE_MAX     1400
    #define CATLINK_PACKET_SIZE_MAX   (CATLINK_DATA_SIZE_MAX + 16)
    #define CATLINK_HOSTNAME_LEN_MAX    128

    HardwareSerial* _hwSerial;
    SoftwareSerial* _swSerial;
    SerialType _serialType;
    uint8_t _pktbuf[CATLINK_PACKET_SIZE_MAX + 8];
    uint8_t _pktidx = 0;
    
    const uint8_t HW_RX_PIN = 0;
    const uint8_t HW_TX_PIN = 1;
    const uint8_t SW_RX_PIN = 7;
    const uint8_t SW_TX_PIN = 8;

    const uint32_t BAUDRATE = 115200;
    
    typedef enum
    {
        CATEGORY_SYS,
        CATEGORY_RS485,
        CATEGORY_GNSS,
        CATEGORY_LTE,
        CATEGORY_UDP,
        CATEGORY_TCP,
        CATEGORY_MQTT,
    } cocolinx_category_e;

    typedef enum
    {
        CMD_SYS_HELLO, // req>null, ack>null
        CMD_SYS_STATE, // data[0](0:booting, 1:ready)
        CMD_SYS_VERSION, // 
        CMD_SYS_RESET, // null
        CMD_SYS_SLEEP, // null
        CMD_SYS_WAKEUP, // null
        CMD_SYS_LED_USER_ON, // null
        CMD_SYS_LED_USER_OFF, // null
        CMD_SYS_BTN_COUNT,
        CMD_SYS_FACTORY_RESET, // modem factory reset + system reset
        CMD_SYS_RTC, // int64_t(ms)
    } cocolinx_cmd_sys_e;
    
    typedef enum
    {
        CMD_RS485_ENABLE, // baudrate
        CMD_RS485_DISABLE,
        CMD_RS485_SEND, // 
        CMD_RS485_RECV, // 
        CMD_RS485_CLEAR_BUF,
    } cocolinx_cmd_rs485_e;    

    typedef enum
    {
        CMD_GNSS_START, // req>interval, timeout
        CMD_GNSS_STOP,
        CMD_GNSS_STATUS,
        CMD_GNSS_READ,
    } cocolinx_cmd_gnss_e;

    typedef enum
    {
        CMD_LTE_CONNECT_SYNC, // req>data[0:3]=timeout_sec, data[4:7]=plmn, ans>data[0]=0:no connection, 1:connected
        CMD_LTE_DISCONNECT, // req>null
        CMD_LTE_STATUS, // req>null, ack>data[0]='x_lte_connection_status_e'    
        CMD_LTE_READ_INFO, // req>null, ack>'x_lte_info_ack_t'
        CMD_LTE_READ_IMEI, // req>null, ack>imei[16]
        CMD_LTE_READ_ICCID, // req>null, ack>string[20]
        CMD_LTE_DDNS_IPV4, // req>hostname[128], ack>'x_ipaddr_t'
        CMD_LTE_PING_IPV4, // req>'x_lte_ping_req_t', ack>'x_lte_ping_ack_t'    
        CMD_LTE_DATETIME_UTC, // unix ms
        CMD_LTE_MODEM_ATCMD, // direct at command to modem(shell)
    } cocolinx_cmd_lte_e;

    typedef enum
    {
        CMD_UDP_OPEN, // req>'x_ipaddr_t'
        CMD_UDP_CLOSE,
        CMD_UDP_STATUS, // req>null, ack>[0]=0:closed, 1:opened
        CMD_UDP_SEND, // req>data, ack>null
        CMD_UDP_RECV, // req>null, ack>recv_data[recv_size]
        CMD_UDP_CLEAR, // req>null, ack>null, *clear rx buffer
    } cocolinx_cmd_udp_e;    

    typedef enum
    {
        CMD_TCP_CONNECT, // req>'x_ipaddr_t'
        CMD_TCP_DISCONNECT,
        CMD_TCP_STATUS,
        CMD_TCP_SEND,
        CMD_TCP_RECV,
        CMD_TCP_CLEAR,
    } cocolinx_cmd_tcp_e;

    typedef enum
    {        
        CMD_MQTT_CONNECT,
        CMD_MQTT_DISCONNECT,
        CMD_MQTT_STATUS,
        CMD_MQTT_PUBLISH,
        CMD_MQTT_SUBSCRIBE, // req>topic[]
        CMD_MQTT_UNSUBSCRIBE,
        CMD_MQTT_RECV_MSG, // req>buffersize, ack>topicLen
        CMD_MQTT_CLEAR,
    } cocolinx_cmd_mqtt_e;

    bool initSerial();
    uint16_t crc16Calculate(uint8_t *data, uint32_t size);
    int32_t transferPkt(uint8_t category, uint8_t cmd, uint16_t datasize, uint32_t timeout_ms);
    bool waitBootup(uint32_t timeout_ms);
    uint16_t getPktDataSize();
};

#endif // CATLINK_H

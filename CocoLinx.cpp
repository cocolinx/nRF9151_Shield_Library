
#include "CocoLinx.h"

CocoLinx::CocoLinx() :
    _hwSerial(nullptr), 
    _swSerial(nullptr), 
    _serialType(SERIAL_HARDWARE) { }

bool CocoLinx::begin(SerialType type) 
{
    bool success;
    _serialType = type;
    if(initSerial() == false) return false;
    waitBootup(5000);
    return sysReset();
}

bool CocoLinx::initSerial() 
{
    if (_serialType == SERIAL_HARDWARE) {
        _hwSerial = &Serial1;
        _hwSerial->begin(BAUDRATE);
        return true;
    } else {
        if (_swSerial) {
            delete _swSerial;
        }
        _swSerial = new SoftwareSerial(SW_RX_PIN, SW_TX_PIN);
        _swSerial->begin(BAUDRATE);
        return true;
    }
}

uint16_t CocoLinx::crc16Calculate(uint8_t *data, uint32_t size) 
{
    uint16_t crc = 0xFFFF; // 초기값은 0xFFFF

    for (uint16_t i = 0; i < size; i++) {
        crc ^= (uint16_t)data[i]; // 데이터의 한 바이트와 XOR 연산

        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) { // 최하위 비트(LSB)가 1이면
                crc = (crc >> 1) ^ 0xA001; // 오른쪽으로 1비트 쉬프트하고, 다항식(0x8005의 반전된 형태)과 XOR
            } else { // LSB가 0이면
                crc = crc >> 1; // 오른쪽으로 1비트 쉬프트
            }
        }
    }
    return crc;
}

bool CocoLinx::waitBootup(uint32_t timeout_ms)
{
    // wait for bootup
    uint32_t start_ms = millis();
    while((millis() - start_ms) <= timeout_ms) {
        if(sysCheck() == 0) return true;
        else delay(100);
    }
    return false;
}

uint16_t CocoLinx::getPktDataSize()
{
    return *((uint16_t *)(&_pktbuf[4]));
}

// return ack code(negative sign)
int32_t CocoLinx::transferPkt(uint8_t category, uint8_t cmd, uint16_t datasize, uint32_t timeout_ms)
{    
    Stream* port = (_serialType == SERIAL_HARDWARE) ? (Stream*)_hwSerial : (Stream*)_swSerial;
    if(port == nullptr) return -(ACK_ERR_SERIAL);
    
    *((uint32_t *)(&_pktbuf[0])) = 0x35A12501;
    *((uint16_t *)(&_pktbuf[4])) = datasize;
    _pktbuf[8] = _pktidx++;
    _pktbuf[9] = category;
    _pktbuf[10] = cmd;
    _pktbuf[11] = 255;
    *((uint32_t *)(&_pktbuf[12])) = 0;
    *((uint16_t *)(&_pktbuf[6])) = crc16Calculate(&_pktbuf[8], datasize + 8);

    // flush rx buffer if exist
    while (port->available()) {
        port->read();
    }

    // send req packet...
    port->write(_pktbuf, (16 + datasize));

    // recv ack packet...
    uint32_t rxcnt = 0;    
    bool rxdone = false;
    uint32_t start_ms = millis();
    while ((millis() - start_ms) <= timeout_ms) 
    {
        if (port->available() == false) continue;        
        _pktbuf[rxcnt++] = port->read();

        if(rxcnt < 4) continue;
        else if(rxcnt > CATLINK_PACKET_SIZE_MAX) break;
        else if(rxcnt == 4) {
            if(*((uint32_t *)(&_pktbuf[0])) == 0x109BC4F0) {
                continue;
            } else {
                rxcnt = 3;
                _pktbuf[0] = _pktbuf[1];
                _pktbuf[1] = _pktbuf[2];
                _pktbuf[2] = _pktbuf[3];
                continue;
            }
        } else if(rxcnt >= 16) {
            if(rxcnt >= (*((uint16_t *)(&_pktbuf[4])) + 16)) {
                rxdone = true;
                break;
            }    
        } else {}
    }

    if(rxdone == false) return -(ACK_ERR_NO_RESPONSE); // no ack

    // crc16            
    uint32_t pktsize = *((uint16_t *)(&_pktbuf[4])) + 16;    
    uint16_t crc16 = crc16Calculate(&_pktbuf[8], pktsize - 8);
    if(*((uint16_t *)(&_pktbuf[6])) != crc16) return -(ACK_ERR_CRC_ACK); // crc error

    if(_pktbuf[11] == 0) return ACK_OKAY;
    else return -(_pktbuf[11]);
}


void CocoLinx::loop()
{
    // nothing to do...
    return;
}

// ===================================================================================
//                                  CATEGORY_SYS
// ===================================================================================

int32_t CocoLinx::sysCheck() 
{
    int32_t ack = transferPkt(CATEGORY_SYS, CMD_SYS_HELLO, 0, 500);
    return ack;
}

bool CocoLinx::sysReset() 
{
    transferPkt(CATEGORY_SYS, CMD_SYS_RESET, 0, 500);    
    delay(1000);
    return waitBootup(5000);
}

int32_t CocoLinx::sysGetVersion(uint8_t *ver4)
{
    int32_t ack = transferPkt(CATEGORY_SYS, CMD_SYS_VERSION, 0, 500);
    if(ack != 0) return ack;
    uint8_t *payload = &(_pktbuf[16]);
    if(ver4 != nullptr) memcpy(ver4, payload, 4);
    return ACK_OKAY;
}

int32_t CocoLinx::sysSetLed(bool onoff)
{
    int32_t ack;
    if(onoff == true) ack = transferPkt(CATEGORY_SYS, CMD_SYS_LED_USER_ON, 0, 500);
    else ack = transferPkt(CATEGORY_SYS, CMD_SYS_LED_USER_OFF, 0, 500);
    return ack;
}

int32_t CocoLinx::sysGetBtnCount(uint32_t *pCount)
{
    int32_t ack = transferPkt(CATEGORY_SYS, CMD_SYS_BTN_COUNT, 0, 500);
    if(ack != 0) return ack;
    if(pCount != nullptr) {
        uint8_t *payload = &(_pktbuf[16]);
        memcpy(pCount, payload, 4);
    }
    return ACK_OKAY;
}

bool CocoLinx::sysFactoryReset() 
{
    transferPkt(CATEGORY_SYS, CMD_SYS_FACTORY_RESET, 0, 1000);
    delay(1000);
    return waitBootup(5000);
}

int32_t CocoLinx::sysGetRtc(int64_t *pRtc)
{
    int32_t ack = transferPkt(CATEGORY_SYS, CMD_SYS_RTC, 0, 500);
    if(ack != 0) return ack;
    if(pRtc == nullptr) {
        uint8_t *payload = &(_pktbuf[16]);
        memcpy(pRtc, payload, 8);
    }
    return ACK_OKAY;
}

// ===================================================================================
//                                  CATEGORY_RS485
// ===================================================================================

int32_t CocoLinx::rs485Enable(uint32_t baudrate) 
{
    uint8_t *payload = &(_pktbuf[16]);
    *((uint32_t *)&payload[0]) = baudrate;
    int32_t ack = transferPkt(CATEGORY_RS485, CMD_RS485_ENABLE, 4, 1000);
    return ack;
}

int32_t CocoLinx::rs485Disable()
{
    int32_t ack = transferPkt(CATEGORY_RS485, CMD_RS485_DISABLE, 0, 1000);
    return ack;
}

int32_t CocoLinx::rs485Send(const uint8_t *data, uint32_t size)
{
    if(size > CATLINK_DATA_SIZE_MAX) return -(ACK_ERR_ARG);
    if(size == 0) return -(ACK_ERR_ARG);
    if(data == nullptr) return -(ACK_ERR_ARG);

    uint8_t *pktdata = &(_pktbuf[16]);
    memcpy(pktdata, data, size);
    int32_t ack = transferPkt(CATEGORY_RS485, CMD_RS485_SEND, size, 10000);
    return ack;
}

int32_t CocoLinx::rs485Send(const char *data, uint32_t size)
{
    return rs485Send((const uint8_t *)data, size);
}

int32_t CocoLinx::rs485Recv(uint8_t *buffer, uint32_t bufferSize)
{
    uint8_t *pktdata = &(_pktbuf[16]);
    *((uint32_t *)&pktdata[0]) = bufferSize;
    int32_t ack = transferPkt(CATEGORY_RS485, CMD_RS485_RECV, 4, 10000);
    if(ack != 0) return ack;
    
    uint16_t pktdata_size = getPktDataSize();
    if(pktdata_size == 0) return 0; // nothing to recv

    if(buffer != nullptr) memcpy(buffer, pktdata, pktdata_size);
    return (int32_t)pktdata_size;
}

int32_t CocoLinx::rs485Recv(char *buffer, uint32_t bufferSize)
{
    return rs485Recv((uint8_t *)buffer, bufferSize);
}

int32_t CocoLinx::rs485ClearBuffer()
{
    int32_t ack = transferPkt(CATEGORY_RS485, CMD_RS485_CLEAR_BUF, 0, 500);
    return ack;
}

// ===================================================================================
//                                  CATEGORY_GNSS
// ===================================================================================
int32_t CocoLinx::gnssStart()
{
    uint8_t *pktdata = &(_pktbuf[16]); 
    memset(pktdata, 0, 16);
    int32_t ack = transferPkt(CATEGORY_GNSS, CMD_GNSS_START, 16, 10000);
    return ack;
}

int32_t CocoLinx::gnssStop()
{
    int32_t ack = transferPkt(CATEGORY_GNSS, CMD_GNSS_STOP, 0, 3000);
    return ack;
}

int32_t CocoLinx::gnssIsRunning()
{
    int32_t ack = transferPkt(CATEGORY_GNSS, CMD_GNSS_STATUS, 0, 3000);
    if(ack != 0) return ack;
    uint8_t *pktdata = &(_pktbuf[16]);    
    if(pktdata[0] == 0) return 0;
    return 1;
}

int32_t CocoLinx::gnssRead(GnssData *pGnssData)
{
    uint8_t *pktdata = &(_pktbuf[16]); 
    int32_t ack = transferPkt(CATEGORY_GNSS, CMD_GNSS_READ, 0, 3000);
    if(ack != 0) return ack;

    if(pGnssData != nullptr) {
        memcpy(pGnssData, pktdata, sizeof(GnssData));
    }
    return ACK_OKAY;
}   


// ===================================================================================
//                                  CATEGORY_LTE
// ===================================================================================
int32_t CocoLinx::lteConnect(uint32_t timeout_ms, int32_t plmn)
{
    if(timeout_ms < 10000) timeout_ms = 10000;

    uint8_t *pktdata = &(_pktbuf[16]);
    memset(pktdata, 0, 24);
    *((uint32_t *)&pktdata[0]) = timeout_ms;
    *((int32_t *)&pktdata[4]) = plmn;

    int32_t ack = transferPkt(CATEGORY_LTE, CMD_LTE_CONNECT_SYNC, 24, timeout_ms + 3000);
    return ack;
}

int32_t CocoLinx::lteDisconnect()
{
    int32_t ack = transferPkt(CATEGORY_LTE, CMD_LTE_DISCONNECT, 0, 5000);
    return ack;
}

int32_t CocoLinx::lteIsConnected()
{
    int32_t ack = transferPkt(CATEGORY_LTE, CMD_LTE_STATUS, 0, 3000);
    if(ack != 0) return ack;
    uint8_t *pktdata = &(_pktbuf[16]);    
    if(pktdata[0] == 0) return 0;
    return 1;
}

int32_t CocoLinx::lteReadInfo(LteInfo *pInfo)
{    
    int32_t ack = transferPkt(CATEGORY_LTE, CMD_LTE_READ_INFO, 0, 3000);
    if(ack != 0) return ack;
    if(pInfo != nullptr) {
        uint8_t *pktdata = &(_pktbuf[16]);
        memcpy(pInfo, pktdata, sizeof(LteInfo));
        pInfo->imei[15] = '\0';
        pInfo->iccid[20] = '\0';
        pInfo->imsi[15] = '\0';
        pInfo->ip_address[15] = '\0';
    }    
    return ACK_OKAY;
}

int32_t CocoLinx::ddnsResolve(const char* hostname, uint8_t* hostIp)
{
    uint8_t *pktdata = &(_pktbuf[16]);
    
    int32_t nameLen = strnlen(hostname, CATLINK_HOSTNAME_LEN_MAX + 4);
    if(nameLen > CATLINK_HOSTNAME_LEN_MAX) return -(ACK_ERR_ARG);
    if(nameLen == 0) return -(ACK_ERR_ARG);

    memcpy(pktdata, hostname, nameLen);
    pktdata[nameLen] = '\0';

    int32_t ack = transferPkt(CATEGORY_LTE, CMD_LTE_DDNS_IPV4, nameLen + 1, 10000);
    if(ack != 0) return ack;

    if(hostIp != nullptr) {
        hostIp[0] = pktdata[4];
        hostIp[1] = pktdata[5];
        hostIp[2] = pktdata[6];
        hostIp[3] = pktdata[7];
    }
    return ACK_OKAY;
}

int32_t CocoLinx::ping(const uint8_t* dstIp, uint32_t *sent, uint32_t *recv, uint32_t *rttAvg, uint32_t *rttMin, uint32_t *rttMax)
{
    uint8_t *pktdata = &(_pktbuf[16]);
    memset(pktdata, 0, 24);
    *((uint16_t *)&pktdata[0]) = 1; // ipv4
    *((uint16_t *)&pktdata[2]) = 0; // port
    pktdata[4] = dstIp[0]; // ip
    pktdata[5] = dstIp[1];
    pktdata[6] = dstIp[2];
    pktdata[7] = dstIp[3];
    
    pktdata[24] = 32; // ping pktdata size
    pktdata[25] = 4; // ping count
    *((uint16_t *)&pktdata[26]) = 5000; // ping timeout(per ping)
    *((uint32_t *)&pktdata[28]) = 0; // 

    int32_t ack = transferPkt(CATEGORY_LTE, CMD_LTE_PING_IPV4, 32, 20000);
    if(ack != 0) return ack;

    if(sent != nullptr) *sent = (uint32_t)pktdata[0];
    if(recv != nullptr) *recv = (uint32_t)pktdata[1];
    if(rttAvg != nullptr) *rttAvg = (uint32_t)(*((uint16_t *)&pktdata[2]));
    if(rttMax != nullptr) *rttMax = (uint32_t)(*((uint16_t *)&pktdata[4]));
    if(rttMin != nullptr) *rttMin = (uint32_t)(*((uint16_t *)&pktdata[6]));
    return ACK_OKAY;
}

int32_t CocoLinx::datetimeUnixMillis(int64_t *unixMillis)
{
    uint8_t *pktdata = &(_pktbuf[16]);
    *((uint32_t *)&pktdata[0]) = 8000; // timeout to get time
    int32_t ack = transferPkt(CATEGORY_LTE, CMD_LTE_DATETIME_UTC, 4, 10000);
    if(ack != 0) return ack;

    if(unixMillis != nullptr) memcpy(unixMillis, pktdata, 8);
    return ACK_OKAY;
}

int32_t CocoLinx::modemAtCmd(char *cmd, char *respBuf, uint32_t respBufSize, int32_t *pRetCode, uint32_t timeoutMs)
{
    int32_t len = strnlen(cmd, 513);
    if(len > 512) return -(ACK_ERR_ARG);
    if(len < 2) return -(ACK_ERR_ARG);

    uint8_t *pktdata = &(_pktbuf[16]);
    memcpy(pktdata, cmd, len);
    if(timeoutMs < 1000) timeoutMs = 1000;
    int32_t ack = transferPkt(CATEGORY_LTE, CMD_LTE_MODEM_ATCMD, len, timeoutMs);
    if(ack != 0) return ack;
    
    uint16_t pktdata_size = getPktDataSize();
    if(pktdata_size < 4) return -(ACK_ERR_PARSE);

    int respSize = pktdata_size - 4;
    if(pRetCode != nullptr) memcpy(pRetCode, &pktdata[0], 4);
    if(respBuf != nullptr && respBufSize > 0) {
        if(respSize >= respBufSize) respSize = respBufSize - 1;
        if(respSize > 0) memcpy(respBuf, &pktdata[4], respSize);
        respBuf[respSize] = '\0';
    }
    return ACK_OKAY;
}

// ===================================================================================
//                                  CATEGORY_UDP
// ===================================================================================
int32_t CocoLinx::udpOpen(const uint8_t* dstIp, uint16_t dstPort)
{
    uint8_t *pktdata = &(_pktbuf[16]);
    memset(pktdata, 0, 24);
    *((uint16_t *)&pktdata[0]) = 1; // ipv4
    *((uint16_t *)&pktdata[2]) = dstPort; // port
    pktdata[4] = dstIp[0]; // ip
    pktdata[5] = dstIp[1];
    pktdata[6] = dstIp[2];
    pktdata[7] = dstIp[3];

    int32_t ack = transferPkt(CATEGORY_UDP, CMD_UDP_OPEN, 24, 15000);
    return ack;
}

int32_t CocoLinx::udpClose()
{
    int32_t ack = transferPkt(CATEGORY_UDP, CMD_UDP_CLOSE, 0, 3000);
    return ack;
}

int32_t CocoLinx::udpIsOpen()
{
    int32_t ack = transferPkt(CATEGORY_UDP, CMD_UDP_STATUS, 0, 3000);
    if(ack != 0) return ack;
    uint8_t *pktdata = &(_pktbuf[16]);
    if(pktdata[0] == 0) return 0;
    else return 1;
}

int32_t CocoLinx::udpSend(const uint8_t* data, uint32_t size)
{    
    if(size > CATLINK_DATA_SIZE_MAX) return -(ACK_ERR_ARG);
    if(size == 0) return -(ACK_ERR_ARG);
    if(data == nullptr) return -(ACK_ERR_ARG);

    uint8_t *pktdata = &(_pktbuf[16]);
    memcpy(pktdata, data, size);
    int32_t ack = transferPkt(CATEGORY_UDP, CMD_UDP_SEND, size, 10000);
    return ack;
}

int32_t CocoLinx::udpSend(const char *data, uint32_t size)
{
    return udpSend((const uint8_t *)data, size);
}

int32_t CocoLinx::udpRecv(uint8_t* buffer, uint32_t bufferSize)
{
    uint8_t *pktdata = &(_pktbuf[16]);
    *((uint32_t *)&pktdata[0]) = bufferSize;
    int32_t ack = transferPkt(CATEGORY_UDP, CMD_UDP_RECV, 4, 10000);
    if(ack != 0) return ack;

    uint16_t pktdata_size = getPktDataSize();
    if(pktdata_size == 0) return 0; // nothing to recv
    if(pktdata_size > bufferSize) pktdata_size = bufferSize;
    if(buffer != nullptr) memcpy(buffer, pktdata, pktdata_size);
    return (uint32_t)pktdata_size;
}

int32_t CocoLinx::udpRecv(char *buffer, uint32_t bufferSize)
{
    return udpRecv((uint8_t *)buffer, bufferSize);
}

int32_t CocoLinx::udpClearBuffer()
{
    int32_t ack = transferPkt(CATEGORY_UDP, CMD_UDP_CLEAR, 0, 500);
    return ack;
}


// ===================================================================================
//                                 CATEGORY_TCP
// ===================================================================================
int32_t CocoLinx::tcpConnect(const uint8_t* dstIp, uint16_t dstPort)
{
    uint8_t *pktdata = &(_pktbuf[16]);
    memset(pktdata, 0, 24);
    *((uint16_t *)&pktdata[0]) = 1; // ipv4
    *((uint16_t *)&pktdata[2]) = dstPort; // port
    pktdata[4] = dstIp[0]; // ip
    pktdata[5] = dstIp[1];
    pktdata[6] = dstIp[2];
    pktdata[7] = dstIp[3];

    int32_t ack = transferPkt(CATEGORY_TCP, CMD_TCP_CONNECT, 24, 15000);
    return ack;
}

int32_t CocoLinx::tcpDisconnect()
{
    int32_t ack = transferPkt(CATEGORY_TCP, CMD_TCP_DISCONNECT, 0, 10000);
    return ack;
}

int32_t CocoLinx::tcpIsConnected()
{
    int32_t ack = transferPkt(CATEGORY_TCP, CMD_TCP_STATUS, 0, 3000);
    if(ack != 0) return ack;
    uint8_t *pktdata = &(_pktbuf[16]);
    if(pktdata[0] == 0) return 0;
    else return 1;
}

int32_t CocoLinx::tcpSend(const uint8_t* data, uint32_t size)
{    
    if(size > CATLINK_DATA_SIZE_MAX) return -(ACK_ERR_ARG);
    if(size == 0) return -(ACK_ERR_ARG);
    if(data == nullptr) return -(ACK_ERR_ARG);

    uint8_t *pktdata = &(_pktbuf[16]);
    memcpy(pktdata, data, size);
    int32_t ack = transferPkt(CATEGORY_TCP, CMD_TCP_SEND, size, 10000);
    return ack;
}
int32_t CocoLinx::tcpSend(const char *data, uint32_t size)
{
    return tcpSend((const uint8_t *)data, size);
}

int32_t CocoLinx::tcpRecv(uint8_t* buffer, uint32_t bufferSize)
{
    uint8_t *pktdata = &(_pktbuf[16]);
    *((uint32_t *)&pktdata[0]) = bufferSize;
    int32_t ack = transferPkt(CATEGORY_TCP, CMD_TCP_RECV, 4, 10000);
    if(ack != 0) return ack;

    uint16_t pktdata_size = getPktDataSize();
    if(pktdata_size == 0) return 0; // nothing to recv
    if(pktdata_size > bufferSize) pktdata_size = bufferSize;
    if(buffer != nullptr) memcpy(buffer, pktdata, pktdata_size);
    return (uint32_t)pktdata_size;
}

int32_t CocoLinx::tcpRecv(char *buffer, uint32_t bufferSize)
{
    return tcpRecv((uint8_t *)buffer, bufferSize);
}

int32_t CocoLinx::tcpClearBuffer()
{
    int32_t ack = transferPkt(CATEGORY_TCP, CMD_TCP_CLEAR, 0, 500);
    return ack;
}


// ===================================================================================
//                                  CATEGORY_MQTT
// ===================================================================================
int32_t CocoLinx::mqttConnect(const uint8_t* dstIp, uint16_t dstPort, 
        uint16_t keepAliveSecs, bool cleanSeesion, 
        const char *clientid, const char *username, const char *password)
{
    uint8_t *pktdata = &(_pktbuf[16]);

    memset(pktdata, 0, 24);
    *((uint16_t *)&pktdata[0]) = 1; // ipv4
    *((uint16_t *)&pktdata[2]) = dstPort; // port
    pktdata[4] = dstIp[0]; // ip
    pktdata[5] = dstIp[1];
    pktdata[6] = dstIp[2];
    pktdata[7] = dstIp[3];

    if(clientid == nullptr) return -(ACK_ERR_ARG);

    memset(&pktdata[24], 0, 140);

    uint32_t len;

    len = strnlen(clientid, 65);
    if(len > 64) return -(ACK_ERR_ARG);
    if(len == 0) return -(ACK_ERR_ARG);
    memcpy(&pktdata[24], clientid, len);

    if(username != nullptr) {
        len = strnlen(username, 33);
        if(len > 32) return -(ACK_ERR_ARG);
        if(len > 0) {
            memcpy(&pktdata[92], username, len);

            if(password != nullptr) {
                len = strnlen(password, 33);
                if(len > 32) return -(ACK_ERR_ARG);
                if(len > 0) {
                    memcpy(&pktdata[128], password, len);
                }        
            }
        }        
    }

    *((uint16_t *)&pktdata[164]) = keepAliveSecs; //
    pktdata[166] = cleanSeesion == true ? 1 : 0;
    pktdata[167] = 0;

    int32_t ack = transferPkt(CATEGORY_MQTT, CMD_MQTT_CONNECT, 168, 30000);
    return ack;
}

int32_t CocoLinx::mqttDisconnect()
{
    int32_t ack = transferPkt(CATEGORY_MQTT, CMD_MQTT_DISCONNECT, 168, 30000);    
    return ack;
}

int32_t CocoLinx::mqttIsConnected()
{
    int32_t ack = transferPkt(CATEGORY_MQTT, CMD_MQTT_STATUS, 0, 3000);
    if(ack != 0) return ack;
    uint8_t *pktdata = &(_pktbuf[16]);
    if(pktdata[0] == 0) return 0;
    else return 1;
}

int32_t CocoLinx::mqttPublish(const char *topic, const uint8_t *payload, uint32_t payloadSize, uint8_t qosLevel, bool retain)
{
    uint8_t *pktdata = &(_pktbuf[16]);
    if(topic == nullptr) return -(ACK_ERR_ARG);
    if(payloadSize > 640) return -(ACK_ERR_ARG);

    pktdata[0] = qosLevel;
    pktdata[1] = retain == true ? 1 : 0;
    pktdata[2] = pktdata[3] = 0;

    uint32_t len;

    // topic
    len = strnlen(topic, 129);
    if(len == 0 || len > 128) return -(ACK_ERR_ARG);
    uint32_t topic_size = len;
    *((uint16_t *)&pktdata[4]) = topic_size; //
    memcpy(&pktdata[8], topic, topic_size);
    
    if(payload == nullptr || payloadSize == 0) {
        *((uint16_t *)&pktdata[6]) = 0; //
        payloadSize = 0;
    } else {        
        *((uint16_t *)&pktdata[6]) = (uint16_t)payloadSize; //
        memcpy(&pktdata[8 + topic_size], payload, payloadSize);
    }

    int32_t ack = transferPkt(CATEGORY_MQTT, CMD_MQTT_PUBLISH, 8 + topic_size + payloadSize, 30000);
    return ack;
}

int32_t CocoLinx::mqttPublish(const char *topic, const char *payload, uint32_t payloadSize, uint8_t qosLevel, bool retain)
{
    return mqttPublish(topic, (const uint8_t *)payload, payloadSize, qosLevel, retain);
}

int32_t CocoLinx::mqttSubscribe(const char *topic, uint8_t qosLevel)
{
    uint8_t *pktdata = &(_pktbuf[16]);
    if(topic == nullptr) return -(ACK_ERR_ARG);

    pktdata[0] = qosLevel;
    pktdata[1] = 0;
    pktdata[2] = pktdata[3] = 0;

    uint32_t len;

    // topic
    len = strnlen(topic, 129);
    if(len == 0 || len > 128) return -(ACK_ERR_ARG);
    uint32_t topic_size = len;
    *((uint16_t *)&pktdata[4]) = topic_size; //
    memcpy(&pktdata[8], topic, topic_size); 

    *((uint16_t *)&pktdata[6]) = 0; //
    
    int32_t ack = transferPkt(CATEGORY_MQTT, CMD_MQTT_SUBSCRIBE, 8 + topic_size, 30000);
    return ack;
}

int32_t CocoLinx::mqttUnSubscribe(const char *topic)
{
    uint8_t *pktdata = &(_pktbuf[16]);
    if(topic == nullptr) return -(ACK_ERR_ARG);

    pktdata[0] = 0;
    pktdata[1] = 0;
    pktdata[2] = pktdata[3] = 0;

    uint32_t len;

    // topic
    len = strnlen(topic, 129);
    if(len == 0 || len > 128) return -(ACK_ERR_ARG);
    uint32_t topic_size = len;
    *((uint16_t *)&pktdata[4]) = topic_size; //
    memcpy(&pktdata[8], topic, topic_size); 

    *((uint16_t *)&pktdata[6]) = 0; //
    
    int32_t ack = transferPkt(CATEGORY_MQTT, CMD_MQTT_UNSUBSCRIBE, 8 + topic_size, 30000);
    return ack;
}

// return topic size
// return negative value: error
int32_t CocoLinx::mqttRecvMsg(char *topicBuffer, uint32_t topicBufferSize, uint8_t *payloadBuffer, uint32_t payloadBufferSize, int32_t *pRecvPayloadSize)
{
    uint8_t *pktdata = &(_pktbuf[16]);

    if(pRecvPayloadSize != nullptr) *pRecvPayloadSize = 0;

    if(topicBuffer == nullptr) return -(ACK_ERR_ARG);
    if(topicBufferSize < 2) return -(ACK_ERR_ARG);

    int32_t ack = transferPkt(CATEGORY_MQTT, CMD_MQTT_RECV_MSG, 0, 10000);
    if(ack != 0) return ack;

    uint16_t pktdata_size = getPktDataSize();
    if(pktdata_size < 8) return 0; // nothing to recv

    uint16_t topic_size = (*((uint16_t *)&pktdata[4]));
    if(topic_size == 0) return 0;

    if(topic_size > (topicBufferSize - 1)) topic_size = (topicBufferSize - 1);
    memcpy(topicBuffer, &pktdata[8], topic_size);
    topicBuffer[topic_size] = '\0';

    uint16_t payload_size = (*((uint16_t *)&pktdata[6]));
    if(payload_size == 0) return topic_size; // only topic exist
    if(payloadBuffer == nullptr) return topic_size;
    if(payloadBufferSize == 0) return topic_size;

    if(payload_size > payloadBufferSize) payload_size = payloadBufferSize;
    memcpy(payloadBuffer, &pktdata[8 + topic_size], payload_size);
    if(pRecvPayloadSize != nullptr) *pRecvPayloadSize = payload_size;

    return topic_size;
}

int32_t CocoLinx::mqttRecvMsg(char *topicBuffer, uint32_t topicBufferSize, char *payloadBuffer, uint32_t payloadBufferSize, int32_t *pRecvPayloadSize)
{
    return mqttRecvMsg(topicBuffer, topicBufferSize, (uint8_t *)payloadBuffer, payloadBufferSize, pRecvPayloadSize);
}

int32_t CocoLinx::mqttClearBuffer()
{
    int32_t ack = transferPkt(CATEGORY_MQTT, CMD_MQTT_CLEAR, 0, 5000);
    return ack;
}
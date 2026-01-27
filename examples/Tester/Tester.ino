#include "CocoLinx.h"

#define TEST_LTE_PLMN_SELECT 	CocoLinx::PLMN_SKT
#define TEST_INTERVAL_SECONDS 	180 // 3minutes

CocoLinx coco;
CocoLinx::LteInfo lteInfo;

// keep lte connection
bool sample_keep_connection()
{
	int32_t ret;

	Serial.println("*** sample_keep_connection() ***");	

	// lte check connection
	Serial.print("check lte connection...");
	ret = coco.lteIsConnected();
	if(ret == 1) {
		Serial.println("connected");
		return true;
	} else if(ret == 0) {
		Serial.println("disconnected");
	} else {
		// error
		Serial.print("error: ");
		Serial.println(ret);
		coco.lteDisconnect();
	}

	// lte connect...
	Serial.print("lte connect...");
	uint32_t duration = millis();	
	ret = coco.lteConnect(180000, TEST_LTE_PLMN_SELECT);
	if(ret != 0) {
		Serial.print("error: ");
		Serial.println(ret);
		return false;
	} else {
		duration = millis() - duration;
		Serial.println("okay");

		Serial.print("duration(ms):");
		Serial.println(duration);
		return true;
	}
}

bool sample_lteinfo()
{
	int32_t ret;

	Serial.println("*** sample_lteinfo() ***");	

	// lte read information...
	Serial.print("lte read info...");
	ret =  coco.lteReadInfo(&lteInfo);
	if(ret != 0) {
		Serial.print("error: ");
		Serial.println(ret);
		return false;
	} else {
		Serial.println("okay");
		Serial.print("imei: ");
		Serial.println(lteInfo.imei);
		Serial.print("iccid: ");
		Serial.println(lteInfo.iccid);
		Serial.print("imsi: ");
		Serial.println(lteInfo.imsi);
		Serial.print("plmn: ");
		Serial.println(lteInfo.plmn);
		Serial.print("cell_id: ");
		Serial.println(lteInfo.cell_id);
		Serial.print("area_code: ");
		Serial.println(lteInfo.area_code);
		Serial.print("band: ");
		Serial.println(lteInfo.band);
		Serial.print("rsrp_dbm: ");
		Serial.println(lteInfo.rsrp_dbm);
		Serial.print("ip address: ");
		Serial.println(lteInfo.ip_address);		
		return true;
	}
}

bool sample_datetime()
{
	int32_t ret;

	Serial.println("*** sample_datetime() ***");	

	// #1. datetime
	Serial.print("kor date time...");
	int64_t unixMillis = 0;

	ret = coco.datetimeUnixMillis(&unixMillis);
	if(ret != 0) {
		Serial.print("error: ");
		Serial.println(ret);
		return false;
	} else {
		Serial.println("okay");

		time_t korSecs = (unixMillis / 1000) + 32400;
		struct tm *tm_info = gmtime(&korSecs);
		Serial.print("time> ");
		Serial.print(tm_info->tm_year + 1900);
		Serial.print("-");
		Serial.print(tm_info->tm_mon  + 1);
		Serial.print("-");
		Serial.print(tm_info->tm_mday);
		Serial.print(" ");
		Serial.print(tm_info->tm_hour);
		Serial.print(":");
		Serial.print(tm_info->tm_min);
		Serial.print(":");
		Serial.print(tm_info->tm_sec);
		Serial.println();
		return true;
	}
}

bool sample_ddns()
{
	int32_t ret;

	Serial.println("*** sample_ddns() ***");	

	const char *host = "echo.cocolinx.com"; // should get 3.36.143.77
	uint8_t ipv4[4] = {0,};

	// #2. ddns resolver
	Serial.print("ddns resolve...'");
	Serial.print(host);
	Serial.print("'...");

	ret = coco.ddnsResolve(host, ipv4);
	if(ret != 0) {
		Serial.print("error: ");
		Serial.println(ret);
		return false;
	} else {
		Serial.println("okay");

		Serial.print("ipv4> ");
		Serial.print(ipv4[0]); Serial.print(".");
		Serial.print(ipv4[1]); Serial.print(".");
		Serial.print(ipv4[2]); Serial.print(".");
		Serial.print(ipv4[3]); Serial.println();
		return true;
	}
}

bool sample_ping()
{
	int32_t ret;

	Serial.println("*** sample_ping() ***");	

	uint32_t txCnt, rxCnt, rttAvg, rttMin, rttMax;
	const uint8_t ipv4[4] = {8, 8, 8, 8}; // google ping

	// # ping
	Serial.print("ping...'");
	Serial.print(ipv4[0]); Serial.print(".");
	Serial.print(ipv4[1]); Serial.print(".");
	Serial.print(ipv4[2]); Serial.print(".");
	Serial.print(ipv4[3]); Serial.print("'...");

	ret = coco.ping(ipv4, &txCnt, &rxCnt, &rttAvg, &rttMin, &rttMax);
	if(ret != 0) {
		Serial.print("error: ");
		Serial.println(ret);
		return false;
	} else {
		Serial.println("okay");

		Serial.print("ping> ");
		Serial.print("sent:");
		Serial.print(txCnt);
		Serial.print(", recv:");
		Serial.print(rxCnt);
		Serial.print(", avg(ms):");
		Serial.print(rttAvg);
		Serial.print(", min(ms):");
		Serial.print(rttMin);
		Serial.print(", max(ms):");
		Serial.print(rttMax);
		Serial.println();
		return true;
	}
}

bool sample_rs485()
{	
	int32_t ret;
	Serial.println("*** sample_rs485() ***");	

	Serial.print("rs485(115200) enable...");
	ret = coco.rs485Enable(115200);
	if(ret != 0) {
		Serial.print("error: ");
		Serial.println(ret);
		coco.rs485Disable();
		return false;
	} else {
		Serial.println("okay");

		// send...
		Serial.print("rs485 send data...");
		char str[] = "hello rs485 world~";
		ret = coco.rs485Send(str, strlen(str));
		if(ret != 0) {
			Serial.print("error: ");
			Serial.println(ret);
		} else {
			Serial.println("okay");
		}

		// recv...for 5secs
		Serial.println("rs485 recv data(5secs)...start");
		char rxbuf[64];
		int rxcnt = 10; // 500ms * 10 = 5secs
		while(rxcnt--) {
			int rxsize = coco.rs485Recv(rxbuf, sizeof(rxbuf) - 1);
			if(rxsize > 0) {
				Serial.print("recv> ");
				for(int i=0; i<rxsize; i++) {
					if(rxbuf[i] < 0x10) Serial.print("0x0");
					else Serial.print("0x");
					Serial.print(rxbuf[i], HEX);
					Serial.print(" ");
				}
				Serial.println();
			} else {
				if(rxcnt > 0) delay(500); // delay
			}
		}
		Serial.print("rs485 disable...");
		coco.rs485Disable();
		Serial.println("okay");
		return true;
	}
}

bool sample_udp()
{
	int32_t ret;
	Serial.println("*** sample_udp() ***");	

	const uint8_t ipv4[4] = {43, 200, 166, 133};
	const uint16_t port = 7777;

	Serial.println("udp echo server: 43, 200, 166, 133:7777 by cocolinx");
	Serial.print("udp open...");
	ret = coco.udpOpen(ipv4, port);
	if(ret != 0) {
		Serial.print("error: ");
		Serial.println(ret);
		coco.udpClose();
		return false;
	} else {
		Serial.println("okay");

		// send...
		char udpTx[] = "hello udp world~~~";
		Serial.print("udp send...");
		ret = coco.udpSend(udpTx, strlen(udpTx));
		if(ret != 0) {
			Serial.print("error: ");
			Serial.println(ret);
		} else {
			Serial.println("okay");
			Serial.print("sent> ");
			Serial.println(udpTx);
		}

		// recv...for 15secs
		Serial.println("udp recv data(max 15secs)...start");
		char udpRx[64];
		int rxcnt = 30; // 500ms * 30 = 15secs
		while(rxcnt--) {
			int rxsize = coco.udpRecv(udpRx, sizeof(udpRx) - 1);
			if(rxsize > 0) {
				Serial.print("recv> ");
				udpRx[rxsize] = '\0';
				Serial.println(udpRx);
				break;
			} else {
				if(rxcnt > 0) delay(500); // delay
			}
		}

		Serial.print("udp close...");
		coco.udpClose();
		Serial.println("okay");
		return true;
	}
}

bool sample_tcp()
{
	int32_t ret;

	Serial.println("*** sample_tcp() ***");	

	const uint8_t ipv4[4] = {43, 200, 166, 133};
	const uint16_t port = 7777;

	Serial.println("tcp echo server: 43, 200, 166, 133:7777 by cocolinx");
	Serial.print("tcp connect...");
	ret = coco.tcpConnect(ipv4, port);
	if(ret != 0) {
		Serial.print("error: ");
		Serial.println(ret);
		coco.tcpDisconnect();
		return false;
	} else {
		Serial.println("okay");

		// send...
		char tcpTx[] = "hello tcp world~~~";
		Serial.print("tcp send...");
		ret = coco.tcpSend(tcpTx, strlen(tcpTx));
		if(ret != 0) {
			Serial.print("error: ");
			Serial.println(ret);
		} else {
			Serial.println("okay");
			Serial.print("sent> ");
			Serial.println(tcpTx);
		}

		// recv...for 15secs
		Serial.println("tcp recv data(max 15secs)...start");
		char tcpRx[64];
		int rxcnt = 30; // 500ms * 30 = 15secs
		while(rxcnt--) {
			int rxsize = coco.tcpRecv(tcpRx, sizeof(tcpRx) - 1);
			if(rxsize > 0) {
				Serial.print("recv> ");
				tcpRx[rxsize] = '\0';
				Serial.println(tcpRx);
				break;
			} else {
				if(rxcnt > 0) delay(500); // delay
			}
		}
		Serial.print("tcp close...");
		coco.tcpDisconnect();
		Serial.println("okay");
		return true;
	}
}

bool sample_mqtt()
{
	int32_t ret;	
	Serial.println("*** sample_mqtt() ***");	

	const char *mqttHost = "test.mosquitto.org";
	const uint16_t mqttPort = 1883;
	const uint8_t keepaliveSecs = 5;

	const char *topicPub = "cocolinx/shield/test/a01/secs";
	const char *topicSub = "cocolinx/shield/test/a00/secs";

	uint8_t mqttIp[4];
	char *clientId = lteInfo.imei;

	// get ip adderss
	Serial.print("mqtt resolve ddns...'");
	Serial.print(mqttHost);
	Serial.print("'...");
	ret = coco.ddnsResolve(mqttHost, mqttIp);
	if(ret != 0) {
		Serial.print("error: ");
		Serial.println(ret);
		return false;
	} 

	Serial.println("okay");
	Serial.print("ipv4> ");
	Serial.print(mqttIp[0]); Serial.print(".");
	Serial.print(mqttIp[1]); Serial.print(".");
	Serial.print(mqttIp[2]); Serial.print(".");
	Serial.print(mqttIp[3]); Serial.println();
	
	// connect
	Serial.print("mqtt connect...");
	ret = coco.mqttConnect(mqttIp, mqttPort, keepaliveSecs, false, clientId, nullptr, nullptr);
	if(ret != 0) {		
		Serial.print("error: ");
		Serial.println(ret);
		coco.mqttDisconnect();
		return false;
	} else {
		Serial.println("okay");

		// publish
		Serial.print("mqtt publish...'");
		Serial.print(topicPub);
		Serial.print("'...");

		uint32_t secs = (millis() / 1000) % 1000000;
		char secsStr[16];
		sprintf(secsStr, "%us", secs);
		secsStr[15] = '\0'; // in case
		ret = coco.mqttPublish(topicPub, secsStr, strlen(secsStr), 0, false);
		if(ret != 0) {
			Serial.print("error: ");
			Serial.println(ret);
		} else {
			Serial.println("okay");
		}

		// subscribe and wait message...			
		Serial.print("mqtt subscribe...'");
		Serial.print(topicSub);
		Serial.print("'...");
		ret = coco.mqttSubscribe(topicSub, 0);
		if(ret != 0) {
			Serial.print("error: ");
			Serial.println(ret);
		} else {
			Serial.println("okay");

			Serial.println("mqtt wait messages(60secs)...start");
			char topicBuf[64];
			char payloadBuf[32];
			int rxcnt = 120; // 500ms * 120 = 60secs
			
			while(rxcnt--) {
				int32_t payloadSize = 0;
				int32_t topicSize = coco.mqttRecvMsg(topicBuf, sizeof(topicBuf) - 1, payloadBuf, sizeof(payloadBuf) - 1, &payloadSize);
				if(topicSize > 0) {
					Serial.print("recv> ");
					Serial.print(topicBuf);
					Serial.print("=");
					if(payloadSize > 0) {
						payloadBuf[payloadSize] = '\0';
						Serial.print(payloadBuf);
						Serial.println();	
					}
				} else {
					if(rxcnt > 0) delay(500); // delay
				}
			}

			coco.mqttUnSubscribe(topicSub);
		}
		Serial.print("mqtt disconnect...");
		coco.mqttDisconnect();
		Serial.println("okay");
		return true;
	}
}

void setup()
{
	int32_t ret;
	bool success;
	// serial monitor
	Serial.begin(115200);
	while (!Serial) ; //
	Serial.println();
	Serial.println("====== setup() start ======");

	// cocolinx begin (hardware:serial1 or software)
	Serial.print("cocolinx begin...");
	success = coco.begin(CocoLinx::SERIAL_HARDWARE);
	if(success == false) {
		Serial.println("error");
		Serial.println("halt forever...");
		while (1);
	} else {
		Serial.println("okay");

		// cocolinx version
		Serial.print("cocolinx version...");
		uint8_t ver[4]; // version length must be 4
		ret = coco.sysGetVersion(ver);
		if(ret != 0) {
			Serial.print("error: ");
			Serial.println(ret);
		} else {
			Serial.println("okay");

			Serial.print("version> ");
			Serial.print(ver[0]); Serial.print(".");
			Serial.print(ver[1]); Serial.print(".");
			Serial.print(ver[2]); Serial.print(".");
			Serial.print(ver[3]); Serial.println();
		}
	}
	
	if(sample_keep_connection() == true) sample_lteinfo();

	#if(0)
	{
		char rxbuf[128] = {0,};
		int32_t retcode;
		ret = coco.modemAtCmd("AT+CGMM", rxbuf, 128, &retcode, 5000);
		Serial.println(retcode);
		Serial.println(rxbuf);
		ret = coco.modemAtCmd("AT+CFUN?", rxbuf, 128, &retcode, 5000);
		Serial.println(retcode);
		Serial.println(rxbuf);

		while(true)
		{
			ret = coco.modemAtCmd("AT+CEREG?", rxbuf, 128, &retcode, 5000);
			Serial.println(retcode);
			Serial.println(rxbuf);
			delay(5000);
		}
		

		while(1);
	}
	#endif

	Serial.println("====== setup() done ======");
	Serial.println();
}

void loop()
{
	static uint32_t millisPrev = -(1000 * 60 * 5); // start first test on first loop
	static uint32_t testCount = 0;
	
	int32_t ret;
	uint32_t testIntervalMs = (1000 * TEST_INTERVAL_SECONDS);

	coco.loop(); //

	if(testIntervalMs < 3000) testIntervalMs = 3000;
	
	if((millis() - millisPrev) >= testIntervalMs)
	{
		testCount++;

		Serial.println();
		Serial.print("====== test start [");
		Serial.print(testCount);
		Serial.println("] ======");

		// # cocolinx user-led
		Serial.println("user-led turn on.");
	    coco.sysSetLed(true);

		// # button click count
		Serial.print("get button click count...");
		uint32_t clickCount;
		ret = coco.sysGetBtnCount(&clickCount);
		if(ret != 0) {
			Serial.print("error: ");
			Serial.println(ret);
		} else {
			Serial.print("okay[");
			Serial.print(clickCount, DEC);
			Serial.print("cnt]");
			Serial.println();
		}

		//# cocolinx-shield rtc(ms)
		Serial.print("get rtc(ms)...");
		int64_t rtcMs;
		ret = coco.sysGetRtc(&rtcMs);
		if(ret != 0) {
			Serial.print("error: ");
			Serial.println(ret);
		} else {
			Serial.print("okay[");
			Serial.print(rtcMs, DEC);
			Serial.print("ms]");
			Serial.println();
		}

		bool connected = sample_keep_connection();
		if(connected == true)
		{			
			sample_lteinfo();
			sample_datetime();
			sample_ping();
			sample_ddns();
			sample_udp();
			sample_tcp();
			sample_mqtt();
			sample_rs485();
		}

		// # cocolinx user-led
		Serial.println("user-led turn off.");
	    coco.sysSetLed(false);

		if(connected == true) Serial.print("====== test done ======");
		else Serial.print("====== test skip ======");

		millisPrev = millis();
	}
}

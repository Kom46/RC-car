typedef enum {
	ESP_Init, ESP_Ready, ESP_CommandState
} ESP_State_TypeDef;

char AT_RST[] = "AT+RST\r\n"; //reset ESP8266
char AT[] = "AT\r\n"; //check if ESP8266 is ready
char ATE0[] = "ATE0\r\n"; //Echo off
char AT_CIOBAUD[] = "AT+CIOBAUD=9600\r\n"; //set baudrate to 9600
char AT_CIPAP[] = "AT+CIPAP=\"192.168.0.30\"\r\n"; //set SoftAP IP
char AT_CWMODE_1[] = "AT+CWMODE=1\r\n"; //create Station
char AT_CWMODE_2[] = "AT+CWMODE=2\r\n"; //create SoftAP
char AT_CWMODE_3[] = "AT+CWMODE=3\r\n"; //create SoftAP+station
char AT_CWSAP[] = "AT+CWSAP=\"abc\",\"123\",1,0\r\n"; // create Wi-Fi network
char AT_CIPMUX1[] = "AT+CIPMUX=1\r\n"; // allow multiple connections
char AT_CIPSERVER[] = "AT+CIPSERVER=1,80\r\n"; // create a TCP/IP server
char AT_CWJAP[] = "AT+CWJAP=\"DIR-320NRU\",\"76543210\"\r\n";//connect to wifi AP
char AT_CIPSTA[] = "AT+CIPSTA=\"192.168.0.30\",\"192.168.0.1\",\"255.255.255.0\"\r\n"; //set station IP, gateaway and mask
char AT_CIPSEND[] = "AT+CIPSEND=";//send TCP packet

typedef enum {
	UP, Down, Hold
} Up_Down_State;

typedef enum {
	Left, Right, CNT
} Left_Right_State;

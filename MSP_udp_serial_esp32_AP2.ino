//#include <HardwareSerial.h>

//HardwareSerial Serial1(1);
//HardwareSerial Serial2(2);

#include <WiFi.h>
//#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#include <sys/fcntl.h>
#include <sys/param.h>
#include <config.h>
#include <string.h>
#include <esp_log.h>
#include <lwip/sockets.h>
#include <driver/uart.h>
#include "msp_ltm_serial.h"
#include "db_protocol.h"

//const char* ssid = "iptime_min";    //DaeYang";
//const char* password = "11111111";     //"daeyang5";

const char* ap_ssid = "DRONE_F450";
const char* ap_password = "11111111";

IPAddress local_ip(192,168,5,100);
IPAddress gateway(192,168,5,1);
IPAddress subnet(255,255,255,0);

IPAddress dest_ip(192,168,5,101);

unsigned int localUdpPort = 1607; //4210;  // local port to listen on
unsigned int remoteUdpPort = 1607; //4210;  // local port to listen on

#define MAX_LTM_FRAMES_IN_BUFFER 5
#define BUILDVERSION 11    //v0.11


#define UDP_BUF_SIZE (1024)
//#define UART_BUF_SIZE (1024)

WiFiUDP Udp;

volatile bool client_connected = true;  //false;
volatile int client_connected_num = 0;
int udp_socket = -1, serial_socket = -1;
uint8_t msp_message_buffer[MSP_PORT_INBUF_SIZE];
uint8_t udp_buffer[UDP_BUF_SIZE];

uint8_t DB_UART_PIN_TX = GPIO_NUM_17;
uint8_t DB_UART_PIN_RX = GPIO_NUM_16;
uint32_t DB_UART_BAUD_RATE = 115200;


unsigned long state_loop_timer = 0;


#define  SER_DEBUG

void parse_msp_ltm();


bool continue_udp = false;

void taskRcvSER( void * parameter )
{
  for(;;){
      parse_msp_ltm();
      continue_udp = false;
      vTaskDelay(5);    //600000/portTICK_PERIOD_MS);  //10 minutes each
  }
  vTaskDelete(NULL);
}

void taskRcvUDP( void * parameter )
{
  for(;;){
    int packetSize = Udp.parsePacket();
    if (packetSize)
    {
      if (packetSize > UDP_BUF_SIZE){
          packetSize = UDP_BUF_SIZE;
      }
      // receive incoming UDP packets
      int len = Udp.read(udp_buffer, packetSize);
      if (len > 0){
        udp_buffer[len] = 0;
      }
  #ifdef SER_DEBUG
      // receive incoming UDP packets
         Serial.printf("Recv UDP len=%d, data: ",len);
         for (int inx=0; inx < len; inx++)
            Serial.printf("%x,",udp_buffer[inx]);
         Serial.printf("\n");
  #endif
  
      if (udp_buffer[0] == '$' && (udp_buffer[1] == 'M' || udp_buffer[1] == 'X') && udp_buffer[2] == '<')
      {
  //      Serial.printf("Sending serial\n");
        uart_write_bytes(UART_NUM_2,(char*)udp_buffer,(size_t)len);
        continue_udp = true;
      }
    
    }
    vTaskDelay(5);  //600000/portTICK_PERIOD_MS);  //10 minutes each
  }
  vTaskDelete(NULL);
}


int open_serial_socket() {
    uart_config_t uart_config = {
            .baud_rate = DB_UART_BAUD_RATE,
            .data_bits = UART_DATA_8_BITS,
            .parity    = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, DB_UART_PIN_TX, DB_UART_PIN_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_2, 1024, 0, 0, NULL, 0);
    if ((serial_socket = open("/dev/uart/2", O_RDWR)) == -1) {
//        ESP_LOGE(TAG, "Cannot open UART2");
           Serial.printf("Cannot open UART2\n");
       close(serial_socket); serial_socket = -1; uart_driver_delete(UART_NUM_2);
    }
    return 1;
}
/**
 * @brief Parses & sends complete MSP & LTM messages
 */
void parse_msp_ltm(){
    uint8_t serial_byte;
    msp_ltm_port_t db_msp_ltm_port;
    bool continue_reading = true;
    uint serial_read_bytes = 0;
    
    uint8_t tmpBuf[MSP_PORT_INBUF_SIZE];
    uint tmp_len = 0;

    db_msp_ltm_port.parse_state = IDLE;

    memset(msp_message_buffer,0,sizeof(msp_message_buffer));
    
    while (continue_reading){
        if (uart_read_bytes(UART_NUM_2, &serial_byte, 1, 200 / portTICK_RATE_MS) > 0) {
//          for(int i = 0; i < tmp_len; i++){
//            serial_byte = tmpBuf[i];    //softSerial.read();
            serial_read_bytes++;
              
            if (parse_msp_ltm_byte(&db_msp_ltm_port, serial_byte)){
                msp_message_buffer[(serial_read_bytes-1)] = serial_byte;
  #ifdef SER_DEBUG
                Serial.printf("%x ",serial_byte);
  #endif
                if (db_msp_ltm_port.parse_state == MSP_PACKET_RECEIVED){
                    continue_reading = false;
 //                   if (client_connected){
   #ifdef SER_DEBUG
                        Serial.println("");
  #endif
                        // send back a reply, to the IP address and port we got the packet from
                        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
                        Udp.write((uint8_t*)msp_message_buffer, (size_t) serial_read_bytes); //&rc,sizeof(rc)); //replyPacket);
                        Udp.endPacket();
                        continue_udp = false;

  #ifdef SER_DEBUG
                        Serial.printf("send udp len=%d, data: ",serial_read_bytes);
                        for (int inx=0; inx < serial_read_bytes; inx++)
                            Serial.printf("%x,",msp_message_buffer[inx]);
                        Serial.printf("\n\n");
  #endif
                       
//                    }
                }
            } 
         
        } 
    }
 
}

void setup()
{
  Serial.begin(115200);
  Serial.println();

  Serial.print("Configuring access point...");
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAPConfig(gateway,gateway,subnet)){
    Serial.println("AP Config Failed.");
  }
  if (WiFi.softAP(ap_ssid, ap_password)){
    Serial.println("");
    IPAddress myIP = WiFi.softAPIP();
    Serial.println("ssid: " + String(ap_ssid) + ",Pass: " + String(ap_password) + " running");
    Serial.print("AP IP address: ");
    Serial.println(myIP);
  } else {
    Serial.println("Starting AP Failed.");
  }
  delay(100);

/*  while (WiFi.softAPgetStationNum()== 0)
  {
     delay(500);
     Serial.println("Wait for clien");
  }
*/
 Serial.printf("Stations connected to soft-AP = %d\n", WiFi.softAPgetStationNum());

  Udp.begin(localUdpPort);
//  Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), localUdpPort);

  #ifdef SER_DEBUG
    Serial.printf("Serial Debugging ON!\n");
  #else
    Serial.printf("Serial Debugging OFF!\n");
  #endif

    bool setup_success = 1;
    if (open_serial_socket() == ESP_FAIL) setup_success = 0;
    if (setup_success){
//        ESP_LOGI(TAG, "setup complete!");
        Serial.printf("setup complete!\n");
    }

  xTaskCreate(taskRcvUDP,          // Task function.
              "TaskUDP",        // String with name of task.
              4096,            // Stack size in words.
              NULL,             // Parameter passed as input of the task
              5,                // Priority of the task. 
              NULL);            // Task handle. 

  xTaskCreate(taskRcvSER,          // Task function.
              "TaskSER",        // String with name of task.
              4096,            // Stack size in words.
              NULL,             // Parameter passed as input of the task
              5,                // Priority of the task. 
              NULL);            // Task handle. 

}
void loop()
{
}

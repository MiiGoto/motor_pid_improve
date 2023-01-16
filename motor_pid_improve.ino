#include <MsTimer2.h>
#include <FlexCAN.h>
#include "PID.h"

typedef struct
{
  int16_t rotation;
  int16_t denryu;
} wheelEscDataSt;

FlexCAN CANTransmitter(1000000);
static CAN_message_t rxmsg;//can受信用buf
CAN_message_t msg;//can送信用buf
wheelEscDataSt wEscData[4];//can受信用

Pid pid0;
Pid pid1;
Pid pid2;
Pid pid3;

float vx, vy, vt;

void setup(void)
{
  CANTransmitter.begin();
  Serial.begin(115200);
  delay(1000);

  msg.len = 8;
  msg.id = 0x200;
  for ( int idx = 0; idx < msg.len; ++idx ) {
    msg.buf[idx] = 0;
  }

  pid0.init(3.0, 0.001, 0.03); //p,i,dの順に指定できる
  pid1.init(3.0, 0.001, 0.03);
  pid2.init(3.0, 0.001, 0.03);
  pid3.init(3.0, 0.001, 0.03);

  MsTimer2::set(2, timerInt);
  MsTimer2::start();

}
int cnt=0;

void loop(void)
{
  int u[4] = {0};
  u[0] = 500;
  u[1] = 500;
  u[2] = 500;
  u[3] = 500; //ここの数字はrpm指定、-5000~5000くらい
  
  Serial.print(u[0]);//目標速度
  Serial.print(",");
  //Serial.print(u[1]);
  //Serial.print(",");
  //Serial.print(u[2]);
  //Serial.print(",");
  //Serial.print(u[3]);
  //Serial.print(",");

  u[0] = pid0.pid_out(u[0]);
  u[1] = pid1.pid_out(u[1]);  
  u[2] = pid2.pid_out(u[2]);
  u[3] = pid3.pid_out(u[3]);
  
    u[0] = (int)(min(max(-16000, -vx - vy + vt), 16000)); 
    u[1] = (int)(min(max(-16000, -vx + vy + vt), 16000));
    u[2] = (int)(min(max(-16000, vx + vy + vt), 16000));
    u[3] = (int)(min(max(-16000, vx - vy + vt), 16000)); 
   
  Serial.println(u[0]);
  Serial.println(u[1]);
  Serial.println(u[2]);
  Serial.println(u[3]);

   for (int i = 0; i < 4; i++) {
    msg.buf[i * 2] = u[i] >> 8;
    msg.buf[i * 2 + 1] = u[i] & 0xFF;
  }
  //Serial.print(pid0.debug());//現在速度
  //Serial.print(",");
  //Serial.print(pid1.debug());
  //Serial.print(",");
  //Serial.print(pid2.debug());  
  //Serial.print(",");
  //Serial.print(pid3.debug());
  //Serial.println("");
  //delay(10);
}

void timerInt() {
  while ( CANTransmitter.read(rxmsg) ) {
    if (rxmsg.id == 0x201) {
      pid0.now_value(rxmsg.buf[2] * 256 + rxmsg.buf[3]);
    }    
    if (rxmsg.id == 0x202) {
      pid1.now_value(rxmsg.buf[2] * 256 + rxmsg.buf[3]);
    } 
    if (rxmsg.id == 0x203) {
      pid2.now_value(rxmsg.buf[2] * 256 + rxmsg.buf[3]);
    }
    if (rxmsg.id == 0x204) {
      pid3.now_value(rxmsg.buf[2] * 256 + rxmsg.buf[3]);
    }
     }
  CANTransmitter.write(msg);
}

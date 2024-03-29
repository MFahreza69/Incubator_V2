#include <ArduinoJson.h>
#include "src/library/SimpleTimer/SimpleTimer.h"
#include "LedControl.h"
#include <stm32g0xx_hal.h>

/*define*/                    /*PCB allisha*/ //    pcb dasar
#define leftAir            47   //47              //22
#define rightAir           46   //46              //23
#define setAir             45   //45             //24
#define leftHumi           11   //11             //25
#define rightHumi          12   //12             //26
#define setHumi            13   //13             //27
#define leftSkin           3   //3             //28
#define rightSkin          4   //4             //29
#define setSkin            5   //5             //30
#define setTimer           25   //25             //4
#define bypassPin          6   //6             //47
#define set_alarm          2   //2             //32
#define led_airway         50   //50             //33
#define led_skin           49   //49             //34
#define led_humi           48   //47             //35
#define led_bypass         7   //7             //48
#define err0               32   //32 //probe fail           //13
#define err1               29   //29 //temp deviation             //12
#define err2               30   //30 //over temp             //23
#define err3               28   //28 //power failure             //23
#define err4               31    //31 //fan failure             //23
//#define err7               57   //
#define lock_btn           10   //10                          //46
#define led_lock           8   //8                           //31
#define sunyiBtn           24   //24  
#define pinBuzzer          27   //8


/*seven segment display define*/
//Pin 34 = DIN                         //7
//PIN 35 = ClK                         //6
//Pin 33 = CS/LOAD                     //5
//LedControl (DIN, CLK, CS/Load, Number of IC used)
LedControl lc = LedControl(34, 35, 33, 3); 
int graphpin[] = {37, 38, 39, 40, 41, 42, 43, 44, 52, 51};                    
//pcb lama
//36, 37, 38, 39, 40, 41, 42, 43, 44, 45
/*End Define*//////////////////////////////////////////////

//display setpoint
float displaysetTemp = 27.0;
float displaysetSkin = 34.0;
float displaysetHumi = 60;

uint8_t displaysetTimer1 = 0;
uint8_t displaysetTimer2 = 0;
uint8_t Datatimer1;
uint8_t Datatimer2;

float debugAirway;
float debugSkin1;
float debugSkin2;
float debugHumi;

float holdAirway;
float holdSkin1;
float holdSkin2;
float holdHumi;
unsigned long timeDilation;
float sumAirway = 0;
float sumSkin1 = 0;
float sumSkin2 = 0;
float sumHumi = 0;

//data from sensor
float chamberTemp0 = 0;
float humidityMid = 0;
float skinTemp1 = 0;
float skinTemp2 = 0; 

//next data dr sensor d inisialisasi ke
float displayTemp = 0;
float displayHumi = 0;
float displaySkin = 0;
float displaySkin2 = 0;

//data setpoint sended to incubator
float sendTemp = 0;
float sendHumi = 0;
float sendSkin = 0;
float sendTime = 0;
uint8_t sendAlarm = 0;
uint8_t silent = 0;

//data Mode sended to incubator
int skinMode = 0;
int highTemp = 0;
int humiMode = 0;
int lockMode = 0;
int sunyiValue = 0;
int last_sunyi_value = 0;
uint8_t timeMode = 0;

//data error received from incubator
int error0 = 0;
int error1 = 0;
int error2 = 0;
int error3 = 0;
int error4 = 0;
int error5 = 0;
int error6 = 0;
int error7 = 0;
int error8 = 0;

//valuePower
static char inputData[256];
int x;
int a = 0;
float b;
int c = 0;
float d;
int e = 0;
float f;
int g = 0;
float h;
int i = 0;
float j;
unsigned long k = 0;
unsigned long l = 0;
uint8_t o;
unsigned long p;
uint8_t q = 0;
uint8_t segmentBlank = 0;

uint8_t sirenAlarm =0;
uint16_t alarmValue  = 0;
uint8_t alarmValue2 = 0;
unsigned long dpTimer;
uint8_t loopTimer;
uint16_t convert;
uint16_t converthumi;

//data button
uint8_t lastPower0;   //tombol kiri airtemp      
uint8_t lastPower1;   //tombol kanan airtemp      
uint8_t lastPower2;   //tombol kiri skintemp1      
uint8_t lastPower3;   //tombol kanan skintemp1
uint8_t lastPower4;   //tombol kiri humi      
uint8_t lastPower5;   //tombol kanan humi   
uint8_t lastPower6;   //tombol setAirTemp     
uint8_t lastPower7;   //tombol setSkinTemp    
uint8_t lastPower8;   //tombol setHumidity
uint8_t lastPower9;   //tombol hightemp  
uint8_t lastPower10;  //Lock
uint8_t lastPower11;  //Alarm
uint8_t lastPower12;  //Sunyi Btn
uint8_t lastPower13;  //set Timer

uint8_t currentPower0;   //read tombol kiri airtemp   
uint8_t currentPower1;   //read tombol kanan airtemp   
uint8_t currentPower2;   //read tombol kiri set skin temp1  
uint8_t currentPower3;   //read tombol kanan set skin temp1
uint8_t currentPower4;   //read tombol kiri set humi 
uint8_t currentPower5;   //read tombol kanan set humi
uint8_t currentPower6;   //read setAirTemp   
uint8_t currentPower7;   //read setSkinTemp   
uint8_t currentPower8;   //read setHumidy
uint8_t currentPower9;   //read hightemp
uint8_t currentPower10;  //Lock
uint8_t currentPower11;  //Alarm
uint8_t currentPower12;  //Sunyi Btn
uint8_t currentPower13;  //set timer

unsigned long incairTime;
unsigned long decairTime;
unsigned long incskinTime;
unsigned long decskinTime;
unsigned long inchumiTime;
unsigned long dechumiTime;
unsigned long lockTimer;
uint8_t timeBtn;
unsigned long debugTime;
bool debugMode = 0;
uint8_t skinSelect = 0;

uint8_t onOff;

uint8_t lastError0          = 0;
uint8_t lastError1          = 0;
uint8_t lastError2          = 0;
uint8_t lastError3          = 0;
uint8_t lastError4          = 0;
uint8_t lastError5          = 0;
uint8_t lastError6          = 0;

uint8_t setHigh = 0;
uint8_t setAirway = 0;
uint8_t setSkinMode = 0;
uint8_t setHumiMode = 0;
uint8_t setLockMode = 0;
uint8_t setAlarmMode = 0;
uint8_t setTimerMode = 0;
uint8_t holdTimer1;
uint8_t holdTimer2;
unsigned long holdtime;
uint8_t holdData = 0;

//initialize digit
uint8_t digit1 = 0;
uint8_t digit2 = 0;
uint8_t digit3 = 0;
uint8_t digit4 = 0;
uint8_t digit5 = 0;
uint8_t digit6 = 0;
uint8_t digit7 = 0;
uint8_t digit8 = 0;
uint8_t digit9 = 0;
uint8_t digit10 = 0;
uint8_t digit11 = 0;
uint8_t digit12 = 0;
uint8_t digit13 = 0;
uint8_t digit14 = 0;
uint8_t digit15 = 0;
uint8_t digit16 = 0;
uint8_t digit17 = 0;
uint8_t digit18 = 0;
uint8_t digit19 = 0;
uint8_t digit20 = 0;
uint8_t digit21 = 0;
uint8_t digit22 = 0;
uint8_t digit23 = 0;
uint8_t digit24 = 0;
uint8_t digit25 = 0;
uint8_t heaterPwm = 0;
uint8_t heatedPower = 0;
uint8_t alarm[6] = {0,0,0,0,0,0};
uint8_t sAlarm[6] = {1,1,1,1,1,1};
unsigned long lcdOn;

unsigned long lastTime3;
uint8_t loopAlarm;
bool fromInt2;

SimpleTimer timer0;
SimpleTimer timer1;
SimpleTimer timer3;
SimpleTimer timer4;


void setup() {
  // analogReference(EXTERNAL);
  Serial1.begin(9600);
 Serial.begin(9600);
  timer0.setInterval(1000, generate_json);
//  timer4.setInterval(5000, plotter);
  lc.setIntensity(0, 2);
  lc.setIntensity(1, 2);
  lc.setIntensity(2, 2);
  lc.setIntensity(3, 2);
  lc.clearDisplay(0);
  lc.clearDisplay(1);
  lc.clearDisplay(2);
  lc.clearDisplay(3);
  lc.shutdown(0, false);
  lc.shutdown(1, false);
  lc.shutdown(2, false);
  
  /*button Pin*/
  pinMode(leftAir, INPUT);
  pinMode(rightAir, INPUT);
  pinMode(setAir, INPUT);
  pinMode(leftSkin, INPUT);
  pinMode(rightSkin, INPUT);
  pinMode(setSkin, INPUT);
  pinMode(leftHumi, INPUT);
  pinMode(rightHumi, INPUT);
  pinMode(setHumi, INPUT);
  pinMode(bypassPin, INPUT);
  pinMode(setTimer, INPUT);
  pinMode(set_alarm, INPUT);
  pinMode(sunyiBtn, INPUT);
  
  /*LED Pin*/
  pinMode(led_airway, OUTPUT);
  pinMode(led_skin, OUTPUT);
  pinMode(led_humi, OUTPUT);
  pinMode(led_bypass, OUTPUT);
  pinMode(led_lock, OUTPUT);
  pinMode(err0, OUTPUT);
  pinMode(err1, OUTPUT);
  pinMode(err2, OUTPUT);
  pinMode(err3, OUTPUT);
  pinMode(err4, OUTPUT);
  pinMode(pinBuzzer, OUTPUT);

  digitalWrite(led_airway, HIGH);
  digitalWrite(led_skin, HIGH);
  digitalWrite(led_humi, HIGH);
  digitalWrite(led_bypass, HIGH);
  digitalWrite(led_lock, HIGH);

  /*led graph pin setup*/
  for(int thisled = 0; thisled < 10; thisled++){
    pinMode(graphpin[thisled], OUTPUT);
  }
  // cli();
  // EICRA = (1 << ISC21)|(1 << ISC20);
  // EIMSK = (1 << INT2);
  // sei();
}

// ISR(INT2_vect){
//     onOff = 0;
// }

void loop() {
   run_program();
}

void run_program(){ 
   if(digitalRead(20)==LOW){
    onOff++;
    if(onOff >=100){  
    error0 = 0;
    error1 = 0;
    error2 = 0;
    error3 = 0;
    error4 = 0;
    error5 = 0;
    error6 = 0;
    skinMode = 0;
    humiMode = 0;
    lockMode = 0;
    sunyiValue = 2;
    highTemp = 0;
    digit_kosong();
    lcdOn = millis();
    // sleepMode(SLEEP_POWER_DOWN);
    // sleep();
    Serial.println("sleep");
    }
    if(onOff > 100){
      onOff = 100;
    }
   }
   
   if(digitalRead(20) == HIGH){ 
    onOff = 0;
    // noSleep();
    if(millis() - lcdOn < 100){
      lc.shutdown(0, false);
      lc.shutdown(1, false);
      lc.shutdown(2, false);
    }
    getData(); 
    timer0.run();//generate_json();
    // timer4.run();
    // timer3.run();
    btn_menu();
    set_btn();
    reset_alarm();
    read_error();
    alarem();
    display_digit();
    // debug_mode();
    // Serial.print(skinSelect);
    // Serial.println(debugMode);
   }
}


/* Send and Receive Data Session*///// 
void generate_json(){
    StaticJsonDocument<512> out1;
    JsonObject DataButton = out1.createNestedObject("dt1");
    DataButton["sn"][0]   = sendTemp;
    // DataButton["sn"][1] = sendSkin;
    DataButton["sn"][1]   = sendHumi;
    DataButton["sn"][2]  = sumAirway;
    DataButton["sn"][3] = sumSkin1;
    DataButton["sn"][4] = sumSkin2;
    DataButton["sn"][5] = sumHumi;
    DataButton["mod"][0] = skinMode;
    DataButton["mod"][1] = humiMode;
    DataButton["mod"][2] = highTemp;
    DataButton["mod"][3] = alarmValue2;
    DataButton["mod"][4] = sirenAlarm;
    DataButton["mod"][5] = timeMode;
    DataButton["mod"][6] = sendAlarm;
    serializeJson(out1, Serial1);
    Serial1.println();
     serializeJson(out1, Serial);
     Serial.println();       
}

void getData(){
  while(Serial1.available()>0){
    inputData[x] = Serial1.read();
    x++;
    if(inputData[x-1] == '\n'){
      // Serial.println(inputData);
      StaticJsonDocument<512>in;
      DeserializationError error = deserializeJson(in, inputData);
      x = 0;
      if(!error){
       chamberTemp0 = in["sh"][0];
       skinTemp1    = in["sh"][1];
       skinTemp2    = in["sh"][2];
       humidityMid  = in["sh"][3];
       heaterPwm    = in["tm"][0];
       Datatimer1   = in["tm"][1];
       Datatimer2   = in["tm"][2];
       error0       = in["er"][0];
       error1       = in["er"][1];
       error2       = in["er"][2];
       error3       = in["er"][3];
       error4       = in["er"][4];
       error5       = in["er"][5];
       error6       = in["er"][6];
       return;
      }   
    }
  }
}
/* END Send and Receive Data Session*///// 


/*increment-Decrement function*/
void btn_menu(){
   lastPower0 = currentPower0;
   lastPower1 = currentPower1;
   lastPower2 = currentPower2;
   lastPower3 = currentPower3;
   lastPower4 = currentPower4;
   lastPower5 = currentPower5;
   currentPower0 = digitalRead(leftAir);
   currentPower1 = digitalRead(rightAir);
   currentPower2 = digitalRead(rightSkin);
   currentPower3 = digitalRead(leftSkin);
   currentPower4 = digitalRead(rightHumi);
   currentPower5 = digitalRead(leftHumi);
  /*kontrol set Humi*/
if(debugMode == 0){  
  if(setHumiMode == 1 && lockMode == 0){
     if(lastPower4 == HIGH && currentPower4 == LOW){
      timeBtn = 0;
        displaysetHumi = displaysetHumi + 1;
        if(displaysetHumi > 90){
           displaysetHumi = 90;
        }
     }
     if(currentPower4 == LOW){
      if(millis() - inchumiTime > 2000){
        displaysetHumi = displaysetHumi + 0.2;
        if(displaysetHumi > 90){
           displaysetHumi = 90;
        }
      }
     }
     if(currentPower4 == HIGH){
      inchumiTime = millis();
     }

     if(lastPower5 == HIGH && currentPower5 == LOW){
      timeBtn = 0;
        displaysetHumi = displaysetHumi - 1;
        if(displaysetHumi < 30){
           displaysetHumi = 30;
        }
     }
     if(currentPower5 == LOW){
      if(millis() - dechumiTime > 2000){
        displaysetHumi = displaysetHumi - 0.2;
        if(displaysetHumi < 30){
           displaysetHumi = 30;
        }        
      }
     }
     if(currentPower5 == HIGH){
      dechumiTime = millis();
     }
  }

  /*Kontrol set airtemp*/
   if(setAirway == 1 && lockMode == 0){
     /*decrement airway*////////////////////////////////////////////////// 
      if(lastPower0 == HIGH && currentPower0 == LOW){
        timeBtn = 0;
         displaysetTemp = displaysetTemp - 0.1;
         if(highTemp == 1 ){
            if(displaysetTemp <= 37){
               displaysetTemp = 37;
            }
           }
         if(highTemp == 0){
            if(displaysetTemp <= 20){
               displaysetTemp = 20;
            }
           }
        }

      if(currentPower0 == LOW){
        if(millis() - decairTime > 2000){
          displaysetTemp = displaysetTemp - 0.02;
          if(highTemp == 1 ){
            if(displaysetTemp <= 37){
               displaysetTemp = 37;
            }
          }
          if(highTemp == 0){
            if(displaysetTemp <= 20){
               displaysetTemp = 20;
            }
          }
        }
      } 
      if(currentPower0 == HIGH){
        decairTime = millis();
      }
/*end decrement airway*//////////////////////////////////////////////////

/*increment Airway*//////////////////////////////////////////////////////
      if(lastPower1 == HIGH && currentPower1 == LOW){
        timeBtn = 0;
         displaysetTemp = displaysetTemp + 0.1;
         if(highTemp == 1){
            if(displaysetTemp >= 39){
               displaysetTemp = 39;
            }
          }
         if(highTemp == 0){
            if(displaysetTemp >= 37){
               displaysetTemp = 37;
            }
         }
      }
    if(currentPower1 == LOW){
      if(millis() - incairTime > 2000){
        displaysetTemp = displaysetTemp + 0.02;
         if(highTemp == 1){
            if(displaysetTemp >= 39){
               displaysetTemp = 39;
            }
          }
         if(highTemp == 0){
            if(displaysetTemp >= 37){
               displaysetTemp = 37;
            }
         }        
      }
    }
    if(currentPower1 == HIGH){
      incairTime = millis();
    }    
    }


/*end increment airway*//////////////////////////////////////////////////

/*increment skin*////////////////////////////////////////////////////////
    if(setSkinMode == 1 && lockMode == 0){
      // segmentBlank = 1;
      /*kontrol Skin Temp*/
       if(lastPower2 == HIGH && currentPower2 == LOW){
           timeBtn = 0;
          displaysetSkin = displaysetSkin + 0.1;
          if(highTemp == 1){
             if(displaysetSkin >= 38){
                displaysetSkin = 38;
             }
           }
          if(highTemp == 0){
             if(displaysetSkin >= 37){
                displaysetSkin = 37;
             }
          }
       }
      if(currentPower2 == LOW){
        if(millis() - incskinTime > 2000){

          displaysetSkin = displaysetSkin + 0.02;
          if(highTemp == 1){
             if(displaysetSkin >= 38){
                displaysetSkin = 38;
             }
           }
          if(highTemp == 0){
             if(displaysetSkin >= 37){
                displaysetSkin = 37;
             }
          }          
        }
      }
      if(currentPower2 == HIGH){
        incskinTime = millis();
      }  

/*end increment skin*///////////////////////////////////////////////////
       if(lastPower3 == HIGH && currentPower3 == LOW){
        timeBtn = 0;
          displaysetSkin = displaysetSkin - 0.1;
          if(highTemp == 1){
             if(displaysetSkin <= 37){
                displaysetSkin = 37;
             }
          }
          if(highTemp == 0){
             if(displaysetSkin <= 34){
                displaysetSkin = 34;
             }
          }
       }       
      if(currentPower3 == LOW){
        if(millis() - decskinTime > 2000){
          displaysetSkin = displaysetSkin - 0.02;
          if(highTemp == 1){
             if(displaysetSkin <= 37){
                displaysetSkin = 37;
             }
          }
          if(highTemp == 0){
             if(displaysetSkin <= 34){
                displaysetSkin = 34;
             }
          }
        }        
      }
      if(currentPower3 == HIGH){
        decskinTime = millis();
      }
    }
  }

 //////////////////////////////////////////////////////////////////////////// 
  
 if(debugMode == 1){
  if(lastPower4 == HIGH && currentPower4 == LOW){
      timeBtn = 0;
        holdHumi = holdHumi + 1;
        if(holdHumi > 90){
           holdHumi = 90;
        }
     }
   if(lastPower5 == HIGH && currentPower5 == LOW){
      timeBtn = 0;
        holdHumi = holdHumi - 1;
        if(holdHumi < 30){
           holdHumi = 30;
        }
     }
   if(lastPower0 == HIGH && currentPower0 == LOW){
       timeBtn = 0;
        holdAirway = holdAirway - 0.1;
            if(holdAirway <= 20){
               holdAirway = 20;
            }
    }
   if(lastPower1 == HIGH && currentPower1 == LOW){
        timeBtn = 0;
        holdAirway = holdAirway + 0.1;
            if(holdAirway >= 39){
               holdAirway = 39;
            }
    }
  if(skinSelect == 0){
    if(lastPower2 == HIGH && currentPower2 == LOW){
        timeBtn = 0;
        holdSkin1 = holdSkin1 + 0.1;
             if(holdSkin1 >= 38.5){
                holdSkin1 = 38.5;
             }
    }
    if(lastPower3 == HIGH && currentPower3 == LOW){
        timeBtn = 0;
          holdSkin1 = holdSkin1 - 0.1;
             if(holdSkin1 <= 25){
                holdSkin1 = 25;
             }
    }
  }
  if(skinSelect == 1){
    if(lastPower2 == HIGH && currentPower2 == LOW){
        timeBtn = 0;
        holdSkin2 = holdSkin2 + 0.1;
             if(holdSkin2 >= 38.5){
                holdSkin2 = 38.5;
             }
    }
    if(lastPower3 == HIGH && currentPower3 == LOW){
        timeBtn = 0;
          holdSkin2 = holdSkin2 - 0.1;
             if(holdSkin2 <= 25){
                holdSkin2 = 25;
             }
    }    
  }          
 }  
}



/*Mode Set Function*/
void set_btn(){
/*Set Lock Mode*/
  lastPower6 = currentPower6;
  currentPower6 = digitalRead(setAir);
  lastPower7 = currentPower7;
  currentPower7 = digitalRead(setSkin);
  lastPower8 = currentPower8;
  currentPower8 = digitalRead(setHumi);
  lastPower9 = currentPower9;
  currentPower9 = digitalRead(bypassPin);
  lastPower10 = currentPower10;
  currentPower10 = digitalRead(lock_btn);
  lastPower13 = currentPower13;
  currentPower13 = digitalRead(setTimer);
  lastPower12 = currentPower12;
  currentPower12 = digitalRead(sunyiBtn);

if(debugMode == 0){
  if(setLockMode == 0){
    if(millis() - lockTimer > 1000){
        lockTimer = millis();
        timeBtn++;
      } 
    }
  if(lastPower10 == HIGH && currentPower10 == LOW){
     setLockMode = setLockMode + 1;
     if(setLockMode == 1){
        lockMode = 1;
        timeBtn = 60;
        lockTimer = millis();
        digitalWrite(led_lock, LOW);
      }

     if(setLockMode == 2){
      timeBtn = 0;
        lockMode = 0;
        setLockMode = 0;
        digitalWrite(led_lock, HIGH);
        if(setLockMode > 1){
           setLockMode = 0;
        }
     } 
  }
    if(timeBtn >= 60){
      setLockMode = 1;
      lockMode = 1;
      timeBtn = 60;
      lockTimer = millis();
      digitalWrite(led_lock, LOW);
  }


  if(lockMode == 0 && skinMode <= 0){
     if(lastPower9 == HIGH && currentPower9 == LOW){
      timeBtn = 0;
        setHigh = setHigh + 1 ;
        if(setHigh == 1){
           highTemp = 1;
           digitalWrite(led_bypass, LOW);
           displaysetTemp = 37;
           displaysetSkin = 37;
        }
        if(setHigh == 2){
           highTemp = 0;
           setHigh = 0;
           digitalWrite(led_bypass, HIGH);
           displaysetTemp = 27;
           displaysetSkin = 34;
        }
     }
  } 


  if(lockMode == 0){
     if(lastPower6 == HIGH && currentPower6 == LOW){
      timeBtn = 0;
        setAirway = setAirway + 1;
        if(setAirway == 1){
        // skinMode = 2;
          setSkinMode = 0;
          segmentBlank = 2;
          digitalWrite(led_skin, HIGH);
          if(setSkinMode > 0){
             setSkinMode = 0;
          }
        }
        /*Send data set airway to incubator*/
        if(setAirway == 2){
          convert = (displaysetTemp*10); 
           sendTemp = (float(convert)/10);
           setAirway = 2;
           skinMode = 2;
           segmentBlank = 2;
           setSkinMode = 0;
           digitalWrite(led_airway, LOW);
           digitalWrite(led_skin, HIGH);
           if(setSkinMode > 0){
              setSkinMode = 0;
           }
        }
        if(setAirway == 3){
           setAirway = 0;
           setSkinMode = 0;
           skinMode = 0;
           segmentBlank = 0;
           digitalWrite(led_airway, HIGH);
           if(setSkinMode > 0){
              setSkinMode = 0;
           }
        }
      } 

    if(lastPower7 == HIGH && currentPower7 == LOW){
      timeBtn = 0;
      setSkinMode = setSkinMode + 1;
      if(setSkinMode == 1){
        // skinMode = 1;
        setAirway = 0;
        segmentBlank = 1;
        digitalWrite(led_airway, HIGH);
        if(setAirway > 0){
        setAirway = 0;
        } 
      }
      /*Send data set skin to incubator*/
      if(setSkinMode == 2){
        convert = (displaysetSkin *10); 
        sendTemp = (float(convert)/10);
        setSkinMode = 2;
        setAirway = 0;
        skinMode = 1;
        segmentBlank = 1;
        digitalWrite(led_skin, LOW);
        digitalWrite(led_airway, HIGH);
        if(setAirway > 0){
          setAirway = 0;
        }
      }
      if(setSkinMode == 3){
        setSkinMode = 0;
        setAirway = 0;
        skinMode = 0;
        segmentBlank = 0;
        digitalWrite(led_skin, HIGH);
        if(setAirway > 0){
          setAirway = 0;
        }
      } 
    }

      /*Set Humi Mode*/
    if(lastPower8 == HIGH && currentPower8 == LOW){
      timeBtn = 0;
       setHumiMode = setHumiMode + 1;
       if(setHumiMode == 1){}
            /*Send data set humidity to incubator*/
       if(setHumiMode == 2){
          converthumi = (displaysetHumi*10);
          sendHumi = (float(converthumi)/10);
          digitalWrite(led_humi, LOW);
          humiMode = 1;
       } 
       if(setHumiMode == 3){
          setHumiMode = 0;
          humiMode = 0;
          sendHumi = 0;
          digitalWrite(led_humi, HIGH);
       }
     }
   }
}
    /* sunyi button */
      if(lastPower12 == HIGH && currentPower12 == LOW){
        timeBtn = 0;
        // sunyiValue = 1;
        k = millis();
        l = millis();
        if(millis() - k < 60000){
          sunyiValue = 1;
        }
        if(millis() - h > 350 && i == 0){
          last_sunyi_value = 1;
          i = 1;
          h = millis();
        }
      }
        if(millis() - h > 350 && i == 1){
            last_sunyi_value = 0;
            i = 0;
            h = millis();
        }
        if(millis() - k > 60000){
          sunyiValue = 0;
            k = millis();
        }
   



if(debugMode == 1){
  if(lastPower7 == HIGH && currentPower7 == LOW){
      timeBtn = 0;
      skinSelect++;
      if(skinSelect >= 2){
        skinSelect = 0;
      } 
  }
 
}   
    /*set Timer BTN*/
    if(lastPower13 == HIGH && currentPower13 == LOW){
      timeBtn = 0;
       setTimerMode++;
       if(setTimerMode == 1){
         timeMode = 1;
       }
       if(setTimerMode == 2){
         timeMode = 2;
         holdTimer1 = displaysetTimer1;
         holdTimer2 = displaysetTimer2;
         setTimerMode = 3;
       }
       if(setTimerMode == 3){
         timeMode = 3;
       }
       if(setTimerMode == 4){
        setTimerMode = 0;
        timeMode = 0;
       }
    }

    if(currentPower13 == LOW){
      if(millis() - debugTime > 7000 && debugMode == 0){
        // if(millis() - timeDilation < 3){
          holdAirway = chamberTemp0;
          holdHumi = humidityMid;
          holdSkin1 = skinTemp1;
          holdSkin2 = skinTemp2;
          Serial.println("--------------------------");
          debugMode = 1;
          timeBtn = 0;
          timeMode = 0;
          debugTime = millis();
      // }
      }
      else if(millis() - debugTime > 7000 && debugMode == 1){
        sumAirway = holdAirway - chamberTemp0;
        sumSkin1 = holdSkin1 - skinTemp1;
        sumSkin2 = holdSkin2 - skinTemp2;
        sumHumi = holdHumi - humidityMid;        
        timeBtn = 0;
        timeMode = 0;
        debugMode = 0;
        debugTime = millis();
      }
    }
    if(currentPower13 == HIGH){
      debugTime = millis();
    }


   
      /*LED Blinking while setpoint changed*/
        if(setSkinMode == 0 && setAirway == 1){
          if(millis() - b > 500 && a == 0) {
            digitalWrite(led_airway, HIGH);
            b = millis();
            a = 1;
          }
          if(millis() - b > 500 && a == 1) {
            digitalWrite(led_airway, LOW);
            b = millis();
            a = 0;
          }   
        } 

        if(setSkinMode == 1 && setAirway == 0){
          if(millis() - d > 500 && c == 0) {
            digitalWrite(led_skin, HIGH);
            d = millis();
            c = 1;
          }
          if(millis() - d > 500 && c == 1) {
            digitalWrite(led_skin, LOW);
            d = millis();
            c = 0;
          }   
        } 

        if(setHumiMode == 1 && humiMode == 0){
          if(millis() - f > 500 && e == 0) {
            digitalWrite(led_humi, HIGH);
            f = millis();
            e = 1;
          }
          if(millis() - f > 500 && e == 1) {
            digitalWrite(led_humi, LOW);
            f = millis();
            e = 0;
          }   
        }
}
 
void reset_alarm(){
   /*Set Alarm Mode*/
    lastPower11 = currentPower11;
    currentPower11 = digitalRead(set_alarm);
    if(lastPower11 == HIGH && currentPower11 == LOW){
      timeBtn = 0;
      //  setAlarmMode = setAlarmMode + 1;
      p = millis();
      //  if(setAlarmMode == 1){       
        if(millis() - p < 20000){
          sendAlarm = 1;
          // q = 1;
          // p = millis();
          // Serial.print("stop");
        }       
    }
        if(millis() - p > 20000){
          sendAlarm = 0;
          // q = 0;
          // p = millis();
          // Serial.print("stop2");
          // setAlarmMode = 0;
        }         
} 
  
void read_error(){
  /*Triggering lamp*/
  /*probe missing*/  
  if(error0 == 1 ){
    digitalWrite(err0, LOW);
  }

  if(error0 == 0){
    digitalWrite(err0, HIGH);
    sAlarm[0] = 1;
  }

  /*temp deviation*/
  if(error1 == 1 || error2 == 1){
    digitalWrite(err1, LOW);
  }
  if(error1 == 0 && error2 == 0){
    digitalWrite(err1, HIGH);
    sAlarm[1] = 1;
    sAlarm[2] = 1;
  }

  /*high temp alarm*/
  if(error3 == 1 || error4 == 1){
    digitalWrite(err2, LOW);
  }
  if(error3 == 0 && error4 == 0){
    digitalWrite(err2, HIGH);
    sAlarm[3] = 1;
    sAlarm[4] = 1;
  }

  /*Power Failure*/
  if(error5 == 1){
    digitalWrite(err3, LOW);
  }
  if(error5 == 0){
    digitalWrite(err3, HIGH);
    sAlarm[5] = 1;
  }

  /*fan failure*/
  if(error6 == 1){
    digitalWrite(err4, LOW);
  }
  if(error6 == 0){
    digitalWrite(err4, HIGH);
    sAlarm[6] = 1;
  }
  /*end Triggering lamp*/


/*Silent Alarm Function*/
  //cek kondisi alarm
  if(error0 == 1 && sAlarm[0] == 1){
    alarm[0] = 1;
  }
  if(error1 == 1 && sAlarm[1] == 1 ){
    alarm[1] = 1;
  }
  if(error2 == 1 && sAlarm[2] == 1){
    alarm[2] = 1;
  }
  if(error3 == 1 && sAlarm[3] == 1){
    alarm[3] = 1;
  }
  if(error4 == 1 && sAlarm[4] == 1){
    alarm[4] = 1;
  }
  if(error5 == 1 && sAlarm[5] == 1){
    alarm[5] = 1;
  }
  if(error6 == 1 && sAlarm[6] == 1){
    alarm[6] = 1;
  }

//trigger off button
  if(alarm[0] == 1 && last_sunyi_value == 1){
    sAlarm[0] = 0;
    alarm[0] = 0;
  }
  if(alarm[1] == 1 && last_sunyi_value == 1){
    sAlarm[1] = 0;
    alarm[1] = 0;
  }  
  if(alarm[2] == 1 && last_sunyi_value == 1){
    sAlarm[2] = 0;
    alarm[2] = 0;
  }
  if(alarm[3] == 1 && last_sunyi_value == 1){
    sAlarm[3] = 0;
    alarm[3] = 0;
  }  
  if(alarm[4] == 1 && last_sunyi_value == 1){
    sAlarm[4] = 0;
    alarm[4] = 0;
  }
  if(alarm[5] == 1 && last_sunyi_value == 1){
    sAlarm[5] = 0;
    alarm[5] = 0;
  }
 if(alarm[6] == 1 && last_sunyi_value == 1){
    sAlarm[6] = 0;
    alarm[6] = 0;
  }  
// check error again after button silent triggering
  for(o=0; o<7; o++){
    if(alarm[o] == 1){
      sirenAlarm = 1;
      // last_sunyi_value == 0;
      alarmValue++;
      if(alarmValue >= 6000){
        alarmValue = 6000;
        alarmValue2 = 1;
      }
    }
    if(last_sunyi_value == 1 ){
        sirenAlarm = 0;
        alarmValue = 0;
        alarmValue2 = 0;
    }

    if(sunyiValue == 0){
      for(uint8_t j=0; j<7; j++){
        sAlarm[j] = 1;
      }
    }

    if(error0 == 0 && error1 == 0 && error2 ==0 && error3 == 0 && error4 == 0 && error5 == 0 && error6 == 0){
      alarmValue = 0;
      alarmValue2 = 0;
      sirenAlarm = 0;    
    }
  }


  /*fan Failure*/
}

void display_digit(){
  nilaidigit();
   /*Display Airway Temp*/            /*PCB Allisha*/
  lc.setDigit(1, 2, digit4, false);     //(1,2)
  lc.setDigit(1, 1, digit5, true);      //(1,1)
  lc.setDigit(1, 5, digit6, false);     //(1,5)
  
  /*Display Skin Temp*/
  lc.setDigit(1, 0, digit10, false);    //(1,0)
  lc.setDigit(1, 4, digit11, true);     //(1,4)
  lc.setDigit(1, 6, digit12, false);    //(1,6)
  
  /*Display SKin 2 */
  lc.setDigit(0, 1, digit13, false);  //sdh di set  
  lc.setDigit(0, 5, digit15, false);     
  lc.setDigit(0, 7, digit14, true);    
 
   /*Display Humidity*/
  lc.setDigit(1, 3, digit16, false);    //(1,3) 
  lc.setDigit(1, 7, digit17, false);    //(1,7)
  /*set Humidity*/ 
  lc.setDigit(2, 3, digit18, false);    //(2,3)
  lc.setDigit(2, 2, digit19, false);    //(2,2)

  /*Display Led Bar Graph*/
  heatedPower = map(heaterPwm, 0, 255, 0, 10);
    for(int thisled = 0; thisled < 10; thisled++){
      if(thisled < heatedPower){
        digitalWrite(graphpin[thisled], LOW);
      }
      else{
        digitalWrite(graphpin[thisled], HIGH);
      }
    }


  /*Display Airway & Skin set Temp*/
  if(segmentBlank == 0){
    /*Airway set*/                      /*PCB ALlisha*/
    lc.setDigit(2, 0, digit1, false);     //(2,0)
    lc.setDigit(2, 4, digit2, true);      //(2,4)
    lc.setDigit(2, 6, digit3, false);     //(2,6)

    /*Skin Set*/
    lc.setDigit(2, 7, digit7, false);     //(2,7)
    lc.setDigit(2, 5, digit8, true);      //(2,5)
    lc.setDigit(2, 1, digit9, false);     //(2,1)
  }   

  if(segmentBlank == 1){
    /*not showing airtemp number when skinmode =1*/
    lc.setChar(2, 0, '-', false);         //(2,0)
    lc.setChar(2, 4, '-', false);         //(2,4)
    lc.setChar(2, 6, '-', false);         //(2,6)
    lc.setDigit(2, 7, digit7, false);     //(2,7)
    lc.setDigit(2, 5, digit8, true);      //(2,5)
    lc.setDigit(2, 1, digit9, false);     //(2,1)
    /*end*/
  }

  if(segmentBlank == 2){
    /*not showing skintemp number when skinmode =2*/ 
    lc.setChar(2, 7, '-', false);         //(2,0)
    lc.setChar(2, 5, '-', false);         //(2,4)
    lc.setChar(2, 1, '-', false);         //(2,6)
    lc.setDigit(2, 0, digit1, false);     //(2,7)
    lc.setDigit(2, 4, digit2, true);      //(2,5)
    lc.setDigit(2, 6, digit3, false);     //(2,1)
  }
}

void digit_kosong(){
  lc.shutdown(0, true);
  lc.shutdown(1, true);
  lc.shutdown(2, true);
  nilaidigit();

    /*Display Airway Temp*/
  lc.setChar(2, 0, 'blank', false);
  lc.setChar(2, 1, 'blank', true);
  lc.setChar(2, 2, 'blank', false);

  lc.setChar(2, 3, 'blank', false);
  lc.setChar(2, 4, 'blank', true);
  lc.setChar(2, 5, 'blank', false);

  /*Display Skin Temp*/
  lc.setChar(1, 3, 'blank', false);
  lc.setChar(1, 4, 'blank', true);
  lc.setChar(1, 5, 'blank', false);

  lc.setChar(1, 1, 'blank', false);
  lc.setChar(1, 7, 'blank', true);
  lc.setChar(1, 6, 'blank', false);
   
   /*Display Humidity*/
  lc.setChar(0, 0, 'blank', false);
  lc.setChar(0, 1, 'blank', false);
  lc.setChar(0, 2, 'blank', false);
  lc.setChar(0, 3, 'blank', false);

  /*Display Timer*/
  lc.setChar(0, 6, 'blank', false);      
  lc.setChar(0, 2, 'blank', false);    
  lc.setChar(0, 4, 'blank', false);  
  lc.setChar(0, 0, 'blank', false);  
}

void nilaidigit() {
  if(debugMode == 0){
   displayTemp = chamberTemp0; 
  //  + (sumAirway);
   displaySkin = skinTemp1; 
  //  + (sumSkin1);
   displaySkin2 = skinTemp2; 
  //  + (sumSkin2);
   displayHumi = humidityMid;
   displaysetTimer1 = Datatimer1;
   displaysetTimer2 = Datatimer2;

    if(timeMode == 0){
      digit20 = 0;
      digit21 = 0;
      digit22 = 0;                          /*PCB ALLISHA*/
      digit23 = 0;    
      lc.setChar(0, 2, '-', false);        //(0,2) 
      lc.setChar(0, 6, '-', false);        //(0,6) 
      lc.setChar(0, 4, '-', false);        //(0,4)
      lc.setChar(0, 0, '-', false);        //(0,0)
    }
    if(timeMode == 1){
      lc.setDigit(0, 2, digit20, false);   //(0,2)
      lc.setDigit(0, 6, digit21, false);   //(0,6)
      lc.setDigit(0, 4, digit22, true);   //(0,4) 
      lc.setDigit(0, 0, digit23, true);   //(0,0)
      digit20 = displaysetTimer1/10;
      digit22 = displaysetTimer2/10;
        if(displaysetTimer2 < 10){
          digit23 = displaysetTimer2;
        }  
        if(displaysetTimer2 >= 10){
          digit23 = (displaysetTimer2)-((displaysetTimer2/10)*(10));  
        }
        if(displaysetTimer1 < 10){
          digit21 = displaysetTimer1;
        }
        if(displaysetTimer1 >= 10){
          digit21 = (displaysetTimer1)-((displaysetTimer1/10)*(10));
        }
    }

    if(timeMode == 3){
      displaysetTimer1 = holdTimer1;
      displaysetTimer2 = holdTimer2;
      if(millis() - dpTimer > 500 && loopTimer == 0){
        lc.setDigit(0, 2, digit20, false);   //(0,2)
        lc.setDigit(0, 6, digit21, false);   //(0,6)
        lc.setDigit(0, 4, digit22, true);   //(0,4) 
        lc.setDigit(0, 0, digit23, true);   //(0,0)
        dpTimer = millis();
        loopTimer = 1;      
      }
      if(millis() - dpTimer > 500 && loopTimer == 1){
        lc.setChar(0, 2, 'blank', false);   //(0,2)
        lc.setChar(0, 6, 'blank', false);   //(0,6)
        lc.setChar(0, 4, 'blank', true);   //(0,4) 
        lc.setChar(0, 0, 'blank', true);   //(0,0)
        dpTimer = millis();
        loopTimer = 0;  
      }
    }
  }

  if(debugMode == 1){
    displayTemp = holdAirway;
    displaySkin = holdSkin1;
    displaySkin2 = holdSkin2;
    displayHumi = holdHumi;
    sumAirway = 0;
    sumSkin1 = 0;
    sumSkin2 = 0;
    sumHumi = 0;
    lc.setChar(0, 2, 'c', false);   //(0,2)
    lc.setChar(0, 6, 'a', false);   //(0,6)
    lc.setChar(0, 4, 'l', false);   //(0,4) 
    lc.setChar(0, 0, 'blank', false);   //(0,0)
  }

  /*airway*/
  digit1 = displaysetTemp / 10;
  digit2 = displaysetTemp - (digit1 * 10);
  digit3 = (displaysetTemp * 10 - (digit1 * 100)) - (digit2 * 10);
  digit4 = displayTemp / 10;
  digit5 = displayTemp - (digit4 * 10);
  digit6 = ((displayTemp * 10) - (digit4 * 100)) - (digit5 * 10);
  
  /*skin*/
  digit7 = displaysetSkin / 10;
  digit8 = displaysetSkin - (digit7 * 10);
  digit9 = (displaysetSkin * 10 - (digit7 * 100)) - (digit8 * 10);
  digit10 = displaySkin / 10;
  digit11 = displaySkin - (digit10 * 10);
  digit12 = ((displaySkin * 10) - (digit10 * 100)) - (digit11 * 10);
  digit13 = displaySkin2 / 10;
  digit14 = displaySkin2 - (digit13 * 10);
  digit15 = (displaySkin2 * 10 - (digit13 * 100)) - (digit14 * 10);
  
  /*humidity*/
  digit16 = displayHumi / 10;
  digit17 = displayHumi - (digit16 * 10);
  digit18 = displaysetHumi / 10;
  digit19 = displaysetHumi - (digit18 * 10);


  /*Timer*/

}

void plotter(){
    Serial.print("DATA, TIME, TIMER,");
    Serial.print(skinTemp1);
    Serial.print(",");
    Serial.print(chamberTemp0);
    Serial.print(",");
    Serial.print(humidityMid);
    Serial.print(",");
    Serial.print(sendHumi);
    Serial.print(",");          
    Serial.print(sendTemp);
    Serial.print(",");
    Serial.print(heaterPwm);
    Serial.println();
}

void alarem(){
  if(sirenAlarm == 1){
    if(alarmValue2 == 0){
      // Serial.println("alarm1");
      if(millis() - lastTime3 > 1000 && loopAlarm == 0){
        lastTime3 = millis();
        loopAlarm = 1;
        tone(pinBuzzer, 2200);
      }
      if(millis() - lastTime3 > 1000 && loopAlarm == 1){
        lastTime3 = millis();
        loopAlarm = 0;
        noTone(pinBuzzer);
      }
    }
    if(alarmValue2 == 1){
      // Serial.println("alarm2");
      if(millis() - lastTime3 > 1000 && loopAlarm == 0){
        lastTime3 = millis();
        loopAlarm = 1;
        tone(pinBuzzer, 2200);
      }
      if(millis() - lastTime3 > 1000 && loopAlarm == 1){
        lastTime3 = millis();
        loopAlarm = 0;
        tone(pinBuzzer, 1800);
      }      
    }
  }
  else{
    noTone(pinBuzzer);
  }
}

//tes
#include "xbaby.h"
#include <Wire.h>
#include <RTClib.h>
#include <ArduinoJson.h>
#include <PID_v1.h>
#include "src/library/SHT15/SHT1X.h"
#include "src/library/SimpleTimer/SimpleTimer.h"
#include "src/library/O2control/XMV20_O2Sensor.h"
#include <SoftwareSerial.h>

/*
    Define pins
*/
/*                 | Select Pin  | PCB NEW ISOLASI   |   Atmega64 |*/
#define shtData         24          /*24*/            /*24*/
#define shtClock        25          /*25*/            /*25*/
#define waterPin        39           /*39*/            /*14*/
#define warmerPin       3            /*3*/ /*auxiliary 13*/
#define heaterPin       4            /*4*/           /*4*/
#define fanPin          5            /*5*/           /*3*/
#define lampPin         13
#define pinBuzzer       16
#define pinSpeaker      14      
#define O2Press         48
// #define pinOnOff     A4          /*A10*/          /*A4*/
#define voltage5V       2//POWRIN          /*2*/          /*A5*/
// #define pinLamp      A7                          /*A7*/
#define pinRst          33          /**/ 
#define dataIn          22          /*22*/
#define clkOut          23          /*23*/
#define relayHeat       36
#define relayWarm       34
#define relayLamp       35
#define pulse_ip        49
#define vbat            50
#define green           43
#define red             42
#define yellow          44
#define valvePin        15
#define o2sensor        47

/*
    Define variables
*/
/*sensor pin*/
float babySkinTemp0         = 0;
float babySkinTemp1         = 0;
float chamberTemp0          = 0;
float humidityMid           = 0;
uint8_t outWarmer;
unsigned long lastTime0     = 0;
unsigned long lastTime1     = 0;
unsigned long lastTime2     = 0;
/* lockButton */
int lock                    = 1;
unsigned long last_time     = 0;
unsigned long last_time1    = 0;
unsigned long tcal          = 0;
//
/*variable data*/
int a                       = 0;
unsigned long b;
uint8_t G                   = 0;
uint8_t B                   = 0;
uint8_t fanPwm              = 0;
uint8_t heaterPwm           = 0;
uint8_t warmerPwm           = 0;
uint8_t valvePwm            = 0;
uint8_t skinMode            = 0; //skinmode =1 = BabyMode || skinmode =2 = Airmode
uint8_t menuMode            = 0;
uint8_t humiMode            = 0;
uint8_t highTemp            = 0;
uint8_t alarmValue;
uint8_t sunyiValue          = 0;
uint8_t last_sunyi_value;
uint8_t ring;
uint8_t silent              = 0;
uint8_t error0              = 0; 
uint8_t error1              = 0; 
uint8_t error2              = 0; 
uint8_t error3              = 0; 
uint8_t error4              = 0; 
uint8_t error5              = 0; 
uint8_t error6              = 0; 
// uint8_t last_sunyi_value    = 0;
float displaySetTemp        = 27;
float displayBabyTemp0      = 0;
float displayBabyTemp1      = 0;
uint16_t convertskin0;
uint16_t convertskin1;
float lastDisplay           = 0;
float lastHumidity          = 0;
float setTemp               = 0;
float setTemp1              = 0;
uint8_t setHumidity         = 0;
//default A= -0.1854, B= 58.551
float setValue0A            = 0.022/*0.3027*/;
float setValue0B            = 21.023/*-214.88*/;
float setValue1A            = 0.022/*0.1154*/;
float setValue1B            = 21.023/*-32.134*/;
float errorHumidity         = 0;
unsigned long lastTime3     = 0;
uint8_t loopAlarm           = 0;
uint8_t suhu                = 0;
uint8_t sensor              = 0;
uint8_t heatedPower         = 0;
uint8_t powerIn             = 0;
uint8_t waterIn             = 0;       

//Alarm variable data
float deviationAir = 0;
float deviationSkin0 = 0;
float deviationSkin1 = 0;
uint8_t steadytime0 = 0;
uint8_t steadytime1 = 0;
float errorAir;
float errorSkin0;
float errorSkin1;
uint8_t alarmRst = 0;
unsigned int DO = 1800;

//timer Variable data
uint8_t timeMode = 0;
unsigned long startup = 0;

static char userInput[255];
static unsigned char x;
static unsigned char x2;
int sampleData[20];
int sampleData1[20];

int g = 0;
float h;
int i = 0;

double setPoint1; 
double input;
double outputHeater;
double outputFan;
uint8_t oxygen;
uint8_t halfBit;
uint8_t fullBit;
int dataArray[20];
unsigned long timePulse;
unsigned long generateData;
uint8_t highState = 0;
uint8_t sampleState = 0;
int dataSensor1;
int dataSensor2;
uint8_t pulsa = 0;
uint8_t start = 1;

int vbatsense;
bool valuePower = 1;


PID myPID(&input, &outputHeater, &setPoint1, 40.68 , 0.23, 0 ,DIRECT);
PID myPID2(&input, &outputFan, &setPoint1, 2, 5, 1, DIRECT);
PID myPID3(&input, &outputHeater, &setPoint1, 32.02, 0.23, 0, DIRECT);
Xbaby Xinfant;
RTC_DS3231 rtc;
SHT1x sht15(shtData, shtClock);
SoftwareSerial mySerial(6, 7); // RX, TX // Only Use RX
O2Sensor O2SensorDev(o2sensor);
SimpleTimer timer0;
SimpleTimer timer1;
SimpleTimer timer3;
SimpleTimer timer4;
SimpleTimer timer5;

void pin_setup(){
  pinMode(12, OUTPUT);
  pinMode(lampPin, OUTPUT);
  pinMode(heaterPin, OUTPUT);
  pinMode(fanPin, OUTPUT);
  pinMode(warmerPin, OUTPUT);
  pinMode(relayHeat, OUTPUT);
  pinMode(relayWarm, OUTPUT);
  pinMode(relayLamp, OUTPUT);
  pinMode(pinRst, OUTPUT);
  pinMode(pinBuzzer, OUTPUT);
  pinMode(pinSpeaker, OUTPUT);
  pinMode(voltage5V, INPUT);
  pinMode(dataIn, INPUT);
  pinMode(clkOut, OUTPUT);
  pinMode(pulse_ip,INPUT);
  pinMode(vbat, INPUT);
  pinMode(red, OUTPUT);
  pinMode(green, OUTPUT);
  pinMode(yellow, OUTPUT);
  pinMode(valvePin, OUTPUT);
  pinMode(o2sensor, INPUT);
}

void setup(){
    analogReference(EXTERNAL);
    mySerial.begin(9600);
//    Serial1.begin(9600);
    rtc.begin();
    Serial.begin(9600);
    pin_setup();
    timer0.setInterval(1000, generate_json); 
    timer3.setInterval(1000, PID);
    timer4.setInterval(1200, read_temperature);
    myPID.SetOutputLimits(40,215);
    myPID.SetMode(AUTOMATIC);
    myPID2.SetOutputLimits(100, 130);
    myPID2.SetMode(AUTOMATIC);
    myPID3.SetOutputLimits(64, 180);
    myPID3.SetMode(AUTOMATIC);
    digitalWrite(relayHeat, LOW);
    digitalWrite(relayWarm, LOW);
    digitalWrite(relayLamp, LOW);  
    digitalWrite(pinBuzzer, LOW);
    digitalWrite(pinSpeaker, LOW);  
    digitalWrite(red, LOW);
    digitalWrite(yellow, LOW);
    digitalWrite(green, LOW);
    analogWrite(valvePin, 0);
    analogWrite(warmerPin, 0);
    pulsa = 0;
    start = 0;
    delay(2000);
    // cli();
    // // Enable Watch Dog Timer
    // WDTCSR = (1 << WDCE) | (1 << WDE);
    // WDTCSR = (1 << WDIE) | (1 << WDP2) | (0 << WDP1) | (1 << WDP0);
    // sei();    
}
////////////////////////////////////////////////////////////////////////////////////////

void loop(){
    run_program();
}

void run_program(){
    vbat_read();
    if(valuePower == 1){
      generate_pulse();
      sample_data();
      communication_serial();
      timer0.run();
      timer3.run();
      timer4.run();
      // timer5.run();
      digitalWrite(12, LOW); 
      read_skin_temperature();
      read_error();
      run_warmer();
      // alarem();
      pewaktu();
      digitalWrite(7, HIGH);
    }
    if(valuePower == 0){
      digitalWrite(7, LOW);
    }
 }


/*Communication Data*/////////////////////////////////////////////////////////////////////////////////////////
//Read data com input using mySerial (RX pin 6)
void communication_serial(){
    while(mySerial.available()>0){
        userInput[x2] = mySerial.read();
        x2++;
        if(userInput[x2-1] == '\n'){
            // Serial.println(userInput); // check the data if it was received or not
            StaticJsonDocument<512>in;
            DeserializationError error = deserializeJson(in, userInput);
            if(!error){
                setTemp     = in["dt1"]["sn"][0];
                setHumidity = in["dt1"]["sn"][1];
                skinMode    = in["dt1"]["mod"][0];
                humiMode    = in["dt1"]["mod"][1];
                highTemp    = in["dt1"]["mod"][2];
                alarmValue  = in["dt1"]["mod"][3];
                sunyiValue  = in["dt1"]["mod"][4];
                timeMode    = in["dt1"]["mod"][5];
                alarmRst    = in["dt1"]["mod"][6];
            }
            x2 = 0;
        }
    }                
}

//Send data using Serial 1 (TX pin 21/PD3)
void generate_json(){
   DateTime now = rtc.now();
   StaticJsonDocument<512> outDbg;
        outDbg["sh"][0] = chamberTemp0;
        outDbg["sh"][1] = babySkinTemp0;
        outDbg["sh"][2] = babySkinTemp1;
        outDbg["sh"][3] = humidityMid;
        // outDbg["pw"][1] = round(outputFan);
        outDbg["tm"][0] = round(outputHeater);
         outDbg["tm"][1] = now.hour();
         outDbg["tm"][2] = now.minute();
//         outDbg["tim"][2] = now.second();
        outDbg["er"][0] = error0; //probe missing
        outDbg["er"][1] = error1; // alarm deviation airway
        outDbg["er"][2] = error2; // alarm deviation skin
        outDbg["er"][3] = error3;
        outDbg["er"][4] = error4;
        outDbg["er"][5] = error5;
        outDbg["er"][6] = error6;
        serializeJson(outDbg, mySerial);
        mySerial.println();
//          serializeJson(outDbg, Serial);
//          Serial.println();    
}
/*END Communication Data*/////////////////////////////////////////////////////////////////////////////////////////


/*Read Skin, Airway, HUMidi*/
void read_temperature(){
    // alarem();
    float read_sht_temperature = 0;
    float read_sht_humidity = 0;
    read_sht_temperature = sht15.readTemperatureC();
    read_sht_humidity = sht15.readHumidity();
    if(read_sht_temperature < 1){
        chamberTemp0 = 0;
    }
    if(read_sht_temperature > 1){
        chamberTemp0 = read_sht_temperature;
    }
    if(read_sht_humidity < 1){
        humidityMid = 0;
    }
    if(read_sht_humidity > 1){
        humidityMid = read_sht_humidity;
    }
//    Serial.print(chamberTemp0);
//    Serial.print("-");
//    Serial.println(humidityMid);
}
/*end read sensor*/

void read_skin_temperature(){
    float read_skin_temperature0 = Xinfant.get_value_baby_skin(setValue0A, setValue0B, dataSensor1);
    float read_skin_temperature1 = Xinfant.get_value_baby_skin(setValue1A, setValue1B, dataSensor2);
    if(read_skin_temperature0 > 21){
        convertskin0 = (read_skin_temperature0*100);
        babySkinTemp0 = (float(convertskin0)/100);
        if(millis() - tcal > 1000){
            displayBabyTemp0 = babySkinTemp0;
        }
    }
    if(read_skin_temperature0 < 22){
        babySkinTemp0 = 0;
        displayBabyTemp0 = babySkinTemp0;
    }
    if(read_skin_temperature1 > 21){
        convertskin1 = (read_skin_temperature1*100);
        babySkinTemp1 = (float(convertskin1)/100);
        if(millis() - tcal > 1000){
            displayBabyTemp1 = babySkinTemp1;
        }
    }
    if(read_skin_temperature1 < 22){
        babySkinTemp1 = 0;
        displayBabyTemp1 = babySkinTemp1;
    }
}

/*Control*/
void run_control(){
    fanPwm = Xinfant.get_value_fan(setTemp, skinMode, chamberTemp0, babySkinTemp0, babySkinTemp1, setHumidity, humidityMid);
    heaterPwm = Xinfant.get_value_heater(setTemp, skinMode , highTemp, chamberTemp0, babySkinTemp0, babySkinTemp1);
    set_pwm(fanPwm, heaterPwm);
        if(skinMode == 0 && humiMode == 0){
        set_pwm(0, 0);
        analogWrite(warmerPin, 0);
    }
}

void setup_fast_pwm(){
//     TCCR3A = (1<<COM3B1)|(1<<COM3B0)|(1<<COM3A1)|(0<<COM3A0)|(1<<WGM32)|(1<<WGM31)|(1<<WGM30); //10bit
    // TCCR3A = (1<<COM3B1)|(1<<COM3B0)|(1<<COM3A1)|(0<<COM3A0)|(1<<WGM32)|(0<<WGM31)|(1<<WGM30); //8bit
    TCCR3B = (1<<CS32)|(0<<CS31)|(1<<CS30); //1024 Prescaler
    // TCCR3B = (1<<CS32)|(0<<CS31)|(0<<CS30); //256 Prescaler
    // TCCR3B = (0<<CS32)|(1<<CS31)|(1<<CS30); //64 Prescaler
//     OCR3A = 0;
//     OCR3B = 1023;
    OCR3B = 255;
}

void set_pwm(int fan, int heater){
    // OCR3A = heater;
    // OCR3B = fan;
    // OCR3B = outputFan;
    analogWrite(heaterPin, heater);
    analogWrite(fanPin, fan);
}

void run_warmer(){
     errorHumidity = (setHumidity * 10) - (humidityMid * 10);
   if(error0 == 0 && error1 == 0 && error2 == 0 && error3 == 0 && error4 == 0 && error5 == 0){ 
     if(humiMode == 1){
       digitalWrite(relayWarm, HIGH);
        if(humidityMid < 0){
            analogWrite(warmerPin, 0);
            warmerPwm = 0;
        }else {
            if(errorHumidity < -0.5 && errorHumidity > -95){
                analogWrite(warmerPin, 0);
                warmerPwm = 0;
            }  
            if(errorHumidity < 20.5 && errorHumidity >= -0.5 ){
                analogWrite(warmerPin, 200);
                warmerPwm = 200;
            }
            if(errorHumidity < 950 && errorHumidity >= 20.5){
                analogWrite(warmerPin, 255); //255
                warmerPwm = 255;
            }
        }
    }
    if(humiMode == 0){
     analogWrite(warmerPin, 0);
     warmerPwm = 0;
      digitalWrite(relayWarm, LOW);
   }
  }
  else{
     analogWrite(warmerPin, 0);
     warmerPwm = 0;
     digitalWrite(relayWarm, LOW);
    }
   
}
/*End Control*/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void pewaktu(){
    if(timeMode == 0){
        rtc.adjust(DateTime(0000,0,00,00,00,1)); 
    }    
}


/*Error Session*/
void read_error(){
  if(millis() - startup > 5000){
    errorAir = (setTemp * 10) - (chamberTemp0 * 10);
    errorSkin0 = (setTemp * 10) - (babySkinTemp0 * 10);
    errorSkin1 = (setTemp * 10) - (babySkinTemp1 * 10);
  if(alarmRst == 0){    
    /*probe sensor missing*///////////////////////
    if(babySkinTemp0 == 0 || babySkinTemp1 == 0 || chamberTemp0 == 0){
        error0 = 1;
    }
    if(babySkinTemp0 != 0 && chamberTemp0 != 0 && babySkinTemp1 != 0){
        error0 = 0;
    }
    /*end probe sensor missing*/////////////////////


    /*Temp Deviation and High Temp Alarm*////////////////////////
    if(skinMode == 1){
        steadytime1 = 0;
        error2 = 0;
        if(errorSkin0 < 1 && errorSkin0 >= -1){
            //|| errorSkin1 < 1 && errorSkin1 >=-1
            steadytime0++;
            error1 = 0;
            if(steadytime0 >= 60){
                steadytime0 = 60;
            }
        }
        if(steadytime0 == 60){
            if(babySkinTemp0 >= setTemp+1 || babySkinTemp0 <= setTemp-1){
            //|| babySkinTemp1 >= setTemp +1 || babySkinTemp1 <= setTemp-1
                error1 = 1;
            }
            else{
                error1 = 0;
            }
        }
        if(highTemp == 0 && babySkinTemp0 >= 38 || babySkinTemp1 >= 38){
          //   
          error3 = 1;
        } 
        if(highTemp == 1 && babySkinTemp0 >= 38.5 || babySkinTemp1 >= 38.5){
          //  
          error3 = 1;
        }         
        else{
          error3 = 0;
        }  
    }
    
    if(skinMode == 2){
        steadytime0 = 0;
        error1 = 0;
        if(errorAir < 1 && errorAir >= -1){
            steadytime1++;
            error2 = 0;
            if(steadytime1 >= 60){
                steadytime1 = 60;
            }
        }
        if(steadytime1 == 60){
            if(chamberTemp0 >= setTemp+2 || chamberTemp0 <= setTemp-2){
                error2 = 1;
            }
            else{
                error2 = 0;
            }
        }
        if(highTemp == 0 && chamberTemp0 >= 38){
            error4 = 1;
        }
        if(highTemp == 1 && chamberTemp0 >= 39.5){
            error4 = 1;
        }        
        else{
            error4 = 0;
        }        

    }
    if(skinMode == 0){
        steadytime0 = 0;
        steadytime1 = 0;
    } 
       
   /*End Temp Deviation and High Temp Alarm*////////////////////////


//    /*High Temperature Alarm*//////////////////////////
//     if(highTemp == 0 && chamberTemp0 >= 38 || babySkinTemp0 >= 38 || babySkinTemp1 >= 38){
//         error3 = 1;
//     }else{
//         error3 = 0;
//     }
//     if(highTemp == 1 && chamberTemp0 >= 39.5 || babySkinTemp0 >= 38.5 || babySkinTemp1 >= 38.5){
//         error4 = 1;
//     }else{
//         error4 = 0;
//     }
//     /*end High Temperature Alarm*/////////////////////////
       
    /*Power Failure*/
    powerIn = digitalRead(voltage5V);
    if(powerIn == 1){
        error5 = 0;
    }
    if(powerIn == 0){
        error5 = 1;
    }

/*Fan Failure*/
  uint16_t freq1 = analogRead(pulse_ip);

     if(freq1 > 1000){
       error6 = 1;
     }
     if(freq1 < 1000){
       error6 = 0;
     }
     Serial.print(freq1);
     Serial.println(error6);          
  }
  if(alarmRst == 1){
    error0 = 0;
    error1 = 0;
    error2 = 0;
    error3 = 0;
    error4 = 0;
    error5 = 0;   
    error6 = 0;     
  }
 }
}

//void sleep_available(){
//  int vbatSense = analogRead(vbat);
//  if(vbatSense <= 3.5 && error5 == 1){
//    sleepMode(SLEEP_IDLE);
//    sleep();
//    sleep_time = 1;
//  }
//  if(vbatSense > 3.5 && error5 == 0){
//    sleep_disable();
//    noSleep();
//    sleep_time = 0;
//  }
//}

/*End read Error data or missing sensor*/////////////////////////////////////////////////////////////////////////////////////

// void plotter(){
//     Serial.print("DATA, TIME, TIMER,");
//     Serial.print(babySkinTemp0);
//     Serial.print(",");
//     Serial.print(chamberTemp0);
//     Serial.print(",");    
//     Serial.print(setTemp);
//     Serial.print(",");
//     Serial.print(outputHeater);
//     Serial.println();
// }

/*tone buzzer function*/
void PID(){
  setPoint1 = setTemp; 
  errorAir = (setTemp * 10) - (chamberTemp0 * 10);
  errorSkin0 = (setTemp * 10) - (babySkinTemp0 * 10);
  errorSkin1 = (setTemp * 10) - (babySkinTemp1 * 10);
  if(error0 == 0 && error1 == 0 && error2 == 0 && error3 == 0 && error4 == 0 && error5 == 0){    
    if(highTemp == 0){
      if(skinMode == 2 && errorAir > 6){
         digitalWrite(relayHeat, HIGH);
        input = chamberTemp0;
        myPID.Compute();
        myPID2.Compute();
        set_pwm(outputFan, outputHeater);
        }
      else if(skinMode == 2 && errorAir <= 6){
        digitalWrite(relayHeat, HIGH);
        run_control();
        outputHeater = heaterPwm;
        outputFan = fanPwm;
        input = chamberTemp0; 
      }
      else if(skinMode == 1 && errorSkin0 > 5){ 
         digitalWrite(relayHeat, HIGH);
        input = babySkinTemp0;
        myPID3.Compute();
        myPID2.Compute();
        set_pwm(outputFan, outputHeater);
      }
      else if(skinMode == 1 && errorSkin0 <= 5){
        digitalWrite(relayHeat, HIGH);
        run_control();
        outputHeater = heaterPwm;
        outputFan = fanPwm;
        input = babySkinTemp0;
      }
      else{
        set_pwm(outputFan, outputHeater);
         digitalWrite(relayHeat, LOW);
        outputHeater = 0;
        outputFan = 100;
      }
    }

    if(highTemp == 1){
      if(skinMode == 2 && errorAir > 12){
         digitalWrite(relayHeat, HIGH);
        input = chamberTemp0;
        myPID.Compute();
        myPID2.Compute();
        set_pwm(outputFan, outputHeater);
      }
      else if(skinMode == 2 && errorAir <= 12){
        digitalWrite(relayHeat, HIGH);
        run_control();
        outputHeater = heaterPwm;
        outputFan = fanPwm;
        input = chamberTemp0; 
      }
      else if(skinMode == 1 && errorSkin0 > 3){ //jika sdh ada sensor skin2 di rata2
         digitalWrite(relayHeat, HIGH);
        input = babySkinTemp0;
        myPID3.Compute();
        myPID2.Compute();
        set_pwm(outputFan, outputHeater);
      }
      else if(skinMode == 1 && errorSkin0 <= 3){
        digitalWrite(relayHeat, HIGH);
        run_control();
        outputHeater = heaterPwm;
        outputFan = fanPwm;
        input = babySkinTemp0;
      }
      else{
        set_pwm(outputFan, outputHeater);
        outputHeater = 0;
        outputFan = 100;
         digitalWrite(relayHeat, LOW);
      }    
    } 
  }
  else{
    set_pwm(outputFan, outputHeater);
     digitalWrite(relayHeat, LOW);
    outputHeater = 0;
    outputFan = 100;
  }  
}


void generate_pulse(){ 
  if(start == 0 && pulsa <= 19){ //20
    if(millis() - timePulse > 2 && highState == 0){
      digitalWrite(clkOut, HIGH);     
      timePulse = millis();
      highState = 1;
    }
    if(millis() - timePulse > 2 && highState == 1){
      dataArray[pulsa] = digitalRead(dataIn);
      pulsa++;
      timePulse = millis();
      highState = 2;
    } 
    if(millis() - timePulse > 4 && highState == 2){
      digitalWrite(clkOut, LOW);
      timePulse = millis();
      highState = 3;
    }
    if(millis() - timePulse > 1 && highState == 3){
      timePulse = millis();
      highState = 0;
    }
  }
}

void sample_data(){ 
    if(pulsa > 19){ //20
      digitalWrite(clkOut, LOW);
        if(millis() - generateData > 100 && sampleState == 0){
          for(halfBit = 0; halfBit < 10; halfBit++){
            bitWrite(dataSensor1, 9- halfBit , dataArray[halfBit]);
            Serial.print(dataArray[halfBit]);
          }
          for(fullBit = 10; fullBit < 20; fullBit++){
            bitWrite(dataSensor2, 19- fullBit , dataArray[fullBit]);
            Serial.print(dataArray[fullBit]);
          }
            Serial.print("-");
            Serial.print(dataSensor1);
            Serial.print("-");
            Serial.print(dataSensor2);
            Serial.println("-");
            start = 1;
            sampleState = 1;
            generateData = millis();
        }
        if(millis() - generateData > 100 && sampleState == 1){
            pulsa = 0;
            start = 0;
            sampleState = 0;
            generateData = millis();            
        }
    }
}

void vbat_read(){
  vbatsense = analogRead(vbat);
  if(vbatsense <= 700 && error5 == 1){
    valuePower = 0;
  }
  if(vbatsense > 700 || error5 == 0){
    valuePower = 1;
  }
}

void oxygen_sense(){
  O2SensorDev.readADC();
  int reado2 = O2SensorDev.getValue();
  oxygen = reado2;
}

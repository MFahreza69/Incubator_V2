#define dataIn  10
#define clkOut  9

uint8_t i;
uint8_t j;
int dataArray[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
unsigned long timePulse; //clk out
unsigned long generateData;
uint8_t b = 0;
uint8_t d = 0;
unsigned long e;
uint8_t f;
int dataSensor1 = 0;
int dataSensor2 = 0;
uint8_t pulsa = 0;
uint8_t start = 1;


void generate_pulse(){ 
  if(start == 0 && pulsa <= 19){ //20
    if(millis() - timePulse > 5 && b == 0){
      digitalWrite(clkOut, HIGH);     
      timePulse = millis();
      b = 1;
    }
    if(millis() - timePulse > 5 && b == 1){
      dataArray[pulsa] = digitalRead(dataIn);
      pulsa++;
      timePulse = millis();
      b = 2;
    } 
    if(millis() - timePulse > 10 && b == 2){
      digitalWrite(clkOut, LOW);
      timePulse = millis();
      b = 3;
    }
    if(millis() - timePulse > 1 && b == 3){
      timePulse = millis();
      b = 0;
    }
  }
}

void sample_data(){ 
    if(pulsa > 19){ //20
      digitalWrite(clkOut, LOW);
        if(millis() - generateData > 250 && d == 0){
          for(i = 0; i < 10; i++){
            bitWrite(dataSensor1, 9- i , dataArray[i]);
            Serial.print(dataArray[i]);
          }
          for(j = 10; j < 20; j++){
            bitWrite(dataSensor2, 19- j , dataArray[j]);
            Serial.print(dataArray[j]);
          }
            Serial.print("-");
            Serial.print(dataSensor1);
            Serial.print("-");
            Serial.print(dataSensor2);
            Serial.println("-");
            start = 1;
            d = 1;
            generateData = millis();
        }
        if(millis() - generateData > 250 && d == 1){
            pulsa = 0;
            start = 0;
            d = 0;
            generateData = millis();            
        }
    }
}

void setup() {
  Serial.begin(9600);
  pinMode(dataIn, INPUT);
  pinMode(clkOut, OUTPUT);
  pulsa = 0;
  start = 0;
  delay(1000);
}

void loop() {
  generate_pulse();
  sample_data();
}

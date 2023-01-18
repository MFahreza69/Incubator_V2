#define dataOut 4 //PB5
#define clkIn 5  //PB4
#define sensor 2 //PA6
#define sensor2 3 //PA7


int a;
int a2;
int dat;
int dat2;
unsigned long low;
int lowvar;
int b = 0;
unsigned long lowtime;
uint16_t sumdata1;
uint16_t sumdata2;
int dataSensor1;
int dataSensor2;
int sampleData[2];
int sampleData2[2];
uint8_t current = 0;
uint8_t last = 0;
String dataArray3[20];
int r = random(0, 1023);
//SoftwareSerial mySerial(5, 4);

void convert_tobinary(int input, int input2){
  dat = 0;
  for(a=512; a>=1; a=a/2){
    if((input-a)>=0){
      dataArray3[dat]= '1';
      input-=a; 
    }
    else{
      dataArray3[dat]='0';
    } 
    dat++;  
  }
    for(a2=512; a2>=1; a2=a2/2){
    if((input2-a2)>=0){
      dataArray3[dat]= '1';
      input2-=a2; 
    }
    else{
      dataArray3[dat]='0';
    } 
    dat++;  
  }
}


void send_data(){
//  mean();
  last = current;
  current = digitalRead(clkIn);
  if(last == 0 && current == 1){
     lowvar = 0;
     digitalWrite(dataOut, dataArray3[b].toInt());
     b++;
     Serial.print(b);
     Serial.println("-");
   }

  if(b > 20){
     b = 0;
     lowvar = 1;
     current = 0;
     last = 0;
     convert_tobinary(dataSensor2, dataSensor1);
   }
     

   if(current == LOW){
    if(millis() - lowtime > 1000){ 
     b = 0;
     }
   }
   if(current == HIGH){
      lowtime = millis();
   }

  else{}
}

void mean(){
  for(uint8_t f=0; f<2; f++){ 
    for(uint8_t mdata = 0; mdata<2; mdata++){ 
      sampleData[mdata] = analogRead(sensor2);
    }
    for(uint8_t x = 0; x < 2; x++){
      sumdata1 += sampleData[x];
    }
    sumdata1 = sumdata1/2;
  }
  
  for(uint8_t i=0; i<2; i++){
    for(uint8_t ndata = 0; ndata<2; ndata++){
      sampleData2[ndata] = analogRead(sensor);
    }
    for(uint8_t y = 0; y < 2; y++){
      sumdata2 += sampleData2[y];
    }
    sumdata2 = sumdata2/2;
    }
  dataSensor1 = sumdata2;
  dataSensor2 = sumdata1;
}

void setup(){
  analogReference(EXTERNAL);
  Serial.begin(9600);
//  mySerial.begin(9600);
    pinMode(dataOut, OUTPUT);
    pinMode(clkIn, INPUT);
    pinMode(sensor, INPUT);
    pinMode(sensor2, INPUT);
//    delay(1000);
    b = 0;
}


void loop(){
  dataSensor1 = analogRead(sensor);
  dataSensor2 = analogRead(sensor2);
//  mean();
  send_data();
}

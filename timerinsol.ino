#include <EEPROM.h>
void showtime(byte );
void keyRead();

/**
 * Timer para insoladora de serigrafía.
 * (c) 2014 - Mario O. Villarroel / Elcan Software
 */

//pin mapping.
#define SEGA 8
#define SEGB 7
#define SEGC 6
#define SEGD 5
#define SEGE 9
#define SEGF 10
#define SEGG 11
#define MPX1 13
#define MPX2 12

#define BUZZER 3
#define RELE   4
#define KEY_DEC    A4
#define KEY_INC    A5
#define KEY_ENT    2

//other constants 
#define TIMELOCATION 0x02
#define KEYREADPERIOD 250
#define BUZZDELAY 450
#define DEBUG 0

//variables
volatile boolean startnow = LOW;
volatile boolean ended = LOW, modified=LOW;
static byte timeSet;
static char buzzout = LOW;
static char keypress=0;
static unsigned long timeStarted;
static unsigned long elapsed,lastBuzz;
unsigned long krtime;
unsigned char seven_seg_digits[22][7] = {
                              //a,b,c,d,e,f,g
                               {0,0,0,0,0,0,0}, //avoid the akward problem 
                               { 1,1,1,1,1,1,0 },  // = 0
                               { 0,1,1,0,0,0,0 },  // = 1
                               { 1,1,0,1,1,0,1 },  // = 2
                               { 1,1,1,1,0,0,1 },  // = 3
                               { 0,1,1,0,0,1,1 },  // = 4
                               { 1,0,1,1,0,1,1 },  // = 5
                               { 1,0,1,1,1,1,1 },  // = 6
                               { 1,1,1,0,0,0,0 },  // = 7
                               { 1,1,1,1,1,1,1 },  // = 8
                               { 1,1,1,0,0,1,1 },   // = 9
                               { 1,1,1,0,1,1,1 },  // A
                               { 0,0,1,1,1,1,1 },  // B
                               { 1,0,0,1,1,1,0 },  // C
                               { 0,1,1,1,1,0,1 },  // =d => digit: 0x0B
                               { 1,0,0,1,1,1,1 },  // E
                               { 1,0,0,0,1,1,1 },  //F => Digit 0x0F
                               { 1,1,0,0,1,1,1 },  // = P => digit: 0x10
                               { 1,1,0,1,0,1,1 },  //= r => digit: 0x11
                               { 0,0,0,0,1,1,0 },  //= I => Digit 0x12
                               { 1,0,0,0,1,1,0 },   //= T => Digit 0x13
                               { 0,1,1,0,1,1,1 }  // H Digit 0x14
                             };
char segments[7] = {SEGA,SEGB,SEGC,SEGD,SEGE,SEGF,SEGG};
//code from here
void setup() {
  char tmp;
  //read the eeprom for the time, just one byte so 
  pinMode(SEGA,OUTPUT);
  pinMode(SEGB,OUTPUT);  
  pinMode(SEGC,OUTPUT);
  pinMode(SEGD,OUTPUT);
  pinMode(SEGE,OUTPUT);
  pinMode(SEGF,OUTPUT);
  pinMode(SEGG,OUTPUT);
  pinMode(MPX1,OUTPUT);
  pinMode(MPX2,OUTPUT);
  pinMode(BUZZER,OUTPUT);  
  pinMode(RELE,OUTPUT);  
  pinMode(KEY_DEC,INPUT_PULLUP);    
  pinMode(KEY_INC,INPUT_PULLUP);
  pinMode(KEY_ENT,INPUT_PULLUP);
  digitalWrite(RELE,LOW);
  timeSet=EEPROM.read(TIMELOCATION);
  timeSet = constrain(timeSet,1,99);
  //timeSet=1;
#if DEBUG  
  Serial.begin(9600);
  Serial.println(timeSet);
#endif  
  attachInterrupt(0,interrupted, FALLING);
  digitalWrite(MPX2,LOW);
  digitalWrite(MPX1,LOW);
  digitalWrite(RELE,LOW);
  krtime=millis();
  keypress=0;
}

void loop() {
  unsigned long curTime = millis();
#if DEBUG
  delay(300);
#endif
  if (startnow) {
    digitalWrite(RELE,HIGH);  //power off the relay.
    if (modified==HIGH) {
      EEPROM.write(TIMELOCATION,timeSet);
      modified = LOW;
#if DEBUG
      Serial.print("Stored!");
      Serial.println(timeSet);
#endif
    }
    elapsed=(curTime/1000)-(timeStarted/1000);
    if (elapsed < (timeSet*60)) {
      if ((timeSet*60-elapsed)<60) {
        showTime(timeSet*60-elapsed);
      } else {
        showTime(timeSet-elapsed/60);
      }
    } else {
      showTime(0);
      ended = HIGH;
      startnow = LOW;
    }
  } else {
    //reading the keys is possible when not started
    keyRead();
    if (ended && !modified ) {
      showTime(0);   
     if ((curTime-lastBuzz > BUZZDELAY)) {
      lastBuzz=curTime;
      buzzout=~buzzout;
      digitalWrite(BUZZER,buzzout);
     }
      digitalWrite(RELE,LOW);
    } else {
      showTime(timeSet);
      digitalWrite(BUZZER,LOW);    
    }
    switch(keypress){
    case 0x01:
      timeSet++;
      modified=HIGH;
      timeSet=constrain(timeSet,1,99);
      break;
    case 0x10:
      timeSet--;
      modified=HIGH;
      timeSet=constrain(timeSet,1,99);
      break;
    }
    keypress=0;
  }
}

void interrupted() {
  if (!startnow) {
    startnow = HIGH;
    timeStarted = millis();
    digitalWrite(RELE,HIGH);  //power on the relay.
  }
}

void showTime(byte ts) {
  //display the time set.
  char ttmp[2];
  char tmp1;
  tmp1=ts; //constrain(0,99,ts);
  ttmp[1]=tmp1/10;
  ttmp[0]=tmp1%10;
#if DEBUG
  Serial.print(ttmp[0],HEX);
  Serial.print("-");
  Serial.println(ttmp[1],HEX);
#endif
  dataDisplay(ttmp);
}

void keyRead() {
  char xtmp;
  unsigned long ctim;
  ctim=millis();
  if (ctim - krtime > KEYREADPERIOD) {
    keypress=0x00;
    xtmp=digitalRead(KEY_INC);
    if (xtmp==LOW) {
      keypress|=0x01;
    } 
    else {
      keypress&=~0x01;
    }
    xtmp=digitalRead(KEY_DEC);
    if (xtmp==LOW) {
      keypress|=0x10;
    } 
    else { 
      keypress&=~0x10;
    }
    krtime=ctim;
  } 
}

void dataDisplay(char *buf) {
  //if (sizeof(buf)>2) return; //we can only show two digits, discard everything else. useless, just display the first two digits, discard the rest...
  digitalWrite(MPX1,LOW);
  sevenSegWrite(buf[0]);
  digitalWrite(MPX1,HIGH);
  delay(5);
  digitalWrite(MPX1,LOW);//*/
  sevenSegWrite(buf[1]);
  digitalWrite(MPX2,HIGH);
  delay(5);
  digitalWrite(MPX2,LOW); //all off;
}

void sevenSegWrite(char digit) {
  uint8_t pin = 0;
  uint8_t segCount = 0;
  //Serial.println(digit,HEX);
  for (segCount = 0; segCount < 7; ++segCount) {
    digitalWrite(segments[pin], seven_seg_digits[digit+1][segCount]);
    pin++;
  }
}


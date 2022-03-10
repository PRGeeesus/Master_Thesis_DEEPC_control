#include <Adafruit_ADS1X15.h>
//#include <Wire.h>
#include <TimerOne.h>
Adafruit_ADS1015 ads;     /* Use this for the 12-bit version */

const int motorPin1 = 9;
const int motorPin2 = 10;

const int LED_pin = 2;
const int POTI_PIN = 6;


// (timer speed (Hz)) = (Arduino clock speed (16MHz)) / prescaler
void setup() {
  //Set pins as outputs
  Serial.begin(19200);
  pinMode(motorPin1, OUTPUT);
  pinMode(LED_pin, OUTPUT);
  pinMode(POTI_PIN, INPUT);
  pinMode(motorPin2, OUTPUT);
  digitalWrite(LED_pin, HIGH);

  Timer1.initialize(40);

  Serial.println("Beginning Initalization");

  if (!ads.begin()) {
    digitalWrite(LED_pin, LOW);
    //Serial.println("Failed to initialize ADS.");
    while (1);
  }
  //Serial.println("Initaialized ADS correctly");
  delay(1000);
  /*
    cli();//stop interrupts

    TCCR2A = 0;// set entire TCCR2A register to 0
    TCCR2B = 0;// same for TCCR2B
    TCNT2  = 0;//initialize counter value to 0
    // set compare match register for 8khz increments
    //OCR2A = 80;// = (16*10^6) / (4,4*100) - 1 (must be <256)
    OCR2A = 160;// = (16*10^6) / (100000*8) - 1 (must be <256)
    // turn on CTC mode
    TCCR2A |= (1 << WGM21);
    // Set CS21 bit for no prescaler
    TCCR2B |= (1 << CS20);
    // enable timer compare interrupt
    TIMSK2 |= (1 << OCIE2A);

    sei();//allow interrupts
  */
  delay(1000);
  //Setup 3V comparator on channel 0 (A0)

  ads.startComparator_SingleEnded(0, 1000);


  //digitalWrite(LED_pin,LOW);
  //Serial.println("Setup Complete");
  // Signals All is good

  //Motor Control A in both directions

}
void blinkLED(int wait_time_on, int wait_time_off, int nr_of_blinks)
{
  for (int i = 0; i < nr_of_blinks; i++)
  {
    digitalWrite(LED_pin, HIGH);
    delay(wait_time_on);
    digitalWrite(LED_pin, LOW);
    delay(wait_time_off);
  }
}
int toggle0 = 0;
int counter_value = 0;
unsigned speed = 1;

/*
  ISR(TIMER2_COMPA_vect){//timer0 interrupt 2kHz toggles pin 8
  ////generates pulse wave of frequency 2kHz/2 = 1kHz (takes two cycles for full wave- toggle high then toggle low)
  counter_value = counter_value + 1; // this will count to 160
  if(counter_value > speed)
  {
    digitalWrite(motorPin1,LOW);
  }
  else
  {
  digitalWrite(motorPin1,HIGH);
  }

  if(counter_value > 100){
  counter_value = 0;
  }
  }
*/
uint16_t adc0 = 0;
int16_t poti_val = 0;
int read_length = 0;
int len = 4;
unsigned int voltage = 0;

/*
  void serialEvent() {
  delayMicroseconds(50);
  //Serial.println("Serial event");
  //string = Serial.readStringUntil('\n');
  voltage = Serial.readString().toInt();
  //Serial.println(voltage);
  Timer1.pwm(motorPin1, (voltage/ 100.0) * 1023);
  delayMicroseconds(3);
  adc0 = ads.getLastConversionResults();
  Serial.print(adc0);

  if(string[0] == '3')
  {
    digitalWrite(LED_pin,LOW);
    //Serial.println("3 DETECTED in first position");
    voltage = (string[1] - 48) * 10 + (string[2] - 48);

    if(voltage >= 0 && voltage <= 100)
    {
      //speed = voltage;
      //sprintf (buf, "%04d", int(RPM)) ;  // leading zeroes, at least 4 chars.
      Timer1.pwm(motorPin1, (voltage/ 100.0) * 1023);
      adc0 = ads.getLastConversionResults();
      //poti_val = analogRead(POTI_PIN);
      Serial.print(adc0);
      digitalWrite(LED_pin,HIGH);
    }
  }
  else
  {
      adc0 = ads.getLastConversionResults();
      //poti_val = analogRead(POTI_PIN);
      Serial.print(adc0);
  }

  }
*/
char string[5];

void loop() {
  if (Serial.available() >= 3)
  {
    digitalWrite(LED_pin, HIGH);
    //string2 = Serial.readString();
    Serial.readBytesUntil('\r', string, 7);
    
      if(string[0] == '3')
      {
      int val = (string[1]-48)*1000 + (string[2]-48)*100 + (string[3]-48)*10 + (string[4]-48);
      if(val >= 0 && val < 1030)
        {
          if(val > 1023){val = 1023;}
          //Timer1.pwm(motorPin1, (val/ 100.0) * 1023);
          Timer1.pwm(motorPin1, val);
          adc0 = ads.getLastConversionResults();
          //poti_val = analogRead(POTI_PIN);
          Serial.print(adc0);
          Serial.flush();
        }
      }
      while (Serial.available() > 3) {
        Serial.read();
          }
  }
}

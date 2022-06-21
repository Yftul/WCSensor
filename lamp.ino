#include <Ultrasonic.h>
#include "GyverTimers.h"
#include <avr/sleep.h>
#include <avr/wdt.h>

#define DEBUG

// Pinout
#define trigger_pin 9
#define echo_pin 8
#define output_pin 11
#define output_led 13
#define sensor_pin A0

// Physics
#define filter_size 4
#define off_delay_value 20
#define gysteresis 15

// Tuning
#define idle_freq 5
#define work_freq 10

// Auxiliary
#define power_on 1
#define power_off 0

Ultrasonic ultrasonic(trigger_pin, echo_pin); // ультразвуковой локатор
volatile bool timer_trigger = false;
volatile bool wdt_trigger = false;

void setup() 
{
#ifdef DEBUG  
  Serial.begin(115200);        // подключаем монитор порта
#endif
  pinMode(sensor_pin, INPUT);  // к аналоговому входу подключаем потенциометр
  pinMode(output_pin, OUTPUT); // подключаем к выходному пину оптрон
  Timer1.setFrequency(work_freq);
  Timer1.enableISR();
  wdt_enable(WDTO_1S);
}

ISR(TIMER1_A) // Прерывание А таймера 2
{
  timer_trigger = true;
}

ISR (WDT_vect)
{
  wdt_trigger = true;
}

void loop () 
{
  static int filterdist[filter_size] = {0};
  static int fdptr = 0;
  static int off_delay = off_delay_value;
  static int old_delay;
  static char old_out;
  static char output;
  int dist = 0;
  int setdist;
  
  wdt_reset();
  
  if (timer_trigger) {
    timer_trigger = false;
    ADCSRA |= bit(ADEN); // Включаем АЦП заранее
    
    filterdist[fdptr] = ultrasonic.distanceRead();
    fdptr = (fdptr == filter_size - 1)?0:fdptr + 1;
    for(int i = 0; i < filter_size; ++i)
      dist += filterdist[i];
    dist /= filter_size;
    
    setdist = analogRead(sensor_pin) >> 2; // (0 .. 1023 / 4) /см/
    ADCSRA &= ~bit(ADEN); // Отключаем АЦП после считывания задания

    old_out = output; 
    output = (dist < (setdist + gysteresis*(off_delay?1:0)))?0:1;
    if (output < old_out) // Мгновенное включение
      off_delay = off_delay_value;
    else if (off_delay && output) // Задержка выключения
      --off_delay;

    if (old_delay == !off_delay) {
      old_delay = !!off_delay;
      if (off_delay) {
        Timer1.setFrequency(work_freq);
        digitalWrite(output_pin, power_on);
        digitalWrite(output_led, power_on);
      } else {
        Timer1.setFrequency(idle_freq);
        digitalWrite(output_pin, power_off);
        digitalWrite(output_led, power_off);
      }
    }
#ifdef DEBUG  
    Serial.print(setdist);  // Заданное расстояние
    Serial.print(" -> ");   
    Serial.print(dist);     // Измеренное расстояние
    Serial.println(" cm");  // в сантиметрах
    if (wdt_trigger)
      Serial.println("*");  // Сработал WDT
#endif
  } else {
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_mode();
  }
}

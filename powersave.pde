/****************************************************************

This file is part of the Arduino powersave example at http://github.com/unchiujar/arduino_powersave

    powersave is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    powersave is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with powersave.  If not, see <http://www.gnu.org/licenses/>.
    
An example on how to put the Arduino to sleep for an arbitrary period
of time. To wake up after a preset time the Watchdog timer is used. 
The watchdog can be set to wake the system after a reset interval.
The intervals are preset (see the chart below or the Watchdog ATmega
technical specs) and depend on the voltage. To sleep for longer periods
of times than the longest preset period (2.2s at 3V) a number of sleep
cycles is calculated and the system is put to sleep immediately after
waking up until all the sleep cycles are completed.

This example uses code from

 * Watchdog Sleep Example 
 * Demonstrate the Watchdog and Sleep Functions
 * Photoresistor on analog0 Piezo Speaker on pin 10
 * 
 * KHM 2008 / Lab3/  Martin Nawrath nawrath@khm.de
 * Kunsthochschule fuer Medien Koeln
 * Academy of Media Arts Cologne


http://interface.khm.de/index.php/lab/experiments/sleep_watchdog_battery/

*****************************************************************/

#include <avr/sleep.h>
#include <avr/wdt.h>

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

byte ledPin = 13;

//sleep time in milliseconds
unsigned long sleepTime = 33000;

/*
From ATMega328 Watchdog specs
 
 Number of WDT     Typical Time-out Typical Time-out
 WDP2 WDP1 WDP0 Oscillator Cycles    at VCC = 3.0V    at VCC = 5.0V
0 0    0    0      16K (16,384)         17.1 ms          16.3 ms
1 0    0    1      32K (32,768)         34.3 ms          32.5 ms
2 0    1    0      64K (65,536)         68.5 ms           65 ms
3 0    1    1     128K (131,072)         0.14 s           0.13 s
4 1    0    0     256K (262,144)         0.27 s           0.26 s
5 1    0    1     512K (524,288)         0.55 s           0.52 s
6 1    1    0   1,024K (1,048,576)       1.1 s            1.0 s
7 1    1    1   2,048K (2,097,152)       2.2 s            2.1 s
 
 */

int wait_periods[7];
/*
 *  Calculates how many times the watchdog should
 *  be set for each sleep state. It goes through progressively
 *  shorter wait times until waking up. Since the 
 *  internal oscillator runs differently for 5V and 3V 
 *  the sleep times are calculated differently for 3V and 5V
 *  See the above chart and the Watchdog Timer and Minimizing Power
 *  consumption in the ATMega328 technical specifications.  
 */
void calculateWaitPeriods5V(unsigned long sleep){
  wait_periods[7] = sleep / 2100;
  sleep = sleep - wait_periods[7] * 2100;
  wait_periods[6] = sleep / 1000;
  sleep = sleep - wait_periods[6] * 1000;
  wait_periods[5] = sleep / 520;
  sleep = sleep - wait_periods[5] * 520;
  wait_periods[4] = sleep / 260;
  sleep = sleep - wait_periods[4] * 260;
  wait_periods[3] = sleep / 130;
  sleep = sleep - wait_periods[3] * 130;
  wait_periods[2] = sleep / 65;
  sleep = sleep - wait_periods[2] * 65;
  wait_periods[1] = sleep / 32;
  sleep = sleep - wait_periods[1] * 32;
  wait_periods[0] = sleep / 16;
}
void calculateWaitPeriods3V(unsigned long sleep){
  wait_periods[7] = sleep / 2200;
  sleep = sleep - wait_periods[7] * 2200;
  wait_periods[6] = sleep / 1100;
  sleep = sleep - wait_periods[6] * 1100;
  wait_periods[5] = sleep / 550;
  sleep = sleep - wait_periods[5] * 550;
  wait_periods[4] = sleep / 270;
  sleep = sleep - wait_periods[4] * 270;
  wait_periods[3] = sleep / 140;
  sleep = sleep - wait_periods[3] * 140;
  wait_periods[2] = sleep / 68;
  sleep = sleep - wait_periods[2] * 68;
  wait_periods[1] = sleep / 34;
  sleep = sleep - wait_periods[1] * 34;
  wait_periods[0] = sleep / 17;
}

/*
 * Puts the system to sleep, the watchdog
 * wakes the system which is then put to sleep
 * again immediately until it cycles through 
 * all the values in wait_periods. 
 */
void doSleep(){
  //sleep routine
  for (byte i=7;i>0;i--){
    //set sleep time
    setup_watchdog(i);
    //sleep for the number of times
    Serial.print(wait_periods[i]);
    Serial.print(" ");
    //remove, this is for testing
    delay(10);
    for (unsigned long j=0;j<wait_periods[i];j++){
      // this can be removed, it just blinks the LED faster as the 
      // sleep times get shorter
      // the only part necessary is systemSleep()
      digitalWrite(ledPin,LOW);
      system_sleep();
      digitalWrite(ledPin, HIGH);
      delay(1);

    }
  }
  Serial.println("");
}

/*
 * Helper method to put the Arduino to sleep for 
 * sleep milliseconds when powered at 3V. 
 */
void sleep3V(unsigned long sleep){
  calculateWaitPeriods3V(sleep);
  doSleep();
}

/*
 * Helper method to put the Arduino to sleep for 
 * sleep milliseconds when powered at 5V. 
 */
void sleep5V(unsigned long sleep){
  calculateWaitPeriods5V(sleep);
  doSleep();
}


volatile boolean f_wdt=1;

void setup(){

  Serial.begin(9600);

  // CPU Sleep Modes 
  // SM2 SM1 SM0 Sleep Mode
  // 0    0  0 Idle
  // 0    0  1 ADC Noise Reduction
  // 0    1  0 Power-down
  // 0    1  1 Power-save
  // 1    0  0 Reserved
  // 1    0  1 Reserved
  // 1    1  0 Standby(1)

  cbi( SMCR,SE );      // sleep enable, power down mode
  cbi( SMCR,SM0 );     // power down mode
  sbi( SMCR,SM1 );     // power down mode
  cbi( SMCR,SM2 );     // power down mode

  pinMode(ledPin, OUTPUT);
  delay(2000);
  Serial.println("Set up done");
}


//****************************************************************
//****************************************************************
//****************************************************************
void loop(){


  if (f_wdt==1) {  // wait for timed out watchdog / flag is set when a watchdog timeout occurs
    f_wdt=0;       // reset flag

    sleep5V(sleepTime);

    //**************** EXECUTE CODE HERE **************
    //**************** EXECUTE CODE HERE **************
    Serial.println("Rubbing the mint - a proud member of http://www.mintrubbing.org/ \n");   
    //**************** EXECUTE CODE HERE **************
    //**************** EXECUTE CODE HERE **************


  }

}


//****************************************************************  
// set system into the sleep state 
// system wakes up when watchdog is timed out
void system_sleep() {

  cbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter OFF

  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();

  sleep_mode();                        // System sleeps here

  sleep_disable();                     // System continues execution here when watchdog timed out 
  sbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter ON

}

//****************************************************************
/* 
 * See the chart at the top for sleep times
 */
void setup_watchdog(int ii) {

  byte bb;
  int ww;
  if (ii > 9 ) ii=9;
  bb=ii & 7;
  if (ii > 7) bb|= (1<<5);
  bb|= (1<<WDCE);
  ww=bb;
  // Serial.println(ww);


  MCUSR &= ~(1<<WDRF);
  // start timed sequence
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  // set new watchdog timeout value
  WDTCSR = bb;
  WDTCSR |= _BV(WDIE);


}
//****************************************************************  
// Watchdog Interrupt Service / is executed when  watchdog timed out
ISR(WDT_vect) {
  f_wdt=1;  // set global flag
}





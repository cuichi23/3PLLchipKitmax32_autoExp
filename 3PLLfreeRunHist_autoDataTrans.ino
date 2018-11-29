// cfaed coupled PLLs demonstrator
// copyright @Alexandros Pollakis, 2015, Lucas Wetzel 2018

// inculde libraries
#include "ClickButton.h"
#include "Wire.h"
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"

// set the baud rate of the serial port
#define BAUDRATE 115200

// set the buffer size (number of samples and maximal possible delay), i.e.: 51200, max:
#define BUFFER_SIZE 65500

// define the waiting time and reference time for the coupling activation
unsigned int period    = 0;
unsigned long time_now = 0;


// divider to set the sampling rate
#define SAMPLERATE 100 //300
// i.e. with samplerate 100/ms we have a sampleperiod of Tsample = 1/100E3 = 10E-6 seconds

// scale for transmission delay (choose according to SAMPLERATE)
#define DELAY_SCALE 1  //3

// system pin definitions
#define BOARD_LED 13

// pins for buttons
#define PIN_BUTTON_A 80
#define PIN_BUTTON_B 78
#define PIN_BUTTON_C 79

// available memory size = 2^17 =  128 KB
// Memory for Datacollection 120 KB = 122880 B
// 3x3 = 9 PLLs: 122880 Byte / 9 = 13653,33..
//            -> 13653 Byte pro PLL -> Mem = 122877 Byte
// 2x2 = 4 PLLs: 13653 Byte pro PLL -> Mem =  54612 Byte
// 1x2 = 2 PLLs: 13653 Byte pro PLL -> Mem =  27306 Byte

// number of collectable samples: 13653 Byte/PLL * 8samples/Byte = 109224 samples/PLL
// assumption: open frequency 1kHz
// sample rate: 10us -> 100 samples/periods -> ca. 1100 periods collectable
//               5us -> 200 samples/periods -> ca.  550 periods collectable [* good choice]

// this is the global trigger variable, it will start a measurement, i.e.,
// 1) free running PLLs for a few periods (Tp =~ 1ms)
// 2) turn on coupling, system is at random initial state, EXTENT: use buffer that looks as if the system was in a synched state
// 3) measure the transients after coupling is turned on, until the buffer is full, then flush it to the Serial port
int triggerRun=0;

// operation mode
uint8_t mode;            // mode = 0: free running PLLs (no coupling)
                         // mode = 1: coupled PLL network
                         // mode = 2: transmit data to PC

// recording BUFFER_SIZE
uint8_t dataBuffer[BUFFER_SIZE];
volatile uint16_t bufferIdx;
volatile uint16_t delayedIdx;
volatile int rnd;

// output bits and port
// bits:  7   6   5   4   3   2   1   0
// pins:  30  31  32  33  34  35  36  37
// plls:  -   -   2   2   1   1   0   0
volatile uint8_t outBits;

// input bits and port
// bits:  7   6   5   4   3   2   1   0
// pins:  A07 A06 A05 A04 A03 A02 A01 A00
// plls:  -   -   -   -   -   2   1   0
volatile uint8_t inBits;

// transmission delay in sampling steps
volatile int transmissionDelayMax = 99;
volatile int transmissionDelay;

// coupling topologies
// "0": 2 bi-directional coupled PLLs
// "1": 2 bi-directional coupled PLLs
// "2": chain of 3 bi-directional coupled PLLs
// "3": ring of 3 bi-directional coupled PLLs
uint8_t couplingTopology = 3;
volatile boolean coupling;
volatile boolean start_it;
volatile boolean writeoutmode = false;
volatile boolean write2buffer = true;
volatile boolean test_dummy = true;
//volatile boolean check_it = true;


// 7 segment LED display
Adafruit_7segment sevseg = Adafruit_7segment();

/**********************************************
                FUNCTIONS
**********************************************/

uint8_t virtualCoupling(uint8_t data){
    uint8_t out = 0x00;                                                         // initialize out with all zeros
    switch(couplingTopology){
        case 0:
            // 2 bi-directionally coupled plls
            bitWrite(out, 0, bitRead(data,1));                                  // PLL 0: 1
            bitWrite(out, 1, bitRead(data,1));                                  // PLL 0: 1
            bitWrite(out, 2, bitRead(data,0));                                  // PLL 1: 0
            bitWrite(out, 3, bitRead(data,0));                                  // PLL 1: 0
            bitWrite(out, 4, 0);                                                // PLL 2: -
            bitWrite(out, 5, 0);                                                // PLL 2: -
            break;
        case 1:
            // 2 bi-directionally coupled plls
            bitWrite(out, 0, 0);                                                // PLL 0: -
            bitWrite(out, 1, 0);                                                // PLL 0: -
            bitWrite(out, 2, bitRead(data,2));                                  // PLL 1: 2
            bitWrite(out, 3, bitRead(data,2));                                  // PLL 1: 2
            bitWrite(out, 4, bitRead(data,1));                                  // PLL 2: 1
            bitWrite(out, 5, bitRead(data,1));                                  // PLL 2: 1
            break;
        case 2:
            // chain of 3 bi-directionally coupled plls
            bitWrite(out, 0, bitRead(data,1));                                  // PLL 0: 1
            bitWrite(out, 1, bitRead(data,1));                                  // PLL 0: 1
            bitWrite(out, 2, bitRead(data,0));                                  // PLL 1: 0
            bitWrite(out, 3, bitRead(data,2));                                  // PLL 1: 2
            bitWrite(out, 4, bitRead(data,1));                                  // PLL 2: 1
            bitWrite(out, 5, bitRead(data,1));                                  // PLL 2: 1
            break;
        case 3:
            // ring of 3 bi-directionally coupled plls
            bitWrite(out, 0, bitRead(data,1));                                  // PLL 0: 1
            bitWrite(out, 1, bitRead(data,2));                                  // PLL 0: 2
            bitWrite(out, 2, bitRead(data,0));                                  // PLL 1: 0
            bitWrite(out, 3, bitRead(data,2));                                  // PLL 1: 2
            bitWrite(out, 4, bitRead(data,0));                                  // PLL 2: 0
            bitWrite(out, 5, bitRead(data,1));                                  // PLL 2: 1
            break;
    }
    return out;
}

// update 7-segment display
void updateDisplay(){
    sevseg.print(transmissionDelay,DEC);
    sevseg.writeDigitNum(0,couplingTopology,false);
    sevseg.writeDigitRaw(1,B01000000);
    if(!writeoutmode){
        sevseg.blinkRate(0);
    }
    else{
        sevseg.blinkRate(1);
    }
    sevseg.writeDisplay();
}

// serial transmit data
void serialWriteDataBuffer(){
    digitalWrite(BOARD_LED, LOW);                                               // turn off board led
    Serial.print("data"); //Serial.flush();                                     // line 'data' to indicate the start of a transmission
    Serial.println(transmissionDelay); //Serial.flush();                        // transmit delay value
    //Serial.write(dataBuffer,BUFFER_SIZE);//Serial.flush();                    // transmit data buffer
    for(int i=0; i<BUFFER_SIZE; i++){
        Serial.print(bitRead(dataBuffer[i],0)); Serial.print("\t"); Serial.print(bitRead(dataBuffer[i],1)); Serial.print("\t"); Serial.print(bitRead(dataBuffer[i],2));
        Serial.println(" ");

        delay(2);
    }
    //Serial.println(" ");//Serial.flush();                                       // newline
    Serial.println("Writeout finished!");
    digitalWrite(BOARD_LED, HIGH);                                              // turn on board led
}

/**********************************************
                    INIT
**********************************************/

void setup() {
    // initialize serial port
    Serial.begin(BAUDRATE);
    Serial.println("Baudrate set!");

    // set boolean variables
    coupling = false;                                                           // this will effect the display blinking vs non/blink
    start_it = true;

    // initialize the board
    pinMode(BOARD_LED, OUTPUT); digitalWrite(BOARD_LED, HIGH);
    // initialize the trigger
    LATFCLR = B11; TRISF = B00;
    // set input ports
    AD1PCFG = 0xFFFF;                                                           // use analogue pins as digital pins
    LATBCLR = 0xFFFF; TRISB = 0xFFFF;                                           // initialize input pins [port: B pins: A0 - A07 ]
    // set output ports
    LATECLR = 0xFFFF; TRISE = 0x0000;                                           // initialize output pins [port: E pins: 37 - 30 ]

    // clear buffer (set zero)
    memset(dataBuffer, 0, BUFFER_SIZE);
    // "0": 2 bi-directional coupled PLLs
    // "1": 2 bi-directional coupled PLLs
    // "2": chain of 3 bi-directional coupled PLLs
    // "3": ring of 3 bi-directional coupled PLLs
    couplingTopology          = 3;
    // set intitial transmission delay
    transmissionDelay         = 45;
    // pause
    delay(200);
    // 7 segment display
    sevseg.begin(0x70);
    sevseg.setBrightness(0.2);
          updateDisplay();

    Serial.print("Doublecheck 1 - time Serial.print output of micros(): ");
    Serial.println(micros());
    Serial.println(micros());
    Serial.print("Doublecheck 1.1 - time Serial.print 'K on:': ");
    Serial.println(micros());
    Serial.println("K on:");
    Serial.println(micros());
    Serial.print("Doublecheck 2 - time set variable: ");
    Serial.println(micros());
    rnd = 1;
    Serial.println(micros());
    Serial.print("Doublecheck 3 - time boolean switch: ");
    Serial.println(micros());
    test_dummy = !test_dummy;
    Serial.println(micros());
    Serial.print("Doublecheck 4 - time to call micros(): ");
    Serial.println(micros());
    time_now = micros();
    Serial.println(micros());
    Serial.print("Doublecheck 5 - time to call while-loop: ");
    Serial.println(micros());
    while(micros() < time_now) {
      // should be fullfilled right away
    }
    Serial.println(micros());

    Serial.println("uC is ready! Start measurement!");
    // start timer interrupt to read/write --> will trigger the loop
    attachCoreTimerService(timerISR);
}

/**********************************************
                MAIN LOOP
**********************************************/

void loop() {
    // within this if condition the measurement is performed; during setup(), the buffer is set up, coupling is turned off, the delay-value is set, the display is initiated
    // and a core timer service is attached (timerISR), start_it is toggled 'true'
    // along with the attachement of the core timer service, the loop() function starts and the first measurement begins
    // --> start_it variable will be toggled false to prevent reentry into this part of the loop before the buffer has been flushed to the serial port...
    if ( start_it ){
        start_it = !start_it;                                                 // switch start_it to FALSE, prevent entering this if-condition
                                                                              // PLLs are still running in coupled mode
        period = 5000;                                                        // set free-running time in microseconds --> e.g. 5000 us = 5 ms
        triggerRun = 1;                                                       // trigger the run

        //coupling = !coupling;                                                 // turn off coupling (only if set to true in setup()-fct.)
        //delay(2000);
        time_now  = micros();                                                 // save current time in microseconds after program has started
        bufferIdx = 0;                                                        // reset buffer index to zero to begin writing the uncoupled case -- good here,
                                                                              // needs almost no time
        //Serial.print("K off: ");                                            // write out the current time, TAKE into account the measured delays due to the
        //Serial.println(micros());                                             // code executed in between!

        while(micros() < time_now + period){                                  // wait approx. [period] ms -- instead of using delay, since
            // Serial.println(micros());
        }                                                                     // while delay, the program halts and no buffer is written etc.
        //Serial.print("K on: ");
        //Serial.println(micros());
        coupling = !coupling;                                                 // toggle coupling state to 'on', now that free-running phase is buffered
        if (bufferIdx == (BUFFER_SIZE-1) && triggerRun == 1){
            // this is just here to make the processing time for coupled and uncoupled state equally long
        }
    }

    while(micros() < time_now){
        // this is just here to make the processing time for coupled and uncoupled state equally long
    }
    // data buffer is full
    if (bufferIdx == (BUFFER_SIZE-1) && triggerRun == 1){                     // flush data buffer to serial port
        //Serial.println("Now detach timerISR!");
        detachCoreTimerService(timerISR);                                     // detach the timerISR to prevent that the buffer is overwritten before it is saved
        //write2buffer = !write2buffer;                                       // alternative to deattaching the timeISR - disable buffer write
        coupling = !coupling;                                                 // turn off coupling
        Serial.print("Buffer full! bufferIdx: ");                             // check the bufferIdx state
        Serial.println(bufferIdx,DEC);
        writeoutmode = !writeoutmode;                                         // toggle boolean variable: here toggle true, such that display blinks to signal write out phase
        updateDisplay();
        serialWriteDataBuffer();                                              // write out buffer
        writeoutmode = !writeoutmode; updateDisplay();                        // toggle boolean variable: here toggle false, such that display does NOT blink, signals measurement phase

        //write2buffer = !write2buffer;                                       // alternative to deattaching the timeISR - enable buffer write
        attachCoreTimerService(timerISR);                                     // reattach the timerISR service to continue
        //Serial.println(random(1,300));
        delay(random(1,300));
        triggerRun = 0;                                                       // reset switch
        //coupling = !coupling;                                               // turn on coupling (only if set true in setup and in above if condition)
        start_it = !start_it;                                                 // enable next measurement
    }
}

/**********************************************
              TIMER INTERRUPT
**********************************************/

uint32_t timerISR(uint32_t currentTime){
    // interrupt trigger [PORT 46]
    LATFSET = B11;
    // Serial.print("Doublecheck 6 - timerISR periods: ");
    // Serial.println(micros());

    // moved here, that it will be carried out in the coupled and uncoupled case
    delayedIdx = (bufferIdx + BUFFER_SIZE - transmissionDelay*DELAY_SCALE)%BUFFER_SIZE;
    outBits    = virtualCoupling(dataBuffer[delayedIdx]);                       // read delayed states

    // free running PLLs
    //if (!coupling and write2buffer ) {
    if (!coupling) {
      //Serial.println("in timerISR: uncoupled");
      LATESET               = 0xFF;                                             // set all output pins to HIGH

      inBits                = PORTB bitand 0xFF;                                // read data from PLLs [NOTE: '&' used to be 'bitand']
      dataBuffer[bufferIdx] = inBits;                                           // write input bits to buffer
      bufferIdx             = (bufferIdx + 1) % BUFFER_SIZE;                    // increment buffer index
    }
    // delay coupled PLLs
    //else if (coupling and write2buffer ) {
    else {
      //Serial.print("Doublecheck 6 - timerISR periods: ");
      //Serial.println(micros());
      //Serial.println("OK");
      //Serial.println("in timerISR: coupled");
      LATESET               = outBits bitand 0xFF;                              // write data [NOTE: '&' used to be 'bitand']
      LATECLR               = ~outBits bitand 0xFF;                             // write data [NOTE: '&' used to be 'bitand']

      inBits                = PORTB bitand 0xFF;                                // read data from PLLs [NOTE: '&' used to be 'bitand']
      dataBuffer[bufferIdx] = inBits;                                           // write input bits to buffer
      bufferIdx             = (bufferIdx + 1) % BUFFER_SIZE;                    // increment buffer index
    }
    LATFCLR = B11;                                                              // trigger OFF

    // CORE_TICK_RATE of chipKit max32: 40000 core_ticks per ms, hence a core-tick takes Tcore = 2.5E-8 s
    // hence, if SAMPLERATE = 300 Samples/ms [i.e. 300kHz], then time is incemented in steps of
    // Tsample = 1/300E3 = 3.33E-6 s, which corresponds in turn to Tsample/Tcore = 133.3 core ticks,
    // i.e., CORE_TICK_RATE has to be divided by SAMPLERATE = 300 1/ms, which is 300 kHz
    // this also implies, that the dataBuffer entries in this case are spaced in time by Tsample = 3.33E-6 s
    // to realize a delay of 1ms, one needs to go back bufferIdx = 300 entries in the dataBuffer, which means
    // that is one wants a transmissionDelayMax = 99% of one period of the free-running frequency of ~1000Hz
    // the DELAY_SCALE = 3
    // HENCE: there are a bit more than 3 periods of the free running frequency that fit into the buffer of length 1024
    return(currentTime+CORE_TICK_RATE/SAMPLERATE);
}

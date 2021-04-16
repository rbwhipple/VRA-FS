/*
 * Sweep Generator Arduino Code
 * Copyright Dick Whipple 2020
 * Version 1.0 (Nuts & Volts Final)
 * 10 April 2021
*/

#include <TFT.h>               // TFT display library: https://github.com/arduino-libraries/TFT
#include <Rotary.h>            // Rotary encoder library: https://github.com/brianlow/Rotary 


// Pin definitions for the TFT display DSP1:
#define CS 10
#define DC 9
#define RST 12
// Note: pins 11 (MOSI) and 13 (SCK) are predefined in the TFT display library
// Operational definitions for the 1.8" TFT display
#define RANGE 100 // Vertical size of the graph in pixels.
#define WIDTH 128 // Horizontal size of Display in pixels.
#define HEIGHT 160 // Vertical size of Display in pixels.
#define PERSIST 10 // persistence of the graph (milliseconds).

// Create TFT object
TFT TFTscreen = TFT(CS, DC, RST);

// Pin definitions for AD9850 module FS1:
#define W_CLK 8    
#define FQ_UD 7       
#define DATA  6       
#define RESET 5     
#define AD9850_CLOCK 125000000         // Module crystal frequency

// Pin definitions for rotary encoder REN1 and REN2:
#define stepPin1 A3                    // REN1 Frequency step encoder clock and data pins
#define stepPin2 A2
#define stepPin3 2                     // REN2 Frequency encoder clock and data pin
#define stepPin4 3 

// Pin definitions for option and mode pus buttons
int stepOpMode = A1;                     // SW2 Option push button pin
int stepOption = 4;                      // SW3 Mode push button pin

//?Rotary i = Rotary(stepPin1, stepPin2); // REN1 Frequency step encoder object
//?Rotary r = Rotary(stepPin3, stepPin4); // REN2 Frequency encoder object

// Create encoder objects
Rotary freqStepObject = Rotary(stepPin1, stepPin2); // REN1 Frequency step encoder object
Rotary freqObject = Rotary(stepPin3, stepPin4); // REN2 Frequency encoder object

// Frequency step labels
const char* stepText[15] = {"  1 Hz", "  5 Hz", " 10 Hz", " 50 Hz", "100 Hz", "500 Hz", "  1 kHz",
                     "  5 kHz", " 10 kHz", " 50 kHz", "100 kHz", "500 kHz", "  1 mHz", "  5 mHz"};

// Initial frequency step (1000 Hz)
int stepPointer = 6; 
unsigned long  incr = 1000;
String units = stepText[stepPointer];

// Sweeep mode labels
// "AM IF(-)" - AM w/ negative going sweep voltage (most tube radios)
// "AM IF(+)" - AM w/ postive going sweep voltage (most transistor radios)
// "FM IF(-)" - FM IF alignment w/ negative going sweep voltage
// "FM IF(+)" - FM IF alignment w/ positive going sweep voltage
// "FM SS" - Single S-curve w/ positive and negative going sweep voltage
// "FM DS" - Doubled S-curve w/ positive and negative going sweep voltage
const char* sweepModeText[7] = {"AM IF(-)KHz","AM IF(+)KHz", "FM IF(-)MHz","FM IF(+)MHz", "FM SS(+-)MHz", "FM DS(+-)MHz"};

// Initial operation mode (AM w/ negative and negative going sweep voltage)
// 0-Sweep mode;1-Set freq; 2-Set sweep width; 3-Set low marker; 4-Set high marker
int opMode = 0;
int opModeOld = 5;

int value ;   //Misc data value
int xPos= 0; // Sweep position; 0 - starting frequency to 127 = ending frequency
int noiseFloor = 0;  // Analog data value of off center frequency noise level
//xxxint analogFloorA0 = floorAnalogA0 * 1023 / 5;
int dsZeroLevelRaw = 1023 / 2, dsZeroLevelScaled = map(dsZeroLevelRaw,0,1023,129,23);
int startFreqRaw;

long unsigned int freq = 455000;         // Set initial frequency.
long unsigned int freqPlot = freq;       // Frequency to plot
long unsigned int freqOld = freq;        // Frequency at xPos on previous sweep
long unsigned int freqAtMax;             // Frequency where maximum amplitude occurs
long unsigned int freqCenter = 455000;   // Center frequency for sweep
long unsigned int freqSweepWidth = 30000;// Width of sweep
long unsigned int freqMarkerLower = freqCenter - 5000;  // Lower marker frequency (below center frequency)
long unsigned int freqMarkerUpper = freqCenter + 5000; // Upper marker frequency (above center frequency)
long unsigned int freqBW = 0;            // Width at -3db frequencies based on amplitude at center frequency

// Misc amplitude and decibel variables
int ampPeakFreqIndex, ampMarkerFreqLowerIndex, ampMarkerFreqUpperIndex, ampMarkerFreqVarIndex, ampMarkerFreqVarIndexOld;
float ampPeakFreq, ampMarkerFreqLower, ampMarkerFreqUpper, ampMarkerFreqVar;
float dbPeakFreq, dbMarkerFreqLower, dbMarkerFreqUpper, dbMarkerFreqVar, dbCenterFreq;
String dbPeakFreqText, dbMarkerFreqLowerText, dbMarkerFreqUpperText, dbMarkerFreqVarText;
String markerFreqBalText, markerFreqAmpText, markerFreqZeroText;
int ampData[128], ampMax, ampMaxIndex, ampBWLowerIndex, ampBWUpperIndex, ampBWLowerIndexOld, ampBWUpperIndexOld; // Amplitude data
int graphBias = 425;
float sqrRoot2 = sqrt(2);

// Sweep mode w/ initial AM (-) sweep
int sweepMode = 0; // 0 - AM IF 455kHz(-)

//
// AD9850 communication code
//
#define pulseHigh(pin) {digitalWrite(pin, HIGH); digitalWrite(pin, LOW); }

 // Transfers a byte, a bit at a time, LSB first to the 9850 via serial DATA line
void tfr_byte(byte data) {
  for (int i = 0; i < 8; i++, data >>= 1) {
    digitalWrite(DATA, data & 0x01);
    pulseHigh(W_CLK);   //after each bit sent, CLK is pulsed high
  }
}

//
// Set frequency of AD9050 module
//
void sendFrequency(double frequency) {
  int32_t freq1 = frequency * 4294967295/AD9850_CLOCK;  // note 125 MHz clock on 9850
  for (int b = 0; b < 4; b++, freq1 >>= 8) {
    tfr_byte(freq1 & 0xFF);
  }
  tfr_byte(0x000);                     // Final control byte, all 0 for 9850 chip
  pulseHigh(FQ_UD);                    // Done!  Should see output
}

//
// Translate step pointer to actual frequency increment
//
void getStep() {
  switch(stepPointer) {
    case 0:  incr = 1; break;
    case 1:  incr = 5; break;
    case 2:  incr = 10; break;
    case 3:  incr = 50; break;
    case 4:  incr = 100; break;
    case 5:  incr = 500; break;
    case 6:  incr = 1000; break;
    case 7:  incr = 5000; break;
    case 8:  incr = 10000; break;
    case 9:  incr = 50000; break;
    case 10: incr = 100000; break;
    case 11: incr = 500000; break;
    case 12: incr = 1000000; break;
    case 13: incr = 5000000; break;
    } 
}

//
// Clear upper text frame
//
void clearUpperFrame(){
TFTscreen.stroke(255,255,255); // white stroke 
TFTscreen.fill( 255, 255, 255 ); // White fill
TFTscreen.rect(0, 0, WIDTH, 21);
}

//
// Display text in upper frame
//
void displayUpperFrame(String line1Text, String line2Text) {
  clearUpperFrame();
  TFTscreen.stroke(0,0,0); // black text
  char tempCharArray[24];
  line1Text.toCharArray(tempCharArray, 24);
  TFTscreen.text(tempCharArray,2,1);
  line2Text.toCharArray(tempCharArray, 24);
  TFTscreen.text(tempCharArray,2,12);
}

//
// Clear lower frame text
//
void clearFrameLower(){
  TFTscreen.stroke(255,255,255); // white stroke 
  TFTscreen.fill( 255, 255, 255 ); // White fill
  TFTscreen.rect(0, 131, WIDTH, 28);
  delay(200);
}

//
// Display text in lower frame
//
void displayFrameLower(String line1Text, String line2Text, String line3Text) {
  clearFrameLower();
  TFTscreen.stroke(0,0,0); // black text
  char tempCharArray[24];
  line1Text.toCharArray(tempCharArray, 24);
  TFTscreen.text(tempCharArray,1,131);
  line2Text.toCharArray(tempCharArray, 24);
  TFTscreen.text(tempCharArray,1,141);
  line3Text.toCharArray(tempCharArray, 24);
  TFTscreen.text(tempCharArray,1,151);
}

//
// Clear graph frame only
//
void clearGraph(){
  TFTscreen.stroke(127,127,127); // grey for the border
  TFTscreen.fill( 180, 0, 0 ); // dark Blue for the background
  TFTscreen.rect(0, 22, 128, 108);
  TFTscreen.line(64, 23, 64, 129); // vertical line
  ampMarkerFreqLowerIndex = (float)(freqMarkerLower - (freqCenter - freqSweepWidth / 2)) / (float)freqSweepWidth * 128;
  TFTscreen.line(ampMarkerFreqLowerIndex, 23, ampMarkerFreqLowerIndex, 129); // vertical line 
  ampMarkerFreqUpperIndex = (float)(freqMarkerUpper - (freqCenter - freqSweepWidth / 2)) / (float)freqSweepWidth * 128;
  TFTscreen.line(ampMarkerFreqUpperIndex, 23, ampMarkerFreqUpperIndex, 129); // vertical line 
  ampMarkerFreqVarIndex = (float)(freq - (freqCenter - freqSweepWidth / 2)) / (float)freqSweepWidth * 128;
}

//
// Clear previous plot in graph frame
//
void clearPlot() {
  int value;
  TFTscreen.stroke(180,0,0); // dark blue stroke for erasing plot.
  
  for (int xPos; xPos < WIDTH; xPos++) {
    
    //
    // Sweepmode dependent code
    //
    if (sweepMode == 5) {
      value= map(ampData[xPos] - startFreqRaw,0,1023,129,23);      
    }
    else if (sweepMode == 4) {
      value= map(ampData[xPos],0,1023,129,23);      
    }
    else {
      value= map(ampData[xPos],0,512,129,23);      
    }
    if(sweepMode == 5) {
      TFTscreen.point(xPos, value );        
      value= map(ampData[127 - xPos] - startFreqRaw,0,1023,129,23);
      if(value <= 23) value= 23; //truncate off screen values.
      TFTscreen.point(xPos, value );               
    }
    else {
      TFTscreen.point(xPos, value );        
    } 
  
    if ((xPos == ampBWLowerIndexOld || xPos == ampBWUpperIndexOld) && sweepMode < 4) {
      for (int j = -3;j < 4;j++) {
        TFTscreen.point(xPos, value + j); // draw line.
      }    
     }
    if (xPos == ampMarkerFreqVarIndexOld) {
      for (int j = -5;j < 6;j++) {
        TFTscreen.point(xPos, value + j); // draw line.
      }    
     }
  }
  TFTscreen.stroke(0,255,255); // back to yellow stroke for plot.
  TFTscreen.line(64, 23, 64, 129); // vertical line
  TFTscreen.line(0, 23, 0, 129); // vertical line
  TFTscreen.line(127, 23, 127, 129); // vertical line
  //
  // Sweepmode dependent code
  //
  if (sweepMode == 4  || sweepMode == 5) {
    TFTscreen.line(1, dsZeroLevelScaled,127, dsZeroLevelScaled); // horizontal line      
  }
  ampMarkerFreqLowerIndex = (float)(freqMarkerLower - (freqCenter - freqSweepWidth / 2)) / (float)freqSweepWidth * 128;
  TFTscreen.line(ampMarkerFreqLowerIndex, 23, ampMarkerFreqLowerIndex, 129); // vertical line 
  ampMarkerFreqUpperIndex = (float)(freqMarkerUpper - (freqCenter - freqSweepWidth / 2)) / (float)freqSweepWidth * 128;
  TFTscreen.line(ampMarkerFreqUpperIndex, 23, ampMarkerFreqUpperIndex, 129); // vertical line 
  ampMarkerFreqVarIndex = (float)(freq - (freqCenter - freqSweepWidth / 2)) / (float)freqSweepWidth * 128;
}
//
// Format text before display
//
String lineFormat(long freqTemp0, long freqTemp1, long freqTemp2, int type) {
  
  String fT0Text,fT1Text,fT2Text, spacesText = "          ";
  int fT0Len, fT1Len, fT2Len, spaces;
 
  if (freqTemp0 < 1000000) {
    fT0Text = String(freqTemp0/1000);fT1Text = String(freqTemp1/1000);fT2Text = String(freqTemp2/1000);
  }
  else {
    fT0Text = String((float)freqTemp0/1000000.,3);fT1Text = String((float)freqTemp1/1000000.);fT2Text = String((float)freqTemp2/1000000.,3);
  }
    fT0Len = fT0Text.length();fT1Len = fT1Text.length();fT2Len = fT2Text.length();
  spaces = 21 - (fT0Len + fT1Len + fT2Len);
  if (type == 0) {
    return spacesText.substring(0, spaces/4) + fT0Text + spacesText.substring(0, spaces/4) + fT1Text + spacesText.substring(0, spaces/4) + fT2Text;
  }
  else {
    return fT0Text + spacesText.substring(0, spaces/2) + fT1Text + spacesText.substring(0, spaces/2) + fT2Text;
  }

}
//
// Update text in upper frame
//
void displayUpdate(String stringTemp) {
  String freqText = stringTemp + String(freq/1000) + "KHz";      
  String unitsText = "Step Size: " + units;
  displayUpperFrame(freqText,unitsText);
}

void setup(){

  Serial.begin(9600);

  pinMode(FQ_UD, OUTPUT);              // Configure pins for output to AD9850 module.
  pinMode(W_CLK, OUTPUT);
  pinMode(DATA, OUTPUT);
  pinMode(RESET, OUTPUT);

  pinMode(stepPin1, INPUT_PULLUP);     // Configure pins for frequency step encoder
  pinMode(stepPin2, INPUT_PULLUP);

  pinMode(2, INPUT_PULLUP);            // Configure pins for frequency encoder
  pinMode(3, INPUT_PULLUP);
  
  pinMode(stepOpMode, INPUT_PULLUP);   // Configure push button for opmode step
  pinMode(stepOption, INPUT_PULLUP);   // Configure push button for pararmeter option step
    
  TFTscreen.begin(); // Initialize the display
  TFTscreen.setRotation(0);
  TFTscreen.background(255,255,255); // Set white background
  TFTscreen.stroke(0,0,0); // Set black text
  TFTscreen.setTextSize(1);
  TFTscreen.text("   WSAB Ver N-V 1",1,75);
  delay(3000);
  clearUpperFrame();
  clearGraph();

  //
  // Initialise the AD9850 module.
  // 
  pulseHigh(RESET);
  pulseHigh(W_CLK);
  pulseHigh(FQ_UD);    // Enable serial mode  
  delay(100);
  
  //
  // Begin freq control processes
  //
  freqStepObject.begin();   // Begin frequency step process
  freqObject.begin();       // Begin frequency process

  //
  // Prepare interrupts
  //
  PCICR |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);
  sei();  
}

void loop(){

  if (digitalRead(stepOpMode) == LOW) {  
    opMode += 1;
    if (opMode == 5) opMode = 0;
    delay(350);
  }

  if (opMode == 0) {
    //
    // Sweep Mode
    //
    if (opMode != opModeOld) {
      displayFrameLower("", "", "     Sweep Mode");
      opModeOld = opMode;
      startFreqRaw = 0;
      sendFrequency(freqCenter / 5);
      delayMicroseconds(150);
      //
      // Sweepmode dependent code
      //
      // Set noise floor at 1/5 of center frequency
      //
      if (sweepMode == 0 || sweepMode == 2) {
        noiseFloor= 512 - analogRead(A0);        
      }
      else if (sweepMode == 1 || sweepMode == 3) {
        noiseFloor= analogRead(A0) - 512;         
      }
      else {
        noiseFloor= 0;
        sendFrequency(freqCenter / 5);
        delayMicroseconds(150);
        startFreqRaw = analogRead(A0) - 512;
      }
      incr = 1000;
      clearGraph();
    }
    //
    // Get response curve amplitudes
    //
    for (int xPos = 0; xPos < WIDTH; xPos++){
      
      freqPlot = freqCenter + ((long)(xPos-64)) * (freqSweepWidth/128);
      sendFrequency(freqPlot);
      delayMicroseconds(150);
      //
      // Sweepmode dependent code
      //   
      if (sweepMode == 0 || sweepMode == 2) {
        value= 512 - analogRead(A0);        
      }
      else if (sweepMode == 1 || sweepMode == 3) {
        value= analogRead(A0) - 512;         
      }
      else {
        value= analogRead(A0);       
      }
      ampData[xPos] = value;

    }
    //
    // Plot response curve
    //
    for (int xPos = 0; xPos < WIDTH; xPos++){
      
      TFTscreen.stroke(0,255,255); // yellow stroke for plotting.

      //
      // Sweepmode dependent code
      //
      if (sweepMode == 5) {
        value= map(ampData[xPos] - startFreqRaw,0,1023,129,23);      
      }
      else if (sweepMode == 4) {
        value= map(ampData[xPos],0,1023,129,23);      
      }
      else {
        value= map(ampData[xPos],0,512,129,23);      
      }
      
      if(value <= 23) value= 23; //truncate off screen values.
      if(sweepMode == 5) {
        TFTscreen.point(xPos, value );        
        value= map(ampData[127 - xPos] - startFreqRaw,0,1023,129,23);
        if(value <= 23) value= 23; //truncate off screen values.
        TFTscreen.point(xPos, value );               
      }
      else {
        TFTscreen.point(xPos, value );        
      } 
      if ((xPos == ampBWLowerIndex || xPos == ampBWUpperIndex) && sweepMode < 4) {
        TFTscreen.stroke(0,0,255);
        for (int j = -3;j < 4;j++) {
          TFTscreen.point(xPos, value + j); // draw line.
        }
        TFTscreen.stroke(0,255,255);
      }
      if (xPos == ampMarkerFreqVarIndex) {
        TFTscreen.stroke(0,255,0);
        for (int j = -5;j < 6;j++) {
          TFTscreen.point(xPos, value + j); // draw line.
        }
        TFTscreen.stroke(0,255,255);    
      }
    }
    
    ampMarkerFreqVarIndexOld = ampMarkerFreqVarIndex;
    ampBWLowerIndexOld = ampBWLowerIndex;
    ampBWUpperIndexOld = ampBWUpperIndex;
    
    // Look for peak frequency
    ampMax = 0;
    for (int i=0;i<WIDTH;i++) {
      if (ampData[i] > ampMax) {
        ampMax = ampData[i];
        ampMaxIndex = i;
      }
    }
    //
    // Look for below center frequency -3 dB point
    //
    ampBWLowerIndex = 0;
    for (int i=0;i<ampMaxIndex;i++) {
      if ((ampMax - noiseFloor) - sqrRoot2 * (ampData[i] - noiseFloor) <= 0) {
        ampBWLowerIndex = i;
        break;
      }
    } 
    //
    // Look for above center frequency -3 dB point
    //
    ampBWUpperIndex = WIDTH - 1;   
    for (int i=ampBWUpperIndex;i>ampMaxIndex;i--) {
      if ((ampMax - noiseFloor) - sqrRoot2 * (ampData[i] - noiseFloor) <= 0) {
        ampBWUpperIndex = i;
        break;
      }
    }
    //
    // Update text in frames
    //
    ampPeakFreq = ampMax - noiseFloor;
    freqAtMax = (freqCenter + (long(ampMaxIndex-64) * (long)freqSweepWidth/128));
    freqBW = ((long)(ampBWUpperIndex - ampBWLowerIndex) * freqSweepWidth/128)/1000;
    TFTscreen.setTextSize(1);
    String freqAtMaxText = "";
    String freqSpreadText = lineFormat((freqCenter - freqSweepWidth/2), freqCenter, (freqCenter + freqSweepWidth/2), 1);
    displayUpperFrame("Swp Mode " + String(sweepModeText[sweepMode]),freqSpreadText);
    
    if (freqAtMax < 1000000) {
     freqAtMaxText = " PF:" + String(freqAtMax/1000) + "KHz"; 
    }
    else {
     freqAtMaxText = " PF:" + String((float)freqAtMax/1000000.) + "MHz";       
    }
    //
    // Sweepmode dependent code
    //
    if (sweepMode < 4) {
      dbMarkerFreqLowerText = String(round(200.0 * log10((float)(ampData[ampMarkerFreqLowerIndex] - noiseFloor) / ampPeakFreq)) / 10.0);
      dbMarkerFreqUpperText = String(round(200.0 * log10((float)(ampData[ampMarkerFreqUpperIndex] - noiseFloor) / ampPeakFreq)) / 10.0);
      dbMarkerFreqVarText = String(round(200.0 * log10((float)(ampData[ampMarkerFreqVarIndex] - noiseFloor) / ampPeakFreq)) / 10.0);
      displayFrameLower(lineFormat(freqMarkerLower, freq, freqMarkerUpper, 0),dbMarkerFreqLowerText.substring(0,4) + "db  " + dbMarkerFreqVarText.substring(0,4) + "db " +  dbMarkerFreqUpperText.substring(0,4) + "db",freqAtMaxText +" BW:" + String (freqBW) + "KHz");
    }
    else {
      markerFreqBalText = String(ampData[ampMarkerFreqUpperIndex] + ampData[ampMarkerFreqLowerIndex] - 1023);
      markerFreqAmpText = String((ampData[ampMarkerFreqUpperIndex] - ampData[ampMarkerFreqLowerIndex]) / 2);
      markerFreqZeroText = String(ampData[64] - 512);
      displayFrameLower(lineFormat(freqMarkerLower, freq, freqMarkerUpper, 0),"  Amp: " + markerFreqAmpText + " Bal: " + markerFreqBalText,"     Zero: " + markerFreqZeroText);
    }
    //
    // Check for sweep mode change
    //
    if (digitalRead(stepOption) == LOW) {
      sweepMode += 1;
      switch (sweepMode) {
        case 0:
          freq = 455000;
          freqCenter = freq;
          freqSweepWidth = 30000;
          freqMarkerLower = 455000 - 5000;
          freqMarkerUpper = 455000 + 5000;
          stepPointer = 6;
          getStep();                            
          units = stepText[stepPointer];
          delay(350);
          break;
          
        case 1:
          freq = 455000;
          freqCenter = freq;
          freqSweepWidth = 30000;
          freqMarkerLower = 455000 - 5000;
          freqMarkerUpper = 455000 + 5000;
          stepPointer = 6;
          getStep();                             
          units = stepText[stepPointer];
          delay(350);
          break;
          
        case 2:
          freq = 10700000;
          freqCenter = freq;
          freqSweepWidth = 400000;
          freqMarkerLower = 10700000 - 100000;
          freqMarkerUpper = 10700000 + 100000;
          stepPointer = 8;
          getStep();                             
          units = stepText[stepPointer];
          delay(350);
          break;
       
        case 3:
          freq = 10700000;
          freqCenter = freq;
          freqSweepWidth = 400000;
          freqMarkerLower = 10700000 - 100000;
          freqMarkerUpper = 10700000 + 100000;
          stepPointer = 8;
          getStep();                             
          units = stepText[stepPointer];
          delay(350);
          clearGraph();
          break;

        case 4:
          freq = 10700000;
          freqCenter = freq;
          freqSweepWidth = 600000;
          freqMarkerLower = 10700000 - 100000;
          freqMarkerUpper = 10700000 + 100000;
          stepPointer = 8;
          getStep();                             
          units = stepText[stepPointer];
          clearGraph();
          delay(350);
          break;

        case 5:
          freq = 10700000;
          freqCenter = freq;
          freqSweepWidth = 600000;
          freqMarkerLower = 10700000 - 100000;
          freqMarkerUpper = 10700000 + 100000;
          stepPointer = 8;
          getStep();                             
          units = stepText[stepPointer];
          clearGraph();
          delay(350);
          break;
          
        case 6:
          freq = 455000;
          freqCenter = freq;
          freqSweepWidth = 30000;
          freqMarkerLower = 455000 - 5000;
          freqMarkerUpper = 455000 + 5000;
          stepPointer = 6;
          getStep();                             
          units = stepText[stepPointer];
          sweepMode = 0;
          break;
      }
       clearGraph();
    }
    delay(PERSIST);
    clearPlot();
  }
  else if (opMode == 1) {
    //
    // Set Center Frequency Mode
    //
    if (opMode != opModeOld) {
      displayFrameLower("Set Center Frequency", "", "");
      opModeOld = opMode;
      freq = freqCenter;
      sendFrequency(freq);
      freqOld = freq;
      stepPointer = 7;
      getStep();                             
      units = stepText[stepPointer];
      displayUpdate("Center Freq: ");
      clearGraph();
    }
      // Check 'Step' rotary encoder.
      unsigned char result = freqStepObject.process();
      if (result) {
        if (result == DIR_CW)  {if (stepPointer < 13) stepPointer++;}
        if (result == DIR_CCW) {if (stepPointer > 0) stepPointer--;} 
        getStep();
        units = stepText[stepPointer];     
        displayUpdate("Center Freq: ");
      }
     
      if (freqOld != freq) {
        sendFrequency(freq);
        delayMicroseconds(10);
        displayUpdate("Center Freq: ");
        freqOld = freq;
        freqCenter = freq;
      }
  }
  else if (opMode == 2) {
    //
    // Set Sweep Width Mode
    //
    if (opMode != opModeOld) {
      displayFrameLower("   Set Sweep Width", "", "");
      opModeOld = opMode;
      freq = freqSweepWidth;
      freqOld = freq;
      stepPointer = 7;
      getStep();                             
      units = stepText[stepPointer];
      sendFrequency(freqCenter);
      displayUpdate("Sweep Width: ");
      clearGraph();
    }
      // Check 'Step' rotary encoder.
      unsigned char result = freqStepObject.process();
      if (result) {
        if (result == DIR_CW)  {if (stepPointer < 13) stepPointer++;}
        if (result == DIR_CCW) {if (stepPointer > 0) stepPointer--;} 
        getStep();
        units = stepText[stepPointer];     
        displayUpdate("Sweep Width: ");
      }
     
      if (freqOld != freq) {
        getStep();
        displayUpdate("Sweep Width: ");
        freqOld = freq;
        freqSweepWidth = freq;
      }
 }
  
  else if (opMode == 3) {
    //
    // Set Lower Marker Frequency Mode
    // 
    if (opMode != opModeOld) {
      displayFrameLower( "   Set Low Marker", "", "");
      opModeOld = opMode;
      freq = freqMarkerLower;
      sendFrequency(freq);
      freqOld = freq;
      stepPointer = 7;
      getStep();                             
      units = stepText[stepPointer];
      displayUpdate("Low Marker: ");
      clearGraph();
    }
      
      // Check 'Step' rotary encoder.
      unsigned char result = freqStepObject.process();
      if (result) {
        if (result == DIR_CW)  {if (stepPointer < 13) stepPointer++;}
        if (result == DIR_CCW) {if (stepPointer > 0) stepPointer--;} 
        getStep();
        units = stepText[stepPointer];     
        displayUpdate("Low Marker: ");
      }
     
      if (freqOld != freq) {
        sendFrequency(freq);
        getStep();
        displayUpdate("Low Marker: ");
        freqOld = freq;
        freqMarkerLower = freq;
      }
   }
  
  else if (opMode == 4) {
    //
    // Set Upper Marker Frequency Mode
    //  
     if (opMode != opModeOld) {
      displayFrameLower("   Set High Marker", "" ,"");
      opModeOld = opMode;
      freq = freqMarkerUpper;
      sendFrequency(freq);
      freqOld = freq;
      stepPointer = 7;
      getStep();                            
      units = stepText[stepPointer];
      displayUpdate("High Marker: ");
      clearGraph();
    }

          // Check 'Step' rotary encoder.
      unsigned char result = freqStepObject.process();
      if (result) {
        if (result == DIR_CW)  {if (stepPointer < 13) stepPointer++;}
        if (result == DIR_CCW) {if (stepPointer > 0) stepPointer--;} 
        getStep();
        units = stepText[stepPointer];     
        displayUpdate("High Marker: ");
      }
     
      if (freqOld != freq) {
        sendFrequency(freq);
        getStep();
        displayUpdate("High Marker: ");
        freqOld = freq;
        freqMarkerUpper = freq;
      }
 }
  
  else {
      
  }

}


ISR(PCINT2_vect) {
  unsigned char result = freqObject.process();
  if (result) {
    if (result == DIR_CW) {
      if ((freq + incr) <= 20000000) freq += incr;
    } else {
      if ((freq - incr) >= 10) freq -= incr;
    }
    if (freq <= 10)  freq = 10;
    if (freq >=20000000) freq = 20000000;
  }
}

//---------------------------------------------------------------------------//`
#define MIC_PIN A0
#define MIN_HZ 75 // E2
unsigned int min_idx;
#define MAX_HZ 800 // E5
unsigned int max_idx;
#define SAMPLES 128             //Must be a power of 2
#define SAMPLING_FREQUENCY 2500 //Hz, must be less than 10000 due to ADC
#define TOP_FREQ_NUM 4
#define NOTE_BUF_SIZE 4
#define NOTE_WINDOW 3 // Wait for three notes to come in before deciding
#define DBL_STRUM_MS 600
#define STAIRS_MODE // STAIRS_MODE /  

const double TWELVE_OVER_LN2 = 17.312340490667562;
const char* const noteTable[] PROGMEM = {"E2", "F2", "F#2", "G2", "G#2", "A2", "A#2", "B2", "C3", "C#3", "D3", "D#3", "E3", "F3", "F#3", "G3", "G#3", "A3", "A#3", "B3", "C4", "C#4", "D4", "D#4", "E4", "F4", "F#4", "G4", "G#4", "A4", "A#4", "B4", "C5", "C#5", "D5", "D#5", "E5", "F5"};
const char* const chromaticAlphabet[] PROGMEM = {'a','b','c','d','e','f','g','h','i','j','k',40,41,'n','o','p','q','r','s','t','u','v','w','x','y','1','2','3','4','5','6','7','8','9','0', '(', ')'};
const char* const scaleAlphabet1[] PROGMEM = {0, 0, 0, 0, 0xB2, 't', 0, 'd', 'u', 0,'s', 0, 'l', 'e', 0, 'f', 0, 'p', 0, 'r', 'i', 0, 'n', 0, 't', 'o', 0, 40, 0, 41, 0, ' ', '\t', 0xB2 ,0, 0,0, 0,'X'};
const char* const scaleAlphabet2[] PROGMEM = {0, 0, 0, 0, 0, '0', 0, '1', '2', 0,'3', 0, '5', '7', 0, '9', 0, '!', 0, '=', '\"', 0, ':', 0, '-', '+', 0, '[', 0, ']', 0, '*', '/', '%'};
const char* const scaleAlphabet3[] PROGMEM = {0, 0, 0, 0, 0, '0', 0, '1', '2', 0,'3', 0, '5', '7', 0, '9', 0, '!', 0, '=', '\"', 0, ':', 0, '-', '+', 0, '[', 0, ']', 0, '*', '/', '%'};
const char* const *scaleAlphabets[3] = {scaleAlphabet1, scaleAlphabet2, scaleAlphabet3};

const char* const stairsAlphabet1[] PROGMEM = {0, 0, 0, 0, 0, '(', 0, 0, ':', 0, 0, 0, 0, ')', 0, 'a', 0, 'f', 0, '(', 'o', 0, 'e', 0, 'r', '\t', 'g', 0, 0, ' ', 0, 'i', 'n', 0 ,0, 0, 0, 0, '9'};
const char* const stairsAlphabet2[] PROGMEM = {0, 0, 0, 0, 0, ')', 0, 0, 'n', 0, 0, 0, 0, 'i', 0, '*', 0, '"', 0, 0, 'r', 0, 0, 0, 'p', 0, 0, 0, 0, 'i', 0, 0, '(', 0 ,0, 0, 't', 0, 0};
const char* const *stairsAlphabets[2] = {stairsAlphabet1, stairsAlphabet2};
#include "arduinoFFT.h"
#include "KickMath.h"
#include "Keyboard.h"
//---------------------------------------------------------------------------//

arduinoFFT FFT = arduinoFFT();
unsigned int sampling_period_us;
unsigned long microseconds;
int noteBuf[NOTE_BUF_SIZE] = {-1, -1, -1, -1};
int prevMed = 0;
int tmp[NOTE_BUF_SIZE] = {0};
int ctr = 0;
boolean silenceInBetween = true; // Whether there was silence inbetween this and last detected note.
float lastMag, prevLastMag;
unsigned long lastStrum = 0;

void setup() {
//  Serial.begin(115200);

  Keyboard.begin();
  sampling_period_us = round(1000000*(1.0/SAMPLING_FREQUENCY));
  min_idx = hzToIdx(MIN_HZ);
  int temp = 800;
  max_idx = hzToIdx(temp);
}

double vReal[SAMPLES];
double vImag[SAMPLES];
void loop() {
  /* Sampling */
  float avg = 0;
  float rms = 0;
  int val = 0;
  for(int i=0; i<SAMPLES; i++) {
        microseconds = micros();    //Overflows after around 70 minutes!

        val = analogRead(MIC_PIN) >> 2;
        vReal[i] = val;
        val -= 500 >> 2;
        avg += val;
        rms += val * val;
        
        vImag[i] = 0;
     
        while(micros() < (microseconds + sampling_period_us)){
        }
    }
    avg = avg / SAMPLES;
    rms = sqrt(rms / SAMPLES);
    float mag = rms-avg;

    /* Strum Detection */
    unsigned long now = millis();
    if (lastStrum != 0 && now - lastStrum > DBL_STRUM_MS) {
      // Waited for doubleStrum but it didn't come.
      lastStrum = 0;
        Keyboard.write('\n');
        
      #ifdef STAIRS_MODE
        Keyboard.write('\t');
      #endif
    }
    else if (lastMag > 35 && prevLastMag < 10 && mag < 10) {
      // New Strum!
      if (lastStrum != 0) {
        // Been waiting for doubleStrum!
        Keyboard.write(0xB2);
        lastStrum = 0;
      } else {
        // Start waiting for doubleStrum.
        lastStrum = now;
      }
    } else if (mag > 25) {
      /* FFT */
      FFT.DCRemoval(vReal, SAMPLES); // Potentially: Remove.
      FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_BLACKMAN, FFT_FORWARD);
      FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
      FFT.ComplexToMagnitude(vReal, vImag, SAMPLES); // mag in vReal
//      Serial.println(FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY));

      
      /* Analyze FFT */
      // Find 4 most dominant frequencies
      double freq_val[TOP_FREQ_NUM] = {0,0,0,0};
      double freq_mag[TOP_FREQ_NUM] = {0,0,0,0};
      findTopFreq(freq_val, freq_mag);

      // Determine Note 
      double note_freq = 0; // Note with E2 (82.41hz) being 0
      if (freq_val[1] < freq_val[0] && freq_mag[1] > (freq_mag[0] / 4)) {
        // Second most powerful frequency is lower than first and still considerable. Probably the fundamental.
        note_freq = freq_val[1];
      } else {
        note_freq = freq_val[0];
      }
      int note = round(TWELVE_OVER_LN2 * log(note_freq/440)) + 29;

      // Compare to previous notes
//      printDiag(freq_val, freq_mag);
      updateNoteBuf(note);
      int med = KickMath<int>::calcMedian(NOTE_BUF_SIZE, noteBuf, tmp);

      
      /*PRINT RESULTS*/
    
      if (ctr == NOTE_WINDOW) {
      if (med != prevMed || silenceInBetween) {
        // Note for sure. Register input.
        #ifdef SCALE_MODE
          scaleModeInput(med);
        #endif

        #ifdef STAIRS_MODE
          stairsModeInput(med);
        #endif
        prevMed = med;
      }
      ctr = 0;
      silenceInBetween = false;
      } else { ctr++; }
    } else {
      ctr  = 0;
      updateNoteBuf(-1);
      silenceInBetween = true;
    }
    prevLastMag = lastMag;
    lastMag = mag;
}

void updateNoteBuf(int note) {
  for (int i = NOTE_BUF_SIZE - 1; i > 0; i--) {
    if (noteBuf[i] == -1) {
      noteBuf[i] = note; // Replace all -1s with new value so if a note comes suddenly, we can detect it.
    } else {
      noteBuf[i] = noteBuf[i-1]; // Shift all elements right
    }
  }
  noteBuf[0] = note;
}

// Handles input in scale mode. note = 0 -> E2 note = 36 => E5
int scaleTriggerPresses = 0;; 
void scaleModeInput(int note) {
  if (note == 5) {
    // if trigger pressed, just increment counter.
    scaleTriggerPresses++;
    return;
  } 
  
  char c = (char)pgm_read_byte_near((scaleAlphabets[scaleTriggerPresses]) + note);
  scaleTriggerPresses = 0;
  if (c == 40 || c == 41 || c == '%' || c == '*' || c == ':' || c == '!' || c == '\"') {
    pressWithShift(c);
  } else {
    Keyboard.write(c); 
  }
}
void pressWithShift(char key) {
  Keyboard.press(KEY_LEFT_SHIFT);
  Keyboard.press(key);
  delay(10);
  Keyboard.releaseAll();
}
int stairsTriggerPresses = 0;
void stairsModeInput(int note) {
  char c = (char)pgm_read_byte_near((stairsAlphabets[stairsTriggerPresses]) + note);
  if (c == ':' || c == '\"' || c == '(' || c == ')' || c == '*') {
    pressWithShift(c);
  } else {
    Keyboard.write(c);
  }
  
  if (c == ':') {
    stairsTriggerPresses++;
  }
}

void findTopFreq(double *freq_val, double *freq_mag) {
  double mag, val;
  for (int i = min_idx; i <= max_idx; i++) {
    if ((vReal[i-1] < vReal[i]) && (vReal[i] > vReal[i+1])) { // if peak
      mag = vReal[i];
      val = idxToHz(i);
      // Insert value into freq_val & _mag if is in top. 
      for (int j = 0 ; j < TOP_FREQ_NUM; j++) {
        if (mag > freq_mag[j]) {
          // Found a top value within HZ range.
          for (int k = TOP_FREQ_NUM-1; k > j; k--) {
            // Scooch all values to the right.
            freq_val[k] = freq_val[k-1];
            freq_mag[k] = freq_mag[k-1];
          }
          // Interpolation magic
          double delta = 0.5 * ((vReal[i - 1] - vReal[i + 1]) / (vReal[i - 1] - (2.0 * vReal[i]) + vReal[i + 1]));
          double interpolatedX = ((i + delta)  * SAMPLING_FREQUENCY) / (SAMPLES - 1);
          // Insert new value
          freq_val[j] = interpolatedX * 0.98;
          freq_mag[j] = mag;
          break; // Inserted this value, go to next.
        }
      }
    }
  }
}

// Type key after chromatic alphaber
void chromaPrint(int key) {
  char c = (char)pgm_read_byte_near(chromaticAlphabet + key);
  if (c == 40 || c == 41) {
    Keyboard.press(KEY_LEFT_SHIFT);
    Keyboard.press(c);
    delay(10);
    Keyboard.releaseAll();
  } else {
    Keyboard.write(c);
  }
}

void serialPrint(int key) {
  char c = (char)pgm_read_byte_near(chromaticAlphabet + key);
    Serial.write(c);
}

void printDiag(double *freq_val, double *freq_mag) {
      // print top freq
      Serial.println("=====");
      Serial.println("Top Frequencies: ");
      for (int i = 0; i < TOP_FREQ_NUM; i++) {
        Serial.print(freq_val[i]);Serial.print('\t');
      }
      Serial.println();
      for (int i = 0; i < TOP_FREQ_NUM; i++) {
        Serial.print(freq_mag[i]);Serial.print('\t');
      }
      Serial.println();

      // print noteBuf
      Serial.print('[');
      Serial.print(noteBuf[0]);
      for (int i = 1; i < NOTE_BUF_SIZE; i++) {
        Serial.print(", "); Serial.print(noteBuf[i]);
      }
      Serial.print("]\t Counter: ");
      Serial.println(ctr);
      Serial.print("\t silence:");
      Serial.println(silenceInBetween);
      Serial.println("=====\n");
}
unsigned int idxToHz(int i) {
  return (i * 1.0 * SAMPLING_FREQUENCY) / SAMPLES;
}
unsigned int hzToIdx(int f) {
  long int temp = f;
  return int(temp * SAMPLES / SAMPLING_FREQUENCY);
}

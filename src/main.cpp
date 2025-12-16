#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_ADXL345_U.h>
#include <arduinoFFT.h>

// -------------------- TFT DEFINES --------------------
#define TFT_CS   9
#define TFT_DC   10
#define TFT_RST  -1

Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_RST);

// -------------------- ADXL345 DEFINES --------------------
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// -------------------- FFT CONFIG --------------------
// sampling at 52 Hz
// a power-of-two FFT size: 128 samples
// -> 2.46 s window

const uint16_t FFT_SAMPLES = 128;
const double SAMPLING_FREQUENCY = 52.0;

double vReal[FFT_SAMPLES];
double vImag[FFT_SAMPLES];

ArduinoFFT<double> FFT(vReal, vImag, FFT_SAMPLES, SAMPLING_FREQUENCY);

// -------------------- FREQUENCY BANDS --------------------
const double TREMOR_MIN_HZ     = 3.0;
const double TREMOR_MAX_HZ     = 5.0;
const double DYSKINESIA_MIN_HZ = 5.0;
const double DYSKINESIA_MAX_HZ = 7.0;

const double TREMOR_MAG_THRESHOLD     = 20;  
const double DYSKINESIA_MAG_THRESHOLD = 60;   

// -------------------- STATE ENUM --------------------
enum MovementState {
  STATE_NEUTRAL,
  STATE_TREMOR,
  STATE_DYSKINESIA
};

MovementState currentState = STATE_NEUTRAL;

// -------------------- FUNCTION PROTOTYPES --------------------
void initTFT();
void initAccel();
void captureSamples();
void analyzeFFT(double &tremorHz, double &tremorMag,
                double &dyskHz, double &dyskMag);
MovementState classifyMovement(double tremorHz, double tremorMag,
                               double dyskHz,   double dyskMag);
void updateScreen(MovementState state, double tremorHz, double dyskHz);
void drawNeutralScreen();
void drawTremorScreen(double tremorMag);
void drawDyskinesiaScreen(double dyskMag);

void setup() {
 
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for USB
  }

  initTFT();
  initAccel();

  tft.fillScreen(ILI9341_ORANGE);
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(3);

  tft.setCursor(40, 80);
  tft.println("Parkinson's");
  tft.setCursor(40, 120);
  tft.println("Detector");
  delay(1500);

  drawNeutralScreen();
}

void loop() {

  captureSamples();

  double tremorHz = 0.0, tremorMag = 0.0;
  double dyskHz   = 0.0, dyskMag   = 0.0;
  analyzeFFT(tremorHz, tremorMag, dyskHz, dyskMag);

  currentState = classifyMovement(tremorHz, tremorMag, dyskHz, dyskMag);

  updateScreen(currentState, tremorMag, dyskMag);

  delay(100);
}

//===========================================================

void initTFT() {
  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(ILI9341_WHITE);
}

void initAccel() {
  if (!accel.begin()) {
    // Sensor not found
    tft.fillScreen(ILI9341_RED);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(2);
    tft.setCursor(10, 10);
    tft.println("ADXL345 not found!");
    while (1) {
      delay(10);
    }
  }

  accel.setRange(ADXL345_RANGE_2_G);
  accel.setDataRate(ADXL345_DATARATE_50_HZ);
}

void captureSamples() {
  const double samplingPeriod_us = (1.0 / SAMPLING_FREQUENCY) * 1e6;

  for (uint16_t i = 0; i < FFT_SAMPLES; i++) {
    sensors_event_t event;
    accel.getEvent(&event);

    double ax = event.acceleration.x;
    double ay = event.acceleration.y;
    double az = event.acceleration.z;

    double mag = sqrt(ax * ax + ay * ay + az * az);

    vReal[i] = mag;
    vImag[i] = 0.0;

    delayMicroseconds((unsigned long)samplingPeriod_us);
  }
}

void analyzeFFT(double &tremorHz, double &tremorMag,
                double &dyskHz,   double &dyskMag) {
  double mean = 0.0;
  for (uint16_t i = 0; i < FFT_SAMPLES; i++) {
    mean += vReal[i];
  }
  mean /= FFT_SAMPLES;
  for (uint16_t i = 0; i < FFT_SAMPLES; i++) {
    vReal[i] -= mean;
  }

  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(FFT_FORWARD);
  FFT.complexToMagnitude();

  tremorHz = 0.0;
  tremorMag = 0.0;
  dyskHz = 0.0;
  dyskMag = 0.0;

  for (uint16_t bin = 1; bin < FFT_SAMPLES / 2; bin++) {
    double freq = (bin * SAMPLING_FREQUENCY) / FFT_SAMPLES;
    double mag  = vReal[bin];

    // Tremor band 3–5 Hz
    if (freq >= TREMOR_MIN_HZ && freq <= TREMOR_MAX_HZ) {
      if (mag > tremorMag) {
        tremorMag = mag;
        tremorHz  = freq;
      }
    }

    // Dyskinesia band 5–7 Hz
    if (freq >= DYSKINESIA_MIN_HZ && freq <= DYSKINESIA_MAX_HZ) {
      if (mag > dyskMag) {
        dyskMag = mag;
        dyskHz  = freq;
      }
    }
  }

  // Debug print
  Serial.print("Tremor peak: ");
  Serial.print(tremorHz, 2);
  Serial.print(" Hz, mag=");
  Serial.println(tremorMag, 3);

  Serial.print("Dyskinesia peak: ");
  Serial.print(dyskHz, 2);
  Serial.print(" Hz, mag=");
  Serial.println(dyskMag, 3);
}

MovementState classifyMovement(double tremorHz, double tremorMag,
                               double dyskHz,   double dyskMag) {
  bool tremorDetected = (tremorHz >= TREMOR_MIN_HZ &&
                         tremorHz <= TREMOR_MAX_HZ &&
                         tremorMag > TREMOR_MAG_THRESHOLD);

  bool dyskDetected   = (dyskHz >= DYSKINESIA_MIN_HZ &&
                         dyskHz <= DYSKINESIA_MAX_HZ &&
                         dyskMag > DYSKINESIA_MAG_THRESHOLD);

  if (dyskDetected) {
    return STATE_DYSKINESIA;
  } else if (tremorDetected) {
    return STATE_TREMOR;
  } else {
    return STATE_NEUTRAL;
  }
}

void updateScreen(MovementState state, double tremorMag, double dyskMag) {
  switch (state) {
    case STATE_NEUTRAL:
      drawNeutralScreen();
      break;

    case STATE_TREMOR:
      drawTremorScreen(tremorMag);
      break;

    case STATE_DYSKINESIA:
      drawDyskinesiaScreen(dyskMag);
      break;
  }
}

void drawNeutralScreen() {
  tft.fillScreen(ILI9341_WHITE);
  tft.setTextColor(ILI9341_BLACK);

  tft.setTextSize(3);
  tft.setCursor(90, 80);
  tft.println("NEUTRAL");

  tft.setTextSize(2);
  tft.setCursor(40, 140);
  tft.println("No tremor or");
  tft.setCursor(40, 155);
  tft.println("dyskinesia detected.");
  tft.setCursor(40, 180);
  tft.println("Monitoring...");
}

void drawTremorScreen(double tremorMag) {
 tft.fillScreen(ILI9341_YELLOW);
  tft.setTextColor(ILI9341_BLACK);

  tft.setTextSize(3);
  tft.setCursor(100, 80);
  tft.println("TREMOR");

  tft.setTextSize(2);
  tft.setCursor(60, 140);
  tft.print("Intensity: ");
  tft.print(tremorMag, 2);
}

void drawDyskinesiaScreen(double dyskMag) {
  tft.fillScreen(ILI9341_RED);
  tft.setTextColor(ILI9341_WHITE);

  tft.setTextSize(3);
  tft.setCursor(65, 80);
  tft.println("DYSKINESIA");

  tft.setTextSize(2);
  tft.setCursor(60, 140);
  tft.print("Intensity: ");
  tft.print(dyskMag, 2);
}
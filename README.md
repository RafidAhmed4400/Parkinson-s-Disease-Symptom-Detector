# Tremor Sense

Tremor Sense is an embedded system that detects and differentiates tremors and dyskinetic hand movements using motion data from a single accelerometer. The system performs real-time frequency analysis using FFTs and displays movement type and intensity on a TFT screen.

## Overview

The device samples acceleration data at 52 Hz, processes approximately 3-second windows using a Fast Fourier Transform (FFT), and classifies motion into three states:
- Neutral  
- Tremor (3–5 Hz)  
- Dyskinesia (5–7 Hz)  

Detection is based on dominant frequency bands and magnitude thresholds, allowing the system to distinguish between different movement patterns in real time.

## Hardware Used
- Adafruit 32u4 development board  
- ADXL345 accelerometer  
- ILI9341 TFT display  
- Power bank for portable operation  

A custom CAD enclosure was designed to securely house the microcontroller, accelerometer, TFT, and power source.

## Software & Libraries
- PlatformIO  
- Arduino framework  
- Adafruit GFX  
- Adafruit ILI9341  
- Adafruit ADXL345 Unified  
- arduinoFFT  

## How It Works
1. Samples acceleration magnitude data at 52 Hz  
2. Captures 128 samples per analysis window  
3. Removes DC offset and applies a Hamming window  
4. Performs FFT to extract frequency content  
5. Identifies dominant peaks in tremor and dyskinesia bands  
6. Displays detected state and intensity on the TFT screen  

## Video Demonstration
https://drive.google.com/file/d/1PS7Av2TzDmVwD4MBKZIZu-iaw3M_mPyn/view?usp=sharing

## Notes
- Uses a single accelerometer for motion detection  
- No additional hardware or wireless interfaces are required  
- Designed to be portable, intuitive, and robust  
```

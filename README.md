# Smart home with voice recognition
### Voice-activated multi-LED control using Edge Impulse & PSoC 6 BLE
> Term project "Smart home with voice recognition" from Principles of Computer Organization

This project implements a real-time voice-controlled lighting system based on the PSoC 6 BLE Pioneer Kit.
A custom Keyword Spotting (KWS) model trained with Edge Impulse recognizes 5 voice commands and controls both onboard and external LEDs through GPIO, PWM, and timers.

### Overview
The system listens continuously through the onboard PDM microphone, processes audio with Mel-frequency features, runs a deep-learning classifier, and triggers different lighting effects:

Supported commands:

- `light` – turn on the white LED

- `off` – turn off all LEDs

- `red` / `green` / `blue` – control external RGB LEDs


The project demonstrates how machine learning models can be optimized to run on microcontrollers with very limited RAM.

### Key Features
Real-time speech recognition

- `PDM - PCM` audio capture

- `MFE` feature extraction

- Lightweight MobileNetV1 0.1 KWS model


LED control

- Multiple external LEDs: red, green, blue, white

- Built-in LED for additional feedback

- Automatic transition handling (turning off previous colors)

Model

- Trained with custom dataset

- 16 kHz audio

- 8-bit quantized MobileNet for embedded inference

- 95.4% training accuracy

- 94.4% testing accuracy

Hardware

- Infineon PSoC 6 Wi-Fi BT Pioneer Kit

- E-INK display shield with onboard PDM microphone

- Breadboard with external LEDs: red, green, blue, white

All LEDs are controlled via GPIO;

### Installation & Flashing
Clone the repository
```
git clone https://github.com/okqsna/poc-smart_home_with_voice_recognition.git`
```
Build a project using ModusToolBox and program PSoC 62 BLE with all the required components.
After flashing, the system will start audio capture, process audio every 1s, classify the keyword and trigger the corresponding lighting behavior.

### Project Structure
```
Exploration-of-data-compression-methods/
├── testing/  simple test of ML model for 1 sample
└── project_audio/   main logic, with main ML model 
```

### Contributors
- [Alina Bodnar](https://github.com/alinabodnarpn)
- [Oksana Moskviak](https://github.com/okqsna)

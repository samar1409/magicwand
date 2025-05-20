# TECHIN 515 – Lab 4: Magic Wand Gesture Recognition

This project implements a gesture recognition wand using the ESP32 XIAO and MPU6050, trained via Edge Impulse. The wand classifies hand-drawn gestures (Z, O, V) and maps them to fantasy spells with RGB LED feedback.

## 🔧 Hardware Setup

- **ESP32 XIAO**
- **MPU6050** (I²C): SDA (D5), SCL (D4)
- **RGB LED** (common cathode): RED (D6), GREEN (D7), BLUE (D8)
- **SPDT Switch**: Common to GND, NO to D3
- **Battery**: 3.7V LiPo connected to XIAO
- Mounted on a solderable breadboard with custom wand enclosure

## 📁 Folder Structure

```
gesture_capture/       # Arduino sketch to collect IMU data
wand/                  # Arduino sketch for gesture inference
src/python-scripts/    # Python script to save data from serial
src/dataset/           # Collected labeled gesture samples (CSV)
media/                 # Demo video
enclosure/             # Images and CAD notes
docs/                  # Final report (PDF)
```

## 🚀 Setup Instructions

### 1. PlatformIO
- Clone the repo and open with VS Code + PlatformIO
- Set correct board in `platformio.ini` (e.g. `esp32-c3-devkitm-1`)
- Add upload port:
  ```
  upload_port = /dev/cu.usbmodem101
  ```

### 2. Install Libraries
`platformio.ini` handles:
- Adafruit MPU6050
- Adafruit Unified Sensor

### 3. Gesture Capture
- Flash `gesture_capture.ino`
- Connect device, open Serial Monitor
- Run:
  ```
  python3 process_gesture_data.py --gesture Z --person Samar
  ```
- Perform gesture (Z, O, V) 20+ times each

### 4. Edge Impulse
- Create new project on [Edge Impulse](https://studio.edgeimpulse.com/)
- Upload data in folders Z/, O/, V/
- Design Impulse:
  - Input: 3-axis accelerometer
  - DSP: Flatten
  - Classifier: Dense NN (1 layer, 20 neurons)
- Train, test, and deploy as Arduino library (Quantized Int8)

### 5. Gesture Recognition
- Flash `wand.ino`
- System captures 1s of motion on switch flip
- Runs inference and outputs gesture:
  - 🔴 Red: Fire Bolt (Z)
  - 🟢 Green: Reflect Shield (O)
  - 🔵 Blue: Healing Spell (V)

## 📊 Performance

| Gesture | Accuracy |
|---------|----------|
| Z       | ~90%     |
| O       | ~95%     |
| V       | ~98%     |

## 🎥 Demo
See `media/demo.mp4`

## 📄 Report
Final documentation located in `docs/report.pdf`

# Servos

The robot consists of a total of 40 servoes. 7 for each arm, 2 for the head, and 12 for each hand. All servos are Waveshare servos, cotntrolled by a [Waveshare servo driver with an ESP32 chip](https://www.waveshare.com/servo-driver-with-esp32.htm). All commands computed on the Jetson are send over USB to the servo driver, that forwards commands to all the servos. 

Two different models of servos are used: 
- [ST3215](https://www.waveshare.com/wiki/ST3215_Servo) - Arms and head
- [SC09](https://www.waveshare.com/wiki/SC09_Servo) - fingers

We supply 12V to the ST3215 and 6V to the SC09. The servo driver requires the same supply voltage as the servos it control, so at least two servo driver boards are needed, one controlling arms/head and one controlling the fingers. 

To increase command bandwidth, we have used two seperate boards for each arm as well, one of them also controlling the head. Thus 3 servo drivers in total. 


## Reprogramming the ESP32 Servo Driver

Out-of-the-box the servo driver does not include software that can control the ST-model servos, so we need to reflash the boards. 

The guide on their website explains the process well, under the title “[Compile Arduino IDE](https://www.waveshare.com/wiki/ST3215_Servo#:~:text=ST%20series%20servos.-,Compile%20Arduino%20IDE,-We%20provide%20ST3215)” (Tested with Windows). 

Some things to note, not mentioned in the guide: 

- If the COM port is not discovered when the board is connected, try installing this USB driver (CP210x VCP Windows). From this trouble shooting guide. 

- Don’t update libraries, as it can result in a compile error. If you need to downgrade, follow this guide. 

- If you get a compile error “A fatal error occurred: Failed to connect to ESP32: Wrong boot mode detected (0x13)! The chip needs to be in download mode”, simply hold down the “boot” button on the board while compiling and writing to the board.  


Some settings in the `ServoDriver.ino` file should be changed:
| Variable name     | Default value | New value     | Note                                       |
| ----------------- | ------------- | ------------- | ------------------------------------------ |
| AP_SSID           | "ESP32_DEV"   | "ESP32_DEV_0" | Must be unique for each servo driver board |
| MAX_ID            | 20            | 50            |                                            |
| SERIAL_FORWARDING | false         | true          |                                            |
| MAX_ID            | 20            | 50            |                                            |



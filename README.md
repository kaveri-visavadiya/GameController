# EE333 - Game Controller Project

## Group Members:
22110208 - Raghavpravin K S
22110269 - Tamizhanban A G
22110114 - Kaveri Visavadiya
22110132 - Mahi Agarwal

## Project Description
Implementation of game controller using STM32 board and USB HID interface class.

## Components Used
Joysticks, pushbuttons, potentiometers, PCB board, jumper wires, acrylic sheet, STM32F439ZI board

Schematic of pins used in SM32F439ZI board
![image](https://github.com/user-attachments/assets/526ae2ae-b71d-484b-b724-b773e5abba68)

Final gamepad
![image](https://github.com/user-attachments/assets/8315416e-2e0e-41fa-8b5d-200570e97cf6)

## Method
Using the USB HID class, we sent data from STM32 board to the computer. Data sent includes 16 button data (each button occupies 1 bit) and 4 axes data for left x, left y, right x and right y joysticks (each axis occupies 1 byte). The computer requests data from the board at an interval of 10 ms and a struct called the ReportDescriptor containing this data is sent through the USB. We attached a detachable PCB board onto the STM32 board to solder the jumper wires onto it. Then, by laser cutting an acrylic sheet into the shape of a game controller, the 10 buttons (although an Xbox has 16 buttons, we mapped only 10 buttons) and 4 joysticks attached to the PCB were arranged and pasted for ease of usage. After checking if Windows recognised the microcontroller as a game controller, we used an emulator (X60CE) that allows our microcontroller to function on our PC as an Xbox 360 controller and allows us to remap buttons and axes to drive cars. It can then be used to play any game of choice.

## Results
The microcontroller was successfully implemented as a game controller and was used to play a car game (Forza). The buttons and joysticks were mapped according to the desired functionality in the game (for eg. left joystick’s y axis was used for acceleration and deceleration, right’s x axis was used for left and right movement of car wheels). Video
 
## References
1. Silicon Laboratories, "Human Interface Device Tutorial," Application Note AN249, 2009. [Online]. Available: https://www.silabs.com/documents/public/application-notes/AN249.pdf
2. O. Ousmoi, "Custom USB HID Interface using STM32 (stm32f103c8)," YouTube, June 24, 2020. [Online]. Available: https://www.youtube.com/watch?v=740XGkC0DfQ 
3. Phil’s Lab, "STM32 USB HID Custom Joystick/Gamepad - Phil's Lab #149," YouTube, Dec. 4, 2024. [Online]. Available: https://www.youtube.com/watch?v=umkD1piCNvc
4. Online Mic Test, "Controller Tester," [Online]. Available: https://www.onlinemictest.com/controller-tester/&#8203;:contentReference[oaicite:3]{index=3}
5. The Linux Kernel Organization, "Introduction to HID report descriptors," The Linux Kernel Archives, Apr. 2024. [Online]. Available: https://docs.kernel.org/hid/hidintro.html 
6. STMicroelectronics, "UM1974: STM32 Nucleo-144 boards (MB1137)," User Manual, Rev. 6, Sep. 2022. [Online]. Available: https://www.st.com/resource/en/user_manual/um1974-stm32-nucleo144-boards-mb1137-stmicroelectronics.pdf
7. X360CE, "Xbox 360 Controller Emulator," [Online]. Available: https://www.x360ce.com/​

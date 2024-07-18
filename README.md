# Version 1
![IMG_0225](https://github.com/user-attachments/assets/ca851782-d328-4e69-80fe-dc5a1077a883)
![IMG_0231](https://github.com/user-attachments/assets/49ae1b70-d374-4118-8489-1003d3685f1b)

This version of the rover has to be plugged in to work, or be powered by a battery that supplies more than 5 volts. It integrates three motor drivers and a gyroscope to correct for the motors' speeds being slightly off from each other.  It has three almost 3d printed omniwheels, which only need ball bearings and screws. It uses an ESP-32 dev board to drive the motors. It also has ESP-NOW integration, which is needed in order to be driven by an external joystick.

# Version 2 - WIP
![roverv2](https://github.com/user-attachments/assets/221ada44-a1a9-4db6-bb32-c641cfd1d982)
This version of the rover supports being completely battery powered with two AA batteries. It is powered by an ESP32-S3, which uses a 3.3 volt regulator. The regulator is also powering the gyroscope and camera *eventually*. Onboard the PCB, there is also a 5v regulator to move the motors. Unlike V1 of the rover, these wheels are completely 3d printed. Unfortunatly, as of right now, the USB connection stopped working after a short and the motors are stuck spinning until shutdown.

# Board 
<img align="left" src="https://github.com/user-attachments/assets/c19179a1-1e81-4120-9a3d-ede7c56958a7">
Version 2 of the board has four layers, with the two internal planes being ground planes, unlike version 1, which just has two layers and a ground fill on each. It uses a Texas Instraments DRV8832DGQR and TPS61040DBVRG4 to drive the motors and boost voltage, respectively. The gyroscope being used is part of the MPU-6050 and, of course, the main component of V2 is the ESP32-S3-WROOM module.



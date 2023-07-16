# STHS34PF80
ST IR temperature sensor with embedded presence and motion detection

Arduino sketch showing how to configure the STHS34PF80 IR sensor for basic ambient and object temperature measurements as well as how to configure the embedded function algorithms and use them to detect presence and motion. The sketch allows either continuous or one-shot mode, allows changing the low-pass filters and detection thresholds to allow a wide variety of detection behavior.

The sketch uses the embedded interrupt engine to detect temperature data ready as well to signal presence and motion events.

The sketch was developed and tested using an STM32L432 development board (Ladybug), although the sketch should work with most Arduino-IDE-compatible MCUs with little or no modification.

![test rig](https://user-images.githubusercontent.com/6698410/253826417-41d274dd-b196-47ee-b2c4-57483e647ef5.jpg)

Data sheet for the STHS34PF80 is reposited [here](https://www.st.com/resource/en/datasheet/sths34pf80.pdf), AN5867 with detailed explanation of how to make use of the sensor is reposited [here](https://www.st.com/resource/en/application_note/an5867-sths34pf80-lowpower-highsensitivity-infrared-ir-sensor-for-presence-and-motion-detection-stmicroelectronics.pdf).

I designed a small breakout board for the STHS34PF80, the bare pcb for which you can order from OSHPark [here](https://oshpark.com/shared_projects/Wqam2MJ5).

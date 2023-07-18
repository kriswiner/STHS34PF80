# STHS34PF80
ST IR temperature sensor with embedded presence and motion detection

Arduino sketch showing how to configure the STHS34PF80 IR sensor for basic ambient and object temperature measurements as well as how to configure the embedded function algorithms and use them to detect presence and motion at up to 4 meters range. The sketch allows either continuous or one-shot mode, allows changing the low-pass filters and detection thresholds to allow a wide variety of detection behavior.

The sketch uses the embedded interrupt engine to detect temperature data ready as well to signal presence and motion events.

The sketch doesn't do everything but it should be a pretty good start for anyone looking to use this sensor for any people or animal counting applications.

The sketch was developed and tested using an STM32L432 development board (Ladybug), although the sketch should work with most Arduino-IDE-compatible MCUs with little or no modification.

![test rig](https://user-images.githubusercontent.com/6698410/253826417-41d274dd-b196-47ee-b2c4-57483e647ef5.jpg)

Data sheet for the STHS34PF80 is reposited [here](https://www.st.com/resource/en/datasheet/sths34pf80.pdf), AN5867 with detailed explanation of how to make use of the sensor is reposited [here](https://www.st.com/resource/en/application_note/an5867-sths34pf80-lowpower-highsensitivity-infrared-ir-sensor-for-presence-and-motion-detection-stmicroelectronics.pdf).

I designed a small breakout board for the STHS34PF80, the bare pcb for which you can order from OSHPark [here](https://oshpark.com/shared_projects/Wqam2MJ5).

I spent a bit of time testing the dependence of the behavior of the sensor on odr, temperature avgeraging, and threshold settings. In general, presence detection is pretty robust even at a few meters. This sensor has the potential to make an ultra-low-power people presence detector and ingress/egress (i.e., directional motion) counter as described [here](https://hackaday.io/project/164131-people-counter). Such a device (same footprint as my [GasCap Asset Tracker](https://www.tindie.com/products/tleracorp/gascap-loragnss-asset-tracker/)), whose prototype is shown below:

![PeopleCounter.v02](https://user-images.githubusercontent.com/6698410/254101989-eecc40b3-6343-428f-b527-0401c9f44051.jpg)

ends up being smaller, cheaper, and easier to use than the previous people counter I designed using the Calipile TPiS1S1385 sensor as well as having longer range. Since the STHS34PF80 has internal ambient temperature comensation, it should not suffer from the ambient temperature drift that required frequent threshold resetting when using the Calipile sensors. Testing is still required to see how reliable the presence and directional detection is, and to see how low the average power is in practical usage. The device above should be able to accurately determine presence, distinguish entry and exit at a threshold (i.e., doorway), and report via LoRaWAN while using ~25 uA continuous current at 3.1 V. The PeopleCounter device is designed to use a AA-sized, 3.6 V, 2400 mAH LiSOCl2 battery, which would last more than ten years at this current usage. Testing will tell...

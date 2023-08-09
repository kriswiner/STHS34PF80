# STHS34PF80
ST IR temperature sensor with embedded presence and motion detection

Arduino sketch showing how to configure the STHS34PF80 IR sensor for basic ambient and object temperature measurements as well as how to configure the embedded function algorithms and use them to detect presence and motion at up to 4 meters range. The sketch allows either continuous or one-shot mode, allows changing the low-pass filters and detection thresholds to provide for a wide variety of detection behavior.

The sketch uses the embedded interrupt engine to detect temperature data ready as well to signal presence and motion events.

The sketch doesn't do everything but it should be a pretty good start for anyone looking to use this sensor for any people or animal counting applications.

The sketch was developed and tested using an STM32L432 development board (Ladybug), although the sketch should work with most Arduino-IDE-compatible MCUs with some modification.

![test rig](https://user-images.githubusercontent.com/6698410/253826417-41d274dd-b196-47ee-b2c4-57483e647ef5.jpg)

Data sheet for the STHS34PF80 is reposited [here](https://www.st.com/resource/en/datasheet/sths34pf80.pdf), application note AN5867 with detailed explanation of how to make use of the sensor is reposited [here](https://www.st.com/resource/en/application_note/an5867-sths34pf80-lowpower-highsensitivity-infrared-ir-sensor-for-presence-and-motion-detection-stmicroelectronics.pdf).

I designed a small breakout board for the STHS34PF80, the bare pcb for which you can order from OSHPark [here](https://oshpark.com/shared_projects/Wqam2MJ5).

I spent a bit of time testing the dependence of the behavior of the sensor on odr, temperature averaging, and threshold settings. In general, presence detection is pretty robust even at a few meters. This sensor has the potential to make an ultra-low-power people presence detector and ingress/egress (i.e., directional motion) counter as described [here](https://hackaday.io/project/164131-people-counter). Such a device (same footprint as my [GasCap Asset Tracker](https://www.tindie.com/products/tleracorp/gascap-loragnss-asset-tracker/)), whose prototype is shown below:

![PeopleCounter.v02](https://user-images.githubusercontent.com/6698410/254101989-eecc40b3-6343-428f-b527-0401c9f44051.jpg)

ends up being smaller, cheaper, and easier to use than the previous people counter I designed using the Calipile TPiS1S1385 sensor as well as having longer range. Since the STHS34PF80 has internal ambient temperature compensation, it should not suffer from the ambient temperature drift that required frequent threshold resetting when using the Calipile sensors. Testing is still required to see how reliable the presence and directional detection is, and to see how low the average power is in practical usage. 

The device above should be able to accurately determine presence, distinguish entry and exit at a threshold (i.e., doorway), and report these events via LoRaWAN while using ~25 uA continuous current at 3.1 V. The PeopleCounter device is designed to use a AA-sized, 3.6 V, 2400 mAH LiSOCl2 battery, which would last more than ten years at this current usage. Testing will tell...

For this prototype, I included a BME280 for environmental (P, H, T) sensing, a LIS2DW12 accel to detect tampering and theft of the device, and an expansion connector in case I need to add some other serial device(s) in development. The mechanical power switch, USB connector, and expansion connector would all likely be dropped in any production version of the device to minimize costs.

The first hurdle in using the STHS34PF80 for people counting is that to determine direction of travel, that is, to distinguish between ingress and egress at a threshold, two widely-spaced IR sensors are required. However, the STHS34PF80 only has one unchangeable I2C address (0x5A). Initial attempts to use two n-type FETs to toggle SDA shared between the two IR sensors didn't work for some reason. Instead, I ended up using a TI switch (SN74LVC1G3157DRYR) which does work.

![initial prototype](https://user-images.githubusercontent.com/6698410/259577490-991a7b28-7501-4bd8-ad37-77533a3238a1.jpg)

The basic people counting sketch shows how to configure and get data from the two STHS34PF80 IR sensors along with the LIS2DW12 accelerometer and BME280 barometer sensors on the prototype board. The setup portion of the sketch checks and configures all sensors for I2C communication and operation. The main loop is entirely interrupt based; if the STM32L082 MCU is not handling an interrupt it is in the lowest attainable power mode until an interrupt wakes it from sleep. The data is output to the serial monitor based on the RTC timer, although a timerMillis object would be more general. The latter is used to schedule LoRaWAN updates of the data at 10 minute intervals. The sketch is simply intended to test correct operation of all of the components on the prototype.  

I measured ~71 uA average current usage in a 10 minute window at 3V0 and 32 MHz CPU speed with one LoRaWAN event (one Tx and two Rxs) in the window using Nordic's Power Profiler II. Without LoRaWAN, the average current usage is ~51 uA dominated by the ~7 mA excursions of the IR sensors at 1 Hz. These average currents will drop when running the CPU at 4.2 MHz. The average current will increase if the IR sensors have to be run at a higher sample rate to sufficiently discriminate comings and goings. TBD.

In the people counter application, one can choose to send a LoRaWAN update every time a counting event is detected or, probably more efficiently, send a periodic update with the number of events detected within the period as well as a running count of ingresses and egresses, etc. Since LoRaWAN at 10 minute interval costs ~20 uA, the longer the period the lower the average current usage, of course. To get to a ten year lifetime using a 2400 mAH 3V6 LiSoCl2 battery, the average current budget is ~27 uA. So the above currents would indicate a device lifetime more like ~3 years unless the power usages can be further reduced. But it is in the ballpark, warranting the effort for further development and optimization.

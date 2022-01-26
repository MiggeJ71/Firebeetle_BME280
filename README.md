Firebeetle_BME280
Battery powered ESP32 based IoT device to measure and report weather data and battery level as MQTT messages. R20 and R21 has been bridged on the Firebeetle board in order to measure battery voltage on pin A0.
Estimated battery life on a single battery is about 940 days if measurement data is sent every 15 minutes and. At that time the battery is estimated to be down at 20% SOC.

Note!
DFRobot driver for BME280 had to be modified (patch uploaded on DFRobot Github page) in order to get correct values for humidity. 

ESP powered by a single Lifepo4 battery
Note! These batteries claim to have a capacity of 7200mA but are in fact 5200mA in average.

Products used
ESP32 https://www.dfrobot.com/product-1590.html <\n>
BME280 https://www.aliexpress.com/item/32849462236.html?spm=a2g0o.9042311.0.0.259e4c4d16GiDw
Battery https://www.aliexpress.com/item/1005001997843247.html?spm=a2g0o.9042311.0.0.259e4c4d16GiDw


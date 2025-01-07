# Portable Environmental Sensor

For this, I wanted to throw together a bunch of sensors I had laying around (and a CO2 sensor because I wanted a CO2 sensor specifically) and make them into something portable I could take with me on a road trip. I still need to design and print a case for this to make it actually portable though.

Components:
Arduino Mega
Inland Capacitive Touch Sensor
Parallax 16x2 LCD Display
Inland LM35 Temperature Sensor
Inland MQ2 Gas Sensor
Dallas Waterproof Temperature Sensor
Adafruit Stemma Soil Moisture Sensor
Inland Hall Effect Sensor
Sensirion SCD41 CO2 Sensor

The full assembly:
![20241119_212345](https://github.com/user-attachments/assets/9c37b49d-bed1-4d7b-b2c4-6399458b366d)

Soil Sensor:
![20241119_212406](https://github.com/user-attachments/assets/2a6122a3-4e2c-4e8e-be1d-382650314916)

Waterproof Temperature Sensor:
![20241119_212426](https://github.com/user-attachments/assets/b76b9360-3933-411b-9274-d63a8c0aaf11)

CO2 Sensor:
![20241119_212446](https://github.com/user-attachments/assets/fb54d054-6732-49bc-8d40-131c98080334)

MQ2 Sensor (Note: The MQ2 sensor cannot discern between types of flammable gasses, just that there is some kind present):

No flammable gasses
![20241120_220002](https://github.com/user-attachments/assets/4f3cbff4-1560-480e-87a9-714ccff16ba1)

Gas from a disposable lighter:
![20241120_220347](https://github.com/user-attachments/assets/542233b0-ff10-4e80-b3fd-b9c7d89a4152)


TODO:
Switch out for a smaller breadboard (I want to maintian customizability, so breadboard it is. However I'm only using half of the breadboard.)
Design and print a case for it.
Solder a better connector for the Waterproof Temperature Sensor, as right now it's just bare wire - which disconnects very easily.
I want to add a UV sensor to this. I thought I had one already but alas I could not find it.
Add a GPS module for tracking where the readings were taken
Add a datalogging module so I'm not manually recording the data.

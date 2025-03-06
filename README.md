# stewart
A self-balancing robot project developed at SproutHacks 2025 over a total of ~12-13 hours.

## Post SproutHacks Briefing
Stewart didn't win anything but he will forever live in my heart as a learning experience that I enjoyed nicely. <a href="https://devpost.com/software/stewart?_gl=1*iwpoki*_gcl_au*MjEzNDczMTM5LjE3NDEyODY2ODU.*_ga*OTcxMzIyMjc4LjE3NDEyODY2ODY.*_ga_0YHJK3Y10M*MTc0MTI4NjY4NS4xLjEuMTc0MTI4NjY5MS4wLjAuMA..">Here is the Devpost link to our project!</a>

## Project Info
<strong>Stewart</strong> is a self-balancing robot made using an ESP32 with a programmed <strong>PID controller</strong>, <strong>a pizza box</strong> for a base, and <strong>LOADS of tape</strong>.

<strong>Mr. Stewwy</strong> had a lot iterations throughout the build process. We nearly swapped over to an Arduino Uno R3 due to library functionality (over using Wire.h, which was a lil buggy and would've resulted in a lot of extra programming), but reading the documentation allowed us to avoid that and use the beloved <a href="https://github.com/RobTillaart/GY521">GY521.h library</a>, which worked fantastically and allowed us to:

- Calibrate the <strong>MPU6050/Accelerometer and Gyroscope</strong> we were using to attain accurate angle measurements for the PID.
- Record and return data from the MPU6050 to get angle data.

From there, it was a matter of calculating the PID value, which was then mapped as a speed value for the motors driving <strong>Monsieur Stewart</strnog>. It was a very careful dance to tune the PID; a lot of Stewart falling over on his face.

## Technologies and Parts Used
- Arduino IDE v2.3.4 for programming
- GY521 library by RobTillaart
- GitHub for version control (not massively needed but makes you feel smart)
- ESP32-WROOM microcontroller
- MPU6050 Accelerometer (there is in fact a gyroscope inside the accelerometer)
- LN298 Motor Driver
- Lots of MLH, Sprouthacks and GoDaddy stickers!
- Plastic forks, paper plates, a domino's pizza box, a rubber band, lots of wires and alligator clips.
- Generic Yellow Wheels
- Yellow TT Motors

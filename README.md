# ESP32-code
Here is the code I'm running on my ESP32 as of October 2026 (This is based on V2 of SensESP)

The setup is using examples found on the excellent SensESP pages here - https://github.com/SignalK/SensESP 

I now have engine temperature, exhaust temp & Oil Temp, engine RPM, engine room environmental, engine runtime using the ESP uptime and (estimated) fuel consumption which is using the engine RPM and fuel burn info from the Volvo Penta documents. I have recently added a bilge monitor to this project.

Please make sure you add the following additional dependencies to your platformio.ini file, located under 'lib_deps', also check the latest version of these dependencies (Thanks Techstyle). I've uploaded my file as a reference.

SensESP/OneWire@^2.0.0 

adafruit/Adafruit BMP280 Library @ ^2.5.0

## RPM calibration - please read

The RPM curve in main.cpp (RPMInterpreter) is calibrated to MY engine, alternator and input circuit, so it won't be right for yours. You'll need to build your own:

1. Flash the code and watch the propulsion.engine.revolutions.raw value in Signal K - this is the raw pulse frequency (Hz) from your sender.
2. Run the engine at a few steady speeds (e.g. idle, 1000, 1500, 2000, 2500, 3000 & max) and note the raw Hz against the RPM on a tacho you trust.
3. In the SensESP web page go to Configuration > /Engine RPM/curve and enter each point - Input is the raw Hz, Output is the RPM divided by 60 (Signal K uses revolutions per second, so 2000 rpm = 33.333).
4. Add a final point well above your max RPM, otherwise readings past the last point will show as 9999.

Once saved in the web page your curve overrides the values in the code. The fuel rate uses this calibrated RPM, so get the RPM right first.

## WiFi fallback

If the ESP can't find its saved WiFi network for 2 minutes, it opens the "Configure SensESP" access point (default password: thisisfine) for 3 minutes so you can enter new WiFi details from your phone, rather than just rebooting forever.

Videos of my setup can be found on our YouTube channel - https://www.youtube.com/c/BoatingwiththeBaileys

Hope this is helpful and my thanks to all the talented people writing the code and continuing to develop these projects!

# ESP32SignalkDisplay
Framework to display Signalk data via an ESP32 to multiple display types.
# Pre-Requisites
This requires the https://github.com/bwssytems/EPDDisplayMarineData and https://github.com/Links2004/arduinoWebSockets. Also see the platformio.ini file for all other libraries that you will need if you are not using PlatformIO in vscode to build the firmware.
# Configuration
Configuration of the wifi connection is through the ESP Async WiFi Manager and is accessed by its SSID of SignalkDisplayWifi_XXXXXX. The password is "signalkdev1."

After WiFi configuration the web page at the device has an input area to add the SignalK Server URI and is defaulted to port 3000.

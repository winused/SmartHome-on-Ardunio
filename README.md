# SmartHome-on-Ardunio
Ardunio smarthome project on esp32

Works on broker.mqtt-dashboard.com domain
Connects Wifi(Change according to your wifi name and password)
Sensors = temperature, motion, humidity, NFC card reader
Can send data to phone app MQTT Dashboard
-Change pin numbers on code according to your own Esp32 microcontroller

#DOORLOCK - reads nfc card
/opens the door if authorized
/ sends message on telegram whether access authorized or denied
-Change telegram CHAT_ID for your own ıd.

#FIRSTROOM - lights room 
/lights different colored light acoording to temperature
/if motion sensor doesn't detect any motion for 5 min, lights are off


#SECONDROOM - 




# esp32minifridge
ESP32 Adafruit Huzzah32 controlled mini-fridge

## features
* controls 12V Peltier element through N-MOSFET
* controls 2-BLDC fans through L293D motor driver
* simple 2-point temperature control
* DS18B20 single-wire digital temperature sensor
* Webserver with GET/POST, multipart-mime upload and TLS support 
* Webclient with TLS support
* Url/Querystring parser
* Captive webserver in Access Point mode (needs improvement for true captive capabilities)
* Responsive Web UI based on Phonon 1.4.5
* Data embedding (such as HTML, CSS, font files, audio...)
* WAV decoder (e.g. 8 or 16bit mono wav files 16khz work very well, but some other rates work too)
* Audio player to I2S devices (e.g. Adafruit MAX98357A) 
* C++, ESP-IDF
* Wifi AP/STA mode (GPIO0 button will toggle mode)
* Stores config in NVS
* Storage class to wrap SPIFFS read/write access for storing uploaded files on flash on dedicated data partition

## screenshots


## build

* Setup ESP-IDF toolchain according to [http://esp-idf.readthedocs.io/en/latest/](http://esp-idf.readthedocs.io/en/latest/)
* run `make menuconfig` and adjust serial port 
* change partition setting to custom and choose `partitions.csv`. the partition table `partitions.csv`setup assumes that the ESP32 has at least 4MB flash available, so that their is a remaining of 896MB flash for SPIFFS data area
*  

## hardware


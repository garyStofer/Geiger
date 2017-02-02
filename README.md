# Geiger Counter module for Air-LCD-Uino

This is a re-implementation of a Geiger counter device, based on [http://rhelectronics.net](http://rhelectronics.net) original work. See link for all the features. 

The PCB is implemented as a daughter module for the back of the AIR-LCD main board. Sources and CAD files are on [GIThub](https://github.com/garyStofer/AIR_LCDuino), PCBs can be ordered from: 

<a href="https://oshpark.com/shared_projects/kT6TDwLU"><img src="https://oshpark.com/assets/badge-5b7ec47045b78aef6eb9d83b3bac6b1920de805e9a0c227658eac6e19a045b9c.png" alt="Order from OSH Park"></img></a>

----------

It uses a Geiger Mueller tube to count, display and record radioactive strikes. 

The device can be operated from 12V using the built in 5V regulator of the main board, or from 4 NiMH batteries by removing the voltage regulator and feeding the battery voltage directly. The Arduino script has a crude High Voltage regulation loop that keeps the Anode voltage in the operation limits of the tube during the decay of the battery voltage. 

Some tuning of the code is required for best operation. 
Specifically the Anode voltage needs to be adjusted for the GM tube chosen, usually around 400V. A 10 to 100Gohm resistor divider needs to be used in order to measure the voltage.  

With attached USB?UART converter the device provides data to a PC tool from RH-Electronics to monitor and graph the readings.

Bare PCB can be ordered from OSH PCB:


<a href="https://oshpark.com/shared_projects/8asBUqMk"><img src="https://a800d827b6de8403a51e-6ffc2e718631809086ea40332b2055f7.ssl.cf1.rackcdn.com/assets/badge-5b7ec47045b78aef6eb9d83b3bac6b1920de805e9a0c227658eac6e19a045b9c.png" alt="Order from OSH Park"></img></a>
  
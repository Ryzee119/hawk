## Hawk - Open Source Xbox Live Communicator

This project is an open source replacement of the [Microsoft Xbox Live Communicator](https://xbox.fandom.com/wiki/Xbox_LIVE_Headset) utilizing modern components and custom open-source firmware.

<p align="center"><img src="./.kitspace/pcb_render.png" alt="hawk!" width="90%"/></p>

It provides feature parity with the original communicator with the following differences:
* A 3.5mm TRRS jack is used to support more generic headsets. (Stereo headsets will output as mono.)
* The volume wheel is replaced with a button. Volume is indicated by LED brightness.

### Usage
* When installing Hawk into your controller, it must be installed in the slot closest to you.
* Press the button to increment the volume. Once maximum volume is reached, it will cycle back to zero.
* Hold the button for 1 second to mute and disable the microphone input (LED will flash at 1Hz).
* Hold the button 2 seconds to enter microphone gain adjustment mode (LED will flash at 4Hz).
* Volume and microphone gain adjustments will be saved internally. You can reset these to default by holding the button down for 8 seconds.

### Assembly
* **When ordering PCB thickness should be 1.2mm!**
* See [Kitspace](https://master.staging.kitspace.dev/Ryzee119/hawk) for BOM, PCB ordering and assembly information.
* See [KiCanvas](https://kicanvas.org/?github=https%3A%2F%2Fgithub.com%2FRyzee119%2Fhawk%2Ftree%2Fmaster%2Fhardware) for schematic view.

### Compilation
* Download and install [Visual Studio Code](https://code.visualstudio.com/).
* Install the [PlatformIO IDE](https://platformio.org/platformio-ide) plugin.
* In Visual Studio Code File > Open Folder... > hawk
* Hit build on the Platform IO toolbar (✓).

### Programming
* [Compile](https://github.com/Ryzee119/hawk?tab=readme-ov-file#compilation) or [Download](https://github.com/Ryzee119/hawk/releases) the pre-built firmware image.
* USB DFU. Connect an xbox controller to your PC, then hold the Hawk button down while inserting it into the controller to enter DFU mode. Your PC should detect this. you can then program with [STM32Cubeprogrammer](https://www.st.com/en/development-tools/stm32cubeprog.html)
* ST-Link interface (5 pin header available on PCB)

### FAQ
* What is the pinout of the headphone jack?
  * Hawk's headphone jack is wired as per [CTIA standard TRRS](https://upload.wikimedia.org/wikipedia/commons/e/ee/3.5mm_TRRS_phone_connector_with_CTIA_standard.png). There is another less common standard called OMTP. This will not work but adapters are available.
* It doesn't fit in my controller slot.
  * PCB should be manufactured with 1.2mm thickness. Most fabrication houses default to 1.6mm so be careful!
* Microphone is really quiet.
  * You can increase the microphone gain above default. Hold the Hawk button down for more than 2 seconds to enter gain adjustment mode. The LED will blink 4 times per second to show it is in the right mode then press the button to increase the gain. Hold for 1 second to exit. If this is still too quiet, you can modify the `MAX_MICROPHONE_GAIN` variable and recompile the code to bump it up even higher.
* Speaker is really quiet
  * Adjust the volume by pressing the button until the LED is at its maximum brightness. Unfortunately, it is already compiled to the maximum allowable by Hawk. If it is still too quiet, your headset's impedance may be too low.
* Does this work on modern controller adapters (ogx360, wingman etc)?
  * Not really, but kind of. To do this, you must connect a USB 1.1 compliant hub to the Xbox player port. Your controller then connects to port 1 of this hub and Hawk to port 2.
* LED just flashes constantly when connected
  * This means the main IC cannot communicate with the audio codec on Hawk. This is likely a soldering issue.

### Attribution
* [Board outline](https://github.com/Zeigren/OXC)
* [EEPROM emulation code](https://github.com/nimaltd/ee)

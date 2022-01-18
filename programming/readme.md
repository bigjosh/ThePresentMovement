# DMY CLock Factory Programming

Each clock needs to get programmed with firmware and the current time before shipping.

This setup assumes you are programming using a Windows computer with a USBASP programmer. AVRDUDE must be installed

## Set up

Get to a command prompt in the directory with this readme. 

Make sure your computer has the right time and date. It is probably a good idea to install an NTP client or GPS slave at some point to keep the time accurate to the second.


### Adjust programming lag offset

The programming procedure takes more than a second to complete, so if we just programmed the current time then the clock would end up slightly behind.

You can specify a number of seconds to add or subtract to the current time when programming the clock to account for these delays in the programming process. 

First try programming a unit with the default offset and then compare the clock time to the the time on the computer. If the clock is off by more than +/- 1 second then adjust the value of `offset` in the `burnthis.bat` file and try again.

### Accurate time base

We want the local windows clock to be as accurate as possible since all the TSLs programmed will be based on this time (+/- the lag offset above). You can manually set the windows clock though Windows' settings, but probably best to install an NTP client on the programming computer to always keep it synced.

Here is one that is free and works...

http://www.timesynctool.com/

You can check against the real time here...

https://time.gov/
  

## Procedure

1. Put batteries in movement.
2. Hold programming connector to programming pads on the back of the clock. 
2. Enter the command `burnthis.bat` on the command line.
3. Watch it program.
4. Axle should rotate 180 degrees to indicate success. 
5. When ready, play hand on axle point to noon.  

### Batch programming mode

If you enter just `burnthis.bat` without specifying a serial number, then you will go into batch programming mode where You will be asked to enter a serial number and after the programming cycle successfully competes then it will ask for the next serial number. 

This works great if you have a bar code scanner to enter the serial numbers.   

## How it works

### tsl-make-block

First the batch file uses the `tsl-make-block` to create a binary file to be programmed into the TSL EEPROM. The program checks the clock on the local PC, converts the current time to GMT, and then adds in the delay offset. Then it writes out the EEPROM data block in the format that the TSL firmware expects to find when it runs. The block contains:

1. The current time GMT
2. A flag that tells the firmware to copy this time to the RX8900 RTC when it sees it
3. The trigger time and flag, which we set to "not triggered yet". 
4. A flag that remembers if the RTC in the TSL has ever reported that it lost power, which we initialize to `not set`.
 
### atprogram
The batch file uses the `atprogram` command to actually download the firmware flash image and the newly created EEPROM data block into the XMEGA on the TSL.  The `atprogram` is equivalent to `avrdude` but it written by Atmel and runs on Windows and uses the Atmel drivers. We used `atprogram`  because `avrdude` can have problems talking to an `MkII` when Atmel Studio is installed because of USB driver conflicts.  

#### Fuses

We leave fuses at factory default:

```
avrdude: safemode: lfuse reads as 62
avrdude: safemode: hfuse reads as D7
avrdude: safemode: efuse reads as FF
```

### Checking your work

You can use the `read-eeprom.bat` batch file to read and print the eeprom of an attached clock. It uses the `tsl-read-block.exe` utility to decode and print the data, and you can also use this utility to work directly with eeprom bin files created by `avrdude`.

### Sample block file

The `party-like.bin` eeprom block will set the clock to just after midnight on Jan 1, 2000. You can use the Atmel Studio "Device programming tools" or AVRDude to load this file into EEPROM for testing.

It was created with this command line...
```
tsl-make-block.exe party-like.bin -s 20000101000000
```  
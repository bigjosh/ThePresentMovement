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
2. Enter the correct command on the command line (`month.bat`,`day.bat`,etc depending on model).
3. Watch it program and make sure no errors.
4. Axle should rotate 180 degrees to indicate success.
5. Clock is now ready. On the next button press it should start ticking at the programmed rate.  
  
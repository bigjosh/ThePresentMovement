# Programming instructions

## Prerequisites

You will need...

1. A windows computer
2. A `USBTiny` compatible programmer (like this [one](https://amzn.to/3AaaWSp))
3. A connector that fits the clock's 6-pin ISP programming port (like [this](https://amzn.to/3nE2BBh) or [this](https://www.sparkfun.com/products/11591))

Note that you can also use a non-Windows computer or a different programmer, but that would require a different procedure.   

## Setup

1. Copy the files from this directory onto the PC 
2. Plug in the programmer to a USB port on the PC
3. Make sure you hear the nice USB noise telling you the drivers installed OK
4. Make sure the little switch on the programmer board is set to `No Power`

## Procedure

1. Install batteries into clock.  
2. Install the hand pointing to 6 o'clock
2. Hold the connector against the clock board like this...

    ![](hold_connector.png)
1. Double click on the appropriate programming file in the directory (either `day.bat`, `month.bat`, or `year.bat`)
2. Wait for programming success (see [Problems](#Problems) below if no success)
3. Remove programming connector
4. Push the button on the movement
5. Verify that the hand then rotates from 6 o'clock to noon position
6. Finished!

## Checking

Once the button has been pressed and the hand has completed its 1/2 revolution, the next button press will begin normal ticking. The clock should tick according to this schedule...

| Mode | Full Rotation Period | Tick Every... |
| - | - | - |
| Day | 24 hours | [24 seconds](https://www.wolframalpha.com/input/?i=%28%28day%29+%2F+3600%29+in+seconds) |
| Moon | 1 lunar month [(29.53059 days)](https://en.wikipedia.org/wiki/Lunar_month) | [11 minutes, 48.73411 seconds](https://www.wolframalpha.com/input/?i=%28%28lunar+month%29+%2F+3600%29+in+minutes) |
| Year | 1 solar year [(365.2422 days)](https://www.google.com/search?q=solar+year+in+days)| [2 hours 26 minutes](https://www.wolframalpha.com/input/?i=%28%28year%29+%2F+3600%29+in+minutes) |

## Problems

* Try again a couple of times. If you happen to be unlucky and a tick happens in the middle of the programming cycle that can interrupt the programming.
* Sometimes you need to unplug and replug the programmer into the USB port if the PC has gone to sleep since the last time you used it.
* Make sure all the pins from the programming connector are firmly touching the pads on the clock board.
* Make sure the programming connector is not backwards. The red stripe on the cable should line up with the little white arrow on the moment. 
* Make sure the clock has good batteries installed.
* Makes sure the little switch on the programmer is set to `NO POWER`.
* Try taking the batteries out for a minute and then putting them back in and then program again. Especially if the movement was last programmed in `hour.bat` or `spin.bat` mode since in this case there is likely to be a tick right in the middle of programming.  
* Try using `erase.bat` to quickly erase the chip and see if that works. After erasing, pull the batteries for a minute and then reinstall and try programming again. 
 

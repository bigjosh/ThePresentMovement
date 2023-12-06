REM pass args for timebase, count
REM timebase is the time per timer cycle. Can be 1=15.625ms 2=1sec 3=1min.
REM count is number of timer cycles per clock tick. Must be 12 bits (4095) or less. 

color 07

REM first program the FLASH since we need to do erase first and it would also erase the EEPROM. 
REM The `-u` means "do not read the fuses"
REM The `-e` means "erase all chip memories first". Note that you do not need to erase EEPROM. 
REM The `-B` sets the speed. 5=slow enough for 1Mhz factory chips. 
REM Found emperically that you need a big `-B` when programming EEPROM or else weird AVRDUDE errors and crashes. 

avrdude -Cavrdude.conf -v -pattiny25 -cusbtiny -e -U flash:w:program.hex:i -B 5 -u
@if NOT ["%errorlevel%"]==["0"] (
    goto error
)

@if %2 GEQ 4096 (
	echo "Timebase must be 12 bits or less"
	goto error
)

set /a thigh=%2 / 256
set /a tlow=%2 %% 256

REM Note that the `0x84,0x03` represents the number of double ticks we do at startup. 0x0384=900 double ticks=1800 ticks=half way around face

avrdude -Cavrdude.conf -pattiny25 -cusbtiny -U eeprom:w:0x4a,0x4c,%1,0x00,%tlow%,%thigh%,0x84,0x03:m -v -B 100 -u
@if NOT ["%errorlevel%"]==["0"] (
    goto error

)

goto done

:error
@color 47
@echo ERROR CLOCK NOT PROGRAMMED!
pause
exit /b %errorlevel%
:done
@color 27
@echo SUCCESS

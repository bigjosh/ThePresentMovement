REM pass args for timebase, count of seconds per tick. Note that assumes count less than 256 (could be full 16 bits, but you'll have to edit this batch file)
REM timebase can be 2=secs 3=mins
REM Note that 0x384=900=The number of double ticks to make it half way around the face. 
REM first program the FLASH since we need to do erase first and it would also erase the EEPROM. 
REM The `-u` means "do not read the fuses"
REM The `-e` means "erase all chip memories first". Note that you do not need to erase EEPROM. 
REM The `-B` sets the speed. 5=slow enough for 1Mhz factor chips. 
REM Found emperically that you need a big `-B` when programming EEPROM or else weired AVRDUDE errors and crashes. 

color 07
avrdude -Cavrdude.conf -v -pattiny25 -cusbtiny -e -U flash:w:program.hex:i -B 5 -u
@if NOT ["%errorlevel%"]==["0"] (
    goto error
)

avrdude -Cavrdude.conf -pattiny25 -cusbtiny -U eeprom:w:0x4a,0x4c,%1,0x00,%2,0x00,0x84,0x03:m -v -B 100 -u
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

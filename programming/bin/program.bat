REM pass args for timebase and count. Note that assumes count less than 256 (could be full 16 bits, but you'll have to edit this batch file)
REM timebase can be 2=secs 3=mins
REM first program the EEPROM which is much slower so needs big B value
color 07
avrdude -Cavrdude.conf -pattiny25 -cusbtiny -U eeprom:w:0x4a,0x4c,2,0x00,4,0x00:m -v -B 50 -u
@if NOT ["%errorlevel%"]==["0"] (
    goto error

)
avrdude -Cavrdude.conf -v -pattiny25 -cusbtiny -e -U flash:w:program.hex:i -u -B 3
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

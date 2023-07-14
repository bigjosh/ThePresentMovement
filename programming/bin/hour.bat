avrdude -Cavrdude.conf -v -pattiny25 -cusbtiny -Uflash:w:hour.hex:i -u 
@if NOT ["%errorlevel%"]==["0"] (
    @color 47
    @echo ERROR CLOCK NOT PROGRAMMED!
    pause
    exit /b %errorlevel%
)
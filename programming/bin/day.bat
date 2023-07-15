avrdude -Cavrdude.conf -v -pattiny25 -cusbtiny -Uflash:w:day.hex:i -u -B 3
@if NOT ["%errorlevel%"]==["0"] (
    @color 47
    @echo ERROR CLOCK NOT PROGRAMMED!
    pause
    exit /b %errorlevel%
)
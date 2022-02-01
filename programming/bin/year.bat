avrdude -Cavrdude.conf -v -pattiny25 -cusbtiny -Uflash:w:year.hex:i -u -B 3
    @color 47
    @echo ERROR CLOCK NOT PROGRAMMED!
    pause
    exit /b %errorlevel%
)
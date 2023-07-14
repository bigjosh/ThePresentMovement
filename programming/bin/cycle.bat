copy D:\Github\DayMoonYearMovement\DayMoonYearMovement\DayMoonYearMovement\Debug\DayMoonYearMovement.hex .
avrdude -Cavrdude.conf -v -pattiny25 -cusbtiny -Uflash:w:DayMoonYearMovement.hex:i -u
pause

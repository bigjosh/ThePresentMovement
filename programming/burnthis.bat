REM Program a TSL unit and then add a record to the airtable units database
REM Args: Label serial number of the unit to be programmed

REM We need the APIKEY to access airtable, but we do not want to store it inside this batch file
REM since then it would be exposed. So we keep it in the local machine registry under the 
REM name `burnthis_airtable_api_key`. If you ever want to delete this key, you run the command...
REM `REG delete HKCU\Environment /V burnthis_airtable_api_key`

color

REM Update delayseconds to reflect the number of seconds to add to the start time to account for
REM the delay in getting everything programmed into the EEPROM


set delayseconds=4
set firmwarefile=tsl.hex

set tempunparsedstarttimefile=%tmp%\unparsedstarttime.txt
set tempeepromfile=%tmp%\burniteeprom.txt
set tempdeviceidfile=%tmp%\burnitdeviceid.txt
set tempfirmwarehashfile=%tmp%\burnitfirmwarehash.txt
set tempfirmwarerecordfile=%tmp%\burnitfirmwarerecord.txt


:nextserial
pause Press any key to program a clock or Control-C to quit

REM Running programming cycle in black background
REM Will turn red or green when done
color

echo Starting time programming sequence at %time%

REM Next lets generate the eeprom block with the current time as start time
eeprom-utils-bin\tsl-make-block %tempeepromfile% -o %delayseconds% | findstr "BURNTIME:" >%tempunparsedstarttimefile%
set /p starttimeline=<%tempunparsedstarttimefile%
REM The time is after the string "Start time:"
REM This funky syntax does a substring starting at pos 9
set "starttime=%starttimeline:~9%"

REM Param -u Disables the default behaviour of reading out the fuses three times before programming, then verifying at the end of programming
REM Param -v Verbose output

"D:\Program Files (x86)\Arduino\hardware\tools\avr/bin/avrdude" -C"D:\Program Files (x86)\Arduino\hardware\tools\avr/etc/avrdude.conf" -v -u -pattiny25 -cusbtiny -Uflash:w:DayMoonYearMovement.hex:i

if errorlevel 1 (
	goto error
)

REM Color white on green
color 27
@echo SUCCESS
REM Loop back for next unit if we started in scanning batch mode
IF "%~1" == "" goto nextserial
goto end


:error


REM Color white on red
color 47
@echo ============================================================================================================
@echo ERROR
@echo ============================================================================================================

:end

echo Ending time programming sequence at %time%


REM testing
REM copy %tempeepromfile% .

REM Clean up after ourselves 
del %tempeepromfile%
del %tempunparsedstarttimefile%
del %tempdeviceidfile%
del %tempfirmwarehashfile%
del %tempfirmwarerecordfile%


@ENDLOCAL


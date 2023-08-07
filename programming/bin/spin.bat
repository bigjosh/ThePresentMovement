REM max spin needs about 50ms per tick ~= 3 1/64ths
REM This results in a 2.8 minute cycle time
REM https://www.google.com/search?q=%28%283%2F64+second%29+*+3600%29+in+minutes
REM note that you will likely need to do a battery pull in order to program a clock in spin mode
REM becuase 3/64ths of a second is not long enough for a program cycle to complete
REM without getting interrupted by an INT signal from the RTC (the INT shares a pin with programming)
program.bat 1 3
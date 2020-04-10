Ti RADAR Firmware:

1. Downloaded mmWave SDK 3.03

2. Create a new cli.c file for hard coded TM config. Attached here. Replaced the original cli.c file in the CLI folder in the SDK

3. Go to setenv.bat in mmWave SDK change to AWR1843. Run .bat file.

4. Change director to CLI folder and run gmake clean, gmake build(gmake all) commands

5. Open up CCS, import 18xx Demo example build DSS and MSS projects as usual.

6. Replace the mss_main.c with attached mss_main.c
7. Include the CAN library file(.aer4f file) in the file search path(properties) and add the library file path also in file search path
8. Build the project and you will be generated with a bin file.
9. Flash the bin file to the AWR1843 (flash mode)
10. Change the board to the functional mode and power up , we will be able to see data streaming over CANBUS
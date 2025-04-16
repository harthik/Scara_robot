1. source esp idf
2. cd Scara_robot/esp32_node
3. idf.py set-target esp32
4. idf.py menuconfig - ( set ip address and wifi credentials)
5. idf.py build
6. sudo chown $USER /dev/ttyUSB0
7. idf.py -p /dev/ttyUSB0 flash
8. start docker (follow instructions on main page)

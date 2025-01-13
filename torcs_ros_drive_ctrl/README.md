# torcs_ros_drive_ctrl

This is a separate implementation of the SimpleDriver contained in the original [SRC C++ client](https://sourceforge.net/projects/cig/files/SCR%20Championship/Client%20C%2B%2B/).
It subscribes to the sensor messsage published by the ```torcs_ros_client```, generates simple drive commands and publishes them as ```TORCSCtrl``` messages.

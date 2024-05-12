USERNAME=${UNAVCO_USERNAME} # make sure that this env var is being set in your bashrc
PASSWORD=${UNAVCO_PASSWORD} # make sure that this env var is being set in your bashrc
PORT=2101
MOUNT_POINT=P178_RTCM3
SERIAL_PORT=ttyUSB0
BAUD_RATE=115200

echo "Starting NTRIP client"
echo " ------------------------ "
echo "Username: ${USERNAME}"
echo "PASSWORD: ${PASSWORD}"
echo "Port: ${PORT}"
echo "Mount point: ${MOUNT_POINT}"
echo "Serial port: ${SERIAL_PORT}"
echo "Baud rate: ${BAUD_RATE}"
echo " ------------------------ "

# TODO: Check if serial port is available
# TODO: Exception handling?

str2str -in ntrip://$USERNAME:$PASSWORD@rtgpsout.unavco.org:$PORT/$MOUNT_POINT -out serial://$SERIAL_PORT:$BAUD_RATE:8:n:1

# str2str -in ntrip://$USERNAME:$PASSWORD@rtgpsout.unavco.org:$PORT/$MOUNT_POINT -out temp.log

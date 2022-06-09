value=$( ps -ef | grep -ic colorDetect.py )

echo $value

if [ $value -eq 1 ]
then
	echo "colorDetect.py not running" >> colordetect.log
	cd /home/pi/Projects
	/usr/bin/python ./colorDetect.py &
fi

value=$( ps -ef | grep -ic off-switch.py )

echo $value

if [ $value -eq 1 ]
then
	echo "off-switch.py not running" >> colordetect.log
	cd /home/pi/Projects
	/usr/bin/python ./off-switch.py &
fi

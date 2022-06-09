value=$( ps -ef | grep -ic off-switch.py )

echo $value

if [ $value -eq 1 ]
then
	echo "off-switch.py not running" >> runOffButton.log
	cd /home/pi/Projects
	/usr/bin/python ./off-switch.py &
fi

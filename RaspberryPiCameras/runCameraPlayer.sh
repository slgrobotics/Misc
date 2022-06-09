value=$( ps -ef | grep -ic raspivid )

echo $value

if [ $value -eq 1 ]
then
	echo "raspivid not running" >> runCameraPlayer.log
	cd /home/pi/Projects
	raspivid -f -t 0 &
fi

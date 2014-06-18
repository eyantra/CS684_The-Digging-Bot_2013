#!/bin/bash

oldts=$(stat -c %Y /media/disk4/esproj.jpg)

firefox -P "default" http://localhost/embedded/index.php?hip=http://10.129.28.153:8080/photo.jpg &

newts=$(stat -c %Y /media/disk4/esproj.jpg)
while [ $oldts = $newts ]
do
	newts=$(stat -c %Y /media/disk4/esproj.jpg)
	#echo "checking"
done

#killall firefox

g++ `pkg-config opencv --cflags` contours.cpp  -o contours `pkg-config opencv --libs`
./contours /media/disk4/esproj.jpg


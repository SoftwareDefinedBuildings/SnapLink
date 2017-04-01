#!/bin/bash

#set -e

yes Y | apt-get install python-imaging
yes Y | apt-get install python-requests
yes Y | apt-get install wget

cmake ..
make 

wget -r --no-parent https://people.eecs.berkeley.edu/~kaifei/download/buildsys16/410_demo/
DATA_PATH=people.eecs.berkeley.edu/~kaifei/download/buildsys16/410_demo/

./cellmate run $DATA_PATH/db/*.db & sleep 30 ; python ../test/http_client.py $DATA_PATH/test/ | tail -1 >  tail.txt
result=1
while read line; do
    for word in $line; do
        if [ $word = tests ]
        then 
            result=0
        fi
    done
done <tail.txt

exit $result

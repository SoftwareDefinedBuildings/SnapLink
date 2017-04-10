#!/bin/bash

#set -e

apt-get install -y python-imaging
apt-get install -y python-requests
apt-get install -y wget

cmake ..
make 

wget -r --no-parent https://people.eecs.berkeley.edu/~kaifei/download/buildsys16/410_demo/
DATA_PATH=people.eecs.berkeley.edu/~kaifei/download/buildsys16/410_demo/


./cellmate run $DATA_PATH/db/*.db |
  while IFS= read -r line
    do
      if [ "$line" == "Initialization Done" ]
      then 
        python ../test/http_client.py $DATA_PATH/test/ | tail -1 >  tail.txt
        result=1
        while read line; do
          for word in $line; do
            if [ $word = tests ]
            then
              result=0
            fi
          done
        done <tail.txt    
        
        if [ result -eq 1 ]
        then 
            exit 1
        fi
        tail_sentence=$(head -n 1 tail.txt)
        read num1 num2 num3 <<<${tail_sentence//[^0-9]/ }
        accuracy=$(( num1 * 100 / num2))
        if [ $accuracy -lt 90 ]
        then
          pkill -f cellmate
          exit 1
        fi
        pkill -f cellmate
        exit 0
      fi
    done

                  


#!/bin/bash
vphpl=$2
for ((agent_num=5; agent_num <= 50; agent_num+=5))
do
    ./AIMILP $agent_num ./data/"$vphpl"vphpl/vehicleArrival_"$vphpl"vphpl_random$1.json ./result/"$vphpl"vphpl/output$1.csv
done

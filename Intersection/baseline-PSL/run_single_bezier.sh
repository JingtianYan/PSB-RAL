#!/bin/bash
vphpl=500
for ((agent_num=1; agent_num <= 50; agent_num++))
do
    ./AIMPBS $agent_num ./data/"$vphpl"vphpl/vehicleArrival_"$vphpl"vphpl_random$1.json ./result/"$vphpl"vphpl/output$1.csv
done

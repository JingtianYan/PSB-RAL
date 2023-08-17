map_name="800"
rm ./result/$map_name"vphpl"/*
for ((traj=1; traj <= 25; traj++))
do
    bash ./run_single_MILP.sh $traj $map_name &
    # if (($traj % 5 == 0))
	# then
	# 	wait	
	# fi
done

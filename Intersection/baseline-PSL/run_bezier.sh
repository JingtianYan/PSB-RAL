for ((traj=1; traj <= 25; traj++))
do
    bash ./run_single_bezier.sh $traj &
    # if (($traj % 5 == 0))
	# then
	# 	wait	
	# fi
done

kill $(ps aux | grep MILP | awk '{print $2}')
kill $(ps aux | grep AIMILP | awk '{print $2}')
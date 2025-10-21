# Steps to recording live data to a csv

1. Ensure that you have all topics already publishing first

2. (To Record all data), you need to pass the `record_all_topics` parameter. Run the following.
```bash 
ros2 run adaptive_navigation_utilities live_data_to_csv --ros-args record_all_topics:=True
```

3. (To Record specific data), you need to pass a list to `list_of_topics` parameter. Run the following
```bash 
ros2 run adaptive_navigation_utilities live_data_to_csv --ros-args list_of_topics:=['/ctrl/cmd_vel', '/p1/pose2D', '/cluster_info']
```

4. To save data to a csv, you must open a new terminal a run `ros2 param set /live_read_to_csv save_to_file True` which will output a file called `output_MM_DD_YY_HHMMSS.csv` to the same directory that the node is running. Run the following
```bash
ros2 param set /live_read_to_csv save_to_file True
```


## Notes
This file `live_data_to_csv.py` is currently stored in the `adaptive_navigation_utilities, but it can really be stored in any package because it is a standalone script. 

Additionally, if the node does not detect any new updates in subscription data, it will timeout after 10 seconds and automatically save a csv file for you. You can modify this timeout parameter by adding the `--ros-args -p timeout:=10.0` flag to `X` seconds.
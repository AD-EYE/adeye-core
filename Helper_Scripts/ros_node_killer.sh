ps aux | grep -E 'op_path_planner|op_local_planner|trajectory_generator|autoware|kinetic|ros' | awk '{print $2}' | xargs kill -9

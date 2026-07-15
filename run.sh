#!/bin/bash

# Line 3 fix: Create directory if it doesn't exist, then enter it
cd build || { echo "[-] Failed to enter build directory"; exit 1; }


execute_benchmark() {
    # create a tmp text file from args
    echo "BOWConnect planner $1"
    # Ensure the executable exists in the build folder
    ./bow_connect_parallel $1 1
}



viz_results() {
    if [ ! -f traj_viewer.py ]; then
        ln -s ../traj_viewer.py .
    fi
    python3 traj_viewer.py "$1" --result "trajectory.csv"
}

# Note: Since we are now inside 'build', we might need to adjust 
# the path to the test file if it was relative to the root.
test_file="../$1"
echo "[+] executing = $test_file"

execute_benchmark "$test_file"
echo "[+] visualizing results"
viz_results "$test_file"

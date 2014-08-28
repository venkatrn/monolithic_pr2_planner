#!/bin/bash

for i in {0..9}; do
  #the planner expects the environment to stored at tableObstacles.yaml
  #so move the numbered one to this path
  cp experiments/tableObstacles$i.yaml experiments/tableObstacles.yaml

  #for method in dts rr; do
  #  #start the planner
  #  echo "starting planner"
  #  screen -dmS planner bash -c "source ~/ros/mha/setup.bash; roslaunch monolithic_pr2_planner_node run_experiments.launch"

  #  #a hack because the planner takes a while to start
  #  sleep 50

  #  #send the tests
  #  echo "sending goals from environment $i using the $method method"
  #  ./bin/runTests experiments/fbp_tests$i.yaml smha $method

  #  #when the trials are done, kill the planner
  #  echo "killing planner"
  #  screen -S planner -X quit
  #  #wait a bit to make sure the planner died
  #  sleep 20

  #  #save the stats
  #  mv mha_stats.csv stats/mha_stats_${method}_${i}.csv
  #done

  for method in rrt; do
    #start the planner
    echo "starting planner"
    screen -dmS planner bash -c "source ~/ros/mha/setup.bash; roslaunch monolithic_pr2_planner_node run_experiments.launch"

    #a hack because the planner takes a while to start
    sleep 50

    #send the tests
    echo "sending goals from environment $i using the $method method"
    ./bin/runTests experiments/fbp_tests$i.yaml $method

    #when the trials are done, kill the planner
    echo "killing planner"
    screen -S planner -X quit
    #wait a bit to make sure the planner died
    sleep 20

    #save the stats
    mv mha_stats.csv stats/mha_stats_${method}_${i}.csv
  done
done

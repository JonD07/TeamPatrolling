# set -e

#Run Exp 1: Increasing Nodes for a fixed standard team (1 UGV, 2 UAVs). Located in /test/IncreasingNodeExps
#     Tests start at 5 nodes and do 50 tests. Increment by 5 nodes each test until 100. Time and PAR results logged

echo "Running Exp 1 with Baseline Solver: Increasing Nodes for a fixed standard team (1 UGV, 2 UAVs). Test files located in /test/IncreasingNodeExps"

for i in {5..100..5}
do
    for j in {1..50}
    do
        echo "Running Exp 1 with Alg 1 IncreaseNodeExps/plot_1_2_${i}_${j}.yaml 1 0 1"
        ../../build/patrolling-solver ../IncreaseNodeExps/plot_1_2_${i}_${j}.yaml 1 0 1 "../IncreaseNodeExps/" $j
        wait
    done
done

echo "Running Exp 1 with Alg 2: Increasing Nodes for a fixed standard team (1 UGV, 2 UAVs). Test files located in /test/IncreasingNodeExps"

for i in {5..100..5}
do
    for j in {1..50}
    do
        echo "Running Exp 1 with Alg 2 IncreaseNodeExps/plot_1_2_${i}_${j}.yaml 2 0 1"
        ../../build/patrolling-solver ../IncreaseNodeExps/plot_1_2_${i}_${j}.yaml 2 0 1 "../IncreaseNodeExps/" $j
        wait
    done
done

echo "Running Exp 1 with ILO Solver (3): Increasing Nodes for a fixed standard team (1 UGV, 2 UAVs). Test files located in /test/IncreasingNodeExps"

for i in {5..100..5}
do
    for j in {1..50}
    do
            echo "Running Alg 3 IncreaseNodeExps/plot_1_2_${i}_${j}.yaml 3 0 1"
        ../../build/patrolling-solver ../IncreaseNodeExps/plot_1_2_${i}_${j}.yaml 3 0 1 "../IncreaseNodeExps/" $j
        wait
    done
done

# #Run exp 2: Increasing the number of UAVs with a fixed number of nodes (50). Located in /test/IncreasedUAVsExps

echo "Running Exp 2 with Baseline Solver: Increasing the number of UAVs with a fixed number of nodes (50)."

for i in {1..10}
do
    for j in {1..50}
    do
        echo "Running Exp 2 with Alg 1 ../IncreaseUAVsExps/plot_1_${i}_50_${j}.yaml 1 0 1"
        ../../build/patrolling-solver ../IncreaseUAVsExps/plot_1_${i}_50_${j}.yaml 1 0 1 "../IncreaseUAVsExps/" $j
        wait
    done
done

echo "Running Exp 2 with Alg 2: Increasing the number of UAVs with a fixed number of nodes (50)."

for i in {1..10}
do
    for j in {1..50}
    do
        echo "Running Exp 2 with Alg 2 ../IncreaseUAVsExps/plot_1_${i}_50_${j}.yaml 2 0 1"
        ../../build/patrolling-solver ../IncreaseUAVsExps/plot_1_${i}_50_${j}.yaml 2 0 1 "../IncreaseUAVsExps/" $j
        wait
    done
done

echo "Running Exp 2 with ILO (3): Increasing the number of UAVs with a fixed number of nodes (50)."

for i in {1..10}
do
    for j in {1..50}
    do
        echo "Running Exp 2 with Alg 3 ../IncreaseUAVsExps/plot_1_${i}_50_${j}.yaml 3 0 1"
        ../../build/patrolling-solver ../IncreaseUAVsExps/plot_1_${i}_50_${j}.yaml 3 0 1 "../IncreaseUAVsExps/" $j
        wait
    done
done

# Run exp 3: Increasing the number of UAV/UGV teams (each team has 2 UAVs and 1 UGV) with a fixed number of nodes (50). Located in /test/IncreasedUAVsExps

echo "Running Exp 3 with Baseline Solver: Increasing the number of UAV/UGV teams with a fixed number of nodes (50)."

for i in {1..10}
do
    droneNumber=$((i*2))
    for j in {1..50}
    do
        echo "Running Exp 3 with Alg 1 ../IncreaseTeamsExps/plot_${i}_${droneNumber}_50_${j}.yaml 1 0 1"
        ../../build/patrolling-solver ../IncreaseTeamsExps/plot_${i}_${droneNumber}_50_${j}.yaml 1 0 1 "../IncreaseTeamsExps/" $j
        wait
    done
done

echo "Running Exp 3 with ILO Solver: Increasing the number of UAV/UGV teams with a fixed number of nodes (50)."

for i in {1..10}
do
    droneNumber=$((i*2))
    for j in {1..50}
    do
        echo "Running Exp 3 with Alg 2 ../IncreaseTeamsExps/plot_${i}_${droneNumber}_50_${j}.yaml 2 0 1"
        ../../build/patrolling-solver ../IncreaseTeamsExps/plot_${i}_${droneNumber}_50_${j}.yaml 2 0 1 "../IncreaseTeamsExps/" $j
        wait
    done
done

echo "Running Exp 3 with VRP Solver: Increasing the number of UAV/UGV teams with a fixed number of nodes (50)."

for i in {1..10}
do
    droneNumber=$((i*2))
    for j in {1..50}
    do
        echo "Running Exp 3 with Alg 3 ../IncreaseTeamsExps/plot_${i}_${droneNumber}_50_${j}.yaml 3 0 1"
        ../../build/patrolling-solver ../IncreaseTeamsExps/plot_${i}_${droneNumber}_50_${j}.yaml 3 0 1 "../IncreaseTeamsExps/" $j
        wait
    done
done


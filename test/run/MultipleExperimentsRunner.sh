#Run Exp 1: Increasing Nodes for a fixed standard team (1 UGV, 2 UAVs). Located in /test/IncreasingNodeExps
#     Tests start at 5 nodes and do 50 tests. Increment by 5 nodes each test until 100. Time and PAR results logged

echo "Running Exp 1 with Baseline Solver: Increasing Nodes for a fixed standard team (1 UGV, 2 UAVs). Test files located in /test/IncreasingNodeExps"

for i in {5..100..5}
do
    for j in {1..50}
    do
        ../../build/patrolling-solver ../IncreasingNodeExps/plot_1_2_$i_$j.yaml 1 >> ../ExpOutputs/Exp1_Baseline.txt
        wait
    done
done

echo "Running Exp 1 with ILO Solver: Increasing Nodes for a fixed standard team (1 UGV, 2 UAVs). Test files located in /test/IncreasingNodeExps"

for i in {5..100..5}
do
    for j in {1..50}
    do
        ../../build/patrolling-solver ../IncreasingNodeExps/plot_1_2_${i}_${j}.yaml 2 >> ../ExpOutputs/Exp1_ILO.txt
        wait
    done
done

echo "Running Exp 1 with VRP Solver: Increasing Nodes for a fixed standard team (1 UGV, 2 UAVs). Test files located in /test/IncreasingNodeExps"

for i in {5..100..5}
do
    for j in {1..50}
    do
        ../../build/patrolling-solver ../IncreasingNodeExps/plot_1_2_$i_$j.yaml 3 >> ../ExpOutputs/Exp1_VRP.txt
        wait
    done
done

#Run exp 2: Increasing the number of UAVs with a fixed number of nodes (50). Located in /test/IncreasedUAVsExps

echo "Running Exp 2 with Baseline Solver: Increasing the number of UAVs with a fixed number of nodes (50)."

for i in {1..10}
do
    for j in {1..50}
    do
        ../../build/patrolling-solver ../IncreaseUAVsExps/plot_1_$i_50_$j.yaml 1 >> ../ExpOutputs/Exp2_Baseline.txt
        wait
    done
done

echo "Running Exp 2 with ILO Solver: Increasing the number of UAVs with a fixed number of nodes (50)."

for i in {1..10}
do
    for j in {1..50}
    do
        ../../build/patrolling-solver ../IncreaseUAVsExps/plot_1_$i_50_$j.yaml 2 >> ../ExpOutputs/Exp2_ILO.txt
        wait
    done
done

echo "Running Exp 2 with VRP Solver: Increasing the number of UAVs with a fixed number of nodes (50)."

for i in {1..10}
do
    for j in {1..50}
    do
        ../../build/patrolling-solver ../IncreaseUAVsExps/plot_1_$i_50_$j.yaml 3 >> ../ExpOutputs/Exp2_VRP.txt
        wait
    done
done

#Run exp 3: Increasing the number of UAV/UGV teams (each team has 2 UAVs and 1 UGV) with a fixed number of nodes (50). Located in /test/IncreasedUAVsExps

echo "Running Exp 3 with Baseline Solver: Increasing the number of UAV/UGV teams with a fixed number of nodes (50)."

for i in {1..10}
do
    droneNumber=$((i*2))
    for j in {1..50}
    do
        echo "Running test with $i UAVs"
        ../../build/patrolling-solver ../IncreaseTeamsExps/plot_$i_$droneNumber_50_$j.yaml 1 >> ../ExpOutputs/Exp3_Baseline.txt
        wait
    done
done

echo "Running Exp 3 with ILO Solver: Increasing the number of UAV/UGV teams with a fixed number of nodes (50)."

for i in {1..10}
do
    droneNumber=$((i*2))
    for j in {1..50}
    do
        echo "Running test with $i UAVs"
        ../../build/patrolling-solver ../IncreaseTeamsExps/plot_$i_$droneNumber_50_$j.yaml 2 >> ../ExpOutputs/Exp2_ILO.txt
        wait
    done
done

echo "Running Exp 3 with VRP Solver: Increasing the number of UAV/UGV teams with a fixed number of nodes (50)."

for i in {1..10}
do
    droneNumber=$((i*2))
    for j in {1..50}
    do
        echo "Running test with $i UAVs"
        ../../build/patrolling-solver ../IncreaseTeamsExps/plot_$i_$droneNumber_50_$j.yaml 3 >> ../ExpOutputs/Exp3_VRP.txt
        wait
    done
done


#!/bin/bash
echo "Current directory: $(pwd)"
source /mnt/data/miniconda3/bin/activate
conda activate CausalityFalsificationLiteRacer
cd /mnt/data/repos/CausalityFalsificationLiteRacer

for i in $(seq 180 199);
do
    echo $i
    python generate_counterexample.py -S $i -O 18 -T 10
    python responsibility.py -C $i -A responsibility
    python responsibility.py -C $i -A responsibility_discrete
done



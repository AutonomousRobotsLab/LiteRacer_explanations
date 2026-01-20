#!/bin/bash
echo "Current directory: $(pwd)"
source /mnt/data/miniconda3/bin/activate
conda activate CausalityFalsificationLiteRacer
cd /mnt/data/repos/CausalityFalsificationLiteRacer


python generate_counterexample.py -S 100 -O 6 -T 5
python responsibility.py -C 100 -A responsibility
python responsibility.py -C 100 -A responsibility_discrete


#!/bin/bash
echo "Current directory: $(pwd)"
source /mnt/data/miniconda3/bin/activate
conda activate CausalityFalsificationLiteRacer
cd /mnt/data/repos/CausalityFalsificationLiteRacer

echo $(date)
python run.py
echo $(date)


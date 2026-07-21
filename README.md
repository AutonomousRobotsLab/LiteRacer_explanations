# Explaining Failures of Cyber-Physical Systems with Actual Causality - Code Artifact

The artifact contains all the necessary code for running the experiment presented in the paper
[*Explaining Failures of Cyber-Physical Systems with Actual Causality*](https://kclpure.kcl.ac.uk/portal/en/publications/explaining-failures-of-cyber-physical-systems-with-actual-causali/)

## Installation 
We recommend using a conda environment with Python 3.11 to install these packages. If conda installation is not possible, you can also try using pip.

Install the following packages:

```shell
numpy==1.26.4
scipy==1.13.0
matplotlib==3.8.4
shapely==2.0.1
gym==0.26.2
stable-baselines3==1.1.0
torch==2.8.0
```

## Usage
To run the evaluation for a single example, run the following three lines, where `-C` indicate the falsifying example ID number (0-31).

```shell
python responsibility.py -C 0 -T 5 -A responsibility_discrete -N 301
python responsibility.py -C 0 -T 5 -A exhaustive_observed -N 301
```

The code creates separate results files in csv formats in the `output` folder. 

## Reproducing the paper results

The `run_all_examples.sh` script runs all the examples and creates separate results files in csv formats in the `output` folder.

You can use the `analyze_results.py` script to combine all results csv files to one results file, `combined_results.csv`, which contains all results presented in the paper.

## Citing the paper
```
@inproceedings{elimelech_explaining_2026,
  author={Elimelech, Khen and Yaacov, Tom and Kelly, David A and Chockler, Hana and Vardi, Moshe Y.},
  title={Explaining Failures of Cyber-Physical Systems with Actual Causality}, 
  booktitle = {International Conference on Robotics and Automation (ICRA)},
  publisher = {{IEEE}},
  year      = {2026},
}
```
#!/bin/bash
### sbatch config parameters must start with #SBATCH and must precede any other command. to ignore just add another # - like so ##SBATCH
#SBATCH --partition main ### specify partition name where to run a job
#SBATCH --time 2-00:00:00 ### limit the time of job running. Format: D-H:MM:SS
#SBATCH --job-name run_responsibility_34 ### name of the job. replace my_job with your desired job name
#SBATCH --output run_responsibility_34.out ### output log for running job - %J is the job number variable
#SBATCH --mail-user=tomya@post.bgu.ac.il ### users email for sending job status notifications ñ replace with yours
#SBATCH --mail-type=FAIL ### conditions when to send the email. ALL,BEGIN,END,FAIL, REQUEU, NONE
#SBATCH --mem=16G ### total amount of RAM // 500
#SBATCH --ntasks=1

### Start you code below ####
module load anaconda ### load anaconda module
source activate LiteRacer ### activating Conda environment. Environment must be configured before running the job
cd ~/repos/CausalityFalsificationLiteRacer/ || exit

~/.conda/envs/LiteRacer/bin/python ~/repos/CausalityFalsificationLiteRacer/responsibility.py -C 34 -T 8 -A responsibility_discrete -M -0.05 -X 0.05 -S 0.01 -N 401
~/.conda/envs/LiteRacer/bin/python ~/repos/CausalityFalsificationLiteRacer/responsibility.py -C 34 -T 8 -A responsibility -M -0.05 -X 0.05 -S 0.01 -N 401
~/.conda/envs/LiteRacer/bin/python ~/repos/CausalityFalsificationLiteRacer/responsibility.py -C 34 -T 8 -A exhaustive_observed -M -0.05 -X 0.05 -S 0.01 -N 401
~/.conda/envs/LiteRacer/bin/python ~/repos/CausalityFalsificationLiteRacer/responsibility.py -C 34 -T 8 -A exhaustive -M -0.05 -X 0.05 -S 0.01 -N 401
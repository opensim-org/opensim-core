#!/usr/bin/env python3

import sys
import yaml
import argparse

with open('config.yaml') as f:
    config = yaml.load(f)

parser = argparse.ArgumentParser(
        description="Submit a Moco job to the Stanford Sherlock computing cluster.")
parser.add_argument('directory' type=str, help="Location of input files.")
parser.add_argument('time' type=str

batch = 
f"""#!/bin/bash
#SBATCH --job-name={name}
#SBATCH --output={name}.out
#SBATCH --error={name}.err
#SBATCH --time={time}
#SBATCH --mail-type=END
#SBATCH --mail-user={sunetid}@stanford.edu
#SBATCH --nodes=1
#SBATCH --partition=owners,normal
module load openblas
module load gcc/8.1.0
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$GROUP_HOME/opensim-moco/moco_dependencies_install/adol-c/lib64
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$GROUP_HOME/opensim-moco/moco_dependencies_install/ipopt/lib
cd {directory}
$GROUP_HOME/opensim-moco/opensim-moco-install/bin/opensim-moco run-tool exampleMocoTrack.omoco
"""

# TODO get moco from DockerHub.

os.system('ssh %s@login.sherlock.stanford.edu' % config['sunetid'])

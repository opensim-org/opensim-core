#!/usr/bin/env python3

import os
import sys
import yaml
import argparse
import datetime
from pathlib import Path

parser = argparse.ArgumentParser(
        description="Submit a Moco job to the Stanford Sherlock computing cluster.")
# TODO ask for a description/note for the job, and write it to the job
# directory.
parser.add_argument('directory', type=str, help="Location of input files.")
parser.add_argument('--duration', type=str, default="00:30:00",
        help="Maximum duration for the job in HH:MM:SS.")
parser.add_argument('--name', type=str, default="",
        help="A name for the job (default: directory name).")

parser.add_argument('--sshmaster', dest='sshmaster', action='store_true',
        help="Start master SSH session (default).")
parser.add_argument('--no-sshmaster', dest='sshmaster', action='store_false',
        help="Do not start master SSH session (if you already started one).")
parser.add_argument('--sshexit', dest='sshexit', action='store_true',
        help="Exit the master session after submitting the job (default).")
parser.add_argument('--no-sshexit', dest='sshexit', action='store_false',
        help=("Do not exit the master session after submitting (if you plan to "
            "submit more jobs."))
parser.set_defaults(sshmaster=True, sshexit=True)

# TODO windows
# TODO initial configuring gdrive on Sherlock.
# TODO building a docker container from a branch (as a separate step?).


args = parser.parse_args()
directory = args.directory
if args.name != "":
    name = args.name
else:
    name = Path(directory).name
duration = args.duration
sshmaster = args.sshmaster
sshexit = args.sshexit

if ' ' in name:
    raise Exception("Cannot have spaces in name.")

with open('config.yaml') as f:
    config = yaml.safe_load(f)

sunetid = config['sunetid']



# Check that the directory contains setup.omoco.
if not os.path.exists(os.path.join(directory, 'setup.omoco')):
    raise Exception(f"setup.omoco is missing from {directory}.")

home = str(Path.home()) # Should work on Windows and UNIX.
if not os.path.exists(f'{home}/.ssh/controlmasters/'):
    os.makedirs(f'{home}/.ssh/controlmasters')
# Create master (-M) SSH session in the background (-f), without running
# a command (-N), with 
# https://unix.stackexchange.com/questions/83806/how-to-kill-ssh-session-that-was-started-with-the-f-option-run-in-background
control_path = "~/.ssh/controlmasters/%C"
server = f"{sunetid}@login.sherlock.stanford.edu"
now = datetime.datetime.now()
time = '%s.%i' % (now.strftime('%Y-%m-%dT%H%M%S'), now.microsecond)
job_directory = '%s-%s' % (time, name)
print(f"Submitting {job_directory}")
mocojobs_dir = f"~/nmbl/mocojobs/"
server_job_dir = f"{mocojobs_dir}{job_directory}"
if sshmaster:
    os.system(f'ssh -M -f -N -S {control_path} {server}')

batch = f"""#!/bin/bash
#SBATCH --job-name={name}
#SBATCH --output={name}.out
#SBATCH --error={name}.err
#SBATCH --time={duration}
#SBATCH --mail-type=END
#SBATCH --mail-user={sunetid}@stanford.edu
#SBATCH --nodes=1
#SBATCH --partition=owners,normal
module load gcc/8.1.0

singularity exec $GROUP_HOME/opensim-moco/opensim-moco_latest.sif /opensim-moco-install/bin/opensim-moco run-tool setup.omoco


# Upload results to Google Drive.
module load system gdrive

opensim_moco_folder_id=$(gdrive list | grep 'opensim-moco' | cut -d" " -f1)
if [[ -z "$opensim_moco_folder_id" ]]; then
    echo "Creating opensim-moco folder."
    opensim_moco_folder_id=$(gdrive mkdir opensim-moco | cut -d" " -f2)
else
    echo "opensim-moco folder exists."
fi

# Copy results.
gdrive upload --recursive --parent $opensim_moco_folder_id {server_job_dir}

"""

with open(f'{directory}/{name}.batch', 'w') as f:
    f.write(batch)

# Re-use existing SSH tunnel.
# Recursively make the job directory.
os.system(f'ssh -S {control_path} {server} "mkdir -p {mocojobs_dir}"')
os.system(f"scp -o ControlPath={control_path} -r '{directory}/' {server}:{server_job_dir}")

os.system(f'ssh -S {control_path} {server} "cd {server_job_dir} && sbatch {name}.batch"')

if sshexit:
    os.system(f'ssh -S {control_path} -O exit {server}')

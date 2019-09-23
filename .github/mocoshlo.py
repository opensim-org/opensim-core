#!/usr/bin/env python3

import os
import sys
import yaml
import argparse
import datetime
from pathlib import Path

parser = argparse.ArgumentParser(
        description="Submit a Moco job to the Stanford Sherlock computing cluster.")
parser.add_argument('directory', type=str, help="Location of input files.")
parser.add_argument('--sunetid', type=str, default=None,
                    help="SUNetID for logging into Sherlock. Overrides the "
                         "the sunetid field in a config.yaml file in the "
                         "current directory.")
parser.add_argument('--duration', type=str, default="00:30:00",
        help="Maximum duration for the job in HH:MM:SS.")
parser.add_argument('--name', type=str, default="",
        help="A name for the job (default: directory name).")
parser.add_argument('--note', type=str, default="",
        help="A note to save to the directory (as note.txt).")

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
parser.add_argument('--command', type=str, default=None,
                    help=("The job should run the given command."
                          "Otherwise, the job is to run an OMOCO file named "
                          "setup.omoco."))
parser.add_argument('--mocotag', type=str, default='latest',
                    help="Use container "
                         "$GROUP_HOME/{sunetid}/opensim-moco/"
                         "opensim-moco_{mocotag}. Default: latest.")
parser.add_argument('--container', type=str, default=None,
                    help="Path to the singularity container (.sif) "
                         "on the cluster. Overrides --mocotag.")
parser.add_argument('--exclude', type=str, default=None,
                    help="Exclude files from copying to the cluster. This is "
                         "passed onto rsync --exclude.")
parser.add_argument('--parallelism', type=int, default=4,
                    help="Number of parallel threads. Default: 4.")

# TODO windows
# TODO initial configuring gdrive on Sherlock.
# TODO building a docker container from a branch (as a separate step?).
# TODO allow customizing where files are saved in Google Drive.
# TODO: support ignoring directories when copying (venv).
# TODO: memory seems fixed at 4GB. use mem-per-cpu=4000M?


args = parser.parse_args()
directory = args.directory
if args.name != "":
    name = args.name
else:
    name = Path(directory).absolute().name
duration = args.duration
note = args.note
sshmaster = args.sshmaster
sshexit = args.sshexit

if ' ' in name:
    raise Exception("Cannot have spaces in name.")

sunetid = None
if args.sunetid:
    sunetid = args.sunetid
else:
    with open('config.yaml') as f:
        config = yaml.safe_load(f)
    sunetid = config['sunetid']



# Check that the directory contains setup.omoco.
if (not args.command and not os.path.exists(
        os.path.join(directory, 'setup.omoco'))):
    raise Exception(f'setup.omoco is missing from {directory}.')

# if note:
#     with open(os.path.join(directory, 'note.txt'), 'w') as f:
#         f.write(note)


home = str(Path.home()) # Should work on Windows and UNIX.
if not os.path.exists(f'{home}/.ssh/controlmasters/'):
    os.makedirs(f'{home}/.ssh/controlmasters')
# Create master (-M) SSH session in the background (-f), without running
# a command (-N), with 
# https://unix.stackexchange.com/questions/83806/how-to-kill-ssh-session-that-was-started-with-the-f-option-run-in-background
control_path = "~/.ssh/controlmasters/%C"
server = f"{sunetid}@login.sherlock.stanford.edu"
now = datetime.datetime.now()
date = now.strftime('%Y-%m-%d')
time = '%s.%i' % (now.strftime('%Y-%m-%dT%H%M%S'), now.microsecond)
# TODO: remove microseconds and put in SLURM JOBID.
job_directory = '%s-%s' % (time, name)
print(f"Submitting {job_directory}")
mocojobs_dir = f"~/nmbl/mocojobs/"
server_job_dir = f"{mocojobs_dir}{job_directory}"
if sshmaster:
    os.system(f'ssh -M -f -N -S {control_path} {server}')


container = f'$GROUP_HOME/{sunetid}/opensim-moco/opensim-moco_{args.mocotag}.sif'
if args.container:
    container = args.container

command = '/opensim-moco-install/bin/opensim-moco run-tool setup.omoco'
if args.command:
    command = args.command

batch = f"""#!/bin/bash
#SBATCH --job-name={name}
#SBATCH --output={name}.out
#SBATCH --error={name}.err
#SBATCH --time={duration}
#SBATCH --mail-type=ALL
#SBATCH --mail-user={sunetid}@stanford.edu
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task={args.parallelism}
#SBATCH --mem-per-cpu=2000M
#SBATCH --partition=owners,normal
module load gcc/8.1.0

echo "nproc: $(nproc)"
# TODO: set to 2 * nproc?
export OPENSIM_MOCO_PARALLEL=$(nproc)
singularity exec {container} {command}


# Upload results to Google Drive.
module load system gdrive

# TODO: What happens when we hit max files to list (--max)?
# TODO: Sometimes, we create duplicate opensim-moco folders in Google Drive.
opensim_moco_folder_id=$(gdrive list --absolute | grep 'opensim-moco ' | cut -d" " -f1)
if [[ -z "$opensim_moco_folder_id" ]]; then
    echo "Creating opensim-moco folder."
    opensim_moco_folder_id=$(gdrive mkdir opensim-moco | cut -d" " -f2)
else
    echo "opensim-moco folder exists."
fi

# date_folder_id=$(gdrive list | grep 'opensim-moco/{date}' | cut -d" " -f1)
# if [[ -z "$date_folder_id" ]]; then
#     date_folder_id=$(gdrive mkdir --parent $opensim_moco_folder_id {date} | cut -d" " -f2)
# fi

# Copy results.
# gdrive upload --recursive --parent $date_folder_id {server_job_dir}
gdrive upload --recursive --parent $opensim_moco_folder_id {server_job_dir}

"""

with open(f'{directory}/{name}.batch', 'w') as f:
    f.write(batch)

# Re-use existing SSH tunnel.
# Recursively make the job directory.
os.system(f'ssh -S {control_path} {server} "mkdir -p {mocojobs_dir}"')
rsync_args = ''
if args.exclude:
    rsync_args = f'--exclude={args.exclude}'

os.system(f"rsync --rsh='ssh -o ControlPath={control_path}' --archive --compress --recursive {rsync_args} '{directory}/' {server}:{server_job_dir}")

os.system(f'ssh -S {control_path} {server} "cd {server_job_dir} && echo \"{note}\" > note.txt && sbatch {name}.batch"')

if sshexit:
    os.system(f'ssh -S {control_path} -O exit {server}')

#!/bin/bash
#SBATCH --job-name=build_vlm2
#SBATCH --partition=YOUR_PARTITION
#SBATCH --account=YOUR_ACCOUNT
#SBATCH --cpus-per-task=4
#SBATCH --mem=16G
#SBATCH --time=01:00:00
#SBATCH --output=%x_%j.log
# Run with VLMNAV_BASE set; the definition files must be in $VLMNAV_BASE (copy containers/*.def there).
BASE=${VLMNAV_BASE:?set VLMNAV_BASE to the project directory}
export APPTAINER_CACHEDIR=$BASE/.apptainer_cache
export APPTAINER_TMPDIR=$BASE/.apptainer_tmp
mkdir -p $APPTAINER_CACHEDIR $APPTAINER_TMPDIR
cd $BASE
apptainer build --fakeroot vlm2.sif vlm2.def
echo "EXIT CODE: $?"
echo DONE

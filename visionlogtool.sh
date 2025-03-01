#!/bin/bash

# A "helpful" shell utility to extract log files from the RIO

DOWNLOAD_PATH=~/extracted_`date +%Y-%m-%d_%H-%M`

echo "Saving logs to $DOWNLOAD_PATH"

mkdir -p $DOWNLOAD_PATH

scp lvuser@10.34.14.2:~/logs/* $DOWNLOAD_PATH

# NUKE IT
# ssh -c "rm ~/logs/*" lvuser@10.34.14.2

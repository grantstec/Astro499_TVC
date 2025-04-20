#!/bin/bash
# Script to clean large files from Git repository history

echo "Cleaning repository of large files..."

# Create a backup branch of current state
git branch backup-full-history || echo "Backup branch already exists or couldn't be created"

# Remove the large files from Git history using filter-repo
# You may need to install git-filter-repo first: pip install git-filter-repo
# Removing specific large files
git filter-repo --path "GUI_GSEv2/windows-amd64 (2).zip" --invert-paths
git filter-repo --path "GUI_GSEv2/windows-amd64.zip" --invert-paths
git filter-repo --path "GUI_GSEv2/windows-amd64/java/lib/modules" --invert-paths
git filter-repo --path "GUI_GSEv2/windows-amd64/" --invert-paths
git filter-repo --path "GUI_GSEv2/data/flight_data_20250418_215027.csv" --invert-paths
git filter-repo --path "GUI_GSEv2/windows-amd64/data/rocket.obj" --invert-paths

# Create a new clean branch
git checkout -b clean-branch

echo "Repository cleaned. Now you can push with: git push -u origin clean-branch --force"
echo "Remember that this has modified history, so other collaborators will need to re-clone or carefully merge."

#!/bin/bash
set -e

while getopts r flag
do
    case "${flag}" in
        r) FORCE_RESET="true";;
        *) echo "Invalid flags! (-r: reset repository)" && exit 1;;
    esac
done

cd "$HOME/ros_robot" || exit 1
git fetch origin
IS_UPTODATE=$(git diff origin/main)

if [ "$IS_UPTODATE" != "" ] || [ "$FORCE_RESET" == "true" ]; then
    git clean -dfx
    git reset --recurse-submodules --hard
    git pull origin main
    git submodule update --recursive
fi

echo "All up to date."
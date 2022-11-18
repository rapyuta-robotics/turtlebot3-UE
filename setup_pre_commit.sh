#!/bin/sh

pre-commit install;
for d in Plugins/*; do
    cd $d
    pre-commit install;
    cd -
done

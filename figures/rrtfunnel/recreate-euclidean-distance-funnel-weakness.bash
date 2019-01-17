#! /bin/bash

# Script which recreats the environment and the figures with the name of the script.

# Checkout the branch named the same as the figures, from the code repo.
${CODE_DIR:?} # Make sure the code dir variable is set.
${IMAGE_SCRIPT:?} # Make sure there is a image script provided for recreating the images.
cd ${CODE_DIR} && git checkout -b $1 && IMAGE_SCRIPT

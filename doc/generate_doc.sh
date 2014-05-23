#!/bin/bash

# Retrieve folder of this script
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Generate the Doxygen documentation
(
	cd "$DIR"
	doxygen Doxyfile
)
# EOF
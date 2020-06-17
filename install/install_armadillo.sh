#!/bin/bash

# ==================================
# define some colors for output
# ==================================
COLOR_RED="\033[1;31m"
COLOR_GREEN="\033[1;32m"
COLOR_YELLOW="\033[1;33m"
COLOR_BLUE="\033[1;34m"
COLOR_CYAN="\033[1;36m"
COLOR_WHITE="\033[1;37m"
COLOR_RESET="\033[0m"

cd $INSTALL_SCRIPTS_DIR/deps

echo -e $COLOR_CYAN"*******************************"$COLOR_RESET
echo -e $COLOR_CYAN"********   Armadillo   ********"$COLOR_RESET
echo -e $COLOR_CYAN"*******************************"$COLOR_RESET

ARMA_VERSION="9.860.2"

# remove other armadillo downloads
find  ./ -type d -name "armadillo-*" | xargs rm -rf

echo -e $COLOR_BLUE"Installing Armadillo Dependencies: cmake, OpenBLAS and LAPACK, wget, xz-utils..."$COLOR_RESET
sudo apt-get update > /dev/null && \
sudo apt-get install -y cmake libopenblas-dev liblapack-dev wget xz-utils > /dev/null && \

echo -e $COLOR_BLUE"Downloading and building Armadillo-"$ARMA_VERSION$COLOR_RESET

wget --no-check-certificate http://sourceforge.net/projects/arma/files/armadillo-$ARMA_VERSION.tar.xz > /dev/null
if [ $? -ne 0 ]; then
  echo -e $COLOR_RED"Failed to download Armadillo-"$ARMA_VERSION"...."$COLOR_RESET
  echo -e $COLOR_RED"Armadillo installation failed..."$COLOR_RESET
  AS64_ERROR=1
  return 1
fi
tar xvf armadillo-$ARMA_VERSION.tar.xz > /dev/null && \
rm -rf armadillo-$ARMA_VERSION.tar.xz && \
mv armadillo-$ARMA_VERSION armadillo
cd armadillo && \
echo -e $COLOR_BLUE"Building locally Armadillo-"$ARMA_VERSION$COLOR_RESET && \
cmake . && \
make && \
cd .. && \
# mv armadillo-$ARMA_VERSION $MAIN_WS_DIR/src/ext_libs && \
#echo -e $COLOR_BLUE"Installing Armadillo-"$ARMA_VERSION$COLOR_RESET && \
#sudo make install > /dev/null

if [ $? -eq 0 ]; then
  echo -e $COLOR_GREEN"Armadillo-"$ARMA_VERSION" successfully installed!"$COLOR_RESET
  AS64_ERROR=0
else
  echo -e $COLOR_RED"Armadillo installation failed..."$COLOR_RESET
  AS64_ERROR=1
fi

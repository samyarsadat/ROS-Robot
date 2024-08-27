#!/usr/bin/env bash
# A stripped down/modified version of Raspberry Pi's setup script for the Pico.
# It installs the C/C++ SDK, Picotool, Picoprobe, OpenOCD, and all required dependencies.
# SET USER TO ROOT BEFORE RUNNING!


# Exit on error
set -e

JNUM=12         # Number of threads (jobs) when running make
OUTDIR="/pico"  # Where will the output go?

# Dependencies
GIT_DEPS="git"
SDK_DEPS="cmake gcc-arm-none-eabi gcc g++"
OPENOCD_DEPS="gdb-multiarch automake autoconf build-essential texinfo libtool libftdi-dev libusb-1.0-0-dev"
DEPS="$GIT_DEPS $SDK_DEPS $OPENOCD_DEPS minicom"  # Full list of dependencies

echo "Installing Dependencies"
apt-get update
apt-get install -y $DEPS

# Create pico directory to put everything in
echo "Creating $OUTDIR"
mkdir -p $OUTDIR
cd $OUTDIR

# Clone sw repos
GITHUB_PREFIX="https://github.com/raspberrypi/"
GITHUB_SUFFIX=".git"
SDK_BRANCH="1.5.1"
PICOTOOL_BRANCH="1.1.2"

echo "" >> ~/.bashrc
echo "" >> /home/$NONROOT_USERNAME/.bashrc

DEST="$OUTDIR/pico-sdk"

if [ -d $DEST ]; then
    echo "$DEST already exists so skipping"
else
    REPO_URL="${GITHUB_PREFIX}pico-sdk${GITHUB_SUFFIX}"
    echo "Cloning $REPO_URL"
    git clone -b $SDK_BRANCH $REPO_URL

    # Any submodules
    cd $DEST
    git submodule update --init
    cd $OUTDIR

    # Define PICO_SDK_PATH in .bashrc
    VARNAME="PICO_SDK_PATH"
    echo "Adding $VARNAME to .bashrc"
    echo "export $VARNAME=$DEST" >> ~/.bashrc
    echo "export $VARNAME=$DEST" >> /home/$NONROOT_USERNAME/.bashrc
    export ${VARNAME}=$DEST
fi

# Pick up new variables we just defined
source ~/.bashrc

cd $OUTDIR

# Picotool
DEST="$OUTDIR/picotool"
REPO_URL="${GITHUB_PREFIX}picotool${GITHUB_SUFFIX}"
git clone --branch $PICOTOOL_BRANCH $REPO_URL

# Build both
cd $DEST
git submodule update --init
mkdir build
cd build
cmake ../
make -j$JNUM

echo "Installing picotool to /usr/local/bin/picotool"
cp picotool /usr/local/bin/

cd $OUTDIR

# Build OpenOCD
echo "Building OpenOCD"
cd $OUTDIR
OPENOCD_BRANCH="rp2040-v0.12.0"
OPENOCD_CONFIGURE_ARGS="--enable-ftdi --enable-sysfsgpio --enable-bcm2835gpio --enable-picoprobe"
git clone "${GITHUB_PREFIX}openocd${GITHUB_SUFFIX}" -b $OPENOCD_BRANCH --depth=1
cd openocd
./bootstrap
./configure $OPENOCD_CONFIGURE_ARGS
make -j$JNUM
make install
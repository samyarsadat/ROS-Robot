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
SDK_BRANCH="master"

echo "" >> ~/.bashrc
echo "" >> /home/$NONROOT_USERNAME/.bashrc

for REPO in sdk examples extras playground
do
    DEST="$OUTDIR/pico-$REPO"

    if [ -d $DEST ]; then
        echo "$DEST already exists so skipping"
    else
        REPO_URL="${GITHUB_PREFIX}pico-${REPO}${GITHUB_SUFFIX}"
        echo "Cloning $REPO_URL"
        git clone -b $SDK_BRANCH $REPO_URL

        # Any submodules
        cd $DEST
        git submodule update --init
        cd $OUTDIR

        # Define PICO_SDK_PATH in .bashrc
        VARNAME="PICO_${REPO^^}_PATH"
        echo "Adding $VARNAME to .bashrc"
        echo "export $VARNAME=$DEST" >> ~/.bashrc
        echo "export $VARNAME=$DEST" >> /home/$NONROOT_USERNAME/.bashrc
        export ${VARNAME}=$DEST
    fi
done

cd $OUTDIR

# Pick up new variables we just defined
source ~/.bashrc

# Build a couple of examples
cd "$OUTDIR/pico-examples"
mkdir build
cd build
cmake ../ -DCMAKE_BUILD_TYPE=Debug

for e in blink hello_world
do
    echo "Building $e"
    cd $e
    make -j$JNUM
    cd ..
done

cd $OUTDIR

# Picoprobe and picotool
for REPO in picoprobe picotool
do
    DEST="$OUTDIR/$REPO"
    REPO_URL="${GITHUB_PREFIX}${REPO}${GITHUB_SUFFIX}"
    git clone $REPO_URL

    # Build both
    cd $DEST
    git submodule update --init
    mkdir build
    cd build
    cmake ../
    make -j$JNUM

    if [[ "$REPO" == "picotool" ]]; then
        echo "Installing picotool to /usr/local/bin/picotool"
        cp picotool /usr/local/bin/
    fi

    cd $OUTDIR
done

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
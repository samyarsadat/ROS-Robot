## CodeQL Analysis for MicroROS & Pico Source Code

At the moment, CodeQL needs to be run in the same environment as the build steps. This means that it cannot analyze code that
is built inside of a Docker container, unless CodeQL itself also runs inside of that container. This becomes a problem when you
try to use the Pico Build Action and the MicroROS Build Action with CodeQL, as both Pico and MicroROS Build are Docker container
actions. Until the developers of the CodeQL action add a way to analyze build that run in a Docker container or until I (maybe)
somehow implement CodeQL into the Pico Build and MicroROS Build actions, I've come up with a temporary workaround.<br>
<br>
I've put modified versions of the build scripts from the build actions into the `codeql-pico-scripts` directory. I've also added
two new re-usable actions (the reason I've decided to use resusable actions isn't because I use them in more than one place, but
because I want them to be separated from everything else) for building and analyzing micro-ROS and the Pico source code using the
aforementioned scripts. You may have also noticed that these re-usable actions use the same Docker images as the actual Pico Build
Action and MicroROS Build Action.<br>
<br>
I have still left the original `pico-build.yml` workflow, as I want it to serve as a usage example for Pico Build Action and MicroROS
Build Action, and also because `pico-codeql.yml` doesn't produce binaries for the Pico (well, it does internally, but they're not 
uploaded as artifacts, so they can't be accessed).
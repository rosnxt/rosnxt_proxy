#!/bin/bash
mkdir -p classes
javac -d classes -cp $NXJ_HOME/lib/pc/pccomm.jar $(find src/main/java/com/github/rosnxt/passtrough -name \*.java)

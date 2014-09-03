#!/bin/bash
java -cp "$(find /opt/leJOS/0.9.1beta-3/lib/pc -name \*.jar -print0 | tr "\0" ":")classes" com.github.rosnxt.passtrough.Passtrough "$@"

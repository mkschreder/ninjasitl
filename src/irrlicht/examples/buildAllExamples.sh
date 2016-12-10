#! /bin/bash
set -e
[ -z $1 ] || TARGET=$1
[ -z $TARGET ] && TARGET=all
for i in [012]* Demo; do
  echo "Building $i";
  pushd $i && LDFLAGS="-L../../build/irrlicht/source/Irrlicht/" make clean $TARGET;
  popd;
done

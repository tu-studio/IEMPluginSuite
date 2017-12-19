#!/bin/sh
cd ${0%/*}
mkdir -p _compiledPlugins/macOS/IEM
for d in */*.jucer; do
  d=${d%/*}
  echo "Compiling $d for macOS..."
    if [ -d "${d}/Builds/MacOSX" ]; then
        cd "${d}/Builds/MacOSX"
        xcodebuild -target "${d} - All" -configuration "Release" build
        echo "done..."
        cd "../../../"
        cp -R -H "${d}/Builds/MacOSX/build/Release/${d}.vst" "_compiledPlugins/macOS/IEM/"
    else
        echo "no xcode project found, moving on..."
    fi
done
echo "all done!"

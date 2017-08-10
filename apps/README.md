# Applications

This directory contains NuttX applications. The full directory must be symlinked in the base NuttX application directory using the name "hn70ap":
Assuming you are in the base nuttx build folder (the one in which you type "make":
```
cd ../apps
ln -s ../nuttx/configs/hn70ap/apps hn70ap
cd ../nuttx
make apps_distclean
tools/configure hn70ap/build/status
make

```

Note that this symlinked directory is NOT required for the basic nsh build described in the top level. If the nsh build does not work, that is a bug, and you should report it.


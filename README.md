# robotworld
RobotWorld TCP/IP application



#TODO
> Niet een n tijd wachten, maar met messages elkaar aangeven dat ze moeten moven/stoppen
> Een config kunnen opslaan
> Achteruit wanneer hij klem loopt.
> Rekening houden met buitenkant map
> Zijkant kunnen kijken.
> Zie image voor verschillende scenario's
> Zonder muren moet hij ook werken.
> Een knop om beide te starten.

#Compilation in eclipse
- Download the zip
-  Use an "Empty Makefile" project as project template for Eclipse and change the build directory 
to the platform compilation directory in the project settings	 
- Go to Project->properties->C/C++ Build
- Untick "Use default build command"
- Enter in the build command box the path to the make.exe of msys. In my case: "C:\mingw-w64\msys64\usr\bin\make"
- Untick "Generate Makefiles automatically"
- Enter in the build location box the path to where the configure will create the make file. In my case: "C:\Users\nicov\Documents\HAN\OSM\Robotwereld\robotworld\mingw"
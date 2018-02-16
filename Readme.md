Load the project file Basestation.ioc in STM32CubeMX and select "Project > Generate Code".
This creates files which have been omitted from the Git Repository. The generated files can differ from system to system.
When the generated project is opened in ST Workbench (Eclipse), there should be a Src and Inc folder containing
main.c and main.h respectively (as well as many other source files). If that is not the case, something went wrong during
the code generation. Open the project once again in STM32CubeMX and select "Project > Settings...".
Check the paths of the project and the toolchain. You can't change them there. Open Basestation.ioc in a text editor and
remove any file paths in the configuration lines. Afterwards, open the project file again (without checking its Settings) and
run the code generation right away.

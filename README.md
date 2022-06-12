# _Test Bench for PTL Nodes Mini Version_


This is a test bench for ESP32 Device using IDF, it focuses mainly on IR sensors.



## How to use
1- Download ESP-IDF framework
2- Connect your ESP32, and choose the Serial port in "idf.portWin" in .vscode or use the VSCode Extension
3- Build, Flash and Monitor the target board.

## Project folder contents

The project contains one source file in C language [main.c](main/main.c). The file is located in folder [main](main).

ESP-IDF projects are built using CMake. The project build configuration is contained in `CMakeLists.txt`
files that provide set of directives and instructions describing the project's source files and targets
(executable, library, or both). 

Below is short explanation of remaining files in the project folder.

```
├── CMakeLists.txt
├── Components
│   ├── led_strip
├── main
│   ├── CMakeLists.txt
│   └── main.c
└── README.md                  This is the file you are currently reading
```


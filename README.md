# About project

This is the buildable files based on esp-idf framework. A versatile esp32 board with on board LED, two buttons, 0.96 inch display, BH1750, MPU6050, and MPU9250.

## Folder contents

The project **main** contains one source file in C language [main.c](main/main.c). The file is located in folder [main](main).

The project build configuration is contained in `CMakeLists.txt` files that provide set of directives and instructions describing the project's source files and targets
(executable, library, or both). 

Below is short explanation of remaining files in the project folder.

```
├── CMakeLists.txt
├── components
│   ├── Keys_on_board         // driver for on board keys
│   └── LED_on_board          // driver for on board LED
├── main
│   ├── CMakeLists.txt
│   └── main.c
└── README.md                  This is the file you are currently reading
```

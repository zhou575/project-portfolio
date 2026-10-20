# Examples of TinyML applications using TFLM
Tensorflow Lite for Microcontrollers (TFLM) is a framework that is a subset of Tensorflow which is designed to execute machine learning models on resource constrained devices i.e. microcontrollers.

The following repository will provide anyone the ability of executing TinyML applications using TFLM on SiFli SF32LB52X family line of ARM based microcontrollers.

For every example, there will be an instruction set on how to execute the example with the given device.

Ported examples
* benchmarks      
* dtln            
* hello_world     
* memory_footprint
* micro_speech    
* network_tester  
* person_detection
* kws (Keyword Spotting)

To compile, please setup SiFli SDK first, 
refer to https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/install/script/index.html

After setup the SDK environment, please enter example project folder, eg. 
For hello world example, using sf32lb52-lcd_n16r8 board,
enter project folder

* cd examples/hellow_world/project

then use command to compile.

* scons --board=sf32lb52-lcd_n16r8

in build folder, issue following command to download to board.

* .\build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat
  

# BRVBI-API
This project provides the necessary code to access the **PrecisionWave BR-VBI** SDR-Interface for **IQ Data Streaming** on **Linux-Machines**. More information about the hardware can be found here: [PrecisionWave BR-VBI](https://www.precisionwave.com/products/signal-analyzers)\
*Please note that IQ-Streaming requires an additional option on the receiver.*

## API

All relevant files can be found in the subfolder [/api](https://github.com/PrecisionWave/BRVBI-API/tree/main/api).\
Public methods are defined in [BRVBIControl.h](https://github.com/PrecisionWave/BRVBI-API/blob/main/api/BRVBIControl.h):

 - checkParams: checks Frequency, SamplingRate and BlockSize for valid values.
 - init: initializes BR-VBI communication and measurement
 - startStream/stopStream: control streaming
 - getStreamData: returns interleaved 16bit IQ-Samples

## Example Project
[Example.cpp](https://github.com/PrecisionWave/BRVBI-API/blob/main/Example.cpp) provides an example for the API usage. This command line application records IQ-data from the BRVBI into a file (16bit IQ interleaved).\
`BRVBI IQ Stream Demo`\
`Usage:`\
`BRVBI2File [OPTION...]`\
`  -i, --ip-address arg        IPv4 Address of the receiver`\
`  -c, --center-frequency arg  Center Frequency [MHz]`\
`  -s, --sampling-clock arg    Sampling Clock [Hz]`\
`  -a, --acquisition-size arg  Acquisition Size [#Samples]`\
`  -f, --iq-file arg           Target IQ data file`\
`  -b, --block-size arg        Block size for file writing [#Samples] (default: 10000)`\
`  -h, --help                  Print usage`\

## Build Instructions
The following instructions allow to build the sample application and provides help to write your own *cmake* application, using the BRVBI-API.
### Prerequisites
The software has been developped and tested on Ubuntu 22.04 LTS, it may run on all compatible platforms. The packages *build-essentials* and *cmake* are required to build the project:\
`sudo apt install build-essentials cmake`

### Build
The following instructions can be used to build a Release-Build:\
`cd this/repo/location`\
`mkdir build`\
`cd build`\
`cmake -DCMAKE_BUILD_TYPE=Release ..`\
`cmake --build .`\

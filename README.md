# CellMate : Visual Appliance Identification and Control in Smart Buildings
[![Build Status](https://travis-ci.com/kaifeichen/CellMate.svg?token=XjizLR77Z2rgJhyHZZ73&branch=master)](https://travis-ci.com/kaifeichen/CellMate)

## What is CellMate? 
CellMate is a system that allows you to interact with smart building appliances by taking pictures of them.


## How to Run CellMate server?

### Server Environment
The easiest way to run CellMate server is using the prebuilt [docker image](https://hub.docker.com/r/kaifeichen/cellmate/). To get the docker image, run
```bash
docker pull kaifeichen/cellmate
```

If you want to build your own environment, follow the [install script](script/install.sh).

### Compile
Clone the CellMate server repository
```bash
git clone https://github.com/SoftwareDefinedBuildings/CellMate
```
Use cmake and make to compile in the [build/](build) folder
```bash
cd build/
cmake ..
make -j $(nproc)
```


### Run

#### standalone
To run the cellmat server
```bash
cellmate run [db_file...]
```

Here is an example that runs it with all data in *~/data/buildsys16/*:
```bash
cellmate run `find ~/data/buildsys16/ -iname *.db`
```


## CellMate Server API

CellMate server has a HTTP fron end and a [BOSSWAVE](https://github.com/immesys/bw2) front end.

### HTTP
You can identify an appliance with a HTTP POST of a HTTP message of type `multipart/form-data` type.

#### Resource URL
`http://$SERVER_IP:SERVER_PORT`

#### Parameters
| Name | Required | Description |
|:----:|:--------:|:-----------:|
| file | required | content of image in JPEG format, a filename is optional |
| fx   | required | fx of the intrincis matrix of the image |
| fy   | required | fy of the intrincis matrix of the image |
| cy   | required | cx of the intrincis matrix of the image |
| cy   | required | cy of the intrincis matrix of the image |

#### Return Data
Return data is a string of the ID of the identified appliance.

### BOSSWAVE
You can also identify an appliance with a [MessagePack](http://msgpack.org/) payload using BOSSWAVE

#### BOSSWAVE Topic
`scratch.ns/cellmate`

#### MessagePack Parameters
| Name | Required | BOSSWAVE Type | Description |
|:----:|:--------:|:-------------:|:-----------:|
| file | required | 64.0.0.0 | content of image in JPEG format, a filename is optional |
| identity | required | 64.0.0.0 | an UUID of the image, which is used in the response topic |
| fx   | required | 64.0.0.0 | fx of the intrincis matrix of the image |
| fy   | required | 64.0.0.0 | fy of the intrincis matrix of the image |
| cy   | required | 64.0.0.0 | cx of the intrincis matrix of the image |
| cy   | required | 64.0.0.0 | cy of the intrincis matrix of the image |

#### Return Data
Return data is a string of the ID of the identified appliance. It is published to BOSSWAVE topic `scratch.ns/cellmate/$IDENTITY`, where `$IDENTITY` is the `identity` sent in the request.


## Do you have CellMate client?
Yes! There is an [Android client](https://github.com/SoftwareDefinedBuildings/CellMate_Android). 
There are also python clients for both BOSSWAVE and HTTP in the [test/](test) folder.


## Citation
You must cite the following paper if you use this code in any ways:

```
@inproceedings{chen2015intuitive,
  title={Intuitive Appliance Identification using Image Matching in Smart Buildings},
  author={Chen, Kaifei and Kolb, John and F{\"u}rst, Jonathan and Hong, Dezhi and Katz, Randy H},
  booktitle={Proceedings of the 2nd ACM International Conference on Embedded Systems for Energy-Efficient Built Environments},
  pages={103--104},
  year={2015},
  organization={ACM}
}
```

## License

```
Copyright (c) 2016-2017, Regents of the University of California
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions 
are met:

 - Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
 - Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the
   distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL 
THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED 
OF THE POSSIBILITY OF SUCH DAMAGE.
```

## Questions? 

Please Email Kaifei Chen <kaifei@berkeley.edu>

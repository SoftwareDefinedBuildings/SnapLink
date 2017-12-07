# SnapLink : Visual Appliance Identification and Control in Smart Buildings
[![Build Status](https://travis-ci.com/kaifeichen/SnapLink.svg?token=XjizLR77Z2rgJhyHZZ73&branch=master)](https://travis-ci.com/kaifeichen/SnapLink)

## What is SnapLink? 
SnapLink is a system that allows you to interact with smart building appliances by taking pictures of them.


## How to Run SnapLink server?

### Server Environment
The easiest way to run SnapLink server is using the prebuilt [docker image](https://hub.docker.com/r/kaifeichen/snaplink/). To get the docker image, run
```bash
docker pull kaifeichen/snaplink
```

If you want to build your own environment, follow the [install script](script/install.sh).

### Compile
Clone the SnapLink server repository
```bash
git clone https://github.com/SoftwareDefinedBuildings/SnapLink
```
Use cmake and make to compile in the [build/](build) folder
```bash
cd build/
cmake ..
make -j $(nproc)
```


### Run

#### standalone
To run a SnapLink server
```bash
snaplink run [db_file...]
```

Here is an example that runs it with all data in *~/data/buildsys16/*:
```bash
snaplink run `find ~/data/buildsys16/ -iname *.db`
```


## SnapLink Server API

SnapLink server has a GRPC front end.

## Do you have SnapLink client?
Yes! There is an [Android client](https://github.com/SoftwareDefinedBuildings/SnapLink_Android). 
There are also python clients for both BOSSWAVE and HTTP in the [test/](test) folder.


## Citation
You must cite the following paper if you use this code in any ways:

```
@article{chen2017snaplink,
  title={{SnapLink: Fast and Accurate Vision-Based Appliance Control in Large Commercial Buildings}},
  author={Chen, Kaifei and F{\"u}rst, Jonathan and Kolb, John and Kim, Hyung-Sin and Jin, Xin and Culler, David E and Katz, Randy H},
  journal={Proceedings of the ACM on Interactive, Mobile, Wearable and Ubiquitous Technologies},
  volume={1},
  number={4},
  pages={129:1--129:27},
  year={2017},
  publisher={ACM}
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

Please Email Kaifei Chen <kaifei@berkeley.edu> or Tong Li <sasrwas@berkeley.edu>

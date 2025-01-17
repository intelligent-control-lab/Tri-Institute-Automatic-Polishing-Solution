
# Tri-Institute Project

This repository contains the implementation of the Tri-Institute project, where an industrial robot arm is applied for automatic surface finishing. The project demonstrates an innovative approach to automating surface finishing tasks in industrial settings.

## Features
- Automates weld finishing using advanced robotics.
- Implements efficient algorithms for precise and consistent finishing.
- Designed for integration with industrial robot arms.

## Getting Started

### Prerequisites

Ensure your system meets the following software and hardware requirements:

#### Software Requirements
- **Operating System**: Ubuntu 20.04 or later
- **MATLAB**: Version 2023 or later
- **Python**: Version 3.6 or later

#### Hardware Requirements
- **Processor**: Intel Xeon(R) CPU E3-1505L v5 @ 2.00GHz (8 cores) or better
- **Graphics**: Mesa Intel HD Graphics P530 (SKL GT2) or better
- **Robot**: Yaskawa GP50
- **Sensors**: ATI FTS Omega85 and scanCONTROL 30x0 series laser profiler 


### Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/<your-repo>/Tri-Institute-Automatic-Polishing-Solution.git
   cd Tri-Institute-Automatic-Polishing-Solution
   ```
2. Create a virtual environment with Python version > 3.6:
    ```bash
    python3 -m venv venv
    source venv/bin/activate
    ```
3. Install the required Python packages:
    ```bash
    pip install open3d numpy matplotlib tqdm scipy qpsolvers[osqp] numpy-quaternion ipdb
    ``` 
4. Setup the hardware and environment for scanCONTROL Laser Profiler and ATI Force Torque Sensor following the official guidelines. 


### Usage
1. Run final Tri-Insitute Demo
    ``bash
    python scan/workpiece_pipline.py
    ``` 

## License
This project is licensed under the **BSD 3-Clause Clear License**. See the full license text below.

```
Copyright (c) 2024 Carnegie Mellon University
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted (subject to the limitations in the disclaimer below) provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions, and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions, and the following disclaimer in the documentation and/or other materials provided with the distribution.
3. Neither the name of Carnegie Mellon University nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY CARNEGIE MELLON UNIVERSITY "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL CARNEGIE MELLON UNIVERSITY BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

## Acknowledgments
Special thanks to Carnegie Mellon University for their support and collaboration on this project.
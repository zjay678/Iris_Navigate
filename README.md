

# Iris_Navigate

## MacOS Configuration

**Download Ardupilot**

```bash
$ git clone git://github.com/ArduPilot/ardupilot.git
$ cd ardupilot
$ git submodule update --init --recursive
```

**Install Homebrew for MacOS**

```bash
$ /usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
```

**Install the following packages using brew**

```bash
$ brew tap ardupilot/homebrew-px4
$ brew update
$ brew install genromfs
$ brew install gcc-arm-none-eabi
$ brew install gawk
$ brew install cmake ccache 
```

**Install some required packages for building Ardupilot**

```bash
$ sudo -H pip install matplotlib pyserial lxml scipy pexpect pymavlink
```

Add the following lines to the end of your `.bash_rc` in your home directory

```bash
export PATH=$PATH:$HOME/ardupilot/Tools/autotest
export PATH=/usr/lib/ccache:$PATH
```

Then reload your `PATH` by using the `source`  command in a terminal

```bash
$ source .bash_rc
```

**Install MAVProxy**

```bash
$ sudo -H pip install wxPython gnureadline billiard numpy pyparsing MAVProxy
```

**Start SITL simulator**

```bash
$ cd ardupilot/ArduCopter
$ sim_vehicle.py -j4 --console --map
```

**Install APM planner for MacOS**

connect with UDP Link 14551

![](https://github.com/Zoneshi/Iris_Navigate/blob/master/apm.png)

**Install `dronekit` and `dronekit-sitl`** 

```bash
$ sudo -H pip install dronekit
$ sudo -H pip install dronekit-sitl
$ dronekit-sitl copter
$ python Iris_Goto.py
```

**Run the navigation code**

```bash
$ python Iris_Goto.py
```


# iflexd - iFlex Server

### Build

Change directory to iflexd source codes and run following commands:
```
cmake .
make
```

This will create `iflexd` executable in bin directory.

`iflexd` has only one argument the configuration file.

### Configuration File Format

The configuration file is regular JSON file with the following schema:

```
{
    "address": "127.0.0.1",                                  - Address to listen
    "port": 9571,                                            - Port to listen
    "api_key": "v5*A2m4d",                                   - Client API key for accessing daemon
    "log_file": "iflex.log",                                 - Path of log file
    "log_level": "debug",                                    - Log level (TRACE|DEBUG|INFO|WARNING|ERROR|CRITICAL|OFF)
    "component_size": 4,                                     - Component length in bytes (supported values are 1 and 4)
    "dataset_path": {                                        - Local dataset path
      "nearist": "/srv/ftp/nearist/",                        - Local shared dataset path
      "user": "/srv/ftp/nickr/"                              - Local user-only dataset path
    },
    "temperature_log": {                                     - Temperature Logging configuration
      "enable": false,                                       - Enable temperature log
      "rate": 10,                                            - Logging rate in seconds
      "file": "/var/log/iflex.tmp"                           - Path to temperature log
    },
    "nodes": [                                               - Connected Nodes (iFlex devices, or Nearist TCP nodes)
        {
            "type" : 0,                                      - Type of node (0 - device, 1 - TCP)
            "revision" : 6,                                  - Device revision (1 - REP_1_0 | 2 - REP_2_0 | 3 - REP_2_1 | 4 - REP_2_2_1 | 5 - REP_2_2_2 | 6 - IFLEX)
            "write": "/dev/xillybus_00_cortical_write",      - PCI device file path (Write direction)
            "read": "/dev/xillybus_00_cortical_read"         - PCI device file path (Read direction)
        },
        ....
        {
            "type" : 1,                                      - Type of node (0 - device, 1 - TCP)
            "address": "127.0.0.1",                          - IP Address of node
            "port": 9571                                     - TCP port of node
        }
    ]
}
```

### Run

There is example configuration files for iFlex and Nearist.

```
    bin/iflexd conf/iflex.json
    bin/iflexd conf/nearist.json
```

## Architecture

Key classes:

* `Session` - The main point of communication between the master server and the client. It receives the Requests over the Internet, issues the appropriate commands to the nodes, and sends back the Responses. 
* `iFlexNode` - Represents (and communicates with) a single VSX card. There is one instance of this class per VSX card in the system.
* `TCPNode` - Represents a slave appliance for multi-appliance systems.


# M5StickC PLUS2 Boilerplate



## Getting started

### Prerequisites

The project uses PlatformIO to collect and manage all dependencies as well as the build pipeline.
Install instructions can be found [here](https://docs.platformio.org/en/latest/core/installation/index.html)

### Installation

Initialize the project:

``` shell
pio init
```

## Development

Update dependencies:

``` shell
pio pkg update --environment m5stick-c
```

Compile, run and monitor the applications serial output executing

``` shell
pio run -t upload -t monitor -e m5stick-c
```
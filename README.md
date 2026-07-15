# FivePortDetector

PlatformIO based firmware for testing and calibrating a Five-Port Detector using IQ bit sequences. The five-port ring is used as a phase/DOA (direction-of-arrival) receiver at 2.45 GHz; the firmware handles in-system calibration, demodulation and angle computation.

![Manufactured PCB](docs/img/Manufactured%20PCB.jpeg)

## Branches

- `main` — stable baseline
- `iq-calibration` — Phase 1: single five-port receiver, in-system IQ calibration and continuous phase readout
- `doa-detector` — Phase 2: two five-port receivers, interferometric DOA computation from the phase difference between both rings

## Hardware targets

- `genericSTM32F411RE` (Nucleo-64, ST-Link upload)
- `lptm4c1294ncpdt` (EK-TM4C129EXL, XDS110 on-board debugger, no external programmer needed)

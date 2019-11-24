# Bluetrackr

## Progress

### Transmitter schematic:

![schematic](https://raw.githubusercontent.com/Haellsigh/Bluetrackr/master/doc/images/transmitter_schematic.png)

### Transmitter PCB:
![schematic](https://raw.githubusercontent.com/Haellsigh/Bluetrackr/master/doc/images/transmitter_pcb.png)

### Transmitter PCB 3D view:

![schematic](https://raw.githubusercontent.com/Haellsigh/Bluetrackr/master/doc/images/transmitter_3d.png)

# Changelog

## 0.1.1

- Initial prototype, in testing as of 24/11/2019.

## 0.1.2

### Added
- [24/11/2019] Replaced the unused programming pin with NRST to support st-link reset.
### Changed
- Use net labels instead of global labels.
### Deprecated
### Removed
### Fixed
- Added circuitry to bypass the battery while USB is connected. This is needed because the TP4056 can't charge properly while under load.
### Security
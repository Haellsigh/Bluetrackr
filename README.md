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

- Initial transmitter prototype, in testing as of 24/11/2019.
- The transmitter failed to work. Using the same code on a STM32F072Discovery board seems to work however.
- The next iteration will remove the 

## 0.1.2

### Added
- [a86c5df] Replaced the unused programming pin with NRST to support stlink reset.
### Changed
- [1984e59] Use net labels instead of global labels.
### Deprecated
### Removed
- 
### Fixed
- Added circuitry to bypass the battery and charger while USB is connected. This is needed because it's not possible to charge properly while the battery is under load. This is called "Load Sharing", see [AN1149](http://ww1.microchip.com/downloads/en/AppNotes/01149c.pdf) from Microchip.
### Security
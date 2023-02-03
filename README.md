# The Modular Mouse Position Surveillance System (ModMoPSS)
Fancy header that gives introduction to the ModMoPSS System
The Modular Mouse Position System is an open source behavioral experimentation platform for mice.

## Modular MoPSS Main Controller Unit

This is the code for the main controller unit (Hive-module) which controls the high level behavior of all Modular MoPSS modules.

The Hive-Gate module is designed to operate a Gating-System for mice. The Gate should only allow one mouse at a time to pass through the gate. In our Example a Homecage is connected to a "Testcage" via the gate. All mice start in the Homecage, at any time a single mouse can transition through the gate into the Testcage. Since the Testcage is now occupied, no additional mouse can transition through the gate until the mouse occupying the Testcage has returned (via the gate) to the Homecage.
This setup allows mice to voluntarily enter a Testcage in which e.g. preference tests can be presented. The mouse can perform these tests without interference from other mice.
In this specific setup we are using four IR-barriers, two fans, one RFID-module with two RFID antennas and one Stepper-Module that controls two Doors.

Due to the flexibility of the modular MoPSS multiple RFID and Stepper modules can be employed to any functions beyond moving doors. It also easily possible to develop additional modules as all PCB files and the complete Code is open source. The limit in software is >200 Modules however current capabilities of the power supply unit and current carrying capabilities of the PCBs must be considered and when necessary adapted. 

# Developer Guide

## Software
### Using the ModMoPSS as a gating system
### Modifying Experiment parameters
### Modifying the experiment

## Hardware
### RFID Antenna placement and setup
### Stepper module setup
### IR barrier setup


# Experimenter Guide

## Assembly
PCB fab, Soldering, 3D printing, Acrylic tubes etc.
Bilder...

## Installation
module testing (solo?)


##### This is a work in progress, documentation and functionality is still in the procress of being fine-tuned and experimentally validated

For further information don't hesitate to contact us.

See also: \
https://github.com/RefinementReferenceCenter/Modular-MoPSS-Hardware \
https://github.com/RefinementReferenceCenter/Modular-MoPSS-RFID \
https://github.com/RefinementReferenceCenter/Modular-MoPSS-Door 

##### Relevant papers
Rating enrichment items by female group-housed laboratory mice in multiple binary choice tests using an RFID-based tracking system. http://doi.org/10.1371/journal.pone.0278709 \
O mouse, where art thou? The Mouse Position Surveillance System (MoPSS)—an RFID-based tracking system https://doi.org/10.3758/s13428-021-01593-7

The contents of this repository are distributed under the GNU General Public License v3.0

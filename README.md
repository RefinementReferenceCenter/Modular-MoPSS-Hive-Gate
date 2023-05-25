# The Modular Mouse Position Surveillance System (ModMoPSS)
The Modular Mouse Position Surveillance System is an open source behavioral experimentation platform (for mice). The System provides Hardware and software for surveillance tasks via infrared light barriers and RFID tracking (with body temperature recording) and control tasks with the implementation of doors.

Due to the flexibility of the modMoPSS, multiple RFID- and door-modules can be employed to any functions beyond moving doors and tracking tasks. It is also easily possible to develop additional modules as all PCB files and the complete code is open source.

## Modular MoPSS Main Controller Unit

This is the repository for the main controller unit (Hive-module) which controls the high level behavior of all Modular MoPSS modules.

## ModMoPSS as a gate (1 RFID-module, 1 door-module)
The Hive-Gate setup is designed to operate a gating-system for mice. The gate should only allow one mouse at a time to pass through the gate. In our example a homecage is connected to a "testcage" via the gate. All mice start in the homecage, at any time a single mouse can transition through the gate into the testcage. Since the testcage is now occupied, no additional mouse can transition through the gate until the mouse occupying the testcage has returned (via the gate) to the homecage.
This setup allows mice to voluntarily enter a testcage in which e.g. preference tests can be presented. The mouse can perform these tests without interference from other mice.
In this specific setup we are using four IR-barriers, two fans, one RFID-module with two RFID antennas and one Door-module that controls two doors.

## ModMoPSS a tracker (1 RFID-module)
The ModMoPSS setup as a simple tracker uses a single RFID module with two antennas. The two antennas can connect two cages and are spaced approximately X cm apart. To avoid interference the antennas are only switched on alternately with a 100 ms interval. Complementary to this setup, the "ModMoPSS Evaluationscript" allows the reconstruction of transitions of mice between the two cages. Allowing the evaluation of stay duration, and (transition)activity of each mouse.


# Developer Guide

## Software
Any changes to the code of the ModMoPSS system should always take into consideration the importance of timing. The system must always be able to respond in the fastest way possible to all inputs.
### Using the ModMoPSS as a gating system
### Modifying experiment parameters
### Modifying the experiment

## Hardware
The limit in software is >200 individual Modules (I²C address limitation) however current capabilities of the power supply unit and current carrying capabilities of the PCBs must be considered and when necessary adapted. 

### RFID antenna placement and setup
RFID antennas need to be placed at an appropriate distance from metal objects to avoid detuning the antenna. During boot-up the resonant frequency is checked and the user will be alerted via the display of the measured resonant frequency and additional input is required if the antenna is too severely out of tune. The standard configuration of tuning capacitors in the design files is optimized towards an Antenna in free air. If the antenna is mounted near a metal object, e.g. a water bottle or the metal lid, the variable tuning capacitor on the PCB can be used to correct the resonant frequency. However it might be the case that additional capacitors have to be soldered to the board to achieve proper tuning. /
Care has to be taken to avoid interference from other antennas. Antennas that are close together and on the same axis (e.g. the gate) will produce interference and prevent successful RFID tag reads. To prevent this, the antennas have to be turned on and off alternately. This is the standard setting and happens at 100ms intervals.

### Door module setup
The door module uses two infrared light barriers as positional feedback for the door. During startup these IR barriers are used to calibrate door movement distance (stepper step count). Standard configuration uses the upper IR barrier to determine that the door is fully open and the calibrated step count during down movent to assure closing. This is necessary as mice like to bite into the door and by pulling on it causing the stepper motor to skip steps which in turn means loss of absolute position which is then reestablished via the IR barriers. 

### IR barrier setup
Each IR-barrier actually contains two individual IR-barriers whose direction is indicatied by the chevrons on the PCB. Two IR-barriers are used to increase confidence in mouse detections as opposed to a singel IR-barrier which can be triggered by mouse-tail passing through the barrier. Signals from both IR-barriers are available at the hive module.
IR-barriers with a solder bridge enable the use of the two perpendicular IR LEDs which are only needed for the door module.

# Experimenter Guide

## Assembly
- PCBs can be ordered online with the appropriate gerber files contained in the ModMoPSS hardware repository. BOM and Pick and place files are included as well.
- Necessary 3D-printing files, additional hardware (screws, microncotroller, cables etc.) are also included in a separate file in the hardware repository.

## Installation
The microcontrollers on each module have to be flashed with the correct firmware which can be found in the appropriate repositories. Care should be taken to assign appropriate I²C addresses (no duplicates) to each microcontroller and adjust the addresses appropriately for the hive module.


### This is a work in progress, documentation and functionality are still in the process of being fine-tuned and experimentally validated
#### A publication for the ModMoPSS with experimental validation is in the works.

For further information don't hesitate to contact us.

See also: \
https://github.com/RefinementReferenceCenter/Modular-MoPSS-Hardware (PCB and fabrication, 3D-print models)\
https://github.com/RefinementReferenceCenter/Modular-MoPSS-RFID (RFID-module code)\
https://github.com/RefinementReferenceCenter/Modular-MoPSS-Door (Door-module code)

##### Relevant papers
Rating enrichment items by female group-housed laboratory mice in multiple binary choice tests using an RFID-based tracking system. http://doi.org/10.1371/journal.pone.0278709 \
O mouse, where art thou? The Mouse Position Surveillance System (MoPSS)—an RFID-based tracking system https://doi.org/10.3758/s13428-021-01593-7

The contents of this repository are distributed under the GNU General Public License v3.0

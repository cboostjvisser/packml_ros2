# PackML State, based on the PackML standard, summarized here:
# http://www.plcopen.org/pages/promotion/publications/downloads/mapping_omac_statediagram.pdf

# Enumeration values are based upon tag guidelines, here:
# http://sesam-world.com/_pdf/make2pack/mode/2010-11-29/Materiale/PackML_Tag_Naming_Guidelines_V2.0.pdf
# Due to changes in the standard some states weren't enumerated.  I've made my best guess
# below

# Jacob Visser: Changed to match packml definition found in;
# Chapter 9 'PACKML INTERFACE STATE DEFINITION' from 'PackML Unit/Machine Implementation Guide Part 1' Ver. 1.00
# Downloaded on 25/07/2024 from: https://www.omac.org/packaging-workgroup-solutions

# Values defined in tag naming guidelines
int8 UNDEFINED = 0
int8 CLEARING = 1
int8 STOPPED = 2
int8 STARTING = 3
int8 IDLE = 4
int8 SUSPENDED = 5
int8 EXECUTE = 6
int8 STOPPING = 7
int8 ABORTING = 8
int8 ABORTED = 9
int8 HOLDING = 10
int8 HELD = 11
int8 UNHOLDING = 12
int8 SUSPENDING = 13
int8 UNSUSPENDING = 14
int8 RESETTING = 15
int8 COMPLETING = 16
int8 COMPLETE = 17

# msg values
int8 val


# README.md - Examples

# Example Programs
- test_ir1838.py


## Example Test Infrared Sensor and Remote  (test_ir1838.py)

By default the program performs test() which tests the NEC 9 protocol.
Simply modify the bottom line program in a text editor.  See the list of protocols below.

For example:

Code .....
..........
#test() 
test(2)

Using Thonny is suggested as you can modify the code and run it from within Thonny.
Protocols available with the IR sensor board:
test() for NEC 8 bit protocol,
test(1) for NEC 16 bit,
test(2) for Sony SIRC 12 bit,
test(3) for Sony SIRC 15 bit,
test(4) for Sony SIRC 20 bit,
test(5) for Philips RC-5 protocol,
test(6) for RC6 mode 0.
test(7) for Microsoft Vista MCE.
test(8) for Samsung.


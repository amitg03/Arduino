# Arduino

This Code written on atmega2560 converts serial string from a weighing scale to modbus.
system acts as a modbus slave.

can cater to -ve and +ve weight reading from sclae at 2400 baud rate.
Modbus details:
De pin-Pin 2
Serial on serial 2.
Slave Details: 
SLAVE_ADDRESS 0x04

READ_HOLDING_REGISTERS 0x03
SEND_BYTE_NUMBERS 54
READ_ADDRESS1 0x00
READ_ADDRESS2 0x03
DATA_LENGTH 0x1B

RX_MAX_SIZE 10
TX_MAX_SIZE 100

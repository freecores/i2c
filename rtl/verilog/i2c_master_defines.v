// I2C registers wishbone addresses

// bitcontroller states
`define I2C_CMD_NOP   4'b0000
`define I2C_CMD_START 4'b0001
`define I2C_CMD_STOP  4'b0010
`define I2C_CMD_WRITE 4'b0100
`define I2C_CMD_READ  4'b1000


// asynchronous reset level
// I2C_RST_LVL == 1'b0 asynchronous active low reset
// I2C_RST_LVL == 1'b1 asynchronous active high reset
`define I2C_RST_LVL 1'b0
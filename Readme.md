# Updates to the PDF spec
The PDF spec does not cover the I2C target extensions in this repo.
The following sections give an overview of the changes.

## Command register
The target extension is the TACK bit.

| Bit | Access | Description                      |
|-----|--------|----------------------------------|
| [7] | W      | STA  - Transaction start         |
| [6] | W      | ST0  - Transaction stop          |
| [5] | W      | RD   - Transaction read          |
| [4] | W      | WR   - Transaction write         |
| [3] | W      | ACK  - Transaction acknowledge   |
| [2] | W      | RSVD                             |
| [1] | W      | TACK - Target acknowledge        |
| [0] | W      | IACK - Interrupt acknowledge     |

## Status register
The target extensions are the target mode, target
data available, and the target data request bits.

| Bit | Access | Description               |
|-----|--------|---------------------------|
| [7] | R      | RxACK                     |
| [6] | R      | Busy                      |
| [5] | R      | Arbitration Lost          |
| [4] | R      | Target Mode               |
| [3] | R      | Target Data Available     |
| [2] | R      | Target Data Request       |
| [1] | R      | Transfer in Progress      |
| [0] | R      | Interrupt Acknowledge     |

### Target Mode
AFAICT, this is NOOP.

### Target Data Available
Indicates that valid data was received by the target extension.

### Target Data Request
Indicates that valid data is required by the target extension
for transmit.

## Target operations
When Target Data Available is high, valid receive data is in the Receive
register.  Acknowledge by setting the Transmit register to 0x00 and then
set TACK in the Command register to 1.

When Target Data Request is is high, the target needs to send data.
Acknowledge by setting the Transmit register with valid data and then
set TACK in the Command register to 1.

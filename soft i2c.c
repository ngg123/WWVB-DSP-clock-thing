Soft I2C:

Phys layer
State machine with 3 states:
1) Bus idle
2) Active Listen
3) Active Send
4) Active Ignore
Requires 4 bytes:
RX buf, TX buf, RX count, TX count
Handle at most 1.5 * bus_speed_Hz interrupts/sec.

Calls 4 software interrupts:
Start Condition, RX Done, TX Done, Stop Condition


GPIO interrupt handler:
Interrupt on SDA edge and SCL edge.  Edge directions set by bus state.  
1) SDA edge in bus idle state:
Call Start software interrupt and goto active listen.

2) SDA edge in active listen, active send, or active ignore
Sample SCL.  If low, do nothing, exit ISR.  If SCL high, call Stop software interrupt and goto bus idle.  

3) SCL edge in bus idle state:
Do nothing (shouldn't happen, as SCL masked in bus idle).

4) SCL edge in active ignore state:
Do nothing (shouldn't happen, as SCL masked in active ignore).

5) SCL edge in active listen
Sample SDA.
Place sampled SDA state to RX buf (MSB), and shift right.
Decrement RX counter.
If RX counter = 0, call RX Done software interrupt and goto active send.  
Else, stay in active listen

6) SCL edge in active send
Extract LSB from TX Buffer and shift right.  
Set SDA to high impedance (LSB = 1) or active low (LSB = 0)
Decrement TX counter
If TX counter = 0, call TX Done software interrupt and goto active listen.  
Else stay in active send


STATE GRAPH EDGES
Transition to Active Listen:
Set SDA interrupt edge to rising
Set SCL interrupt edge to rising
(higher level protocol expected to clear RX buf and RX count)

Transition to Active Send:
Set SDA interrupt edge to rising
Set SCL interrupt edge to falling
(higher level protocol expected to set TX buf and TX count)

Transition to Bus Idle:
Set SDA interrupt edge to falling
Mask SCL interrupt
Send Proto layer goto Idle

Transition to Active Ignore:
Set SDA interrupt edge to rising
Mask SCL interrupt


In bus idle state, software waits for falling edge on SDA, which triggers Start condition.  If SCL line is high, recieve buffer and buffer count are cleared, SDA interrupt is set to rising to listen for bus stop condition.  Start Condition software interrupt is sent to higher protocol, and state transitions to Active Listen to get I2C address.  

In Active Listen, SCL rising edge interrupt is enabled (data valid while SCL high).  Rising edge interrupt handler samples SCLline and shifts sampled value into receive buffer.  Buffer count is decremented. If buffer count = 0, RX Done software interrupt is sent, and state transitions to Active Send (so that higher protocol can send ACK/NACK).  Otherwise, continue in current state.  SDA rising edge interrupt handler samples CLK to look for stop condition.

In Active Send, SCL falling edge interrupt is enabled (data can be changed while SCL low).  Falling edge interrupt handler extracts LSB from transmit buffer, shifts buffer right, sets SDA to high impedance (LSB = 1) or active low (LSB = 0), and decrements transmit count.  If transmit count = 0, TX Done software interrupt is generated, transition to Active Listen state (to listen for ACK/NACK).  Otherwise continue in current state.  SDA rising edge interrupt handler samples CLK to look for stop condition.

In Active Ignore, SCL is ignored.  SDA rising edge interrupt handler samples CLK to look for stop condition.


Proto layer
Recognizes I2C address(es), handles traffic flow

RX Done handler, Waiting for address:
Check if RX buf matches our RX address(es)
If match, set TX_buf = 0 (ACK), set TX_count = 1, goto ACKing address RX
Check if RX buf matches our TX address(es)
IF match, set TX_buf = 0 (ACK), set TX_count = 1, goto ACKing address TX
Else, set TX_buf = 1, set Phy to active ignore state, goto Idle

TX Done handler, ACKing address RX
Goto Waiting for Command

TX Done handler, ACKing address TX
Goto TXing

RX Done handler, Waiting for Command
call application command number
goto ACKing command

TX Done handler, ACKing command
Goto Waiting for RX

RX Done handler, Waiting for RX:
Call application RX byte (expecting ACK/NACK for return from call)
set TX_buf = return value, set TX_count = 1
If TX_buf == 0 goto ACKing RX
Else goto idle

TX Done handler, ACKing RX
Goto Waiting for RX

RX Done handler, Waiting for TX ACK
If RX_buf == 0 (ACK), goto TXing
If RX_buf == 1 (NACK), goto idle

TX Done handler, TXing
Call application TX byte
Goto Waiting for TX ACK

States:
1) Idle
2) Waiting for address RX
3a) ACKing address RX
3b) ACKing address TX
4) Waiting for Command
5) ACKing command
6) Waiting for RX
7) ACKing RX
8) TXing
9) Waiting for TX ACK
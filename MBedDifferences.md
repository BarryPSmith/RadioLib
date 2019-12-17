MBed differences

This file described differences between Radiolib and the Semtech provided SX126xLib mbed driver.
See https://os.mbed.com/teams/Semtech/code/SX126xLib/
This file is up-to-date on the 15th of December, 2019
This file was created for SX126xLib revision 6:1e2345700991
This file was created for Radiolib revision e61bd588b06a9e2

This considers LoRa use only.

Use of the SX126xLib is inferred from the SX126x Dev Kit demo application (https://os.mbed.com/teams/Semtech/code/SX126xDevKit/file/b96176a4ccb8/Demo/DemoApplication.cpp/)

As there is not a 1 to 1 mapping between functions in the libraries, this document compares the libraries on the basis of common user actions.

Initialisation
====================================
Minimum actions that must be performed at startup regardless of how the radio is used.
**API:**
Radiolib:
Create Module
Create SX126x instance
Call begin

SX126xLib:
Create SX126xHal instance
Call Init

**Parameters:**
Radiolib:
Module:
Set Pins

Begin:
Frequency
Bandwidth
Spreading Factor
SyncWord
Tx Power
Current Limit
Preamble Length
TCXO Voltage

SX126xLib:
Define pins and radiocallbacks

**Implementation:**
Radiolib:
 - Write select HIGH (module::init)
 - Set radio to Standby RC mode
 - Set regulator to DC_DC
 - Set RX and TX base addresses to zero
 - Set radio to LoRa modem
 - Set RX/TX fallback mode to Standby RC (Note: Datasheet indicates this is the default anyway)
 - Set CAD paramaters:
	  -  8 symbols, spreading factor + 13, 10, Return to Standby_RC, zero timeout
 - Clear IRQ status
 - Set DioIrqParams to no action
 - Perform calibration with 'calibrate all'
     - Wait for Busy to go low to indicate end of calibration
 - Set DIO3 TCXO Voltage if required based on input paramater
 - Set Spreading factor
 - Set Bandwidth
 - Set Coding Rate
 - Set Sync Word
 - Set Current limit
 - Set Preamble length
 - Disable Dio2 as RF switch
 - Set Frequency
 - Set Output Power
 - Fix Pa Clamping

SX126xLib:
- Constructor:
	- Write Select HIGH
	- Write Radio Reset HIGH
 - Init:
	 - Reset radio
	    - Write radio reset LOW
	    - Wait 50ms
	    - Write radio reset HIGH
	- Wakeup:
	    - Write Nss low
	    - Call GetStatus on radio
	    - Write NSS High
	    - Turn on antenna switch
	- Set Standby RC Mode
	- If Pin A3 is High [Seems to be a specific hardware indication of TCXO crystal]
	    - Set TCXO voltage on DIO3: 1.7V, 5ms wakeup
	    - Perform calibration with all.
	- Turn on antenna switch
	- Set Dio2 as RF Switch
	- Set LoRa packet type
	- Set sync word

**Differences:**
 - SX126xLib resets the radio
 - SX126xLib enables the RF switch by default, Radiolib turns it off
 - SX126xLib turns on the RF switch, Radiolib does not control this (This is through another uC pin)
 - Radiolib initialises far more parameters
 - SX126xLib sets the TCXO voltage before calibration, Radiolib calibrates then sets TCXO voltage
 - Radiolib fixes PA clamping

Major Functions
====================================

Wakup:
------------------------------------
**API:**
Radiolib:
Call SetStandby()

SX126xLib:
Call Wakeup();

**Implementation:**
Radiolib:
Set radio to StandbyRC

SX126xLib:
Set NSS low, query radio status

**Differences:**
Nothing obvious


Sleep:
------------------------------------
API:
Radiolib:
Call sleep()

SX126xLib:
Call SetSleep()

**Implementation:**
Radiolib:
 - Call sleep with a cold start
 - Delay 500 us for device to go to sleep

SX126xLib:
 - Set Antenna switch power low
 - Set Radio to sleep (warm or cold as user configured)

**Differences:**
 - SX126xLib handles the antenna switch power
 - SX126xLib allows the user to select sleep type
 - SX126xLib doesn't wait half a millisecond

**Possible Radiolib issues:**
 - The device is by default put into a cold sleep where it will loose all settings. Is this a desirable default?
 - Is the wait necessary? 
    - If the user is putting the device to sleep, they are likely concerned about power and might want to put the uC to ASAP too.
    - If they try to issue another command within the 500 us required for the device to sleep, that command will pull NSS low and wake the device, then wait for BUSY. The default 5ms timeout on SPITransfer should take care of this.

Blocking Transmit:
------------------------------------
**API:**
Radiolib:
Call Transmit()

SX126xLib:
Not Implemented

**Implementation:**
Radiolib:
 - Set radio to Standby RC
 - Set a timeout at 150% of expected time on air
 - Call asychronous transmit
 - Read DIO1 until it goes high or a uC determined timeout
 - Clear IrqStatus (even in the event of a timeout - why?)
 - Set radio to Standby RC (It should already be here)

**Possible radiolib issue:**
 - In the event of a uC timeout, the radio is not returned to StandBy RC (Perhaps sx126x.cpp:204 should do this instead of clear IRQ?)

Blocking Receive:
------------------------------------
API:
Radiolib:
Call Receive()

SX126xLib:
Not Implemented

**Implementation:**
Radiolib:
 - Set radio to Standby RC
 - Set a timeout to 100 symbol lengths
 - Call asynchronous receive
 - Wait for DIO1 to go high or uC timeout
 - Fix Implicit Timeout on radio
 - Clear IRQ status
 - Read data if DIO1 went high

**Possible radiolib issues:**
 - Radio is not instructed to return to StandbyRC in the event of a uC timeout
 - Receive timeout is not programmable
 - Receive timeout can occur even if a packet is currently being received. Does this limit receive to 100 symbols (or less if the tx doesn't start simulatenous with Rx)?
 - 
Asynchronous Transmit:
---------------------------------
**API:**
Radiolib:
Call startTransmit
Handle completed event on Dio1 Action

SX126xLib:
Call SetPacketParams for packet size
Call SetDioIrqParams for TX_DONE (and maybe TX_TIMEOUT)
Call SendPayload
Handle completed event in ???

**Implementation:**
Radiolib:
 - Fix inverted IQ
 - setPacketParams
 - SetIRQ to TX_DONE on DIO1 (also globally enable timeout IRQ, but it's not used anywhere)
 - Set TX and RX buffer base addresses to 0
 - Write buffer from address 0
 - Clear IRQ status
 - fix Sensitivity
 - set Tx
 - wait for busy (PA ramp)

SX126xLib:
 - (Set PacketParams called by user)
 - (Set DioIrqParams called by user)
 - Write buffer
 - SetTx

**Differences:**
 - Radiolib does far more work.
 - Separated functions mean SX126xLib can make fewer radio calls in a static environment (constant packet size, always Tx)
 - SX126xLib allows a user defined timeout (why?)
 - SX126xLib separates different causes for interrupts into different callbacks

**Possible SX126xLib issues:**
 - Never calls setBufferBaseAddress (is it always zero if not set?) (SX1262 datasheet v1.2, 14.2 point 6)

**Possible Radiolib issues:**
 - No TX timeout is set (SX1262 datasheet v1.2, 13.1.4 final paragraph)

Start Asynchronous Receive:
------------------------------------
**API:**
Radiolib:
Call startReceive

SX126xLib:
Call SetPacketParams with packet size
Call SetDioIrqParams with RX_DONE, maybe also: preamble detect, syncword detect, CRC error, rx_timeout
Call setRx

**Implementation:**
Radiolib:
 - Set DIO IRQ Params to SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT | SX126X_IRQ_CRC_ERR | SX126X_IRQ_HEADER_ERR
 - Set buffer base addresses to zero
 - Clear IRQ status
 - Set Rx Mode

SX126xLib:
 - (SetPacketParams called by user)
 - (Set DioIrqParams called by user)
 - Set Rx Mode

**Differences:**
As with Asynchronous Transmit, SX126xLib is lower level allowing more control by the user

**Possible Radiolib issues:**
 - SetBufferBaseAddress logic doesn't play well with continous receive.
 - User can only be notified on RxDone or CRC error, not preamble or syncword detected.

End Asynchronous Receive:
------------------------------------
**API:**
Radiolib:
Wait for event on Dio1 Action
Call readData
Call getPacketLength

SX126xLib:
Wait for event (???)
Call getPayload

**Implementation:**
Radiolib:
 - Set radio to standby
 - Check IRQ registers for CRC mismatch
 - Determine length to read:
     - Request packet length (if supplied buffer is large enough to hold any packet)
         - uses SX126X_CMD_GET_RX_BUFFER_STATUS, only takes first parameter
     - Use supplied length if less than maximum is supplied
 - Read buffer from offset zero
 - Clear IRQ status
 - (request packet length again using SX126X_CMD_GET_RX_BUFFER_STATUS to provide user with packet length)

SX126xLib
 - Get Rx buffer size and offset
 - Read buffer from returned offset

**Differences:**
 - Radiolib sets the radio to standby
 - Radiolib always reads from buffer offset zero. This works because it sets the buffer params in StartReceive

**Possible Radiolib issues:**
 - By putting the radio into standby, continuous receive is disabled and the radio always works in "Single receive"
 - In the following situation, reading from buffer offset zero doesn't work:
    - The user sets the radio in continous receive
    - The user ignores at least one packet, and waits for a second packet
    - The user then calls readData
    - The library will return data of length of the last received packet, with data from the first received packet

Asynchronous Receive with Duty Cycle
------------------------------------
**API:**
Radiolib:
Call startReceiveDutyCycle
(Optional Alternative: Call startReceiveDutyCycleAuto for radiolib to calculate appropriate parameters)

SX126xLib:
Call SetRxDutyCycle

**Implementation:**
Radiolib:
 - Account for radio sleep and wakeup time, subtract them from sleep time.
 - Common with startReceive:
     - Set DioIrqParams
     - Set BufferBaseAddress to zero
     - Clear interrupt flags
 - Set RxDutyCycle

SX126xLib:
 - Call SetRxDutyCycle with as-provided parameters

**Differences:**
 - Radiolib accepts parameters in microseconds, SX126xLib in radio units (1 unit = 15.25us)
 - Radiolib accounts for unit powerdown and wakeup time: 1000ms + TCXO delay, and reduces requested sleep time appropriately.
 - Radiolib offers convenience function for automatic determination of parameters

Detect Activity:
---------------------
**API:**
Radiolib:
(Blocking)
Call scanChannel

SX126xLib:
(Asynchronous)
Call setCadParams
Call SetDioIrqParams
Call setCad
Wait for event ???
Read IrqParams to determine whether channel has activity or CAD finished

**Implementation:**
Radiolib:
 - Put modem into standby
 - Set DIO Irq Params for CAD_DETECTED and CAD_DONE on DIO1
 - Clear IrqStatus
 - Set CAD
 - Wait until DIO1 goes high
 - Read IRQ status (To determine if activity was detected)
 - Clear IRQ status

**SX126xLib:**
(Library does minimal wrapping)
 - Call setCadParams
 - Call SetDioIrqParams
 - Call setCad

**Differences:**
 - Radiolib more user friendly (1 function call)
 - Radiolib cannot change CAD parameters

 Configuration
====================================
**Pins:**
Radiolib:
Busy Pin
DIO1 Pin
Select Pin

SX126xLib:
All SPI Pins: Mosi, miso, sclk, Select (nss)
3 DIO pins: 1, 2, 3 (Only DIO1 is used anywhere)
pin: Reset 
pin: FreqSel 
pin: Device Select 
pin: antSwPower

**Callbacks:**
Radiolib:
Dio1Action for _all_ possible interrupts

SX126xLib:
txDone
rxDone
rxPreambleDetect
rxSyncwordDone
rxHeaderDone
txTimeout
rxTimeout
rxError
cadDone

ModulationParams:
--------------------------
**API:**
Radiolib:
setBandwidth
setSpreadingFactor
setCodingRate

SX126xLib:
SetModulationParams

**Implementation:**
Radiolib:
 - Apply Low Data Rate Optimisation iff symbol length > 16.0ms
 - Set Modulation Params

SX126xLib:
 - Apply Low Data Rate Optimisation depending on spreading factor + BW
 - Set Modulation Params

**Differences:**
 - Radiolib offers three functions, but just remembers values internally and calls setModulationParams with all parameters every time.
 - Exact logic for low data rate optimisation is different but should give the same results. (CHECK THIS)

**Possible Radiolib Issues:**
 - It's probably not necessary to call SetModulationParams on the modem every time, but only before the modem is put into Rx or Tx mode.

Sync Word:
--------------------------
**API:**
Radiolib:
Call setSyncWord

SX126xLib:
 #define / #undefine USE_CONFIG_PUBLIC_NETWORK

**Imeplementation:**
Radiolib:
 - Write two bytes to register 0x0740

SX126xLib:
 - Write to register 0x0740 then 0x0741

**Differences:**
 - Compile time vs runtime
 - SX126xLib uses to write reg calls, Radiolib does it in one.

Packet Parameters
--------------------------
**API:**
Radiolib:
 Call setPreambleLength
 Call setCRC

SX126xLib:
 Call SetPacketParams

**Implementation:**
Radiolib and SX126xLib (identical):
 -  Set Packet Params

**Differences:**
 - Radiolib cannot change from these defaults:
    - Header Type: Explicit
    - InvertIQ: Standard
 - Radiolib provides multiple functions but just remembers values internally and calls SetPacketParams every time.
 - Radiolib sets packet length on startReceive

**Possible Radiolib Issues:**
 - It's probably not necessary to call SetModulationParams on the modem every time, but only before the modem is put into Rx or Tx mode.

TCXO Voltage on DIO3
--------------------------
**API:**
Radiolib:
setTCXO

SX126xLib:
SetDio3AsTcxoCtrl

**Implementation:**
Identical:
Call SetTCXO

**Differences:**
 - Radiolib accepts TCXO as float, SX126x has predefined values
 - Initialisation logic is different: Radiolib is based on parameters to begin(...), SX126X is based on the value of a particular input pin

Dio2 as RF Switch
--------------------------
**API:**
Radiolib:
setDio2AsRfSwitch

SX126xLib:
SetDio2AsRfSwitchCtrl

**Implementation:**
Identical:
Set DIO2 as Rf Switch

**Differences:**
 - Initialisation logic is different: Radiolib has RF Switch off by default, SX126x lib has it on
 
Frequency:
--------------------------
**API:**
Radiolib:
Call setFrequency

SX126xLib:
Call SetRfFrequency

**Implementation:**
Radiolib:
 - Calibrate Image
 - Adjust user input appropriately
 - Set RF Frequency

SX126xLib:
 - Calibrate Image - only once!
 - Adjust user input appropriately
 - Set RF Frequency

**Differences:**
 - SX126xLib will only calibrate the image once.

**Possible Radiolib Issues:**
 - Likely doesn't need to calibrate every time the frequency changes, only if the calibration image changes

Transmit Power
--------------------------
**API:**
Radiolib:
Call setOutputPower

SX126xLib:
Call SetTxParams

**Implementation:**
Radiolib:
 - Reject outside range -17 to +22 (or +14, for SX1261)
 - Store existing over current protection value
 - Set optimal hi power pa config
     - Call SetPaConfig with appropriate values as in SX1262 datasheet v1.2, Table 13-21
     - Scale value passed to SetTxParams according to final column of table
     - Note: SX1261 isn't configured for +15dBm
 - Call SetTxParams with scaled power and default ramp time of 200us
 - Restore over current protection value

SX126xLib:
 - SX1261:
     - Clamp to -3 to +15 dBm
     - Set PA Config, but only for +15 or +14 dBm as described in SX1262 datasheet v1.2, Table 13-21
     - Write Over current protection to 80mA
 - SX1262/8:
     - Clamp to -3 to +22 dBm
     - Set Pa Config to 0x04, 0x07, 0x00, 0x01 (Value for +22dBm in SX1262 datasheet v1.2, Table 13-21)
     - Write Over current protection to 160mA
 - All:
     - Ramp time is limited by presence of TCXO: If present, ramp time is never less than 200 us
     - Call SetTxParams

Differences:
 - Radiolib sets optimal PA config
 - Radiolib restores OCP last, SX126xLib does it before calling setTxParams

--------------------------
The following are not configurable in Radiolib, but are in SX126xLib:
 - Regulator Mode
 - Rx/Tx Fallback Mode
 - Cad Params

Informational Functions
========================================
Packet Status:
------------------
**API:**
Radiolib:
getRSSI
getSNR

SX126xLib:
GetPacketStatus

**Implementation:**
Radiolib:
 - Get Packet Status from modem
 - RSSI = -byte0 / 2.0
 - SNR = byte1 / 4.0

SX126xLib:
 - Record frequency error in ProcessIRQs:
     - Read (REG_FREQUENCY_ERROR = 0x76B)
 - Get Packet Status from modem
 - Rssi = -byte0 / 2
 - SNR = byte1 < 128 ? byte1 / 4.0 : (byte1 - 256) / 4.0
 - SignalRssi = byte2 / 2

**Differences:**
 - Radiolib provides multiple public functions, but both call the same function on the modem
 - SX126xLib treats byte1 as signed byte
 - SX126xLib reports SignalRSSI (???)
 - SX126xLib reads undocumented register 0x076B for frequency error

Instant RSSI
--------------------------
Radiolib:
Not Implemented

SX126x:
GetRSSIInst

Utility Functions
=========================================
Coninuous Wave transmission
------------------------
Radiolib optionally sets the frequency first, otherwise these are identical

Identical Functions
=========================================
 - GetStatus
 - GetDeviceErrors

SX126xLib only
=========================================
 - CheckDeviceReady: Calls wakeup iff necessary
 - GetRandom:
     - Puts radio into continuous receive, 
     - reads a random number from 0x0819
     - Puts radio into standby
 - SetFs
 - SetRxBoosted:
     - Writes RX_GAIN register with maximum gain
     - Calls SetRx
 - SetTxInfinitePreamble
 - SetStopRxTimerOnPreambleDetect
 - SetLoRaSymbNumTimeout

===========================================
Radiolib only
=========================================
 - setCurrentLimit: SX126xLib has unalterable default of 80mA for SX1261, 160mA for SX1262/8
 - getDataRate: Calculated based on the length and time required of the last packet. uC only function, doesn't touch the radio.
 - getTimeOnAir: Calculated based on datasheet formulae, does not communicate with modem.
 - clearDeviceErrors
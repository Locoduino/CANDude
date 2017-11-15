/*============================================================================
 * CANDude.cpp
 *----------------------------------------------------------------------------
 * Classes to manage a MCP 2515 CAN controller
 *
 * LOCODUINO, http://www.locoduino.org
 *
 * Author : Jean-Luc B\'echennec
 *
 * This software is distributed under the GNU Public License v2 (GPLv2)
 *
 * Please read the LICENCE file
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <SPI.h>
#include <CANDude.h>
#include <HardwareSerial.h>

/*============================================================================
 * CANDudeSettings class handle a setting of the CAN bus
 *----------------------------------------------------------------------------
 * Constructor. Does computation of bit timing
 */
CANDudeSettings::CANDudeSettings(const uint32_t inCANCrystal,
                                 const uint32_t inBaudRate,
                                 const uint32_t inMaxPPMError) :
mConfigOk(false),
mBaudRate(inBaudRate),
mCANCrystal(inCANCrystal),
mSendQueueSize(16),
mReceiveQueueSize(0)
{
  /* out of range crytal frequency according to MCP2515 datasheet */
  if (inCANCrystal < mcp2515::CRYSTAL_MIN ||
      inCANCrystal > mcp2515::CRYSTAL_MAX) return;

  /* CAN clock is half of crystal frequency */
  uint32_t CANClock = inCANCrystal / 2;

  uint32_t TQCount = 25; 			// TQCount ranges from 5 to 25
  uint32_t smallestError = UINT32_MAX;
  uint32_t bestBRP = 64;			// settings for slowest baud rate	
  uint32_t bestTQCount = 25;	// settings for slowest baud rate	
  uint32_t BRP = CANClock / inBaudRate / TQCount;
  
  /* Find the best BRP and best TQCount fitting the inBaudRate */
  while ((TQCount >= 5) && (BRP <= 64)) {
  	/* Compute error with BRP */
  	if (BRP > 0) {
  		/* error is always > = 0 */
  		const uint32_t error = (CANClock / TQCount / BRP) - inBaudRate;
  		if (error < smallestError) {
  			smallestError = error;
  			bestBRP = BRP;
  			bestTQCount = TQCount;
  		}
  	}
  	/* Compute error with BRP + 1 */
  	if (BRP < 64) {
  		/* error is always > = 0 */
  		const uint32_t error = inBaudRate - (CANClock / TQCount / (BRP + 1));
  		if (error < smallestError) {
  			smallestError = error;
  			bestBRP = BRP + 1;
  			bestTQCount = TQCount;
  		}
  	}
  	/* Continue with the next value of TQCount and BRP */
  	TQCount--;
  	BRP = CANClock / inBaudRate / TQCount;
  } 

	// Set the BRP
	mBRP = bestBRP - 1; /* mBRP is the value that will be stored in 2515 */
	// Compute PS2 so that it is lower or equal to 40% of TQCount
	uint8_t PS2 = (7 * bestTQCount) / 20;
	if (PS2 < 2) PS2 = 2;
	mPS2 = PS2 - 1;
	// Compute the remaining number of TQ once PS2 and SyncSeg are removed
	bestTQCount -= PS2 + 1;
	// Set PS1 to half of remaining TQCount
	uint8_t PS1 = bestTQCount / 2;
	if (PS1 < 1) PS1 = 1;
	mPS1 = PS1 - 1;
	// Set PS to what is left
	uint8_t PS = bestTQCount - PS1;
	mPS = PS - 1;
	// Set SJW to PS2 minus 1
	mSJW = mPS2 < 4 ? mPS2 - 1 : 3;

	// Adjust the sample point.
	if (samplePoint() > 700) {
		mPS2++;
		if (mPS1 > 0) mPS1--;
		else if (mPS > 0) mPS--;
		else mPS2--; // unable tu reduce mPS1 or mPS2, so we stay with a sample point greater than 70%
	}
	if (samplePoint() < 600 && mPS2 > 1) {
		mPS2--;
		mPS1++;
	}

	// Final check of the configuration
	mConfigOk = (PS > 0) && (PPMError() <= inMaxPPMError);
}

/*----------------------------------------------------------------------------
 * Prints a CANDudeSettings
 */
static void printSequence(
  const char delimiter,
  const char fill,
  const uint8_t * const segs,
  const uint8_t count)
{
  Serial.print(delimiter);
  for (uint8_t seg = 0 ; seg < count ; seg++) {
    for (uint8_t i = 0 ; i < segs[seg]; i++) Serial.print(fill);
    Serial.print(delimiter);
  }
}

static void printlnSequence(
  const char delimiter,
  const char fill,
  const uint8_t * const segs,
  const uint8_t count)
{
  printSequence(delimiter, fill, segs, count);
  Serial.println();
}

void CANDudeSettings::print() const
{
  uint8_t segTable[] = { 1, mPS + 1, mPS1 + 1, mPS2 + 1 };
  printlnSequence('+', '-', segTable , 4);
  printSequence('|', ' ', segTable , 4);
	Serial.print(" Baud="); Serial.print(mBaudRate);
	Serial.print(" BRP="); Serial.print(mBRP+1);
	Serial.print(" TQCount="); Serial.print(timeQuantaCount());
	Serial.print(" SamplePoint="); Serial.print(samplePoint());
	Serial.print(" error="); Serial.print(absoluteError());
	Serial.print(" PPM="); Serial.println(PPMError());
  printlnSequence('+', '-', segTable , 4);
}

/*----------------------------------------------------------------------------
 * Returns the number of Time Quanta in a bit
 */
uint8_t CANDudeSettings::timeQuantaCount() const
{
  return 1 /* SyncSeg */ + (mPS + 1) + (mPS1 + 1) + (mPS2 + 1);
}

/*----------------------------------------------------------------------------
 * Returns the number of Time Quanta in a bit
 */
uint32_t CANDudeSettings::actualBaudRate() const
{
	uint8_t TQCount = timeQuantaCount();
  return TQCount > 0 ? (mCANCrystal / 2) / (TQCount * (mBRP + 1)) : 0;
}

/*----------------------------------------------------------------------------
 * Returns the ppm error of a configuration
 */
 uint32_t CANDudeSettings::PPMError() const
 {
   uint32_t adjustedError = absoluteError();
   uint32_t multiplier = 1;
   while (adjustedError > 4294) {
     adjustedError /= 10;
     multiplier *= 10;
   }
   return ((1000000 * adjustedError) / mBaudRate) * multiplier;
 }

/*----------------------------------------------------------------------------
 * Returns the sample point in ‰
 */
uint16_t CANDudeSettings::samplePoint() const
{
	uint16_t divisor = timeQuantaCount();
	uint16_t phaseSeg2 = mPS2 + 1;
	return divisor > 0 ? (1000 * (divisor - phaseSeg2)) / divisor : 0;
}

/*============================================================================
 * CANDudeMessage is the class to handle both sent and receive messages
 *----------------------------------------------------------------------------
 */
void CANDudeSendMessage::setStandardId(const uint16_t inId)
{
	sidh = (inId >> 3) & 0x00FF;
	sidl = ((inId & mcp2515::TXBnSIDL_SID_MASK) << mcp2515::TXBnSIDL_SID_SHIFT);
}

/*----------------------------------------------------------------------------
 */
void CANDudeSendMessage::setExtendedId(const uint32_t inId)
{
	sidh = (inId >> 3) & 0x000000FF;
	sidl = ((inId & mcp2515::TXBnSIDL_SID_MASK) << mcp2515::TXBnSIDL_SID_SHIFT) |
	       mcp2515::TXBnSIDL_EXIDE |
         ((inId >> 27) & mcp2515::TXBnSIDL_EID_MASK);
	eid8 = (inId >> 19) & 0x000000FF;
	eid0 = (inId >> 11) & 0x000000FF;
}

/*----------------------------------------------------------------------------
 */
void CANDudeSendMessage::setRemote(const bool inRemote)
{
	if (inRemote) dlc |= mcp2515::TXBnDLC_RTR;
	else dlc &= ~mcp2515::TXBnDLC_RTR;
}

/*----------------------------------------------------------------------------
 */
void CANDudeSendMessage::setLength(const uint8_t inLength)
{
	dlc &= ~mcp2515::TXBnDLC_DLC_MASK;
	dlc |= inLength & mcp2515::TXBnDLC_DLC_MASK;
}

/*----------------------------------------------------------------------------
 */
uint8_t CANDudeSendMessage::sizeInBytes() const
{
	// 4 bytes for id and one for length + the number of bytes
	return 5 + length();
}

/*----------------------------------------------------------------------------
 */
void CANDudeSendMessage::print() const
{
  uint32_t id = ((uint32_t)sidh) << 3 | ((sidl >> mcp2515::TXBnSIDL_SID_SHIFT) & mcp2515::TXBnSIDL_SID_MASK);
  char type = isExtended() ? 'e' : 's';
  if (type == 'e') {
    id |= (((uint32_t)eid8) << 19) | (((uint32_t)eid0) << 11) | (((uint32_t)(sidl & mcp2515::TXBnSIDL_EID_MASK)) << 27) ;
  }
  Serial.print(type);
  Serial.print('[');
  Serial.print(id);
  Serial.print('/');
  Serial.print(id, HEX);
  Serial.print(']');
  if (isRemote()) {
    Serial.println('r');
  }
  else {
    for (uint8_t i = 0; i < length(); i++) {
      Serial.print(' ');
      Serial.print(data[i], HEX);
    }
    Serial.println();
  }
}

/*----------------------------------------------------------------------------
 */
void CANDudeSendMessage::printRaw() const
{
  Serial.print("SIDH="); Serial.print(sidh, HEX);
  Serial.print(" SIDL="); Serial.print(sidl, HEX);
  Serial.print(" (");
  Serial.print((sidl >> mcp2515::TXBnSIDL_SID_SHIFT) & mcp2515::TXBnSIDL_SID_MASK);
  Serial.print(',');
  Serial.print(isExtended() ? 'e' : 's');
  Serial.print(',');
  Serial.print(sidl & mcp2515::TXBnSIDL_EID_MASK);
  Serial.print(')');
  Serial.print(" EID8="); Serial.print(eid8, HEX);
  Serial.print(" EID0="); Serial.print(eid0, HEX);
  Serial.print(" DLC="); Serial.print(dlc, HEX);
  Serial.print(" DATA=");
  for (uint8_t i = 0; i < 8; i++) {
    Serial.print(' ');
    Serial.print(data[i], HEX);
  }
  Serial.println();
}

/*============================================================================
 * CANDudeQueue is the class to handle queues used to send and receive
 * messages
 *----------------------------------------------------------------------------
 * Constructor, init an empty queue.
 */
CANDudeQueue::CANDudeQueue(const uint8_t inSize) :
mSize(0),
mIndex(0)
{
	uint8_t mask = 0xFF;
	uint8_t size = inSize > 16 ? inSize - 1 : 15;
	while (!(size & 0x80)) {
		mask >>= 1;
		size <<= 1;
	}
  mMask = mask;
	mQueue = new uint8_t[((uint16_t)mask) + 1];
}

/*----------------------------------------------------------------------------
 * Push a byte in the queue if room available
 */
uint8_t CANDudeQueue::pushByte(const uint8_t inByte)
{
	if (!isFull()) {
		mQueue[mIndex++] = inByte;
		mSize++;
		mIndex &= mMask;
    return 1;
	}
  else {
    return 0;
  }
}

/*----------------------------------------------------------------------------
 * Push bytes in the queue if room available. Returns how many bytes have
 * been pushed.
 */
uint8_t CANDudeQueue::pushByte(const uint8_t * const inBytes, const uint8_t inCount)
{
  uint16_t freeSpace = available();
  uint8_t howMany = inCount > freeSpace ? freeSpace : inCount;
  for (uint8_t i = 0; i < howMany; i++) {
    mQueue[mIndex++] = inBytes[i];
    mIndex &= mMask;
  }
	mSize += howMany;
  return howMany;
}

/*----------------------------------------------------------------------------
 * Pop a byte from the queue if not empty
 */
uint8_t CANDudeQueue::popByte()
{
	if (mSize > 0) {
    uint8_t readIndex = (mIndex - mSize) & mMask;
    mSize--;
    return mQueue[readIndex];
  }
  else return 0;
}

/*----------------------------------------------------------------------------
 * Pop a bytes from the queue if available, returns how many bytes have been
 * popped
 */
uint8_t CANDudeQueue::popByte(uint8_t * const outBytes, const uint8_t inCount)
{
  uint8_t howMany = inCount > mSize ? mSize : inCount;
  uint8_t readIndex = (mIndex - mSize) & mMask;
  for (uint8_t i = 0; i < howMany; i++) {
    outBytes[i] = mQueue[readIndex++];
    readIndex &= mMask;
  }
  return howMany;
}

/*----------------------------------------------------------------------------
 * Prints the queue (for debug)
 */
void CANDudeQueue::print() const
{
	uint8_t readIndex = (mIndex - mSize) & mMask;
	uint8_t size = mSize;
	Serial.print('['); Serial.print(mSize); Serial.print("] ");
	while (size-- > 0) {
		Serial.print(mQueue[readIndex++], HEX);
		Serial.print(' ');
		readIndex &= mMask;
	}
	Serial.println();
}
/*============================================================================
 */
static const uint8_t CAN_MESSAGE_MAX_LENGTH = 8;

#if SPI_HAS_TRANSACTION == 1
static const SPISettings kMcp2515Settings(10000000, MSBFIRST, SPI_MODE0);
#define SPIBeginTransaction() SPI.beginTransaction(kMcp2515Settings)
#define SPIEndTransaction()   SPI.endTransaction()
#else
#define SPIBeginTransaction()
#define SPIEndTransaction()
#endif

/*============================================================================
 * CANDude is the base class to talk with a MCP2515
 *----------------------------------------------------------------------------
 * Constructor. Set the chip select pin
 */
CANDude::CANDude(const uint8_t inSlaveSelectPin,
								 const uint8_t inInterruptPin,
								 const CANDudeInterrupt inInterruptFunction) :
mSlaveSelectPin(inSlaveSelectPin),
mInterruptPin(inInterruptPin),
mInterruptFunction(inInterruptFunction),
mReceiveQueue(NULL),
mSendQueue(NULL),
mSendCount(0)
{
}

/*----------------------------------------------------------------------------
 * Reset the MCP2515 by sending a reset command
 */
void CANDude::reset()
{
  SPIBeginTransaction();
  select();
  SPI.transfer(mcp2515::RESET);
  unselect();
  SPIEndTransaction();
}

/*----------------------------------------------------------------------------
 * Read a register of the MCP2515 by sending a read command
 * returns the value of the register
 */
uint8_t CANDude::read(const uint8_t inAddress)
{
  SPIBeginTransaction();
  select();
  SPI.transfer(mcp2515::READ);
  SPI.transfer(inAddress);
  uint8_t value = SPI.transfer(0);
  unselect();
  SPIEndTransaction();
  return value;
}

/*----------------------------------------------------------------------------
 * Write a register of the MCP2515 by sending a WRITE command
 */
void CANDude::write(const uint8_t inAddress, const uint8_t inData)
{
  SPIBeginTransaction();
  select();
  SPI.transfer(mcp2515::WRITE);
  SPI.transfer(inAddress);
  SPI.transfer(inData);
  unselect();
  SPIEndTransaction();
}

/*----------------------------------------------------------------------------
 * Read registers in sequential order by sending a READ command
 * startAddress is the address of the register where reading starts
 * numberOfRegisters is the number of registers to read
 * buffer is a pointer to where the read value will be stored.
 * Size of the storage should be at least numberOfRegisters
 */
void CANDude::read(
  const uint8_t inStartAddress,
  uint16_t inNumberOfRegisters,
  uint8_t *outBuffer)
{
  SPIBeginTransaction();
  select();
  SPI.transfer(mcp2515::READ);
  SPI.transfer(inStartAddress);
  while (inNumberOfRegisters--) *outBuffer++ = SPI.transfer(0);
  unselect();
  SPIEndTransaction();
}

/*----------------------------------------------------------------------------
 * Write registers in sequential order by sending a WRITE command.
 * startAddress is the address of the register where writing starts.
 * numberOfRegisters is the number of registers to write.
 * buffer is a pointer from where the value to write will be stored.
 * Size of the storage should be at least numberOfRegisters.
 */
void CANDude::write(
  const uint8_t inStartAddress,
  uint16_t inNumberOfRegisters,
  uint8_t *inBuffer)
{
  SPIBeginTransaction();
  select();
  SPI.transfer(mcp2515::WRITE);
  SPI.transfer(inStartAddress);
  while (inNumberOfRegisters--) SPI.transfer(*inBuffer++);
  unselect();
  SPIEndTransaction();
}

/*----------------------------------------------------------------------------
 * Read a RX buffer without the identifier of the message by
 * sending a READ RX BUFFER command
 * bufferID is the identifier of the RX buffer.
 * It can be mcp2515::RX_BUFFER_0 or mcp2515::RX_BUFFER_1
 * inNumberOfBytes is the number of uint8_ts to read from the message
 * buffer id a pointer to where the message will be stored.
 * Size of the storage should be at least numberOfuint8_t
 */
void CANDude::readRawMessage(
  const uint8_t inBufferID,
  uint8_t inNumberOfBytes,
  uint8_t *outBuffer)
{
  inNumberOfBytes = min(inNumberOfBytes, CAN_MESSAGE_MAX_LENGTH);
  SPIBeginTransaction();
  select();
  SPI.transfer(mcp2515::READ_RX_BUFFER | inBufferID | mcp2515::RX_BUFFER_NO_ID);
  while (inNumberOfBytes--) *outBuffer++ = SPI.transfer(0);
  unselect();
  SPIEndTransaction();
}

/*----------------------------------------------------------------------------
 * Load a TX buffer without the header of the message by
 * sending a LOAD TX BUFFER command.
 * bufferId is the identifier of the TX buffer.
 * It can be mcp2515::TX_BUFFER_0, mcp2515::TX_BUFFER_1 or
 * mcp2515::TX_BUFFER_2.
 * inNumberOfBytes is the number of uint8_ts to write to the message.
 * buffer is a pointer from where the message will be read.
 * Size of the buffer should be at least inNumberOfBytes
 */
void CANDude::loadMessage(
  const uint8_t inBufferID,
  const CANDudeSendMessage &inMessage)
{
  uint8_t numberOfBytes = inMessage.sizeInBytes();
  uint8_t *dataPtr = inMessage.start();
  Serial.print("S: ");
  Serial.print(mcp2515::LOAD_TX_BUFFER(inBufferID), BIN);
  for (uint8_t i = 0; i < numberOfBytes; i++) {
    Serial.print(' ');
    Serial.print(*dataPtr++, BIN);
  }
  Serial.println();
  dataPtr = inMessage.start();
  SPIBeginTransaction();
  select();
  SPI.transfer(mcp2515::LOAD_TX_BUFFER(inBufferID));
  while (numberOfBytes--) SPI.transfer(*dataPtr++);
  unselect();
  SPIEndTransaction();
}

/*----------------------------------------------------------------------------
 * Request to send one or more TX buffer(s)
 * query may be mcp2515::RTS_TXB0, mcp2515::RTS_TXB1, mcp2515::RTS_TXB2 or
 * a combination (|) of them.
 */
void CANDude::requestToSend(const uint8_t inQuery)
{
  SPIBeginTransaction();
  select();
  SPI.transfer(mcp2515::RTS(inQuery));
  unselect();
  SPIEndTransaction();
}

/*----------------------------------------------------------------------------
 * Read status instruction
 */
uint8_t CANDude::readStatus()
{
  SPIBeginTransaction();
  select();
  uint8_t status = SPI.transfer(mcp2515::READ_STATUS);
  unselect();
  SPIEndTransaction();
  return status;
}

/*----------------------------------------------------------------------------
 * Read RX status instruction
 */
uint8_t CANDude::RXStatus()
{
  SPIBeginTransaction();
  select();
  uint8_t status = SPI.transfer(mcp2515::RX_STATUS);
  unselect();
  SPIEndTransaction();
  return status;
}

/*----------------------------------------------------------------------------
 * Bit modify instruction
 */
void CANDude::modifyBit(
  const uint8_t address,
  const uint8_t mask,
  const uint8_t data)
{
  SPIBeginTransaction();
  select();
  SPI.transfer(mcp2515::BIT_MODIFY);
  SPI.transfer(address);
  SPI.transfer(mask);
  SPI.transfer(data);
  unselect();
  SPIEndTransaction();
}

/*----------------------------------------------------------------------------
 * Set mode of operation.
 */
CANDudeResult CANDude::setMode(const uint8_t inMode)
{
  if (inMode <= mcp2515::CONFIG_MODE) {
    // Set CANCTRL.REQOP to request a new mode of operation
    modifyBit(
      mcp2515::CANCTRL,
      mcp2515::CANCTRL_REQOP_MASK << mcp2515::CANCTRL_REQOP_SHIFT,
      inMode << mcp2515::CANCTRL_REQOP_SHIFT
    );
    // Loop until CANSTAT.OPMOD get the same value or 1 ms elapsed
    uint8_t opmod;
    uint32_t currentTime = micros();
    do {
      opmod = read(mcp2515::CANSTAT) >> mcp2515::CANSTAT_OPMOD_SHIFT;
    } while (opmod != inMode && (micros() - currentTime) < 1000);
    if (opmod == inMode) {
      return CANDudeOk;
    }
    else {
      return CANDudeSetModeTimeout;
    }
  }
  else {
    return CANDudeBadMode;
  }
}

/*----------------------------------------------------------------------------
 * Start and configure
 */
CANDudeResult CANDude::begin(const CANDudeSettings & inSettings)
{
	// Allocate the send and receive queues if any
	if (inSettings.receiveQueueSize() > 0) {
		mReceiveQueue = new CANDudeQueue(inSettings.receiveQueueSize());
		if (mReceiveQueue == NULL || mReceiveQueue->allocFailed()) return CANDudeOutOfMemory;
	}
	if (inSettings.sendQueueSize() > 0) {
		mSendQueue = new CANDudeQueue(inSettings.sendQueueSize());
		if (mSendQueue == NULL || mSendQueue->allocFailed()) return CANDudeOutOfMemory;
	}

  unselect(); // Turn off the chip select before programming the pin as output
  pinMode(mSlaveSelectPin, OUTPUT); // Program the chip select as output
  SPI.begin();
  // reset the MCP2515
  reset();
  // Test the communication with the MCP2515 by writing and reading data
  // to and from byte 0 of TX buffer 0
  write(mcp2515::TXB0D0, 0xAA);
  if (read(mcp2515::TXB0D0) != 0xAA) return CANDudeSPIFailed;
  write(mcp2515::TXB0D0, 0x55);
  if (read(mcp2515::TXB0D0) != 0x55) return CANDudeSPIFailed;

  // Check settings to avoid configuration if settings are not ok
  if (! inSettings.configOk()) return CANDudeBadConfig;

  // Enter config mode
  CANDudeResult result = setMode(mcp2515::CONFIG_MODE);
  if (result == CANDudeOk) {
    // Program bit timing according to the configuration
    uint8_t cnf[3];
    // CNF register ar stored in consecutive registers from CNF3 to CNF1
    // that is address in reverse order compared to register number
    cnf[2] = // This is CNF1
      (inSettings.sjw() << mcp2515::CNF1_SJW_SHIFT) | // SJW
      inSettings.brp();                             // BRP
    cnf[1] = // This is CNF2
      mcp2515::CNF2_BTLMODE | // PS2 is in CNF3
      (inSettings.tripleSampling() ? mcp2515::CNF2_SAM : 0) | // Triple sample point is set only when there is enough TQ
      (inSettings.ps1() << mcp2515::CNF2_PHSEG1_SHIFT) | // PS1
      inSettings.ps();                                 // PS
    cnf[0] = // This is CNF3
      inSettings.ps2();                                // PS2

    for (uint8_t i=0; i < 3; i++) {
      Serial.print("CNF");
      Serial.print(3-i);
      Serial.print(": ");
      Serial.print(cnf[i], BIN);
      Serial.print(" / ");
    }
    Serial.println();

    // Write the bit timing configuration
    write(mcp2515::CNF3, 3 /* 3 registers */, cnf);

		// Program Interrupts
		pinMode(mInterruptPin, INPUT);
		uint8_t interruptNumber = digitalPinToInterrupt(mInterruptPin);
		SPI.usingInterrupt(interruptNumber);
		attachInterrupt(interruptNumber, mInterruptFunction, FALLING);
		// Enable RXB0 and TXB0 Interrupts
		write(mcp2515::CANINTE, mcp2515::CANINTE_TX0IE, mcp2515::CANINTE_RX0IE);

    // Go back to normal mode
    result = setMode(mcp2515::NORMAL_MODE);
  }
  return result;
}

/*----------------------------------------------------------------------------
 * Send a message
 */
CANDudeResult CANDude::sendMessage(const CANDudeSendMessage & inMessage)
{
  if (mSendCount > 0) {
  	if (mSendQueue->available() >= inMessage.sizeInBytes()) {
  		mSendQueue->pushByte(inMessage.start(), 5);
      mSendQueue->pushByte(inMessage.dataPart(), inMessage.length());
  		return CANDudeOk;
  	}
  	else {
  		return CANDudeNoRoom;
  	}
  }
  else {
    loadMessage(0, inMessage);
  }
}

/*----------------------------------------------------------------------------
 * Handle the MCP2515 interrupt
 */
void CANDude::handleInterrupt()
{
}

// /*----------------------------------------------------------------------------
//  * Read a RX buffer with the header of the message by
//  * sending a READ RX BUFFER command.
//  * bufferID is the identifier of the RX buffer.
//  * It can be mcp2515::RX_BUFFER_0 or mcp2515::RX_BUFFER_1.
//  * The message is dispatched to the CanReceiveMessage having the corresponding
//  * identifier or discarded.
//  */
// void CanController::readMessage(const uint8_t bufferID)
// {
//   select();
//   SPI.transfer(mcp2515::READ_RX_BUFFER | bufferID | mcp2515::RX_BUFFER_ID);
//   uint8_t sidh = SPI.transfer(0);
//   uint8_t sidl = SPI.transfer(0);
//   uint8_t eid8 = SPI.transfer(0);
//   uint8_t eid0 = SPI.transfer(0);
//   uint8_t dlc  = SPI.transfer(0);
//   uint8_t size = dlc & mcp2515::RXBnDLC_DLC_MASK;
//   for (uint8_t i = 0; i < size; i++) buffer[i] = SPI.transfer(0);
//   unselect();
//
//   /* standard id part */
//   long frameId = sidh;
//   frameId = (frameId << 3)  | (sidl >> 5);
//
//   /* look for the kind of message, standard or extended */
//   if (sidl & mcp2515::RXBnSIDL_IDE) {
//     /* extended frame, add the extended id part */
//     frameId = (frameId << 2)  | (sidl & 0x03);
//     frameId = (frameId << 8)  | eid8;
//     frameId = (frameId << 8)  | eid0;
//     /* mark the frame as extended by setting the sign bit */
//     frameId |= 0x80000000;
//   }
//
//   /* look for the destination message */
//   CanReceiveMessage *destinationMessage;
//   if (destinationMessage = findReceiveMessage(frameId)) {
//     buffer = destinationMessage.set(size, buffer);
//   }
// }
//
// /*----------------------------------------------------------------------------
//  * Load a TX buffer with the header of the message by
//  * sending a LOAD TX BUFFER command.
//  * bufferID is the identifier of the TX buffer.
//  * It can be mcp2515::TX_BUFFER_0, mcp2515::TX_BUFFER_1 or
//  * mcp2515::TX_BUFFER_2.
//  * message is a pointer to a CanSendSync message
//  */
// void CanController::loadMessage(const uint8_t bufferID, CanSendsync *message)
// {
//   if (lastMessageForTXBuffer[bufferID] != message) {
//     /* The header have to be loaded */
//     select();
//     SPI.transfer(mcp2515::LOAD_TX_BUFFER(bufferID));
//     unsigned long identifier = message->identifier();
//     if (identifier & 0x80000000) {
//       /* extended message, the sidh is bits 28 to 21 */
//       SPI.transfer(uint8_t((identifier >> 21) & 0xFF));
//       /* the sidl is bits 20 to 18 and 17 to 16 */
//       SPI.transfer(uint8_t(
//         ((identifier >> 13) & (TXBnSIDL_SID_MASK << TXBnSIDL_SID_SHIFT)) |
//         TXBnSIDL_EXIDE |
//         ((identifier >> 16) & TXBnSIDL_EID_MASK)
//       ));
//       /* extended high */
//       SPI.transfer(uint8_t((identifier >> 8) & 0xFF));
//       /* extended low */
//       SPI.transfer(uint8_t(identifier & 0xFF));
//     }
//     else {
//       /* extended message, the sidh is bits 10 to 3 */
//       SPI.transfer(uint8_t((identifier >> 3) & 0xFF));
//       /* the sidl is bits 2 to 0 */
//       SPI.transfer(
//         uint8_t((identifier & TXBnSIDL_SID_MASK) << TXBnSIDL_SID_SHIFT)
//       );
//       /* extended unused */
//       SPI.transfer(0);
//       SPI.transfer(0);
//     }
//     /* Data length is 0 */
//     SPI.transfer(0);
//     unselect();
//   }
//   /* request to send of the TX buffer */
//   requestToSend(1 << bufferID);
// }
//
// /*----------------------------------------------------------------------------
//  */
// void CanController::computeTXBufferAttribution()
// {
//   /* remaining messages to attribute is set to the number of messages */
//   uint8_t remainingMessagesToAttribute = 0;
//   CanSendMessage *currentMessage = mSendMessages;
//   while (currentMessage != NULL) {
//     remainingMessagesToAttribute++;
//     currentMessage = currentMessage->nextMessage();
//   }
//
//   /* compute the number of messages per TX buffer */
//   uint8_t TXBufferId = 0;
//   uint8_t TXBufferMessageCount[3] = { 0, 0, 0 };
//   /* This is done by ++ the number of messages using a round-robin scheme */
//   while (remainingMessagesToAttribute > 0) {
//     TXBufferMessageCount[TXBufferId]++;
//     TXBufferId = (TXBufferId + 1) % 3;
//   }
//
//   /* set the TX buffer ID for each message */
//   currentMessage = mSendMessages;
//   TXBufferId = 0;
//   while (currentMessage != NULL) {
//     if (TXBufferMessageCount[TXBufferId] == 0) TXBufferId++;
//     currentMessage->setTXBufferId(TXBufferId);
//     currentMessage = currentMessage->next();
//     TXBufferMessageCount[TXBufferId]--;
//   }
// }
//
// /*----------------------------------------------------------------------------
//  */
// void CanController::addReceiveMessage(CanReceiveMessage *message)
// {
//   message->setNext(mReceiveMessages);
//   mReceiveMessages = message;
// }
//
// /*----------------------------------------------------------------------------
//  * Add a sending message to the controller. Messages are sorted according
//  * to their priority
//  */
// void CanController::addSendMessage(CanSendMessage *message)
// {
//   if (message != NULL) {
//     message->setController(this);
//     mSendMessages = mSendMessages->addMessage(message);
//   }
// }
//
// /*----------------------------------------------------------------------------
//  */
// void CanController::begin()
// {
//   CanReceiveMessage *message = mReceiveMessages;
//   while (message != NULL) {
//
//     message = message->nextMessage();
//   }
// }
//
// /*----------------------------------------------------------------------------
//  */
// CanReceiveMessage *CanController::findReceiveMessage(unsigned long id)
// {
//   CanReceiveMessage *message = mReceiveMessages;
//   while (message != NULL) {
//     if (message->accept(id)) break;
//     message = message->nextMessage();
//   }
//   return message;
// }

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
#include <MCP2515Definitions.h>

static const uint32_t kMaxBaudRateForTripleSampling = 125000;

static void printByteWithLeadingZeroes(
  const uint8_t inValue,
  const bool    inNewline = false)
{
  uint8_t mask = B10000000;
  while (mask != 0) {
    Serial.print((mask & inValue) != 0);
    mask >>= 1;
  }
  if (inNewline) Serial.println();
}

static void printExtendedID(
  const uint32_t inValue,
  const bool     inNewline = false)
{
  uint32_t mask = 0x10000000;
  while (mask != 0) {
    Serial.print((mask & inValue) != 0);
    mask >>= 1;
  }
  if (inNewline) Serial.println();
}

static void printStandardID(
  const uint32_t inValue,
  const bool     inNewline = false)
{
  uint32_t mask = 0x00000400;
  while (mask != 0) {
    Serial.print((mask & inValue) != 0);
    mask >>= 1;
  }
  if (inNewline) Serial.println();
}

static const bool EOL = true;

/*----------------------------------------------------------------------------
 * Constructor. Does computation of bit timing
 */
CANDudeSettings::CANDudeSettings(const uint32_t inCANCrystal,
                                 const uint32_t inWishedBaudRate,
                                 const uint32_t inMaxPPMError) :
mWishedBaudRate(inWishedBaudRate),
mCANCrystal(inCANCrystal),
mConfigOk(false)
{
  /*
   * out of range crytal frequency according to MCP2515 datasheet. So let
   * mConfigOk set to false
   */
  if (inCANCrystal < mcp2515::CRYSTAL_MIN ||
      inCANCrystal > mcp2515::CRYSTAL_MAX) return;

  /*
   * CAN clock is half of crystal frequency
   */
  const uint32_t CANClock = inCANCrystal / 2;

  uint32_t TQCount = 25;      // TQCount ranges from 5 to 25
  uint32_t smallestError = UINT32_MAX;
  uint32_t bestBRP = 64;      // settings for slowest baud rate
  uint32_t bestTQCount = 25;  // settings for slowest baud rate
  uint32_t BRP = CANClock / inWishedBaudRate / TQCount;

  //---- Find the best BRP and best TQCount fitting the inWishedBaudRate
  while ((TQCount >= 5) && (BRP <= 64)) {
    //---- Compute error with BRP
    if (BRP > 0) {
      //---- error is always >= 0
      const uint32_t error = (CANClock / TQCount / BRP) - inWishedBaudRate;
      if (error < smallestError) {
        smallestError = error;
        bestBRP = BRP;
        bestTQCount = TQCount;
      }
    }
    //---- Compute error with BRP + 1
    if (BRP < 64) {
      //---- error is always >= 0
      const uint32_t error = inWishedBaudRate -
                             (CANClock / TQCount / (BRP + 1));
      if (error < smallestError) {
        smallestError = error;
        bestBRP = BRP + 1;
        bestTQCount = TQCount;
      }
    }
    //---- Continue with the next value of TQCount and BRP
    TQCount--;
    BRP = CANClock / inWishedBaudRate / TQCount;
  }

  //---- Set the BRP
  mBRP = bestBRP - 1; /// mBRP is the value that will be stored in the MCP2515
  //---- Compute PS2 so that it is lower or equal to 40% of TQCount
  const uint8_t PS2 = 1 + 2 * bestTQCount / 7; // necessarily between 2 and 8
  mPS2 = PS2 - 1;
  //---- Compute the remaining number of TQ once PS2 and SyncSeg are removed
  const uint8_t PSplusPS1 = bestTQCount - (PS2 + 1);
  //---- Set PS1 to half of remaining TQCount
  const uint8_t PS1 = PSplusPS1 / 2;           // necessarily between 1 and 8
  mPS1 = PS1 - 1;
  //---- Set PS to what is left
  const uint8_t PS = PSplusPS1 - PS1;
  mPS = PS - 1;
  //---- Adjust the sample point so that it is not greater than 70%
  if (samplePoint() >= 700) {
    mPS2++;
    mPS--;
  }
  //---- Adjust the sample point so that it is not lower than 60%
  if ((samplePoint() <= 600) && (mPS2 > 1)) {
    mPS2--;
    if (mPS1 < mPS) mPS1++; else mPS++;
  }

  //---- Set SJW to PS2 minus 1 with a maximum of 3
  mSJW = mPS2 < 4 ? mPS2 - 1 : 3;
  //---- Is triple sampling possible ?
  mTripleSampling = (inWishedBaudRate <= kMaxBaudRateForTripleSampling) &&
                    (mPS2 >= 1);

  // Final check of the configuration
  mConfigOk = (PPMError() <= inMaxPPMError);
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
  uint8_t segTable[] = {
    1,
    (uint8_t)(mPS + 1),
    (uint8_t)(mPS1 + 1),
    (uint8_t)(mPS2 + 1)
  };
  printlnSequence('+', '-', segTable , 4);
  printSequence('|', ' ', segTable , 4);
  Serial.print(" Baud="); Serial.print(mWishedBaudRate);
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
   return ((1000000 * adjustedError) / mWishedBaudRate) * multiplier;
 }

/*----------------------------------------------------------------------------
 * Returns the sample point in ‰
 */
uint16_t CANDudeSettings::samplePoint() const
{
  const uint16_t divisor = timeQuantaCount();
  const uint16_t phaseSeg2 = mPS2 + 1;
  return divisor > 0 ? (1000 * (divisor - phaseSeg2)) / divisor : 0;
}


/*-------------------------------------------------------------------------------------------------
 * Constructor. Initialize the masks and the filters so that no message is accepted
 */
CANDudeFilters::CANDudeFilters()
{
  for (uint8_t m = 0; m < 2; m++) {
    mMask[m].isSet = false;
  }
  for (uint8_t f = 0; f < 6; f++) {
    mFilter[f].isSet = false;
  }
}

/*-------------------------------------------------------------------------------------------------
 * Set the mask of one of the buffers (0 or 1). Return true if ok, false otherwise
 */
bool CANDudeFilters::setMask(const uint8_t inBuffer, const uint32_t inMask)
{
  bool status = true;
  if (inBuffer < 2) {
    mMask[inBuffer].isSet = true;
    mMask[inBuffer].mask = inMask;
  }
  else status = false;
  return status;
}

/*-------------------------------------------------------------------------------------------------
 * Set one of the filter of one of the buffers (0 or 1). Return true if ok, false otherwise
 */
bool CANDudeFilters::setFilter(const uint8_t  inFilterNum,
                               const bool     inIsExtended,
                               const uint32_t inFilter)
{
  bool status = true;
  if (inFilterNum < 6) {
    mFilter[inFilterNum].isSet = true;
    mFilter[inFilterNum].isExtended = inIsExtended;
    mFilter[inFilterNum].filter = inFilter;
  }
  else status = false;
  return status;
}

/*-------------------------------------------------------------------------------------------------
 * Returns one of the 4 MCP2515 registers falue for a filter or a mask. inPos should be one of the
 * following:
 * CANDudeFilters::SIDH
 * CANDudeFilters::SIDL (Here for filters the EXIDE bit is not set)
 * CANDudeFilters::EID8
 * CANDudeFilters::EID0
 */
bool CANDudeFilters::byteInMaskOrFilter(const uint32_t        inMaskOrFilter,
                                        const maskOrFilterPos inPos,
                                        uint8_t&              result)
{
  bool status = true;
  switch (inPos) {
    case SIDH:
      result = (uint8_t) ((inMaskOrFilter >> 3) & 0xFF);
      break;
    case SIDL:
      result = (uint8_t) (((inMaskOrFilter & mcp2515::RXFnSIDL_SID_MASK)
                                            << mcp2515::RXFnSIDL_SID_SHIFT) |
               ((inMaskOrFilter >> 27) & 0x03));
      break;
    case EID8:
      result = (uint8_t) ((inMaskOrFilter >> 19) & 0xFF);
      break;
    case EID0:
      result = (uint8_t) ((inMaskOrFilter >> 11) & 0xFF);
      break;
    default:
      status = false;
  }
  return status;
}

/*-----------------------------------------------------------------------------
 * Returns one of the 4 MCP2515 registers values for a mask.
 * inPos should be one of the following:
 * CANDudeFilters::SIDH
 * CANDudeFilters::SIDL
 * CANDudeFilters::EID8
 * CANDudeFilters::EID0
 *
 * Returns 0 if inBuffer or inPos is out of range
 */
uint8_t CANDudeFilters::mask(const uint8_t inBuffer, const maskOrFilterPos inPos)
{
  uint8_t result = 0;

  if (inBuffer < 2) {
    byteInMaskOrFilter(mMask[inBuffer].mask, inPos, result);
  }
  return result;
}

/*-----------------------------------------------------------------------------
 * Returns one of the 4 MCP2515 registers value for a filter.
 * inPos should be one of the following:
 * CANDudeFilters::SIDH
 * CANDudeFilters::SIDL
 * CANDudeFilters::EID8
 * CANDudeFilters::EID0
 *
 * Returns all significant bits filled with 1 if inBuffer or inPos
 * is out of range
 */
uint8_t CANDudeFilters::filter(const uint8_t          inFilter,
                               const maskOrFilterPos  inPos)
{
  uint8_t result = (uint8_t) (-1);
  Filter_t filter;

  if (inFilter < 6) {
    filter = mFilter[inFilter];
    if (byteInMaskOrFilter(filter.filter, inPos, result)) {
      if (inPos == SIDL && filter.isExtended) {
        result |= mcp2515::RXFnSIDL_EXIDE;
      }
    }
  }
  else {
    result = B11101011;
  }
  return result;
}

/*-----------------------------------------------------------------------------
 * Check at least one mask and filter are configured for a message buffers
 */
bool CANDudeFilters::isConfigured(const uint8_t inBuffer)
{
  bool status = false;
  if (inBuffer < 2) {
    if (mMask[inBuffer].isSet) {
      uint8_t startFilterNum = (inBuffer == 0) ? 0 : 2;
      uint8_t endFilterNum = (inBuffer == 0) ? 2 : 6;
      for (uint8_t f = startFilterNum; f < endFilterNum; f++) {
        if (mFilter[f].isSet) {
          status = true;
          break;
        }
      }
    }
  }
  return status;
}

/*-----------------------------------------------------------------------------
 * Compute the unset filters if mask is set and at least one filter is set
 * copy the set filter to the unset filters.
 */
void CANDudeFilters::finalize()
{
  for (uint8_t m = 0; m < 2; m++) {
    if (mMask[m].isSet) {
      uint8_t startFilterNum = (m == 0) ? 0 : 2;
      uint8_t endFilterNum = (m == 0) ? 2 : 6;
      /* search for a filter that is set */
      uint8_t filterSet;
      for (uint8_t f = startFilterNum; f < endFilterNum; f++) {
        if (mFilter[f].isSet) {
          for (uint8_t ff = startFilterNum; ff < endFilterNum; ff++) {
            if (!mFilter[ff].isSet) {
              mFilter[ff] = mFilter[f];
            }
          }
          break;
        }
      }
    }
  }
}

/*-----------------------------------------------------------------------------
 * print the filter inSettings
 */
void CANDudeFilters::print()
{
  Serial.println(F("** Buffer 0"));
  Serial.print(mMask[0].isSet ? '+' : '-');
  Serial.print(F("MASK : "));
  printExtendedID(mMask[0].mask);
  Serial.print(F(" : SIDH = "));
  printByteWithLeadingZeroes(mask(0,SIDH));
  Serial.print(F(" SIDL = "));
  printByteWithLeadingZeroes(mask(0,SIDL));
  Serial.print(F(" EID8 = "));
  printByteWithLeadingZeroes(mask(0,EID8));
  Serial.print(F(" EID0 = "));
  printByteWithLeadingZeroes(mask(0,EID0), EOL);
  for (int f = 0; f < 2; f++) {
    Serial.print(mFilter[f].isSet ? '+' : '-');
    Serial.print(F("FILTER "));
    Serial.print(f);
    Serial.print(F(" : "));
    if (mFilter[f].isExtended) {
      printExtendedID(mFilter[f].filter);
    }
    else {
      printStandardID(mFilter[f].filter);
    }
    Serial.print(F(" : SIDH = "));
    printByteWithLeadingZeroes(filter(f, SIDH));
    Serial.print(F(" SIDL = "));
    printByteWithLeadingZeroes(filter(f, SIDL));
    Serial.print(F(" EID8 = "));
    printByteWithLeadingZeroes(filter(f, EID8));
    Serial.print(F(" EID0 = "));
    printByteWithLeadingZeroes(filter(f, EID0), EOL);
  }
  Serial.println(F("** Buffer 1"));
  Serial.print(mMask[1].isSet ? '+' : '-');
  Serial.print(F("MASK : "));
  printExtendedID(mMask[1].mask, EOL);
  Serial.print(F(" : SIDH = "));
  printByteWithLeadingZeroes(mask(1,SIDH));
  Serial.print(F(" SIDL = "));
  printByteWithLeadingZeroes(mask(1,SIDL));
  Serial.print(F(" EID8 = "));
  printByteWithLeadingZeroes(mask(1,EID8));
  Serial.print(F(" EID0 = "));
  printByteWithLeadingZeroes(mask(1,EID0), EOL);
  for (int f = 2; f < 6; f++) {
    Serial.print(mFilter[f].isSet ? '+' : '-');
    Serial.print(F("FILTER "));
    Serial.print(f);
    Serial.print(F(" : "));
    if (mFilter[f].isExtended) {
      printExtendedID(mFilter[f].filter);
    }
    else {
      printStandardID(mFilter[f].filter);
    }
    Serial.print(F(" : SIDH = "));
    printByteWithLeadingZeroes(filter(f, SIDH));
    Serial.print(F(" SIDL = "));
    printByteWithLeadingZeroes(filter(f, SIDL));
    Serial.print(F(" EID8 = "));
    printByteWithLeadingZeroes(filter(f, EID8));
    Serial.print(F(" EID0 = "));
    printByteWithLeadingZeroes(filter(f, EID0), EOL);
  }
}

// /*============================================================================
//  * CANDudeMessage is the class to handle both sent and receive messages
//  *----------------------------------------------------------------------------
//  */
// void CANDudeSendMessage::setStandardId(const uint16_t inId)
// {
//   sidh = (inId >> 3) & 0x00FF;
//   sidl = ((inId & mcp2515::TXBnSIDL_SID_MASK) << mcp2515::TXBnSIDL_SID_SHIFT);
// }
//
// /*----------------------------------------------------------------------------
//  */
// void CANDudeSendMessage::setExtendedId(const uint32_t inId)
// {
//   sidh = (inId >> 3) & 0x000000FF;
//   sidl = ((inId & mcp2515::TXBnSIDL_SID_MASK) << mcp2515::TXBnSIDL_SID_SHIFT) |
//          mcp2515::TXBnSIDL_EXIDE |
//          ((inId >> 27) & mcp2515::TXBnSIDL_EID_MASK);
//   eid8 = (inId >> 19) & 0x000000FF;
//   eid0 = (inId >> 11) & 0x000000FF;
// }
//
// /*----------------------------------------------------------------------------
//  */
// void CANDudeSendMessage::setRemote(const bool inRemote)
// {
//   if (inRemote) dlc |= mcp2515::TXBnDLC_RTR;
//   else dlc &= ~mcp2515::TXBnDLC_RTR;
// }
//
// /*----------------------------------------------------------------------------
//  */
// void CANDudeSendMessage::setLength(const uint8_t inLength)
// {
//   dlc &= ~mcp2515::TXBnDLC_DLC_MASK;
//   dlc |= inLength & mcp2515::TXBnDLC_DLC_MASK;
// }
//
// /*----------------------------------------------------------------------------
//  */
// uint8_t CANDudeSendMessage::sizeInBytes() const
// {
//   // 4 bytes for id and one for length + the number of bytes
//   return 5 + length();
// }
//
// /*----------------------------------------------------------------------------
//  */
// void CANDudeSendMessage::print() const
// {
//   uint32_t id = ((uint32_t)sidh) << 3 | ((sidl >> mcp2515::TXBnSIDL_SID_SHIFT) & mcp2515::TXBnSIDL_SID_MASK);
//   char type = isExtended() ? 'e' : 's';
//   if (type == 'e') {
//     id |= (((uint32_t)eid8) << 19) | (((uint32_t)eid0) << 11) | (((uint32_t)(sidl & mcp2515::TXBnSIDL_EID_MASK)) << 27) ;
//   }
//   Serial.print(type);
//   Serial.print('[');
//   Serial.print(id);
//   Serial.print('/');
//   Serial.print(id, HEX);
//   Serial.print(']');
//   if (isRemote()) {
//     Serial.println('r');
//   }
//   else {
//     for (uint8_t i = 0; i < length(); i++) {
//       Serial.print(' ');
//       Serial.print(data[i], HEX);
//     }
//     Serial.println();
//   }
// }
//
// /*----------------------------------------------------------------------------
//  */
// void CANDudeSendMessage::printRaw() const
// {
//   Serial.print("SIDH="); Serial.print(sidh, HEX);
//   Serial.print(" SIDL="); Serial.print(sidl, HEX);
//   Serial.print(" (");
//   Serial.print((sidl >> mcp2515::TXBnSIDL_SID_SHIFT) & mcp2515::TXBnSIDL_SID_MASK);
//   Serial.print(',');
//   Serial.print(isExtended() ? 'e' : 's');
//   Serial.print(',');
//   Serial.print(sidl & mcp2515::TXBnSIDL_EID_MASK);
//   Serial.print(')');
//   Serial.print(" EID8="); Serial.print(eid8, HEX);
//   Serial.print(" EID0="); Serial.print(eid0, HEX);
//   Serial.print(" DLC="); Serial.print(dlc, HEX);
//   Serial.print(" DATA=");
//   for (uint8_t i = 0; i < 8; i++) {
//     Serial.print(' ');
//     Serial.print(data[i], HEX);
//   }
//   Serial.println();
// }
//
// /*============================================================================
//  * CANDudeQueue is the class to handle queues used to send and receive
//  * messages
//  *----------------------------------------------------------------------------
//  * Constructor, init an empty queue.
//  */
// CANDudeQueue::CANDudeQueue(const uint8_t inSize) :
// mSize(0),
// mIndex(0)
// {
//   uint8_t mask = 0xFF;
//   uint8_t size = inSize > 16 ? inSize - 1 : 15;
//   while (!(size & 0x80)) {
//     mask >>= 1;
//     size <<= 1;
//   }
//   mMask = mask;
//   mQueue = new uint8_t[((uint16_t)mask) + 1];
// }
//
// /*----------------------------------------------------------------------------
//  * Push a byte in the queue if room available
//  */
// uint8_t CANDudeQueue::pushByte(const uint8_t inByte)
// {
//   if (!isFull()) {
//     mQueue[mIndex++] = inByte;
//     mSize++;
//     mIndex &= mMask;
//     return 1;
//   }
//   else {
//     return 0;
//   }
// }
//
// /*----------------------------------------------------------------------------
//  * Push bytes in the queue if room available. Returns how many bytes have
//  * been pushed.
//  */
// uint8_t CANDudeQueue::pushByte(const uint8_t * const inBytes, const uint8_t inCount)
// {
//   uint16_t freeSpace = available();
//   uint8_t howMany = inCount > freeSpace ? freeSpace : inCount;
//   for (uint8_t i = 0; i < howMany; i++) {
//     mQueue[mIndex++] = inBytes[i];
//     mIndex &= mMask;
//   }
//   mSize += howMany;
//   return howMany;
// }
//
// /*----------------------------------------------------------------------------
//  * Pop a byte from the queue if not empty
//  */
// uint8_t CANDudeQueue::popByte()
// {
//   if (mSize > 0) {
//     uint8_t readIndex = (mIndex - mSize) & mMask;
//     mSize--;
//     return mQueue[readIndex];
//   }
//   else return 0;
// }
//
// /*----------------------------------------------------------------------------
//  * Pop a bytes from the queue if available, returns how many bytes have been
//  * popped
//  */
// uint8_t CANDudeQueue::popByte(uint8_t * const outBytes, const uint8_t inCount)
// {
//   uint8_t howMany = inCount > mSize ? mSize : inCount;
//   uint8_t readIndex = (mIndex - mSize) & mMask;
//   for (uint8_t i = 0; i < howMany; i++) {
//     outBytes[i] = mQueue[readIndex++];
//     readIndex &= mMask;
//   }
//   return howMany;
// }
//
// /*----------------------------------------------------------------------------
//  * Prints the queue (for debug)
//  */
// void CANDudeQueue::print() const
// {
//   uint8_t readIndex = (mIndex - mSize) & mMask;
//   uint8_t size = mSize;
//   Serial.print('['); Serial.print(mSize); Serial.print("] ");
//   while (size-- > 0) {
//     Serial.print(mQueue[readIndex++], HEX);
//     Serial.print(' ');
//     readIndex &= mMask;
//   }
//   Serial.println();
// }
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
CANDude::CANDude(const uint8_t inSlaveSelectPin) :
mSlaveSelectPin(inSlaveSelectPin)
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
  uint8_t *     inBuffer,
  uint8_t       inNumberOfBytes)
{
  Serial.print("S: ");
  Serial.print(mcp2515::LOAD_TX_BUFFER(inBufferID), BIN);
  for (uint8_t i = 0; i < inNumberOfBytes; i++) {
    Serial.print(' ');
    Serial.print(inBuffer[i], BIN);
  }
  Serial.println();
  SPIBeginTransaction();
  select();
  SPI.transfer(mcp2515::LOAD_TX_BUFFER(inBufferID));
  while (inNumberOfBytes--) SPI.transfer(*inBuffer++);
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
    //---- Set CANCTRL.REQOP to request a new mode of operation
    modifyBit(
      mcp2515::CANCTRL,
      mcp2515::CANCTRL_REQOP_MASK << mcp2515::CANCTRL_REQOP_SHIFT,
      inMode << mcp2515::CANCTRL_REQOP_SHIFT
    );
    //---- Loop until CANSTAT.OPMOD get the same value or 1 ms elapsed
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
  unselect(); // Turn off the chip select before programming the pin as output
  pinMode(mSlaveSelectPin, OUTPUT); // Program the chip select as output
  SPI.begin();
  //---- reset the MCP2515
  reset();
  //---- Test the communication with the MCP2515 by writing and reading data
  //---- to and from byte 0 of TX buffer 0
  write(mcp2515::TXB0D0, 0xAA);
  if (read(mcp2515::TXB0D0) != 0xAA) return CANDudeSPIFailed;
  write(mcp2515::TXB0D0, 0x55);
  if (read(mcp2515::TXB0D0) != 0x55) return CANDudeSPIFailed;

  //---- Check settings to avoid configuration if settings are not ok
  if (! inSettings.configOk()) return CANDudeBadConfig;

  //---- Enter config mode
  CANDudeResult result = setMode(mcp2515::CONFIG_MODE);
  if (result == CANDudeOk) {
    //---- Program bit timing according to the configuration
    uint8_t cnf[3];
    //---- CNF register ar stored in consecutive registers from CNF3 to CNF1
    //---- that is address in reverse order compared to register number
    cnf[2] = // This is CNF1
      (inSettings.sjw() << mcp2515::CNF1_SJW_SHIFT) | // SJW
      inSettings.brp();                               // BRP
    cnf[1] = // This is CNF2
      mcp2515::CNF2_BTLMODE | // PS2 is in CNF3
      (inSettings.tripleSampling() ? mcp2515::CNF2_SAM : 0) |
      (inSettings.ps1() << mcp2515::CNF2_PHSEG1_SHIFT) | // PS1
      inSettings.ps();                                   // PS
    cnf[0] = // This is CNF3
      inSettings.ps2();                                  // PS2

    for (uint8_t i=0; i < 3; i++) {
      Serial.print("CNF");
      Serial.print(3-i);
      Serial.print(": ");
      Serial.print(cnf[i], BIN);
      Serial.print(" / ");
    }
    Serial.println();

    //---- Write the bit timing configuration
    write(mcp2515::CNF3, 3 /* 3 registers */, cnf);

    //---- Program Interrupts
    // pinMode(mInterruptPin, INPUT);
    // uint8_t interruptNumber = digitalPinToInterrupt(mInterruptPin);
    // SPI.usingInterrupt(interruptNumber);
    // attachInterrupt(interruptNumber, mInterruptFunction, FALLING);
    // //---- Enable RXB0 and TXB0 Interrupts
    // write(mcp2515::CANINTE, mcp2515::CANINTE_TX0IE | mcp2515::CANINTE_RX0IE);
    //
    // Go back to normal mode
    result = setMode(mcp2515::NORMAL_MODE);
  }
  return result;
}

// /*----------------------------------------------------------------------------
//  * Send a message
//  */
// CANDudeResult CANDude::sendMessage(const CANDudeSendMessage & inMessage)
// {
//   if (mSendCount > 0) {
//     if (mSendQueue->available() >= inMessage.sizeInBytes()) {
//       mSendQueue->pushByte(inMessage.start(), 5);
//       mSendQueue->pushByte(inMessage.dataPart(), inMessage.length());
//       return CANDudeOk;
//     }
//     else {
//       return CANDudeNoRoom;
//     }
//   }
//   else {
//     loadMessage(0, inMessage);
//   }
// }

// /*----------------------------------------------------------------------------
//  * Handle the MCP2515 interrupt
//  */
// void CANDude::handleInterrupt()
// {
// }

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

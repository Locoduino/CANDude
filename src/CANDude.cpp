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

static void printByteWithLeadingZeroes(const uint8_t inValue)
{
  uint8_t mask = B10000000;
  while (mask != 0) {
    Serial.print((mask & inValue) != 0);
    mask >>= 1;
  }
}

static inline void printByteWithLeadingZeroesLn(const uint8_t inValue)
{
  printByteWithLeadingZeroes(inValue);
  Serial.println();
}

static void printExtendedID(const uint32_t inValue)
{
  uint32_t mask = 0x10000000;
  while (mask != 0) {
    Serial.print((mask & inValue) != 0);
    mask >>= 1;
  }
}

static inline void printExtendedIDLn(const uint32_t inValue)
{
  printExtendedID(inValue);
  Serial.println();
}

static void printStandardID(const uint32_t inValue)
{
  uint32_t mask = 0x00000400;
  while (mask != 0) {
    Serial.print((mask & inValue) != 0);
    mask >>= 1;
  }
}

static inline void printStandardIDLn(const uint32_t inValue)
{
  printStandardID(inValue);
  Serial.println();
}

/*=============================================================================
 * CANDudeSettings Class
 *
 * Settings of the CAN controller. An instance of this class is passed to them
 * constructor of the CAN controller.
 *-----------------------------------------------------------------------------
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

/*=============================================================================
 * CANDudeFilters Class
 *
 * It manages a set of filters. An instance of this class is passed to the
 * constructor of the controller.
 *-----------------------------------------------------------------------------
 * Constructor.
 * Initialize the masks and the filters so that no message is accepted
 */
CANDudeFilters::CANDudeFilters()
{
  for (uint8_t m = 0; m < 2; m++) {
    mMask[m] = 0UL;
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
  if (inBuffer < 2) {
    mMask[inBuffer] = inMask;
    return true;
  }
  return false;
}

/*-------------------------------------------------------------------------------------------------
 * Set one of the filter of one of the buffers (0 or 1). Return true if ok, false otherwise
 */
bool CANDudeFilters::setFilter(const uint8_t  inFilterNum,
                               const bool     inIsExtended,
                               const uint32_t inFilter)
{
  if (inFilterNum < 6) {
    mFilter[inFilterNum].isSet = true;
    mFilter[inFilterNum].isExtended = inIsExtended;
    mFilter[inFilterNum].filter = inFilter;
    return true;
  }
  return false;
}

/*-------------------------------------------------------------------------------------------------
 * Returns one of the 4 MCP2515 registers falue for a filter or a mask. inPos should be one of the
 * following:
 * CANDudeFilters::SIDH
 * CANDudeFilters::SIDL (Here for filters the EXIDE bit is not set)
 * CANDudeFilters::EID8
 * CANDudeFilters::EID0
 */
void CANDudeFilters::maskOrFilter(
  uint32_t inMaskOrFilter,
  uint8_t * const outMF) const
{
  outMF[0] = (uint8_t) ((inMaskOrFilter >> 3) & 0xFF);
  outMF[1] = (uint8_t) (((inMaskOrFilter & mcp2515::RXFnSIDL_SID_MASK)
                       << mcp2515::RXFnSIDL_SID_SHIFT) |
                       ((inMaskOrFilter >> 27) & 0x03));
  outMF[2] = (uint8_t) ((inMaskOrFilter >> 19) & 0xFF);
  outMF[3] = (uint8_t) ((inMaskOrFilter >> 11) & 0xFF);
}

/*-----------------------------------------------------------------------------
 * Put MCP2515 registers values for a mask in table pointed by outMask
 * and returns true. Returns false if inBuffer or inPos is out of range
 */
bool CANDudeFilters::mask(
  const uint8_t inBuffer,
  uint8_t * const outMask) const
{
  if (inBuffer < 2) {
    maskOrFilter(mMask[inBuffer], outMask);
    return true;
  }
  return false;
}

/*-----------------------------------------------------------------------------
 * Put the 4 values for a filter in a table pointed to by outFilter.
 * Return true if filters are correctly set, false otherwise
 */
bool CANDudeFilters::filter(
  const uint8_t inFilter,
  uint8_t * const outFilter) const
{
  if (inFilter < 6 && mFilter[inFilter].isSet) {
    maskOrFilter(mFilter[inFilter].filter, outFilter);
    if (mFilter[inFilter].isExtended) outFilter[1] |= mcp2515::RXFnSIDL_EXIDE;
    return true;
  }
  else return false;
}

/*-----------------------------------------------------------------------------
 * Put the 4 values of all the filters of a buffer in a table pointed to by
 * outFilter. 2 x 4 = 8 bytes for buffer 0 and 4 x 4 = 16 bytes for buffer 1
 * Return true if filters are correctly set, false otherwise
 */
bool CANDudeFilters::filtersOfBuffer(
  const uint8_t inBuffer,
  uint8_t * const outFilters) const
{
  bool status = true;

  if (inBuffer < 2) {
    for (uint8_t f = inBuffer * 2, loc = 0; f < inBuffer * 4 + 2; f++, loc += 4) {
      if (mFilter[f].isSet) {
        maskOrFilter(mFilter[f].filter, outFilters + loc);
        if (mFilter[f].isExtended) outFilters[loc + 1] |= mcp2515::RXFnSIDL_EXIDE;
      }
      else status = false;
    }
  }
  return status;
}

/*-----------------------------------------------------------------------------
 * Compute the unset filters if mask is set and at least one filter is set
 * copy the set filter to the unset filters.
 *
 * Returns true if :
 * - mask is 0 or
 * - mask is not 0 and a least one filter is set
 */
bool CANDudeFilters::finalize()
{
  bool status = false;
  for (uint8_t m = 0; m < 2; m++) {
    if (mMask[m] != 0) {
      /* search for a filter that is set */
      for (uint8_t f = m * 2; f < m * 4 + 2; f++) {
        if (mFilter[f].isSet) {
          /* copy it to all unset filters */
          for (uint8_t ff = m * 2; ff < m * 4 + 2; ff++) {
            if (!mFilter[ff].isSet) {
              mFilter[ff] = mFilter[f];
            }
          }
          /* At least one filter is set and has been replicated */
          status = true;
          break;
        }
      }
    }
    else status = true;
  }
  return status;
}

/*-----------------------------------------------------------------------------
 * print the filter settings
 */
void CANDudeFilters::print() const
{
  uint8_t data[4];

  Serial.println(F("** Buffer 0"));
  Serial.print(F("MASK : "));
  printExtendedID(mMask[0]);
  if (mask(0, data)) {
    Serial.print(F(" : SIDH = "));
    printByteWithLeadingZeroes(data[0]);
    Serial.print(F(" SIDL = "));
    printByteWithLeadingZeroes(data[1]);
    Serial.print(F(" EID8 = "));
    printByteWithLeadingZeroes(data[2]);
    Serial.print(F(" EID0 = "));
    printByteWithLeadingZeroesLn(data[3]);
  }
  else Serial.println();
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
    if (filter(f, data)) {
      Serial.print(F(" : SIDH = "));
      printByteWithLeadingZeroes(data[0]);
      Serial.print(F(" SIDL = "));
      printByteWithLeadingZeroes(data[1]);
      Serial.print(F(" EID8 = "));
      printByteWithLeadingZeroes(data[2]);
      Serial.print(F(" EID0 = "));
      printByteWithLeadingZeroesLn(data[3]);
    }
    else Serial.println();
  }
  Serial.println(F("** Buffer 1"));
  Serial.print(F("MASK : "));
  printExtendedID(mMask[1]);
  if (mask(1, data)) {
    Serial.print(F(" : SIDH = "));
    printByteWithLeadingZeroes(data[0]);
    Serial.print(F(" SIDL = "));
    printByteWithLeadingZeroes(data[1]);
    Serial.print(F(" EID8 = "));
    printByteWithLeadingZeroes(data[2]);
    Serial.print(F(" EID0 = "));
    printByteWithLeadingZeroesLn(data[3]);
  }
  else Serial.println();
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
    if (filter(f, data)) {
      Serial.print(F(" : SIDH = "));
      printByteWithLeadingZeroes(data[0]);
      Serial.print(F(" SIDL = "));
      printByteWithLeadingZeroes(data[1]);
      Serial.print(F(" EID8 = "));
      printByteWithLeadingZeroes(data[2]);
      Serial.print(F(" EID0 = "));
      printByteWithLeadingZeroesLn(data[3]);
    }
    else Serial.println();
  }
}

/*-----------------------------------------------------------------------------
 * Load the filter configuration in the controller
 */
void CANDudeFilters::loadInController(CANDude * inController)
{
  uint8_t filterOrMask[16];

  /* Complete the filters that have not been set */
  finalize();

  /*
   * Load the filters for buffer 0 and set the RXM bits of RXB0CTRL register
   * so that the filters are used
   */
  if (mask(0, filterOrMask)) {
    inController->write(mcp2515::RXM0SIDH, 4, filterOrMask);
  }
  if (filtersOfBuffer(0, filterOrMask)) {
    inController->write(mcp2515::RXF0SIDH, 8, filterOrMask);
  }
  inController->modifyBit(mcp2515::RXB0CTRL, mcp2515::RXB0CTRL_RXM_MASK, 0);

  /*
   * Load the filters for buffer 1 and set the RXM bits of RXB1CTRL register
   * so that the filters are used
   */
  if (mask(1, filterOrMask)) {
    inController->write(mcp2515::RXM1SIDH, 4, filterOrMask);
  }
  if (filtersOfBuffer(1, filterOrMask)) {
    inController->write(mcp2515::RXF2SIDH, 4, filterOrMask);
    inController->write(mcp2515::RXF3SIDH, 12, filterOrMask);
  }
  inController->modifyBit(mcp2515::RXB1CTRL, mcp2515::RXB1CTRL_RXM_MASK, 0);
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
 * Load a TX buffer with the header of the message by
 * sending a LOAD TX BUFFER command.
 * inBufferId is the identifier of the TX buffer.
 * It can be mcp2515::TX_BUFFER_0, mcp2515::TX_BUFFER_1 or
 * mcp2515::TX_BUFFER_2.
 * inHeader is a pointer to a 4 bytes buffer containing the identifier
 * inBuffer is a pointer from where the message will be read.
 * inNumberOfbytes is the number of bytes to write to the message.
 * Size of the buffer should be at least numberOfbytes
 */
void CANDude::loadMessage(
  const uint8_t inBufferID,
  uint8_t *inHeader,
  const uint8_t inDLC,
  uint8_t *inBuffer)
{
  const uint8_t dataLength = inDLC & 0x0F;
  const bool anyData = ((inDLC & mcp2515::TXBnDLC_RTR) != 0) && (dataLength > 0);

  // Serial.print("S: ");
  // Serial.print(mcp2515::LOAD_TX_BUFFER(inBufferID), BIN);
  // for (uint8_t i = 0; i < inNumberOfBytes; i++) {
  //   Serial.print(' ');
  //   Serial.print(inBuffer[i], BIN);
  // }
  // Serial.println();

  SPIBeginTransaction();
  select();
  SPI.transfer(mcp2515::LOAD_TX_BUFFER(inBufferID));
  SPI.transfer(inHeader, 4);
  SPI.transfer(inDLC);
  if (anyData) SPI.transfer(inBuffer, dataLength);
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
CANDudeResult CANDude::begin(
  const CANDudeSettings & inSettings,
  CANDudeFilters & inFilters)
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
    //---- Program the filters according to their configuration
    Serial.println('*');
    dumpRegisters();
    Serial.println('*');

    inFilters.loadInController(this);

    Serial.println('*');
    dumpRegisters();
    Serial.println('*');

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

/*-----------------------------------------------------------------------------
 * Print the registers of the 2515
 */
void CANDude::dumpRegisters()
{
  for (uint8_t low = 0 ; low < 16 ; low++) {
    for (uint8_t high = 0; high < 8 ; high++) {
      uint8_t address = high << 4 | low ;
      uint8_t reg = read(address);
      for (uint8_t b = 7; b > 0; b--) {
        if ((reg & (1 << b)) == 0) Serial.print('0');
        else break;
      }
      Serial.print(reg, BIN);
      Serial.print(' ');
    }
    Serial.println();
  }
}


CANDudeResult CANDudeController::sendMessage(
  const AbstractCANSendMessage *inMessage,
  const uint8_t *inData,
  const uint8_t inLength)
{
  loadMessage(inMessage->transmitBuffer(), );
}

AbstractCANSendMessage::AbstractCANSendMessage(
  CANDudeController * const inController) :
mSidh(0),
mSidl(0),
mEid8(0),
mEid0(0),
mController(inController)
{
}

/*
 * Set the standard Id of the send message. As a result the frame
 * is a standard one.
 */
CANDudeResult AbstractCANSendMessage::setStandardId(const uint16_t inId)
{
  mSidh = (inId >> mcp2515::TXBnSIDH_SID_SHIFT) & 0xFF;
  /* while setting the SIDL, preserve the unused bits which contain the buffer */
  mSidl = ((inId & mcp2515::TXBnSIDL_SID_MASK) << mcp2515::TXBnSIDL_SID_SHIFT)
            | (mSidl & mcp2515::TXBnSIDL_UNUSED_MASK);
  /* EXIDE bit is set to 0, so that the frame is a standard one */
  return (inId & ~((1UL << 11) - 1)) ? CANDudeOutOfRange : CANDudeOk;
}

/*
 * Set the standard Id of the send message. As a result the frame
 * is an extended one.
 */
CANDudeResult AbstractCANSendMessage::setExtendedId(const uint32_t inId)
{
  setStandardId((uint16_t)inId);
  mSidl |= mcp2515::TXBnSIDL_EXIDE; /* it's and extended id */
  mSidl |= (inId >> 27) & mcp2515::TXBnSIDL_EID_MASK;
  mEid8 = (inId >> 19) & 0xFF;
  mEid0 = (inId >> 11) & 0xFF;
  return (inId & ~((1UL << 29) - 1)) ? CANDudeOutOfRange : CANDudeOk;
}

/*
 * Set the transmit buffer the message uses. inBuffer can range from 0 to 3
 * When 0 to 2, it is the transmit buffers
 * When 3, it means any buffer.
 * For sake of memory footprint unused 2 bits in SIDL are used to set
 * the buffer for the message.
 */
CANDudeResult AbstractCANSendMessage::setTransmitBuffer(const uint8_t inBuffer)
{
  mSidl &= ~mcp2515::TXBnSIDL_UNUSED_MASK;
  mSidl |= (inBuffer & 0x2) << (mcp2515::TXBnSIDL_UNUSED_SHIFT_1 - 1);
  mSidl |= (inBuffer & 0x1) << mcp2515::TXBnSIDL_UNUSED_SHIFT_2;
  return (inBuffer & ~0x03) ? CANDudeOutOfRange : CANDudeOk;
}

uint8_t AbstractCANSendMessage::transmitBuffer() const
{
  return ((mSidl & mcp2515::TXBnSIDL_UNUSED_MASK_1) >>
            (mcp2515::TXBnSIDL_UNUSED_SHIFT_1 - 1)) |
         ((mSidl & mcp2515::TXBnSIDL_UNUSED_MASK_2) >>
            mcp2515::TXBnSIDL_UNUSED_SHIFT_2);
}

/*
 * Send a data message.
 * Arguments are:
 * - inData: a pointer to a byte array containing the bytes to send
 * - inLength: the number of bytes
 */
CANDudeResult CANSendMessage::send(const uint8_t *inData,
                                   const uint8_t inLength)
{
  return mController->sendMessage(this, inData, inLength);
}

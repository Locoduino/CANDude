/*=============================================================================
 * CANDudeSettings.h
 *-----------------------------------------------------------------------------
 * Class to manage a MCP 2515 Can controller settings
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

#include <CANDudeSettings.h>
#include <MCP2515Definitions.h>
#include <HardwareSerial.h>

static const uint32_t kMaxBaudRateForTripleSampling = 125000;

/*----------------------------------------------------------------------------
 * Constructor. Does computation of bit timing
 */
CANDudeSettings::CANDudeSettings(const uint32_t inCANCrystal,
                                 const uint32_t inWishedBaudRate,
                                 const uint32_t inMaxPPMError) :
mConfigOk(false),
mWishedBaudRate(inWishedBaudRate),
mCANCrystal(inCANCrystal)
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
      const uint32_t error = inWishedBaudRate - (CANClock / TQCount / (BRP + 1));
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


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

#ifndef __CANDUDESETTINGS_H__
#define __CANDUDESETTINGS_H__

#include <Arduino.h>

/*-----------------------------------------------------------------------------
 * CANDudeSettings class handle a setting of the CAN bus
 *
 * Initialization of CANDude starts with the instantiation of a CANDudeSettings
 * object. The constructor takes at least 2 arguments:
 * 1) the crystal frequency of the MCP2515
 * 2) the wished baudrate of the CAN bus
 * From that, a configuration of the bit timing is done
 */
class CANDudeSettings
{	
  public : CANDudeSettings(const uint32_t inCANCrystal,
                           const uint32_t inWishedBaudRate,
                           const uint32_t inMaxPPMError = 600);

  /*
   * Wished baud rate in byte/s as specified in the constructor
   */
  private : uint32_t mWishedBaudRate;
  public : uint32_t baudRate() const { return mWishedBaudRate; }
  
  /*
   * Frequency in Hertz of the MCP2515 crystal
   */
  private : uint32_t mCANCrystal;
  public : uint32_t CANCrystal() const { return mCANCrystal; }
  
  /*
   * At start mConfigOk is set to false. If the configuration is doable
   * it is set to true
   */
  private : bool mConfigOk;
  public : bool configOk() const { return mConfigOk; }
  
  /*
   * Baud Rate Prescaler. Ranges from 0 to 63. Actual Value is mBRP+1
   */
  private : uint8_t mBRP;
  public : uint8_t brp() const { return mBRP; }
  
  /*
   * Propagation Segment. Ranges from 0 to 7. Actual value is mPS + 1
   */
  private : uint8_t mPS;
  public : uint8_t ps() const { return mPS; }
  
  /*
   * Phase Segment 1. Ranges from 0 to 7. Actual value is mPS1 + 1
   */
  private : uint8_t mPS1;
  public : uint8_t ps1() const { return mPS1; }
  
  /*
   * Phase Segment 2. Ranges from 1 to 7. Actual value is mPS2 + 1
   */
  private : uint8_t mPS2;
  public : uint8_t ps2() const { return mPS2; }
  
  /*
   * Sync Jump Width. Ranges from 0 to 3. Actual value is mSJW + 1
   */
  private : uint8_t mSJW;  
  public : uint8_t sjw() const { return mSJW; }
  
  /*
   * actualBaudRate computes the actual baud rate of the configuration that
   * may be different from the wished baudrate if the latter is no possible
   */
  public : uint32_t actualBaudRate() const;

  /*
   * absoluteError returns the difference between the actual baud rate and the
   * wished baud rate.
   */
  public : int32_t absoluteError() const { return actualBaudRate() - mWishedBaudRate; }

  /*
   * samplePoint returns the position of the sample point multiplied by 1000
   */
  public : uint16_t samplePoint() const;

  /*
   * PPMError returns the relative baud rate error expressed in parts per million
   */
  public : uint32_t PPMError() const;

  /*
   * timeQuantaCount returns the number of time quantum (TQ) in a bit
   */
  public : uint8_t timeQuantaCount()const;

  /*
   * tripleSampling returns true if triple sampling is used in the computed
   * configuration
   */
  public : bool tripleSampling() const { return timeQuantaCount() > 15; }

  /*
   * print the configuration for debug purpose
   */
  public : void print() const;
};

#endif

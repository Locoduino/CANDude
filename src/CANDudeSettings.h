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

  private : uint32_t mWishedBaudRate;   // asked baud rate in byte/s
  private : uint32_t mCANCrystal; // MCP2515 crystal frequency
  private : bool mConfigOk; // true is the computed configuration is doable
  private : uint8_t mBRP;   // 0..63, actual value of Baud Rate Prescaler is mBRP+1
  private : uint8_t mPS;    // 0..7, actual value of Propagation Segment is mPS+1
  private : uint8_t mPS1;   // 0..7, actual value of Phase Segment 1 is mPS1+1
  private : uint8_t mPS2;   // 1..7, actual value of Phase Segment 2 is mPS2+1
  private : uint8_t mSJW;   // 0..3, actual value of Sync Jump Width is mSJW+1
  private : uint8_t mSendQueueSize;    // size of sending queue in bytes
  private : uint8_t mReceiveQueueSize; // size of receiving queue in bytes

  public : uint32_t baudRate() const { return mBaudRate; }
  public : uint32_t CANCrystal() const { return mCANCrystal; }
  public : uint8_t brp() const { return mBRP; }
  public : uint8_t ps() const { return mPS; }
  public : uint8_t ps1() const { return mPS1; }
  public : uint8_t ps2() const { return mPS2; }
  public : uint8_t sjw() const { return mSJW; }
  public : uint32_t actualBaudRate() const;
  public : bool configOk() const { return mConfigOk; }
  public : uint32_t absoluteError() const { return actualBaudRate() - mBaudRate; }
  public : uint16_t samplePoint() const;
  public : uint32_t PPMError() const;
  public : uint8_t timeQuantaCount()const;
  public : bool tripleSampling() const { return timeQuantaCount() > 15; }
  public : void print() const;
  // Size of receive and send queues
  public : uint8_t sendQueueSize() const { return mSendQueueSize; }
  public : uint8_t receiveQueueSize() const { return mReceiveQueueSize; }
  public : void setSendQueueSize(const uint8_t inSize) { mSendQueueSize = inSize > 250 ? 250 : inSize; }
  public : void setReceiveQueueSize(const uint8_t inSize) { mReceiveQueueSize = inSize > 250 ? 250 : inSize; }
};

#endif

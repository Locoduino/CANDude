/*=============================================================================
 * CANDude.h
 *-----------------------------------------------------------------------------
 * Classes to manage a MCP 2515 Can controller
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

#ifndef __CANDUDE_H__
#define __CANDUDE_H__

#include <Arduino.h>
#include <MCP2515Definitions.h>

class CANDude;

/*=============================================================================
 * CANDudeSettings class handles a setting of the CAN bus
 *
 * Initialization of CANDude starts with the instantiation of a CANDudeSettings
 * object. The constructor takes at least 2 arguments:
 * 1) the crystal frequency of the MCP2515
 * 2) the wished baudrate of the CAN bus
 * 3) the optional maximum PPM to accept a configuration. 600 is the default
 *    value
 * From that, a configuration of the bit timing is done
 */
class CANDudeSettings
{
private:
  /* Wished baud rate in bits/s as specified in the constructor              */
  uint32_t  mWishedBaudRate;
  /* Frequency in Hertz of the MCP2515 crystal                               */
  uint32_t  mCANCrystal;
  /* At start mConfigOk is set to false. If the configuration is doable,
     it is set to true                                                       */
  bool      mConfigOk;
  /* Baud Rate Prescaler. Ranges from 0 to 63. Actual Value is mBRP+1        */
  uint8_t   mBRP;
  /* Propagation Segment. Ranges from 0 to 7. Actual value is mPS+1          */
  uint8_t   mPS;
  /* Phase Segment 1. Ranges from 0 to 7. Actual value is mPS1+1             */
  uint8_t   mPS1;
  /* Phase Segment 2. Ranges from 1 to 7. Actual value is mPS2+1             */
  uint8_t   mPS2;
  /* Sync Jump Width. Ranges from 0 to 3. Actual value is mSJW+1             */
  uint8_t   mSJW;
  /* Triple sampling                                                         */
  bool      mTripleSampling;

public:
  /*
   * constructor
   */
  CANDudeSettings(const uint32_t inCANCrystal,
                  const uint32_t inWishedBaudRate,
                  const uint32_t inMaxPPMError = 600);
  /*
   * Accessors
   */
  uint32_t baudRate() const   { return mWishedBaudRate; }
  uint32_t CANCrystal() const { return mCANCrystal;     }
  bool configOk() const       { return mConfigOk;       }
  uint8_t brp() const         { return mBRP;            }
  uint8_t ps() const          { return mPS;             }
  uint8_t ps1() const         { return mPS1;            }
  uint8_t ps2() const         { return mPS2;            }
  uint8_t sjw() const         { return mSJW;            }
  bool tripleSampling() const { return mTripleSampling; }

  /*
   * actualBaudRate computes the actual baud rate of the configuration that
   * may be different from the wished baudrate if the latter is no possible
   */
  public : uint32_t actualBaudRate() const;

  /*
   * absoluteError returns the difference between the actual baud rate and the
   * wished baud rate.
   */
  public : int32_t absoluteError() const {
    return actualBaudRate() - mWishedBaudRate;
  }

  /*
   * samplePoint returns the position of the sample point multiplied by 1000
   */
  public : uint16_t samplePoint() const;

  /*
   * PPMError returns the relative baud rate error
   * expressed in parts per million
   */
  public : uint32_t PPMError() const;

  /*
   * timeQuantaCount returns the number of time quantum (TQ) in a bit
   */
  public : uint8_t timeQuantaCount() const;

  /*
   * print the configuration for debug purpose
   */
  public : void print() const;
};

/*=============================================================================
 * CANDudeFilters class handles the filters and the masks
 */
class CANDudeFilters {

private:
  typedef struct {
    bool     isSet : 1;
    bool     isExtended : 1;
    uint32_t filter : 30;
  } Filter_t;

  typedef uint32_t Mask_t;

  Mask_t   mMask[2];
  Filter_t mFilter[6];

public:
  /*
   * Constructor. Init both masks to 0 and all filters to unset.
   */
  CANDudeFilters();

  /*
   * Set the mask of a buffer
   * inBuffer is the id of the buffer (0 or 1), inMask is the mask to set.
   * Return true if successful (inBuffer in range), false otherwise.
   */
  bool setMask(const uint8_t inBuffer, const uint32_t inMask);

  /*
   * Set a filter. inFilterNum is the id of the filter (from 0 to 5),
   * inIsExtended is true if the filter id for extended frames, false if the
   * filter is for standard frames. inFilter is the filter value.
   * Return true if successful (inFilterNum in range), false otherwise.
   */
  bool setFilter(const uint8_t  inFilterNum,
                 const bool     inIsExtended,
                 const uint32_t inFilter);

  /*
   * Get the mask corresponding to a buffer as an array of 4 bytes encoded
   * as expected by the MCP2515. inBuffer is the id of the buffer (0 or 1),
   * outMask is a pointer to an array of at least 4 bytes.
   * Return true if inBuffer is in range 0-1, false otherwise.
   */
  bool mask(const uint8_t inBuffer, uint8_t * const outMask) const;

  /*
   * Get a filter as an array of 4 bytes encoded as expected by the MCP2515
   * inFilter is the id of the filter (0 to 5), outFilter is a pointer to
   * an array of at least 4 bytes.
   * Return true if successful (inFilter is range and the filter has been set),
    * false otherwise.
   */
  bool filter(const uint8_t inFilter, uint8_t * const outFilter) const;

  /*
   * Get all filters corresponding to a buffer as an array of the number of
   * filters x 4 bytes encoded as expected by the MCP2515.
   * inBuffer is the id of the buffer (0 or 1), outFilters is a pointer to
   * an array of at least 8 bytes if inBuffer is 0 and 16 bytes if inBuffer
   * is 1.
   * Return true if successful (inBuffer is range and the filters have been
   * set), false otherwise.
   */
  bool filtersOfBuffer(const uint8_t inBuffer, uint8_t * const outFilters) const;

  bool finalize();
  void print() const;
  void loadInController(CANDude * inController);

private:
  void maskOrFilter(uint32_t inMaskOrFilter, uint8_t * const outMF) const;
};

/*-----------------------------------------------------------------------------
 * function pointer for interrupt handlers
 */
typedef  void (*CANDudeInterrupt)();

// /*-----------------------------------------------------------------------------
//  * Macro to intantiante a controller as long as its interrupt handler
//  * and links them together.
//  * _controller is the name of the controller object
//  * _CSPin is the chip select pin to communicate with the MCP2515
//  * _interruptPin is the external interrupt pin use to notify the MCU something
//  * happened on the MCP2515
//  */
// #define CANDudeController(_controller, _CSPin, _interruptPin) \
// void _controller ## InterruptFunction(); \
// CANDude _controller(_CSPin, _interruptPin, _controller ## InterruptFunction); \
// void _controller ## InterruptFunction() \
// { \
//   _controller.handleInterrupt();\
// }

typedef enum {
  CANDudeOk,
  CANDudeSPIFailed,
  CANDudeBadConfig,
  CANDudeBadFilterConfig,
  CANDudeBadMode,
  CANDudeSetModeTimeout,
	CANDudeOutOfMemory,
	CANDudeNoRoom,
	CANDudeWrongInterruptPin,
  CANDudeOutOfRange,
  CANDudeControllerNotStarted
} CANDudeResult ;

class CANDude;

/*-------------------------------------------------------------------------------------------------
 * CANDudeMessage is the class to store a CAN send message
 */
// class CANDudeSendMessage {
//   public : uint8_t sidh; 		// bits 10-3 of standard identifier
//   public : uint8_t sidl;		// bits 2-0 of sid, EXIDE and bits 17 and 16 of eid
//   public : uint8_t eid8;		// bits 15-8 of extended identifier
//   public : uint8_t eid0;  	// bits 7-0 of extended identifier
//   public : uint8_t dlc;			// data length and remote frame bit
//   public : uint8_t data[8];	// data
//
//   public : bool isExtended() const { return (sidl & mcp2515::TXBnSIDL_EXIDE) != 0; }
//   public : bool isRemote() const { return (dlc & mcp2515::TXBnDLC_RTR) != 0; }
//   public : void setStandardId(const uint16_t inId);
//   public : void setExtendedId(const uint32_t inId);
//   public : void setRemote(const bool inRemote);
//   public : uint8_t length() const { return dlc & mcp2515::TXBnDLC_DLC_MASK; }
//   public : void setLength(const uint8_t inLength);
//   public : uint8_t sizeInBytes() const;
//   private : const uint8_t * start() const { return &sidh; }
//   private : const uint8_t * dataPart() const { return data; }
//
//   public : void print() const;
//   public : void printRaw() const;
//
//   friend class CANDude;
// };

/*-------------------------------------------------------------------------------------------------
 *
 */
// class CANDudeQueue {
//   private : uint8_t *mQueue;
//   private : uint16_t mSize;
//   private : uint8_t mIndex;
//   private : uint8_t mMask;
//   private : uint8_t mLockMask;
//   private : uint8_t mSavedMask;
//
//   public : CANDudeQueue(const uint8_t inSize);
//   public : uint8_t pushByte(const uint8_t inByte);
//   public : uint8_t pushByte(const uint8_t * const inBytes, const uint8_t inCount);
//   public : uint8_t popByte();
//   public : uint8_t popByte(uint8_t * const outBytes, const uint8_t inCount);
//   public : bool isEmpty() const { return mSize == 0; }
//   public : bool isFull() const { return mSize == ((uint16_t)mMask) + 1; }
//   public : uint16_t available() const { return mMask - mSize + 1; }
//   public : bool allocFailed() const { return mQueue == NULL; }
//   public : void print() const;
// };

/*-----------------------------------------------------------------------------
 * CANDude is the base class to talk with a MCP2515
 */
class CANDude
{
private:
  uint8_t mSlaveSelectPin;

protected:
  void select()   { digitalWrite(mSlaveSelectPin, LOW);  }
  void unselect() { digitalWrite(mSlaveSelectPin, HIGH); }


  // flag indicating how many send message are pending in the MCP2515
  // it is incremented when a message is loaded in the MCP2515
  // it is decremented for each TX flag set when an interrupt occurs

public:

  // Constructor, init the slave select pin
  CANDude(const uint8_t inSlaveSelectPin);

  // Reset the MCP2515 by sending a reset command
  void reset();

  /*
   * Read a register of the MCP2515 by sending a READ command
   * returns the value of the register
   */
  uint8_t read(const uint8_t inAddress);

  /*
   * Write a register of the MCP2515 by sending a WRITE command
   */
  void write(const uint8_t inAddress, const uint8_t inData);

	/*
	 * Read registers in sequential order by sending a READ command.
	 * startAddress is the address of the register where reading starts.
	 * numberOfRegisters is the number of registers to read.
	 * buffer is a pointer to where the read value will be stored.
	 * Size of the storage should be at least numberOfRegisters.
	 */
	void read(const uint8_t inStartAddress,
            uint16_t inNumberOfRegisters,
            uint8_t *outBuffer);

	/*
	 * Read a RX buffer without the header of the message by
	 * sending a READ RX BUFFER command.
	 * bufferID is the identifier of the RX buffer.
	 * It can be mcp2515::RX_BUFFER_0 or mcp2515::RX_BUFFER_1.
	 * numberOfuint8_t is the number of uint8_ts to read from the message.
	 * buffer is a pointer to where the message will be stored.
	 * Size of the storage should be at least numberOfuint8_t
	 */
	void readRawMessage(const uint8_t inBufferID,
                      uint8_t inNumberOfBytes,
                      uint8_t *outBuffer);

	/*
	 * Write registers in sequential order by sending a WRITE command.
	 * startAddress is the address of the register where writing starts.
	 * numberOfRegisters is the number of registers to write.
	 * buffer is a pointer from where the value to write will be stored.
	 * Size of the storage should be at least numberOfRegisters.
	 */
	void write(const uint8_t inStartAddress,
             uint16_t inNumberOfRegisters,
             uint8_t *inBuffer);

	/*
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
	void loadMessage(const uint8_t inBufferID,
                   uint8_t *inHeader,
                   const uint8_t inDLC,
                   uint8_t *inBuffer = NULL);

	/*
	 * Request to send one or more TX buffer(s)
	 * query may be mcp2515::RTS_TXB0, mcp2515::RTS_TXB1, mcp2515::RTS_TXB2 or
	 * a combination (|) of them.
	 */
	void requestToSend(const uint8_t inQuery);

	/*
	 * Read status instruction. The value returned is as follow :
	 * bit 7: interrupt flag of TX buffer 2
	 * bit 6: rts bit of TX buffer 2
	 * bit 5: interrupt flag of TX buffer 1
	 * bit 4: rts bit of TX buffer 1
	 * bit 3: interrupt flag of TX buffer 0
	 * bit 2: rts bit of TX buffer 0
	 * bit 1: interrupt flag of RX buffer 1
	 * bit 0: interrupt flag of RX buffer 0
	 */
	uint8_t readStatus();

	/*
	 * Read RX status instruction
	 * bits 7 and 6:
	 *    00  no RX message
	 *    01  RX message in buffer 0
	 *    10  RX message in buffer 1
	 *    11  RX messages in both buffers
	 * bits 4 and 3 (RX buffer 0 status if both buffers are full):
	 *    00  standard data frame
	 *    01  standard remote frame
	 *    10  extended data frame
	 *    11  extended remote frame
	 * bits 2 to 0:
	 *    000 RXF0 filter match
	 *    001 RXF1 filter match
	 *    010 RXF2 filter match
	 *    011 RXF3 filter match
	 *    100 RXF4 filter match
	 *    101 RXF5 filter match
	 *    110 RXF0 (rollover to RXB1) filter match
	 *    111 RXF1 (rollover to RXB1) filter match
	 */
	uint8_t RXStatus();

	/*
	 * Bit modify instruction.
	 * address is the address of the register to modify
	 * mask is the mask applied to select which bits to modify
	 * data is the bits to set or reset according to the mask.
	 * basically: reg <- (reg & ~mask) | (data & mask)
	 */
	void modifyBit(const uint8_t inAddress,
                 const uint8_t inMask,
                 const uint8_t inData);

	/*
	 * setMode method.
	 * This method change the mode of operation of the MCP2515. When reset, the
	 * controller is in the config mode and both CANCTRL.REQOP and
	 * CANSTAT.OPMOD contains value 100 (configuration mode).
	 * The argument should be one of the following:
	 *    mcp2515::NORMAL_MODE
	 *    mcp2515::SLEEP_MODE
	 *    mcp2515::LOOP_MODE
	 *    mcp2515::LISTEN_MODE
	 *    mcp2515::CONFIG_MODE
	 */
	CANDudeResult setMode(const uint8_t inMode);

	/*
	 * Start the Can bus
	 */
	CANDudeResult begin(const CANDudeSettings & inSettings,
                      CANDudeFilters & inFilters);

  /*
   * Print the registers of the 2515
   */
  void dumpRegisters();
	/*
	 * Send a message
	 */
//	CANDudeResult sendMessage(const CANDudeSendMessage & inMessage);

	/*
	 * Handle the MCP2515 interrupt
	 */
	void handleInterrupt();
};

class CANDudeController;

class AbstractCANSendMessage
{
protected:
  uint8_t mSidh;  /* bits 10-3 of standard identifier                       */
  uint8_t mSidl;  /* bits 2-0 of sid, EXIDE and bits 17 and 16 of eid       */
  uint8_t mEid8;  /* bits 15-8 of extended identifier                       */
  uint8_t mEid0;  /* bits 7-0 od extended identifiers                       */
  CANDudeController *mController; /* A pointer to the CAN controller object */
public:
  AbstractCANSendMessage(CANDudeController * const inController);
  CANDudeResult setStandardId(const uint16_t inId);
  uint16_t standardId() const;
  CANDudeResult setExtendedId(const uint32_t inId);
  uint16_t extendedId() const;
  CANDudeResult setTransmitBuffer(const uint8_t inBuffer);
  uint8_t transmitBuffer() const;
  virtual CANDudeResult send(const uint8_t *inData, const uint8_t inLength) = 0;
};

class CANDudeController : public CANDude
{
public:
  CANDudeController(const uint8_t inSlaveSelectPin) :
    CANDude(inSlaveSelectPin)
  {}
  CANDudeResult sendMessage(const AbstractCANSendMessage *inMessage,
                            const uint8_t *inData,
                            const uint8_t inLength);
};

class CANSendMessage : public AbstractCANSendMessage
{
public:
  CANSendMessage(CANDudeController * const inController) :
    AbstractCANSendMessage(inController) {}
  virtual CANDudeResult send(const uint8_t *inData,
                             const uint8_t inLength);
};

#endif

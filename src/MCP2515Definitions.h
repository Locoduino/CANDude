/*============================================================================
 * MCP2515Definitions.h
 *----------------------------------------------------------------------------
 * Register Definition for MCP 2515 CAN controller
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

#ifndef __MCP2515DEFINITIONS_H__
#define __MCP2515DEFINITIONS_H__

#include <Arduino.h>

namespace mcp2515 {

/*============================================================================
 * Maximum clock frequency of the SPI on MCP2515
 */
static const uint32_t SPI_CLOCK       = 10000000;

/*============================================================================
 * MCP2515 crystal frequencies. Internal clock frequency is the crytsal
 * frequency divided by 2
 */
static const uint32_t CRYSTAL_16MHZ   = 16000000;
static const uint32_t CRYSTAL_8MHZ    = 8000000;
static const uint32_t CRYSTAL_MAX     = 40000000;
static const uint32_t CRYSTAL_MIN     = 1000000;

/*============================================================================
 * Configuration registers
 *----------------------------------------------------------------------------
 * CNF1 – CONFIGURATION 1 (ADDRESS: 2Ah).See page 44 of the datasheet
 */
static const uint8_t CNF1             = 0x2A;

/*
 *      7      6      5      4      3      2      1      0
 *    R/W-0  R/W-0  R/W-0  R/W-0  R/W-0  R/W-0  R/W-0  R/W-0
 *	+------+------+------+------+------+------+------+------+
 *  | SJW1 | SJW0 | BRP5 | BRP4 | BRP3 | BRP2 | BRP1 | BRP0 |
 *  +------+------+------+------+------+------+------+------+
 */

/*
 * SJW<1:0>: Synchronization Jump Width Length bits
 * 11 : Length = 4xTQ
 * 10 : Length = 3xTQ
 * 01 : Length = 2xTQ
 * 00 : Length = 1xTQ
 */
static const uint8_t CNF1_SJW_SHIFT   = 6;
static const uint8_t CNF1_SJW_MASK    = 0x03; /* shift then mask */

/*
 * BRP<5:0>: Baud Rate Prescaler bits
 * TQ = 2 x (BRP + 1)/FOSC
 */
static const uint8_t CNF1_BRP_MASK    = 0x3F;

/*----------------------------------------------------------------------------
 * CNF2 – CONFIGURATION 2 (ADDRESS: 29h).See page 44 of the datasheet
 */
static const uint8_t CNF2             = 0x29;

/*
 *       7       6       5         4         3         2        1        0
 *     R/W-0   R/W-0   R/W-0     R/W-0     R/W-0     R/W-0    R/W-0    R/W-0
 *	+---------+-----+---------+---------+---------+--------+--------+--------+
 *  | BTLMODE | SAM | PHSEG12 | PHSEG11 | PHSEG10 | PRSEG2 | PRSEG1 | PRSEG0 |
 *  +---------+-----+---------+---------+---------+--------+--------+--------+
 */

/*
 * BTLMODE: PS2 Bit Time Length bit (bit 7)
 * 1 : Length of PS2 determined by PHSEG22:PHSEG20 bits of CNF3
 * 0 : Length of PS2 is the greater of PS1 and IPT (2 TQ)
 */
static const uint8_t CNF2_BTLMODE     = 1 << 7;

/*
 * SAM: Sample Point Configuration bit (bit 6)
 * 1 : Bus line is sampled three times at the sample point
 * 0 : Bus line is sampled once at the sample point
 */
static const uint8_t CNF2_SAM         = 1 << 6;

/*
 * PHSEG1<2:0>: PS1 Length bits (bits 5 to 3)
 * (PHSEG1 + 1) x TQ
 */
static const uint8_t CNF2_PHSEG1_SHIFT = 3;
static const uint8_t CNF2_PHSEG1_MASK  = 0x07; /* shift then mask */

/*
 * PRSEG<2:0>: Propagation Segment Length bits (bits 2 to 0)
 * (PRSEG + 1) x TQ
 */
static const uint8_t CNF2_PRSEG_MASK   = 0x07;

/*----------------------------------------------------------------------------
 * CNF3 – CONFIGURATION 3 (ADDRESS: 28h).See page 45 of the datasheet
 */
static const uint8_t CNF3             = 0x28;

/*
 *     7       6      5     4     3       2         1         0
 *   R/W-0   R/W-0   U-0   U-0   U-0    R/W-0     R/W-0     R/W-0
 *	+-----+--------+-----+-----+-----+---------+---------+---------+
 *  | SOF | WAKFIL |  -  |  -  |  -  | PHSEG22 | PHSEG21 | PHSEG20 |
 *  +-----+--------+-----+-----+-----+---------+---------+---------+
 */

/*
 * SOF: Start-of-Frame signal bit (bit 7)
 * If CANCTRL.CLKEN = 1:
 * 1 : CLKOUT pin enabled for SOF signal
 * 0 : CLKOUT pin enabled for clockout function
 * If CANCTRL.CLKEN = 0, Bit is don’t care.
 */
static const uint8_t CNF3_SOF         = 1 << 7;

/*
 * WAKFIL: Wake-up Filter bit (bit 6)
 * 1 : Wake-up filter enabled
 * 0 : Wake-up filter disabled
 */
static const uint8_t CNF3_WAKFIL      = 1 << 6;

/*
 * PHSEG2<2:0>: PS2 Length bits (bits 2 to 0)
 * (PHSEG2 + 1) x TQ
 * Minimum valid setting for PS2 is 2 TQ
 */
static const uint8_t CNF3_PHSEG2_MASK = 0x07;

/*============================================================================
 * Error handling registers
 *----------------------------------------------------------------------------
 * TEC - TRANSMIT ERROR COUNTER (ADDRESS: 1Ch). See page 49 of the datasheet
 */
static const uint8_t TEC               = 0x1C;

/*----------------------------------------------------------------------------
 * REC - RECEIVER ERROR COUNTER (ADDRESS: 1Dh). See page 49 of the datasheet
 */
static const uint8_t REC               = 0x1D;

/*----------------------------------------------------------------------------
 * EFLG – ERROR FLAG (ADDRESS: 2Dh). See page 50 of the datasheet
 */
static const uint8_t EFLG              = 0x2D;

/*
 *       7        6       5      4      3      2       1       0
 *     R/W-0   R/W-0     R-0    R-0    R-0    R-0     R-0     R-0
 *	+--------+--------+------+------+------+-------+-------+-------+
 *  | RX1OVR | RX0OVR | TXB0 | TXEP | RXEP | TXWAR | RXWAR | EWARN |
 *	+--------+--------+------+------+------+-------+-------+-------+
 */

/*
 * RX1OVR: Receive Buffer 1 Overflow Flag bit (bit 7)
 * - Set when a valid message is received for RXB1 and CANINTF.RX1IF = 1
 * - Must be reset by MCU
 */
static const uint8_t EFLG_RX1OVR       = 1 << 7;

/*
 * RX0OVR: Receive Buffer 0 Overflow Flag bit (bit 6)
 * - Set when a valid message is received for RXB0 and CANINTF.RX0IF = 1
 * - Must be reset by MCU
 */
static const uint8_t EFLG_RX0OVR       = 1 << 6;

/*
 * TXBO: Bus-Off Error Flag bit (bit 5)
 * - Bit set when TEC reaches 255
 * - Reset after a successful bus recovery sequence
 */
static const uint8_t EFLG_TXB0         = 1 << 5;

/*
 * TXEP: Transmit Error-Passive Flag bit (bit 4)
 * - Set when TEC is equal to or greater than 128
 * - Reset when TEC is less than 128
 */
static const uint8_t EFLG_TXEP         = 1 << 4;

/*
 * RXEP: Receive Error-Passive Flag bit (bit 3)
 * - Set when REC is equal to or greater than 128
 * - Reset when REC is less than 128
 */
static const uint8_t EFLG_RXEP         = 1 << 3;

/*
 * TXWAR: Transmit Error Warning Flag bit (bit 2)
 * - Set when TEC is equal to or greater than 96
 * - Reset when TEC is less than 96
 */
static const uint8_t EFLG_TXWAR        = 1 << 2;

/*
 * RXWAR: Receive Error Warning Flag bit (bit 1)
 * - Set when REC is equal to or greater than 96
 * - Reset when REC is less than 96
 */
static const uint8_t EFLG_RXWAR        = 1 << 1;

/*
 * EWARN: Error Warning Flag bit (bit 0)
 * - Set when TEC or REC is equal to or greater than 96 (TXWAR or RXWAR = 1)
 * - Reset when both REC and TEC are less than 96
 */
static const uint8_t EFLG_EWARN        = 1 << 0;

/*============================================================================
 * Interrupts related registers
 *----------------------------------------------------------------------------
 * CANINTE – INTERRUPT ENABLE (ADDRESS: 2Bh). See page 53 of the datasheet
 */
static const uint8_t CANINTE           = 0x2B;

/*
 *      7       6       5       4       3       2       1       0
 *    R/W-0   R/W-0   R/W-0   R/W-0   R/W-0   R/W-0   R/W-0   R/W-0
 *	+-------+-------+-------+-------+-------+-------+-------+-------+
 *  | MERRE | WAKIE | ERRIE | TX2IE | TX1IE | TX0IE | RX1IE | RX0IE |
 *	+-------+-------+-------+-------+-------+-------+-------+-------+
 */

/*
 * MERRE: Message Error Interrupt Enable bit (bit 7)
 * 1 : Interrupt on error during message reception or transmission
 * 0 : Disabled
 */
static const uint8_t CANINTE_MERRE     = 1 << 7;

/*
 * WAKIE: Wake-up Interrupt Enable bit (bit 6)
 * 1 : Interrupt on CAN bus activity
 * 0 : Disabled
 */
static const uint8_t CANINTE_WAKIE     = 1 << 6;

/*
 * ERRIE: Error Interrupt Enable bit (multiple sources in EFLG register)
 * (bit 5)
 * 1 : Interrupt on EFLG error condition change
 * 0 : Disabled
 */
static const uint8_t CANINTE_ERRIE     = 1 << 5;

/*
 * TX2IE: Transmit Buffer 2 Empty Interrupt Enable bit (bit 4)
 * 1 = Interrupt on TXB2 becoming empty
 * 0 = Disabled
 */
static const uint8_t CANINTE_TX2IE     = 1 << 4;

/*
 * TX1IE: Transmit Buffer 1 Empty Interrupt Enable bit (bit 3)
 * 1 : Interrupt on TXB1 becoming empty
 * 0 : Disabled
 */
static const uint8_t CANINTE_TX1IE     = 1 << 3;

/*
 * TX0IE: Transmit Buffer 0 Empty Interrupt Enable bit (bit 2)
 * 1 : Interrupt on TXB0 becoming empty
 * 0 : Disabled
 */
static const uint8_t CANINTE_TX0IE     = 1 << 2;

/*
 * RX1IE: Receive Buffer 1 Full Interrupt Enable bit (bit 1)
 * 1 : Interrupt when message received in RXB1
 * 0 : Disabled
 */
static const uint8_t CANINTE_RX1IE     = 1 << 1;

/*
 * RX0IE: Receive Buffer 0 Full Interrupt Enable bit (bit 0)
 * 1 : Interrupt when message received in RXB0
 * 0 : Disabled
 */
static const uint8_t CANINTE_RX0IE     = 1 << 0;

/*----------------------------------------------------------------------------
 * CANINTF – INTERRUPT FLAG (ADDRESS: 2Ch). See page 54 of the datasheet
 */
static const uint8_t CANINTF           = 0x2C;

/*
 *      7       6       5       4       3       2       1       0
 *    R/W-0   R/W-0   R/W-0   R/W-0   R/W-0   R/W-0   R/W-0   R/W-0
 *	+-------+-------+-------+-------+-------+-------+-------+-------+
 *  | MERRF | WAKIF | ERRIF | TX2IF | TX1IF | TX0IF | RX1IF | RX0IF |
 *	+-------+-------+-------+-------+-------+-------+-------+-------+
 */

/*
 * MERRF: Message Error Interrupt Flag bit (bit 7)
 * 1 : Interrupt pending (must be cleared by MCU to reset interrupt condition)
 * 0 : No interrupt pending
 */
static const uint8_t CANINTF_MERRF     = 1 << 7;

/*
 * WAKIF: Wake-up Interrupt Flag bit (bit 6)
 * 1 : Interrupt pending (must be cleared by MCU to reset interrupt condition)
 * 0 : No interrupt pending
 */
static const uint8_t CANINTF_WAKIF     = 1 << 6;

/*
 * ERRIF: Error Interrupt Flag bit (multiple sources in EFLG register) (bit 5)
 * 1 : Interrupt pending (must be cleared by MCU to reset interrupt condition)
 * 0 : No interrupt pending
 */
static const uint8_t CANINTF_ERRIF     = 1 << 5;

/*
 * TX2IF: Transmit Buffer 2 Empty Interrupt Flag bit (bit 4)
 * 1 : Interrupt pending (must be cleared by MCU to reset interrupt condition)
 * 0 : No interrupt pending
 */
static const uint8_t CANINTF_TX2IF     = 1 << 4;

/*
 * TX1IF: Transmit Buffer 1 Empty Interrupt Flag bit (bit 3)
 * 1 : Interrupt pending (must be cleared by MCU to reset interrupt condition)
 * 0 : No interrupt pending
 */
static const uint8_t CANINTF_TX1IF     = 1 << 3;

/*
 * TX0IF: Transmit Buffer 0 Empty Interrupt Flag bit (bit 2)
 * 1 : Interrupt pending (must be cleared by MCU to reset interrupt condition)
 * 0 : No interrupt pending
 */
static const uint8_t CANINTF_TX0IF     = 1 << 2;

/*
 * RX1IF: Receive Buffer 1 Full Interrupt Flag bit (bit 1)
 * 1 : Interrupt pending (must be cleared by MCU to reset interrupt condition)
 * 0 : No interrupt pending
 */
static const uint8_t CANINTF_RX1IF     = 1 << 1;

/*
 * RX0IF: Receive Buffer 0 Full Interrupt Flag bit (bit 0)
 * 1 : Interrupt pending (must be cleared by MCU to reset interrupt condition)
 * 0 = No interrupt pending
 */
static const uint8_t CANINTF_RX0IF     = 1 << 0;

/*============================================================================
 * Control and status registers
 *----------------------------------------------------------------------------
 * CANCTRL – CAN CONTROL REGISTER (ADDRESS: XFh). See page 60 of the datasheet
 */
static const uint8_t CANCTRL           = 0x0F;

/*
 *       7        6        5       4     3      2        1         0
 *     R/W-1    R/W-0    R/W-0  R/W-0  R/W-0  R/W-1    R/W-1     R/W-1
 *	+--------+--------+--------+------+-----+-------+---------+---------+
 *  | REQOP2 | REQOP1 | REQOP0 | ABAT | OSM | CLKEN | CLKPRE1 | CLKPRE0 |
 *	+--------+--------+--------+------+-----+-------+---------+---------+
 */

/*
 * REQOP<2:0>: Request Operation mode bits (bits 7 to 5)
 * 000 : Set normal operation mode
 * 001 : Set sleep mode
 * 010 : Set loopback mode
 * 011 : Set listen-only mode
 * 100 : Set configuration mode
 * All other values for REQOP bits are invalid and should not be used
 * On power-up, REQOP : b’111’
 *
 * NOTE: the REQOP value of 111 at Power Up contradicts the reset value of the
 * register as shown in the datasheet (100) aka Configuration Mode.
 * When reading this register, the value is indeed 100. So the datasheet claim
 * (On power-up, REQOP : b’111’), an invalid value by the way, is false or
 * the silicon has been updated to fit the reset value of the datasheet and the
 * claim is obsolet.
 */
static const uint8_t CANCTRL_REQOP_SHIFT = 5;
static const uint8_t CANCTRL_REQOP_MASK  = 0x07;
static const uint8_t NORMAL_MODE = 0;
static const uint8_t SLEEP_MODE  = 1;
static const uint8_t LOOP_MODE   = 2;
static const uint8_t LISTEN_MODE = 3;
static const uint8_t CONFIG_MODE = 4;

/*
 * ABAT: Abort All Pending Transmissions bit (bit 4)
 * 1 : Request abort of all pending transmit buffers
 * 0 : Terminate request to abort all transmissions
 */
static const uint8_t CANCTRL_ABAT        = 1 << 4;

/*
 * OSM: One-Shot mode bit (bit 3)
 * 1 : Enabled. Message will only attempt to transmit one time
 * 0 : Disabled. Messages will reattempt transmission, if required
 */
static const uint8_t CANCTRL_OSM         = 1 << 3;

/*
 * CLKEN: CLKOUT Pin Enable bit (bit 2)
 * 1 : CLKOUT pin enabled
 * 0 : CLKOUT pin disabled (Pin is in high-impedance state)
 */
static const uint8_t CANCTRL_CLKEN       = 1 << 2;

/*
 * CLKPRE<1:0>: CLKOUT Pin Prescaler bits (bits 1 and 0)
 * 00 : FCLKOUT = System Clock/1
 * 01 : FCLKOUT = System Clock/2
 * 10 : FCLKOUT = System Clock/4
 * 11 : FCLKOUT = System Clock/8
 */
static const uint8_t CANCTRL_CLKPRE_MASK = 0x03;
static const uint8_t CLKOUTDIV1          = 0;
static const uint8_t CLKOUTDIV2          = 1;
static const uint8_t CLKOUTDIV4          = 2;
static const uint8_t CLKOUTDIV8          = 3;

/*----------------------------------------------------------------------------
 * CANSTAT – CAN STATUS REGISTER (ADDRESS: XEh). See page 61 of the datasheet
 */
static const uint8_t CANSTAT             = 0x0E;

/*
 *       7         6         5      4     3       2       1     0
 *      R-1       R-0       R-0    U-0   R-0     R-0     R-0   U-0
 *	+---------+---------+---------+---+-------+-------+-------+---+
 *  | OPMODE2 | OPMODE1 | OPMODE0 | - | ICOD2 | ICOD1 | ICOD0 | - |
 *	+---------+---------+---------+---+-------+-------+-------+---+
 */

/*
 * OPMOD<2:0>: Operation mode bits (bits 7 to 5)
 * 000 : Device is in the Normal Operation mode
 * 001 : Device is in Sleep mode
 * 010 : Device is in Loopback mode
 * 011 : Device is in Listen-Only mode
 * 100 : Device is in Configuration mode
 */
static const uint8_t CANSTAT_OPMOD_SHIFT = 5;
static const uint8_t CANSTAT_OPMOD_MASK  = 0x07;

/*
 * ICOD<2:0>: Interrupt Flag Code bits (bits 3 to 1)
 * 000 : No Interrupt
 * 001 : Error Interrupt
 * 010 : Wake-up Interrupt
 * 011 : TXB0 Interrupt
 * 100 : TXB1 Interrupt
 * 101 : TXB2 Interrupt
 * 110 : RXB0 Interrupt
 * 111 : RXB1 Interrupt
 */
static const uint8_t CANSTAT_ICOD_SHIFT = 1;
static const uint8_t CANSTAT_ICOD_MASK  = 0x07;
static const uint8_t NO_INTERRUPT       = 0;
static const uint8_t ERROR_INTERRUPT    = 1;
static const uint8_t WAKEUP_INTERRUPT   = 2;
static const uint8_t TXB0_INTERRUPT     = 3;
static const uint8_t TXB1_INTERRUPT     = 4;
static const uint8_t TXB2_INTERRUPT     = 5;
static const uint8_t RXB0_INTERRUPT     = 6;
static const uint8_t RXB1_INTERRUPT     = 7;

/*============================================================================
 * Registers related to messages transmission
 *----------------------------------------------------------------------------
 * TXB0CTRL – TRANSMIT BUFFER 0 CONTROL REGISTER (ADDRESS: 30h)
 */
static const uint8_t TXB0CTRL            = 0x30;

/*----------------------------------------------------------------------------
 * TXB1CTRL – TRANSMIT BUFFER 1 CONTROL REGISTER (ADDRESS: 40h)
 */
static const uint8_t TXB1CTRL            = 0x40;

/*----------------------------------------------------------------------------
 * TXB2CTRL – TRANSMIT BUFFER 2 CONTROL REGISTER (ADDRESS: 50h)
 */
static const uint8_t TXB2CTRL            = 0x50;

/*
 * See page 18 of the datasheet
 *
 *    7     6      5      4       3     2     1      0
 *   U-0   R-0    R-0    R-0    R/W-0  U-0  R/W-0  R/W-0
 *	+---+------+------+-------+-------+---+------+------+
 *  | - | ABTF | MLOA | TXERR | TXREQ | - | TXP1 | TXP0 |
 *	+---+------+------+-------+-------+---+------+------+
 */

/*
 * ABTF: Message Aborted Flag bit (bit 6)
 * 1 : Message was aborted
 * 0 : Message completed transmission successfully
 */
static const uint8_t TXBnCTRL_ABTF       = 1 << 6;

/*
 * MLOA: Message Lost Arbitration bit (bit 5)
 * 1 : Message lost arbitration while being sent
 * 0 : Message did not lose arbitration while being sent
 */
static const uint8_t TXBnCTRL_MLOA       = 1 << 5;

/*
 * TXERR: Transmission Error Detected bit (bit 4)
 * 1 : A bus error occurred while the message was being transmitted
 * 0 : No bus error occurred while the message was being transmitted
 */
static const uint8_t TXBnCTRL_TXERR      = 1 << 4;

/*
 * TXREQ: Message Transmit Request bit (bit 3)
 * 1 : Buffer is currently pending transmission
 *     (MCU sets this bit to request message be transmitted -
 *     bit is automatically cleared when the message is sent)
 * 0 : Buffer is not currently pending transmission
 *     (MCU can clear this bit to request a message abort)
 */
static const uint8_t TXBnCTRL_TXREQ      = 1 << 3;

/*
 * TXP<1:0>: Transmit Buffer Priority bits (bits 1 and 0)
 * 11 : Highest Message Priority
 * 10 : High Intermediate Message Priority
 * 01 : Low Intermediate Message Priority
 * 00 : Lowest Message Priority
 */
static const uint8_t TXBnCTRL_TXP_MASK   = 0x03;

/*----------------------------------------------------------------------------
 * TXRTSCTRL – TXnRTS PIN CONTROL AND STATUS REGISTER (ADDRESS: 0Dh).
 * See page 19 of the datasheet
 */
static const uint8_t TXRTSCTRL           = 0x0D;

/*
 *    7   6     5       4       3        2        1        0
 *   U-0 U-0   R-x     R-x     R-x     R/W-0    R/W-0    R/W-0
 *	+---+---+-------+-------+-------+--------+--------+--------+
 *  | - | - | B2RTS | B1RTS | B0RTS | B2RTSM | B1RTSM | B0RTSM |
 *	+---+---+-------+-------+-------+--------+--------+--------+
 */

/*
 * B2RTS: TX2RTS Pin State bit (bit 5)
 * - Reads state of TX2RTS pin when in Digital Input mode
 * - Reads as ‘0’ when pin is in ‘Request-to-Send’ mode
 */
static const uint8_t TXRTSCTRL_B2RTS     = 1 << 5;

/*
 * B1RTS: TX1RTS Pin State bit (bit 4)
 * - Reads state of TX1RTS pin when in Digital Input mode
 * - Reads as ‘0’ when pin is in ‘Request-to-Send’ mode
 */
static const uint8_t TXRTSCTRL_B1RTS     = 1 << 4;

/*
 * B0RTS: TX0RTS Pin State bit (bit 3)
 * - Reads state of TX0RTS pin when in Digital Input mode
 * - Reads as ‘0’ when pin is in ‘Request-to-Send’ mode
 */
static const uint8_t TXRTSCTRL_B0RTS     = 1 << 3;

/*
 * B2RTSM: TX2RTS Pin mode bit (bit 2)
 * 1 : Pin is used to request message transmission of TXB2 buffer
 *     (on falling edge)
 * 0 : Digital input
 */
static const uint8_t TXRTSCTRL_B2RTSM    = 1 << 2;

/*
 * B1RTSM: TX1RTS Pin mode bit (bit 1)
 * 1 : Pin is used to request message transmission of TXB1 buffer
 *     (on falling edge)
 * 0 : Digital input
 */
static const uint8_t TXRTSCTRL_B1RTSM    = 1 << 1;

/*
 * B0RTSM: TX0RTS Pin mode bit (bit 0)
 * 1 : Pin is used to request message transmission of TXB0 buffer
 *     (on falling edge)
 * 0 : Digital input
 */
static const uint8_t TXRTSCTRL_B0RTSM    = 1 << 0;

/*----------------------------------------------------------------------------
 * TXB0SIDH – TRANSMIT BUFFER 0 STANDARD IDENTIFIER HIGH (ADDRESS: 31h)
 */
static const uint8_t TXB0SIDH            = 0x31;

/*----------------------------------------------------------------------------
 * TXB1SIDH – TRANSMIT BUFFER 1 STANDARD IDENTIFIER HIGH (ADDRESS: 41h)
 */
static const uint8_t TXB1SIDH            = 0x41;

/*----------------------------------------------------------------------------
 * TXB2SIDH – TRANSMIT BUFFER 2 STANDARD IDENTIFIER HIGH (ADDRESS: 51h)
 */
static const uint8_t TXB2SIDH            = 0x51;

/*
 * See page 20 of the datasheet
 *
 *      7       6      5      4      3      2      1      0
 *    R/W-x   R/W-x  R/W-x  R/W-x  R/W-x  R/W-x  R/W-x  R/W-x
 *	+-------+------+------+------+------+------+------+------+
 *  | SID10 | SID9 | SID8 | SID7 | SID6 | SID5 | SID4 | SID3 |
 *	+-------+------+------+------+------+------+------+------+
 */
static const uint8_t TXBnSIDH_SID_SHIFT  = 3;

/*----------------------------------------------------------------------------
 * TXB0SIDL – TRANSMIT BUFFER 0 STANDARD IDENTIFIER LOW (ADDRESS: 32h)
 */
static const uint8_t TXB0SIDL            = 0x32;

/*----------------------------------------------------------------------------
 * TXB1SIDL – TRANSMIT BUFFER 1 STANDARD IDENTIFIER LOW (ADDRESS: 42h)
 */
static const uint8_t TXB1SIDL            = 0x42;

/*----------------------------------------------------------------------------
 * TXB2SIDL – TRANSMIT BUFFER 2 STANDARD IDENTIFIER LOW (ADDRESS: 52h)
 */
static const uint8_t TXB2SIDL            = 0x52;

/*
 * See page 20 of the datasheet
 *
 *      7      6      5     4      3      2      1       0
 *    R/W-x  R/W-x  R/W-x R/W-x  R/W-x  R/W-x  R/W-x   R/W-x
 *	+------+------+------+-----+-------+-----+-------+-------+
 *  | SID2 | SID1 | SID0 |  -  | EXIDE |  -  | EID17 | EID16 |
 *	+------+------+------+-----+-------+-----+-------+-------+
 */

/*
 * SID<2:0>: Standard Identifier bits (bits 7 to 5)
 */
static const uint8_t TXBnSIDL_SID_SHIFT  = 5;
static const uint8_t TXBnSIDL_SID_MASK   = 0x07;

/*
 * EXIDE: Extended Identifier Enable bit (bit 3)
 * 1 : Message will transmit extended identifier
 * 0 : Message will transmit standard identifier
 */
static const uint8_t TXBnSIDL_EXIDE      = 1 << 3;

/*
 * EID<17:16>: Extended Identifier bits (bits 1 and 0)
 */
static const uint8_t TXBnSIDL_EID_MASK   = 0x03;

/*
 * Mask to isolate the unused bits
 */
static const uint8_t TXBnSIDL_UNUSED_SHIFT_1 = 4;
static const uint8_t TXBnSIDL_UNUSED_SHIFT_2 = 2;
static const uint8_t TXBnSIDL_UNUSED_MASK_1 = (1 << TXBnSIDL_UNUSED_SHIFT_1);
static const uint8_t TXBnSIDL_UNUSED_MASK_2 = (1 << TXBnSIDL_UNUSED_SHIFT_2);
static const uint8_t TXBnSIDL_UNUSED_MASK = TXBnSIDL_UNUSED_MASK_1 |
                                            TXBnSIDL_UNUSED_MASK_2;

/*----------------------------------------------------------------------------
 * TXB0EID8 – TRANSMIT BUFFER 0 EXTENDED IDENTIFIER HIGH (ADDRESS: 33h)
 */
static const uint8_t TXB0EID8            = 0x33;

/*----------------------------------------------------------------------------
 * TXB1EID8 – TRANSMIT BUFFER 1 EXTENDED IDENTIFIER HIGH (ADDRESS: 43h)
 */
static const uint8_t TXB1EID8            = 0x43;

/*----------------------------------------------------------------------------
 * TXB2EID8 – TRANSMIT BUFFER 2 EXTENDED IDENTIFIER HIGH (ADDRESS: 53h)
 */
static const uint8_t TXB2EID8            = 0x53;

/*
 * See page 21 of the datasheet
 *
 *      7       6       5       4       3       2       1      0
 *    R/W-x   R/W-x   R/W-x   R/W-x   R/W-x   R/W-x   R/W-x  R/W-x
 *	+-------+-------+-------+-------+-------+-------+------+------+
 *  | EID15 | EID14 | EID13 | EID12 | EID11 | EID10 | EID9 | EID8 |
 *	+-------+-------+-------+-------+-------+-------+------+------+
 */

/*----------------------------------------------------------------------------
 * TXB0EID0 – TRANSMIT BUFFER 0 EXTENDED IDENTIFIER LOW (ADDRESS: 34h)
 */
static const uint8_t TXB0EID0            = 0x34;

/*----------------------------------------------------------------------------
 * TXB1EID0 – TRANSMIT BUFFER 1 EXTENDED IDENTIFIER LOW (ADDRESS: 44h)
 */
static const uint8_t TXB1EID0            = 0x44;

/*----------------------------------------------------------------------------
 * TXB2EID0 – TRANSMIT BUFFER 2 EXTENDED IDENTIFIER LOW (ADDRESS: 54h)
 */
static const uint8_t TXB2EID0            = 0x54;

/*
 * See page 21 of the datasheet
 *
 *      7      6      5      4      3      2      1      0
 *    R/W-x  R/W-x  R/W-x  R/W-x  R/W-x  R/W-x  R/W-x  R/W-x
 *	+------+------+------+------+------+------+------+------+
 *  | EID7 | EID6 | EID5 | EID4 | EID3 | EID2 | EID1 | EID0 |
 *	+------+------+------+------+------+------+------+------+
 */

/*----------------------------------------------------------------------------
 * TXB0DLC - TRANSMIT BUFFER 0 DATA LENGTH CODE (ADDRESS: 35h)
 */
static const uint8_t TXB0DLC             = 0x35;

/*----------------------------------------------------------------------------
 * TXB1DLC - TRANSMIT BUFFER 1 DATA LENGTH CODE (ADDRESS: 45h)
 */
static const uint8_t TXB1DLC             = 0x45;

/*----------------------------------------------------------------------------
 * TXB2DLC - TRANSMIT BUFFER 2 DATA LENGTH CODE (ADDRESS: 55h)
 */
static const uint8_t TXB2DLC             = 0x55;

/*
 * See page 22 of the datasheet
 *
 *     7     6     5     4      3      2      1      0
 *   R/W-x R/W-x R/W-x R/W-x  R/W-x  R/W-x  R/W-x  R/W-x
 *	+-----+-----+-----+-----+------+------+------+------+
 *  |  -  | RTR |  -  |  -  | DLC3 | DLC2 | DLC1 | DLC0 |
 *	+-----+-----+-----+-----+------+------+------+------+
 */

/*
 * RTR: Remote Transmission Request bit (bit 6)
 * 1 : Transmitted Message will be a Remote Transmit Request
 * 0 : Transmitted Message will be a Data Frame
 */
static const uint8_t TXBnDLC_RTR         = 1 << 6;

/*
 * DLC<3:0>: Data Length Code bits (bit 3 to 0)
 * Sets the number of data bytes to be transmitted (0 to 8 bytes)
 * Note: It is possible to set the DLC to a value greater than eight,
 * however only eight bytes are transmitted
 */
static const uint8_t TXBnDLC_DLC_MASK    = 0x0F;

/*----------------------------------------------------------------------------
 * TXB0Dm – TRANSMIT BUFFER 0 DATA byte m (ADDRESS: 36h to 3Dh)
 */
static const uint8_t TXB0D0           = 0x36;
static const uint8_t TXB0D1           = 0x37;
static const uint8_t TXB0D2           = 0x38;
static const uint8_t TXB0D3           = 0x39;
static const uint8_t TXB0D4           = 0x3A;
static const uint8_t TXB0D5           = 0x3B;
static const uint8_t TXB0D6           = 0x3C;
static const uint8_t TXB0D7           = 0x3D;

/*----------------------------------------------------------------------------
 * TXB1Dm – TRANSMIT BUFFER 0 DATA byte m (ADDRESS: 46h to 4Dh)
 */
static const uint8_t TXB1D0           = 0x46;
static const uint8_t TXB1D1           = 0x47;
static const uint8_t TXB1D2           = 0x48;
static const uint8_t TXB1D3           = 0x49;
static const uint8_t TXB1D4           = 0x4A;
static const uint8_t TXB1D5           = 0x4B;
static const uint8_t TXB1D6           = 0x4C;
static const uint8_t TXB1D7           = 0x4D;

/*----------------------------------------------------------------------------
 * TXB2Dm – TRANSMIT BUFFER 0 DATA byte m (ADDRESS: 56h to 5Dh)
 */
static const uint8_t TXB2D0           = 0x56;
static const uint8_t TXB2D1           = 0x57;
static const uint8_t TXB2D2           = 0x58;
static const uint8_t TXB2D3           = 0x59;
static const uint8_t TXB2D4           = 0x5A;
static const uint8_t TXB2D5           = 0x5B;
static const uint8_t TXB2D6           = 0x5C;
static const uint8_t TXB2D7           = 0x5D;

/*============================================================================
 * Registers related to messages reception
 *----------------------------------------------------------------------------
 * RXB0CTRL – RECEIVE BUFFER 0 CONTROL (ADDRESS: 60h).
 * See page 27 of the datasheet
 */
static const uint8_t RXB0CTRL         = 0x60;

/*
 *     7      6      5     4      3       2      1        0
 *    U-0   R/W-0  R/W-0  U-0    R-0    R/W-0   R-0      R-0
 *	+-----+------+------+-----+-------+------+-------+---------+
 *  |  -  | RXM1 | RXM0 |  -  | RXRTR | BUKT | BUKT1 | FILHIT0 |
 *	+-----+------+------+-----+-------+------+-------+---------+
 */

/*
 * RXM<1:0>: Receive Buffer Operating mode bits (bits 6 and 5)
 * 11 : Turn mask/filters off; receive any message
 * 10 : Receive only valid messages with extended identifiers that meet filter
 *      criteria
 * 01 : Receive only valid messages with standard identifiers that meet filter
 *      criteria. Extended ID filter registers RXFnEID8:RXFnEID0 are ignored
 *      for the messages with standard IDs.
 * 00 : Receive all valid messages using either standard or extended
 *      identifiers that meet filter criteria. Extended ID filter registers
 *      RXFnEID8:RXFnEID0 are applied to first two bytes of data in the
 *      messages with standard IDs.
 */
static const uint8_t RXB0CTRL_RXM1    = 1 << 6;
static const uint8_t RXB0CTRL_RXM0    = 1 << 5;
static const uint8_t RXB0CTRL_RXM_MASK = RXB0CTRL_RXM1 | RXB0CTRL_RXM0;

/*
 * RXRTR: Received Remote Transfer Request bit (bit 3)
 * 1 : Remote Transfer Request Received
 * 0 : No Remote Transfer Request Received
 */
static const uint8_t RXB0CTRL_RXRTR   = 1 << 3;

/*
 * BUKT: Rollover Enable bit (bit 2)
 * 1 : RXB0 message will rollover and be written to RXB1 if RXB0 is full
 * 0 : Rollover disabled
 */
static const uint8_t RXB0CTRL_BUKT    = 1 << 2;

/*
 * BUKT1: Read-only Copy of BUKT bit (used internally by the MCP2515) (bit 1)
 */
static const uint8_t RXB0CTRL_BUKT1   = 1 << 1;

/*
 * FILHIT0: Filter Hit bit – indicates which acceptance filter enabled
 *          reception of message (bit 0)
 * 1 : Acceptance Filter 1 (RXF1)
 * 0 : Acceptance Filter 0 (RXF0)
 *     Note: If a rollover from RXB0 to RXB1 occurs, the FILHIT bit will
 *     reflect the filter that accepted the message that rolled over.
 */
static const uint8_t RXB0CTRL_FILHIT0 = 1 << 0;

/*----------------------------------------------------------------------------
 * RXB1CTRL – RECEIVE BUFFER 1 CONTROL (ADDRESS: 70h).
 * See page 28 of the datasheet
 */
static const uint8_t RXB1CTRL         = 0x70;

/*
 *     7      6      5     4      3        2         1         0
 *    U-0   R/W-0  R/W-0  U-0    R-0      R-0       R-0       R-0
 *	+-----+------+------+-----+-------+---------+---------+---------+
 *  |  -  | RXM1 | RXM0 |  -  | RXRTR | FILHIT2 | FILHIT1 | FILHIT0 |
 *	+-----+------+------+-----+-------+---------+---------+---------+
 */

/*
 * RXM<1:0>: Receive Buffer Operating mode bits (bits 6 and 5)
 * 11 : Turn mask/filters off; receive any message
 * 10 : Receive only valid messages with extended identifiers that meet filter
 *      criteria
 * 01 : Receive only valid messages with standard identifiers that meet filter
 *      criteria. Extended ID filter registers RXFnEID8:RXFnEID0 are ignored
 *      for the messages with standard IDs.
 * 00 : Receive all valid messages using either standard or extended
 *      identifiers that meet filter criteria. Extended ID filter registers
 *      RXFnEID8:RXFnEID0 are applied to first two bytes of data in the
 *      messages with standard IDs.
 */
static const uint8_t RXB1CTRL_RXM1    = 1 << 6;
static const uint8_t RXB1CTRL_RXM0    = 1 << 5;
static const uint8_t RXB1CTRL_RXM_MASK = RXB1CTRL_RXM1 | RXB1CTRL_RXM0;

/*
 * RXRTR: Received Remote Transfer Request bit (bit 3)
 * 1 : Remote Transfer Request Received
 * 0 : No Remote Transfer Request Received
 */
static const uint8_t RXB1CTRL_RXRTR   = 1 << 3;

/*
 * FILHIT<2:0>: Filter Hit bits - indicates which acceptance filter enabled
 *              reception of message (bits 2 to 0)
 * 101 : Acceptance Filter 5 (RXF5)
 * 100 : Acceptance Filter 4 (RXF4)
 * 011 : Acceptance Filter 3 (RXF3)
 * 010 : Acceptance Filter 2 (RXF2)
 * 001 : Acceptance Filter 1 (RXF1) (Only if BUKT bit set in RXB0CTRL)
 * 000 : Acceptance Filter 0 (RXF0) (Only if BUKT bit set in RXB0CTRL)
 */
static const uint8_t RXB1CTRL_FILHIT2 = 1 << 2;
static const uint8_t RXB1CTRL_FILHIT1 = 1 << 1;
static const uint8_t RXB1CTRL_FILHIT0 = 1 << 0;

/*----------------------------------------------------------------------------
 * BFPCTRL – RXnBF PIN CONTROL AND STATUS (ADDRESS: 0Ch)
 * See page 29 of the datasheet
 */
static const uint8_t BFPCTRL          = 0x0C;

/*
 *     7     6      5       4       3       2       1       0
 *    U-0   U-0   R/W-0   R/W-0   R/W-0   R/W-0   R/W-0   R/W-0
 *	+-----+-----+-------+-------+-------+-------+-------+-------+
 *  |  -  |  -  | B1BFS | B0BFS | B1BFE | B0BFE | B1BFM | B0BFM |
 *	+-----+-----+-------+-------+-------+-------+-------+-------+
 */

/*
 * B1BFS: RX1BF Pin State bit (Digital Output mode only) (bit 5)
 * Reads as ‘0’ when RX1BF is configured as interrupt pin
 */
static const uint8_t BFPCTRL_B1BFS    = 1 << 5;

/*
 * B0BFS: RX0BF Pin State bit (Digital Output mode only) (bit 4)
 * Reads as ‘0’ when RX0BF is configured as interrupt pin
 */
static const uint8_t BFPCTRL_B0BFS    = 1 << 4;

/*
 * B1BFE: RX1BF Pin Function Enable bit (bit 3)
 * 1 : Pin function enabled, operation mode determined by B1BFM bit
 * 0 : Pin function disabled, pin goes to high-impedance state
 */
static const uint8_t BFPCTRL_B1BFE    = 1 << 3;

/*
 * B0BFE: RX0BF Pin Function Enable bit (bit 2)
 * 1 : Pin function enabled, operation mode determined by B0BFM bit
 * 0 : Pin function disabled, pin goes to high-impedance state
 */
static const uint8_t BFPCTRL_B0BFE    = 1 << 2;

/*
 * B1BFM: RX1BF Pin Operation mode bit (bit 1)
 * 1 : Pin is used as interrupt when valid message loaded into RXB1
 * 0 : Digital Output mode
 */
static const uint8_t BFPCTRL_B1BFM    = 1 << 1;

/*
 * B0BFM: RX0BF Pin Operation mode bit (bit 0)
 * 1 : Pin is used as interrupt when valid message loaded into RXB0
 * 0 : Digital Output mode
 */
static const uint8_t BFPCTRL_B0BFM    = 1 << 0;

/*----------------------------------------------------------------------------
 * RXB0SIDH – RECEIVE BUFFER 0 STANDARD IDENTIFIER HIGH (ADDRESS: 61h)
 * These bits contain the eight Most Significant bits of the
 * Standard Identifier for the received message
 */
static const uint8_t RXB0SIDH         = 0x61;

/*----------------------------------------------------------------------------
 * RXB1SIDH – RECEIVE BUFFER 1 STANDARD IDENTIFIER HIGH (ADDRESS: 71h)
 * These bits contain the eight Most Significant bits of the
 * Standard Identifier for the received message
 */
static const uint8_t RXB1SIDH         = 0x71;

/*
 * See page 30 of the datasheet
 *
 *      7       6      5      4      3      2      1      0
 *     R-x     R-x    R-x    R-x    R-x    R-x    R-x    R-x
 *	+-------+------+------+------+------+------+------+------+
 *  | SID10 | SID9 | SID8 | SID7 | SID6 | SID5 | SID4 | SID3 |
 *	+-------+------+------+------+------+------+------+------+
 */

/*----------------------------------------------------------------------------
 * RXB0SIDL – RECEIVE BUFFER 0 STANDARD IDENTIFIER LOW (ADDRESS: 62h)
 */
static const uint8_t RXB0SIDL         = 0x62;

/*----------------------------------------------------------------------------
 * RXB1SIDL – RECEIVE BUFFER 1 STANDARD IDENTIFIER LOW (ADDRESS: 72h)
 */
static const uint8_t RXB1SIDL         = 0x72;

/*
 * See page 30 of the datasheet
 *
 *      7      6      5     4     3     2      1       0
 *     R-x    R-x    R-x   R-x   R-x   U-0    R-x     R-x
 *	+------+------+------+-----+-----+-----+-------+-------+
 *  | SID2 | SID1 | SID0 | SRR | IDE |  -  | EID17 | EID16 |
 *	+------+------+------+-----+-----+-----+-------+-------+
 */

/*
 * SID<2:0>: Standard Identifier bits (bits 7 to 5)
 * These bits contain the three Least Significant bits of the Standard
 * Identifier for the received message
 */
static const uint8_t RXBnSIDL_SID_MASK  = 0xE0;
static const uint8_t RXBnSIDL_SID_SHIFT = 5;

/*
 * SRR: Standard Frame Remote Transmit Request bit (valid only if IDE
 * bit = ‘0’) (bit 4)
 * 1 : Standard Frame Remote Transmit Request Received
 * 0 : Standard Data Frame Received
 */
static const uint8_t RXBnSIDL_SRR     = 1 << 4;

/*
 * IDE: Extended Identifier Flag bit (bit 3)
 * This bit indicates whether the received message was a Standard or an
 * Extended Frame
 * 1 : Received message was an Extended Frame
 * 0 : Received message was a Standard Frame
 */
static const uint8_t RXBnSIDL_IDE     = 1 << 3;

/*
 * EID<17:16>: Extended Identifier bits (bits 1 and 0)
 * These bits contain the two Most Significant bits of the Extended
 * Identifier for the received message
 */
static const uint8_t RXBnSIDL_EID_MASK = 0x03;

/*----------------------------------------------------------------------------
 * RXB0EID8 – RECEIVE BUFFER 0 EXTENDED IDENTIFIER HIGH (ADDRESS: 63h)
 * EID<15:8>: Extended Identifier bits
 * These bits hold bits 15 through 8 of the Extended Identifier for the
 * received message
 */
static const uint8_t RXB0EID8         = 0x63;

/*----------------------------------------------------------------------------
 * RXB1EID8 – RECEIVE BUFFER 1 EXTENDED IDENTIFIER HIGH (ADDRESS: 73h)
 * EID<15:8>: Extended Identifier bits
 * These bits hold bits 15 through 8 of the Extended Identifier for the
 * received message
 */
static const uint8_t RXB1EID8         = 0x73;

/*
 * See page 31 of the datasheet
 *
 *      7       6       5       4       3       2       1      0
 *     R-x     R-x     R-x     R-x     R-x     R-x     R-x    R-x
 *	+-------+-------+-------+-------+-------+-------+------+------+
 *  | EID15 | EID14 | EID13 | EID12 | EID11 | EID10 | EID9 | EID8 |
 *	+-------+-------+-------+-------+-------+-------+------+------+
 */

/*----------------------------------------------------------------------------
 * RXB0EID0 – RECEIVE BUFFER 0 EXTENDED IDENTIFIER LOW (ADDRESS: 63h)
 * EID<7:0>: Extended Identifier bits
 * These bits hold the Least Significant eight bits of the Extended
 * Identifier for the received message
 */
static const uint8_t RXB0EID0         = 0x64;

/*----------------------------------------------------------------------------
 * RXB1EID0 – RECEIVE BUFFER 1 EXTENDED IDENTIFIER LOW (ADDRESS: 73h)
 * EID<7:0>: Extended Identifier bits
 * These bits hold the Least Significant eight bits of the Extended
 * Identifier for the received message
 */
static const uint8_t RXB1EID0         = 0x74;

/*
 * See page 31 of the datasheet
 *
 *      7      6      5      4      3      2      1      0
 *     R-x    R-x    R-x    R-x    R-x    R-x    R-x    R-x
 *  +------+------+------+------+------+------+------+------+
 *  | EID7 | EID6 | EID5 | EID4 | EID3 | EID2 | EID1 | EID0 |
 *	+------+------+------+------+------+------+------+------+
 */

/*----------------------------------------------------------------------------
 * RXB0DLC - RECEIVE BUFFER 0 DATA LENGTH CODE (ADDRESS: 65h)
 */
static const uint8_t RXB0DLC          = 0x65;

/*----------------------------------------------------------------------------
 * RXB1DLC - RECEIVE BUFFER 1 DATA LENGTH CODE (ADDRESS: 75h)
 */
static const uint8_t RXB1DLC          = 0x75;

/*
 * See page 32 of the datasheet
 *
 *     7     6     5     4      3      2      1      0
 *    R-x   R-x   R-x   R-x    R-x    R-x    R-x    R-x
 *  +-----+-----+-----+-----+------+------+------+------+
 *  |  -  | RTR | RB1 | RB0 | DLC3 | DLC2 | DLC1 | DLC0 |
 *  +-----+-----+-----+-----+------+------+------+------+
 *
 * RB1 and RB0 are reserved
 */

/*
 * RTR: Extended Frame Remote Transmission Request bit (bit 6)
 * (valid only when RXB1SIDL.IDE = 1)
 * 1 : Extended Frame Remote Transmit Request Received
 * 0 : Extended Data Frame Received
 */
static const uint8_t RXBnDLC_RTR      = 1 << 6;

/*
 * DLC<3:0>: Data Length Code bits (bit 3 to 0)
 * Indicates number of data bytes that were received
 */
static const uint8_t RXBnDLC_DLC_MASK = 0x07;

/*----------------------------------------------------------------------------
 * RXB0Dm - RECEIVE BUFFER 0 DATA byte m (ADDRESS: 66h to 6Dh)
 */
static const uint8_t RXB0D0           = 0x66;
static const uint8_t RXB0D1           = 0x67;
static const uint8_t RXB0D2           = 0x68;
static const uint8_t RXB0D3           = 0x69;
static const uint8_t RXB0D4           = 0x6A;
static const uint8_t RXB0D5           = 0x6B;
static const uint8_t RXB0D6           = 0x6C;
static const uint8_t RXB0D7           = 0x6D;

/*----------------------------------------------------------------------------
 * RXB1Dm - RECEIVE BUFFER 1 DATA byte m (ADDRESS: 76h to 7Dh)
 */
static const uint8_t RXB1D0           = 0x76;
static const uint8_t RXB1D1           = 0x77;
static const uint8_t RXB1D2           = 0x78;
static const uint8_t RXB1D3           = 0x79;
static const uint8_t RXB1D4           = 0x7A;
static const uint8_t RXB1D5           = 0x7B;
static const uint8_t RXB1D6           = 0x7C;
static const uint8_t RXB1D7           = 0x7D;

/*----------------------------------------------------------------------------
 * RXFnSIDH – FILTER n STANDARD IDENTIFIER HIGH
 * (ADDRESS: 00h, 04h, 08h, 10h, 14h, 18h)
 */
static const uint8_t RXF0SIDH         = 0x00;
static const uint8_t RXF1SIDH         = 0x04;
static const uint8_t RXF2SIDH         = 0x08;
static const uint8_t RXF3SIDH         = 0x10;
static const uint8_t RXF4SIDH         = 0x14;
static const uint8_t RXF5SIDH         = 0x18;

/*
 * See page 35 of the datasheet
 *
 *      7       6      5      4      3      2      1      0
 *    R/W-x   R/W-x  R/W-x  R/W-x  R/W-x  R/W-x  R/W-x  R/W-x
 *	+-------+------+------+------+------+------+------+------+
 *  | SID10 | SID9 | SID8 | SID7 | SID6 | SID5 | SID4 | SID3 |
 *	+-------+------+------+------+------+------+------+------+
 */

/*----------------------------------------------------------------------------
 * RXFnSIDL – FILTER n STANDARD IDENTIFIER LOW
 * (ADDRESS: 01h, 05h, 09h, 11h, 15h, 19h)
 */
static const uint8_t RXF0SIDL         = 0x01;
static const uint8_t RXF1SIDL         = 0x05;
static const uint8_t RXF2SIDL         = 0x09;
static const uint8_t RXF3SIDL         = 0x11;
static const uint8_t RXF4SIDL         = 0x15;
static const uint8_t RXF5SIDL         = 0x19;

/*
 * See page 35 of the datasheet
 *
 *      7      6      5     4      3      2      1       0
 *    R/W-x  R/W-x  R/W-x  U-0   R/W-x   U-0   R/W-x   R/W-x
 *	+------+------+------+-----+-------+-----+-------+-------+
 *  | SID2 | SID1 | SID0 |  -  | EXIDE |  -  | EID17 | EID16 |
 *	+------+------+------+-----+-------+-----+-------+-------+
 */

/*
 * SID<2:0>: Standard Identifier Filter bits (bits 7 to 5)
 * These bits hold the filter bits to be applied to bits <2:0> of the
 * Standard Identifier portion of a received message
 */
static const uint8_t RXFnSIDL_SID_SHIFT  = 5;
static const uint8_t RXFnSIDL_SID_MASK   = 0x07;

/*
 * EXIDE: Extended Identifier Enable bit (bit 3)
 * 1 : Filter is applied only to Extended Frames
 * 0 : Filter is applied only to Standard Frames
 */
static const uint8_t RXFnSIDL_EXIDE      = 1 << 3;

/*
 * EID<17:16>: Extended Identifier Filter bits (bits 1 and 0)
 * These bits hold the filter bits to be applied to bits <17:16> of the
 * Extended Identifier portion of a received message
 */
static const uint8_t RXFnSIDL_EID_MASK   = 0x03;

/*----------------------------------------------------------------------------
 * RXFnEID8 – FILTER n EXTENDED IDENTIFIER HIGH
 * (ADDRESS: 02h, 06h, 0Ah, 12h, 16h, 1Ah)
 */
static const uint8_t RXF0EID8            = 0x02;
static const uint8_t RXF1EID8            = 0x06;
static const uint8_t RXF2EID8            = 0x0A;
static const uint8_t RXF3EID8            = 0x12;
static const uint8_t RXF4EID8            = 0x16;
static const uint8_t RXF5EID8            = 0x1A;

/*
 * See page 36 of the datasheet
 *
 *      7       6       5       4       3       2       1      0
 *    R/W-x   R/W-x   R/W-x   R/W-x   R/W-x   R/W-x   R/W-x  R/W-x
 *	+-------+-------+-------+-------+-------+-------+------+------+
 *  | EID15 | EID14 | EID13 | EID12 | EID11 | EID10 | EID9 | EID8 |
 *	+-------+-------+-------+-------+-------+-------+------+------+
 */

/*----------------------------------------------------------------------------
 * RXFnEID0 – FILTER n EXTENDED IDENTIFIER LOW
 * (ADDRESS: 03h, 07h, 0Bh, 13h, 17h, 1Bh)
 */
static const uint8_t RXF0EID0            = 0x03;
static const uint8_t RXF1EID0            = 0x07;
static const uint8_t RXF2EID0            = 0x0B;
static const uint8_t RXF3EID0            = 0x13;
static const uint8_t RXF4EID0            = 0x17;
static const uint8_t RXF5EID0            = 0x1B;

/*
 * See page 36 of the datasheet
 *
 *      7      6      5      4      3      2      1      0
 *    R/W-x  R/W-x  R/W-x  R/W-x  R/W-x  R/W-x  R/W-x  R/W-x
 *  +------+------+------+------+------+------+------+------+
 *  | EID7 | EID6 | EID5 | EID4 | EID3 | EID2 | EID1 | EID0 |
 *	+------+------+------+------+------+------+------+------+
 */

/*----------------------------------------------------------------------------
 * RXMnSIDH – MASK n STANDARD IDENTIFIER HIGH (ADDRESS: 20h, 24h)
 */
static const uint8_t RXM0SIDH            = 0x20;
static const uint8_t RXM1SIDH            = 0x24;

/*
 * See page 37 of the datasheet
 *
 *      7       6      5      4      3      2      1      0
 *    R/W-0   R/W-0  R/W-0  R/W-0  R/W-0  R/W-0  R/W-0  R/W-0
 *	+-------+------+------+------+------+------+------+------+
 *  | SID10 | SID9 | SID8 | SID7 | SID6 | SID5 | SID4 | SID3 |
 *	+-------+------+------+------+------+------+------+------+
 */

/*----------------------------------------------------------------------------
 * RXMnSIDL – MASK n STANDARD IDENTIFIER LOW (ADDRESS: 21h, 25h)
 */
static const uint8_t RXM0SIDL            = 0x21;
static const uint8_t RXM1SIDL            = 0x25;

/*
 * See page 37 of the datasheet
 *
 *      7      6      5     4     3     2      1       0
 *    R/W-0  R/W-0  R/W-0  U-0   U-0   U-0   R/W-0   R/W-0
 *	+------+------+------+-----+-----+-----+-------+-------+
 *  | SID2 | SID1 | SID0 |  -  |  -  |  -  | EID17 | EID16 |
 *	+------+------+------+-----+-----+-----+-------+-------+
 */

/*----------------------------------------------------------------------------
 * RXMnEID8 – MASK n EXTENDED IDENTIFIER HIGH (ADDRESS: 22h, 26h)
 */
static const uint8_t RXM0EID8            = 0x22;
static const uint8_t RXM1EID8            = 0x26;

/*
 * See page 38 of the datasheet
 *
 *      7       6       5       4       3       2       1      0
 *    R/W-0   R/W-0   R/W-0   R/W-0   R/W-0   R/W-0   R/W-0  R/W-0
 *	+-------+-------+-------+-------+-------+-------+------+------+
 *  | EID15 | EID14 | EID13 | EID12 | EID11 | EID10 | EID9 | EID8 |
 *	+-------+-------+-------+-------+-------+-------+------+------+
 */

/*----------------------------------------------------------------------------
 * RXMnEID0 – MASK n EXTENDED IDENTIFIER LOW (ADDRESS: 23h, 27h)
 */
static const uint8_t RXM0EID0            = 0x23;
static const uint8_t RXM1EID0            = 0x27;

/*
 * See page 38 of the datasheet
 *
 *      7      6      5      4      3      2      1      0
 *    R/W-0  R/W-0  R/W-0  R/W-0  R/W-0  R/W-0  R/W-0  R/W-0
 *  +------+------+------+------+------+------+------+------+
 *  | EID7 | EID6 | EID5 | EID4 | EID3 | EID2 | EID1 | EID0 |
 *	+------+------+------+------+------+------+------+------+
 */

/*============================================================================
 * MCP2515 SPI Insctructions. See page 67 of the datasheet
 *----------------------------------------------------------------------------
 * RESET instruction
 */
static const uint8_t RESET               = B11000000;

/*----------------------------------------------------------------------------
 * READ instruction
 */
static const uint8_t READ                = B00000011;

/*----------------------------------------------------------------------------
 * READ RX BUFFER instructions
 */
static const uint8_t READ_RX_BUFFER      = B10010000;
static const uint8_t RX_BUFFER_0         = B00000010;
static const uint8_t RX_BUFFER_0_ID      = B00000000;
static const uint8_t RX_BUFFER_1         = B00000110;
static const uint8_t RX_BUFFER_1_ID      = B00000100;
static const uint8_t RX_BUFFER_NO_ID     = B00000000;

/*----------------------------------------------------------------------------
 * WRITE instruction
 */
static const uint8_t WRITE               = B00000010;

/*----------------------------------------------------------------------------
 * LOAD TX BUFFER instructions
 */
static const uint8_t LOAD_TX_BUFFER_0_ID  = B01000000;
static const uint8_t LOAD_TX_BUFFER_0     = B01000001;
static const uint8_t LOAD_TX_BUFFER_1_ID  = B01000010;
static const uint8_t LOAD_TX_BUFFER_1     = B01000011;
static const uint8_t LOAD_TX_BUFFER_2_ID  = B01000100;
static const uint8_t LOAD_TX_BUFFER_2     = B01000101;
static const uint8_t LOAD_TX_BUFFER_NO_ID = B01000000;

static inline uint8_t __attribute__((always_inline, unused)) LOAD_TX_BUFFER(
  uint8_t idAsInteger)
{
  return (LOAD_TX_BUFFER_NO_ID | (idAsInteger << 1));
}

static inline uint8_t __attribute__((always_inline, unused)) LOAD_TX_BUFFER_WITHOUT_ID(
  uint8_t idAsInteger)
{
  return (LOAD_TX_BUFFER_NO_ID | (idAsInteger << 1) | 0x01);
}

/*----------------------------------------------------------------------------
 * RTS instructions
 */
static const uint8_t RTS_NONE            = B10000000;
static const uint8_t RTS_TXB0            = B00000001;
static const uint8_t RTS_TXB1            = B00000010;
static const uint8_t RTS_TXB2            = B00000100;
static inline uint8_t __attribute__((always_inline, unused)) RTS(uint8_t txMask)
{
  return (RTS_NONE | (txMask & 0x07));
}

/*----------------------------------------------------------------------------
 * READ STATUS instruction
 */
static const uint8_t READ_STATUS         = B10100000;

/*----------------------------------------------------------------------------
 * RX STATUS instruction
 */
static const uint8_t RX_STATUS           = B10110000;

/*----------------------------------------------------------------------------
 * BIT MODIFY instruction
 */
static const uint8_t BIT_MODIFY          = B00000101;

/* End of namespace mpc2515 */
};

#endif

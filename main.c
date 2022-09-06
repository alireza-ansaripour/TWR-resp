/*! ----------------------------------------------------------------------------
 *  @file    tx_with_cca.c
 *  @brief   Here we implement a simple Clear Channel Assessment (CCA) mechanism
 *           before frame transmission. The CCA can be used to avoid collisions
 *           with other frames on the air. See Note 1 for more details.
 *
 *           Note this is not doing CCA the way a continuous carrier radio would do it by
 *           looking for energy/carrier in the band. It is only looking for preamble so
 *           will not detect PHR or data phases of the frame. In a UWB data network it
 *           is advised to also do a random back-off before re-transmission in the event
 *           of not receiving acknowledgement to a data frame transmission.
 *
 *           This example has been designed to operate with the transmitter in
 *           Continuous Frame mode (CF).
 *           The transmitter  will fill the air with frames.
 *           The receiver is set in dwt_starttx(DWT_START_TX_CCA) mode.
 *           In this mode transmission will occurr if the receiver does not detect a preamble,
 *           otherwise the transmission will be cancelled.
 *           Note, the Continuous Frame example actually stops after 2 minutes
 *           interval (thus the user should toggle the reset button on the unit running
 *           CF example to restart it if they wish to continue observing this pseudo CCA
 *           experiencing an environment of high air-utilisation). Thus the radio configuration
 *           used here matches that of CF example.
 * @attention
 *
 * Copyright 2019 - 2020 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */
#include "instance.h"
#include "deca_regs.h"
#include "deca_device_api.h"
uint8_t rx_buffer_main[100];
int main(void){
  uint32_t status_reg;
  instance_init();
  while(1){
    instance_loop();
  }


}
/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. For Wireless Sensor Networks application, most of the MAC protocols rely on Clear Channel Assessment (CCA) to avoid collisions with other frames
 *    in the air. This consists in sampling the air for a short period to see if the medium is idle before transmitting. For most radios this involves
 *    looking for the RF carrier, but for UWB where this is not possible, one approach is to just look for preamble to avoid conflicting transmissions,
 *    since any sending of preamble during data will typically not disturb those receivers who are demodulating in data mode.
 *    The idea then is to sample the air for a small amount of time to see if a preamble can be detected, then if preamble is not seen the transmission
 *    is initiated, otherwise we defer the transmission typically for a random back-off period after which transmission is again attempted with CCA.
 *    Note: we return to idle for the back-off period and do not receive the frame whose preamble was detected, since the MAC (and upper layer) wants
 *    to transmit and not receive at this time.
 *    This example has been designed to operate with example 4b - Continuous Frame. The 4b device will fill the air with frames which will be detected by the CCA
 *    and thus the CCA will cancel the transmission and will use back off to try sending again at later stage.
 *    This example will actually get to send when the CCA preamble detection overlaps with the data portion of the continuous TX or inter frame period,
 *    Note the Continuous Frame example actually stops after 30s interval (thus the user should toggle the reset button on the unit running example 4b
 *    to restart it if they wish to continue observing this pseudo CCA experiencing an environment of high air-utilisation).
 * 2. The device ID is a hard coded constant in the blink to keep the example simple but for a real product every device should have a unique ID.
 *    For development purposes it is possible to generate a DW3000 unique ID by combining the Lot ID & Part Number values programmed into the
 *    DW3000 during its manufacture. However there is no guarantee this will not conflict with someone else?s implementation. We recommended that
 *    customers buy a block of addresses from the IEEE Registration Authority for their production items. See "EUI" in the DW3000 User Manual.
 * 3. In a real application, for optimum performance within regulatory limits, it may be necessary to set TX pulse bandwidth and TX power, (using
 *    the dwt_configuretxrf API call) to per device calibrated values saved in the target system or the DW3000 OTP memory.
 * 4. The preamble timeout of 3 PACs is recommended as sufficient for this CCA example for all modes and data rates. The PAC size should be different
 *    when is different for different preamble configurations, as per User Manual guidelines.
 * 5. dwt_writetxdata() takes the tx_msg buffer and copies it into devices TX buffer memory, the two byte check-sum at the end of the frame is
 *    automatically appended by the DW3000, thus the dwt_writetxfctrl() should be given the total length.
 * 6. We use polled mode of operation here to keep the example as simple as possible, but the TXFRS and CCA_FAIL status events can be used to generate an interrupt.
 *    Please refer to DW3000 User Manual for more details on "interrupts".
 * 7. Desired configuration by user may be different to the current programmed configuration. dwt_configure is called to set desired
 *    configuration.
 ****************************************************************************************************************************************************/

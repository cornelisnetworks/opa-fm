/* BEGIN_ICS_COPYRIGHT5 ****************************************

Copyright (c) 2015-2017, Intel Corporation

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Intel Corporation nor the names of its contributors
      may be used to endorse or promote products derived from this software
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 * ** END_ICS_COPYRIGHT5   ****************************************/

/**
 * @file test_em_basic.c
 *
 * @brief
 *
 * This file uses cmocka to test basic functionality for the
 * Etherent Manager by calling EM specific functions and
 * testing their return information. In the future, it may
 * be possible to call em_main() or sm_main() as the setup
 * for some tests, but there may be problems associated with
 * the state of the SM or the EM.
 *
 */

#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <cmocka.h>

#include <stl_em_eostl.h>
#include <em_eostl.h>
#include <fm_xml.h>
#include <cs_vesw.h>

Status_t sm_main(void);
extern Status_t sm_parse_xml_config(void);

/**
* @brief test_parse_xml - parse the default XML file and
* verify that the parsing succeeds.
*
* @param state - cmocka required variable
*/
static void test_parse_xml(void **state)
{
    (void) state; /* unused */
	Status_t status = VSTATUS_OK;
	status = sm_parse_xml_config();
	assert_int_equal(status, VSTATUS_OK);
}

/**
* @brief test_mac_gen - generate a mac address and verify
* that the mac address is constructed as expected.
*
* How to check mac address generation:
*     cd eostl-dev/Eostl_HD/Tools
*     gcc -o lmac_encode lmac_encode.c
*     %                   fabricId vEswId encSLid Multicast
*     [Tools]$ ./lmac_encode 0x0301 0x02 0x05 0
*
* @param state - cmocka required variable
*/
static void test_mac_gen(void **state)
{
    (void) state; /* unused */
	int i = 0;
	uint8_t *tmac = NULL, *emac = NULL;
	MACAddr test_mac;
	MACAddr expected_mac = {0x06, 0x2c, 0x00, 0x05, 0x00, 0x00};

	// fabric id = 0x301
	// slid = 0x05
	// virtual etherent switch id = 0x02
	cs_gen_lmac(test_mac, 0x301, 0x05, 0x02);

	tmac = test_mac;
	emac = expected_mac;
	for (i = 0; i < 6; i++) {
		assert_int_equal(tmac[i], emac[i]);
	}
}

/**
* @brief main function for this group of tests.
*
* @return cmocka return value.
*/
int main(void)
{
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_parse_xml),
        cmocka_unit_test(test_mac_gen),
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}

/* BEGIN_ICS_COPYRIGHT5 ****************************************

Copyright (c) 2015, Intel Corporation

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
/* [ICS VERSION STRING: unknown] */

#ifndef _stl_convertfuncs_h_included_
#define _stl_convertfuncs_h_included_ 1

#include "iba/ib_sm_priv.h"
#include "iba/stl_sm_priv.h"

/**
	@file Provides functions for converting IB to STL data structures.
*/

/**
	IB->STL: copy/convert values from @c src to @c dest.

	@param cpyVerInfo If 0, set (Base|Class)Version to STL values, otherwise, copy from @c src.

	@return 0 on success, non-zero otherwise.
*/
int stl_CopyIbNodeInfo(STL_NODE_INFO * dest, NODE_INFO * src, int cpyVerInfo);

/**
	STL->IB: copy/convert values from @c src to @c dest.

	@param cpyVerInfo If 0, set (Base|Class)Version to IB values, otherwise, copy from @c src.

	@return 0 on success, non-zero otherwise.
*/
int stl_CopyStlNodeInfo(NODE_INFO * dest, STL_NODE_INFO * src, int cpyVerInfo);

#if 1
/* These functions facilitate conversions between IB and STL formats. */
/* Primary user is intended to be the SM */
/* A non-zero return value indicates a problem with the conversion and results should not be used. */
/* IB can be exactly mapped to STL. Extra info in STL format is defaulted. */
/* STL cannot be exactly mapped to IB. If STL value is not compatible with IB an error may be returned. */
int IB2STL_NODE_INFO(NODE_INFO *pIb, STL_NODE_INFO *pStl);
int STL2IB_NODE_INFO(STL_NODE_INFO *pStl, NODE_INFO *pIb);
int IB2STL_PORT_INFO(PORT_INFO *pIb, STL_PORT_INFO *pStl);
int STL2IB_PORT_INFO(STL_PORT_INFO *pStl, PORT_INFO *pIb);
int IB2STL_SWITCH_INFO(SWITCH_INFO *pIb, STL_SWITCH_INFO *pStl);
int STL2IB_SWITCH_INFO(STL_SWITCH_INFO *pStl, SWITCH_INFO *pIb);
STL_LID_32 IB2STL_LID(IB_LID lid);
IB_LID STL2IB_LID(STL_LID_32 lid);
#endif 


#endif /* _stl_convertfuncs_h_included_ */

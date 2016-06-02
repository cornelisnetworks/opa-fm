/* BEGIN_ICS_COPYRIGHT2 ****************************************

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

 * ** END_ICS_COPYRIGHT2   ****************************************/

//=======================================================================
//									/
// FILE NAME								/
//    cs_util.h								/
//									/
// DESCRIPTION								/
//									/
//	This is the OPA FM common services transport utility header	/
//									/
// DATA STRUCTURES							/
//    [global component data structures defined in this file]		/
//									/
// DEPENDENCIES								/
//    none								/
//									/
// HISTORY								/
//									/
//    NAME	DATE  REMARKS						/
//     trp  01/24/01  Initial creation of file.				/
//     trp  02/05/01  Changed IBerror_t to Status_t			/
//     trp  02/05/01  Removed cs_mad stuff				/
//     trp  02/09/01  Moved VIEO specific stuff here			/
//     trp  02/13/01  Added prototypes for media to/from struct funcs	/
//     trp  02/14/01  Moved prototypes for media to/from to tapi	/
//     trp  03/14/01  Changed cs_tapi to cs_g for newly merged headers	/
//     trp  04/16/01  Removed vs_pkt.h include				/
//     trp  05/31/01  Added cep create/destroy prototypes		/
//									/
//=======================================================================

#ifndef	_CS_UTIL_H
#define	_CS_UTIL_H

#include "cs_g.h"
#include "cs_ds.h"


Status_t	cs_clone_hca	 (CShca_t *hp, CShca_t **clonep);
Status_t	cs_get_con_t_ptr ( CShca_t *hp, CScon_t **cpp);
Status_t	cs_get_pathp 	 (CScon_t *conp, uint8_t flag, CSpath_t **pathp);
void		cs_mad_dump 	 (Mai_t *pkt);


#endif	// _CS_UTIL_H


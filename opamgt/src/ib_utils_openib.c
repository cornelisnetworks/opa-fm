/* BEGIN_ICS_COPYRIGHT3 ****************************************

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

** END_ICS_COPYRIGHT3   ****************************************/
 
/* [ICS VERSION STRING: unknown] */

#ifdef IB_STACK_OPENIB

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <inttypes.h>
#define _GNU_SOURCE
#include <getopt.h>
#include <syslog.h>
#include <signal.h>
#include <sys/wait.h>
#include <errno.h>

#define OPAMGT_PRIVATE 1

#include "ibt.h"  

#include "infiniband/umad.h"
#include "ib_utils_openib.h"
#include <opamgt_priv.h>

#define MAX_NUM_CAS 20

FSTATUS convert_umad_ca_attribs_to_iba( IN  void *pumad_ca_attribs,
										OUT IB_CA_ATTRIBUTES *pCaAttributes );
FSTATUS convert_umad_port_attribs_to_iba( IN  void *pumad_port_attribs,
										  OUT IB_PORT_ATTRIBUTES  *pPortAttributes );

/**
 * @brief Get the HFI number
 *
 * @param hfi_name       Name of the HFI device
 *
 * @return
 *  On Success  the HFI Number
 *  On Error    -1
 */
int omgt_get_hfi_num(char *hfi_name)
{
	char ca_names[MAX_NUM_CAS][UMAD_CA_NAME_LEN];
	int i;

	// Always clear the count.
	int caCount = 0;

	// get ca count and names
	if (omgt_get_hfi_names((void *)ca_names, MAX_NUM_CAS, &caCount) != OMGT_STATUS_SUCCESS) {
		return -1;
	}

	for (i = 0; i < caCount; i++) {
		if (0 == strcmp(ca_names[i], hfi_name))
			return i + 1;
	}
	return -1;
} // int omgt_get_hfiNum

/******************************************************************************/
/******************************************************************************/

/**
 * Return the GUIDs for the FIs.  The GUID array pointer must be null and
 * will always get filled-in, sets the number of FIs in the system
 * regardless of the initial value of the
 * count.
 *
 * @param pCaCount
 * @param pCaGuidArray, If pCaGuidArray not null, Caller must call MemoryDeallocate()
 *
 * @return FSTATUS
 */
FSTATUS omgt_get_caguids(OUT uint32 *pCaCount, OUT uint64 **pCaGuidArray)
{
    FSTATUS fstatus = FSUCCESS;
   int i, c;
   char ca_names[MAX_NUM_CAS][UMAD_CA_NAME_LEN];

   // Check that a valid pointer was provided for the CA count.
   if( !pCaCount )
   {
      return FINVALID_PARAMETER;
   }
   if( !pCaGuidArray )
   {
      return FINVALID_PARAMETER;
   }

   // Always clear the count.
   *pCaCount = 0;

   // get ca count and names
   (void)omgt_get_hfi_names((void *)ca_names, MAX_NUM_CAS, &c);
   if (c <= 0)
   {
      // _DBG_LEAVE_FUNC();
      *pCaCount = 0;
      return FNOT_FOUND;
   }

   *pCaCount = 0;


   *pCaGuidArray = (EUI64*)MemoryAllocateAndClear( sizeof(EUI64)*(*pCaCount), FALSE, OMGT_MEMORY_TAG);


   if ( *pCaGuidArray==NULL )
   {
      return FINSUFFICIENT_MEMORY;
   }

   umad_ca_t ca;
   uint64     *pCaGuid;

   for (i=0, pCaGuid=*pCaGuidArray; i<*pCaCount; ++i,++pCaGuid)
   {


      fstatus = umad_get_ca(ca_names[i], &ca);
      if (fstatus != FSUCCESS) {
         break;
      }  


      *pCaGuid = ca.node_guid;
	  umad_release_ca(&ca);
      // *pCaGuid = ca.system_guid;

   }  // for (i=0, pCaGuid=g_caMon; i<*pCaCount; ++i,++pCaGuid)

   if (fstatus != FSUCCESS) {
      if (pCaGuidArray != NULL)
            MemoryDeallocate(pCaGuidArray);
   }  

   return fstatus;


}  // omgt_get_caguids()


/**
 * Return the the hfi name and hfi port for the given port guid.
 * Return value is success / failure.
 *
 * @param portGuid
 * @param pointer to hfiName
 * @param pointer to hfi number (1=1st CA)
 * @param pointer to port number (1=1st port)
 * @param pointer to port gid prefix
 *
 * @return 0 for success, else error code
 */
FSTATUS omgt_get_hfi_from_portguid(uint64_t portGuid, struct omgt_port *port, char *pCaName,
	int *caNum, int *portNum, uint64_t *pPrefix, uint16 *pSMLid, uint8 *pSMSL, uint8 *pPortState)
{
	FSTATUS fstatus = FSUCCESS;
	char ca_names[MAX_NUM_CAS][UMAD_CA_NAME_LEN];
	umad_ca_t Ca;
	umad_port_t port_attr;
	int i, j;
	int found = 0;

	// Always clear the count, and output parameters
	int caCount = 0;

	// Default the return values to error cases.
	if (pCaName) *pCaName = '\0';
	if (caNum) *caNum = -1;
	if (portNum) *portNum = -1;
	if (pPrefix) *pPrefix = 0;
	if (pSMLid) *pSMLid = -1;
	if (pSMSL) *pSMSL = 0;
	if (pPortState) *pPortState = PortStateNop;

	// get ca count and names
	if (omgt_get_hfi_names((void *)ca_names, MAX_NUM_CAS, &caCount) != OMGT_STATUS_SUCCESS || caCount <= 0) {
		OMGT_OUTPUT_ERROR(port, "No hfi names found, no data to be found.\n");
		return FERROR;
	}

	for (i = 0; i < caCount; i++) {
		fstatus = umad_get_ca(ca_names[i], &Ca);

		if (0 != fstatus) {
			OMGT_OUTPUT_ERROR(port, "Cannot query CA %s: %s\n", ca_names[i], FSTATUS_MSG(fstatus));
			return FERROR;
		}

		for (j = 1; j <= Ca.numports; j++) {
			fstatus = umad_get_port(ca_names[i], j, &port_attr);
			OMGT_OUTPUT_INFO(port, "returned umad_get_port (%s,%u)...Status=%u \n", ca_names[i], j, fstatus);
			if (FSUCCESS != fstatus) {
				OMGT_OUTPUT_ERROR(port, "Failed. Returned umad_get_port (%s,%u)...Status=%u \n", ca_names[i], j, fstatus);
				umad_release_ca(&Ca);
				return FERROR;
			}

			// Check for the port
			if (portGuid == ntoh64(port_attr.port_guid)) {
				found = 1;
				if (pCaName) strncpy(pCaName, ca_names[i], UMAD_CA_NAME_LEN);
				if (caNum) *caNum = i + 1;
				if (portNum) *portNum = j;
				if (pPrefix) *pPrefix = ntoh64(port_attr.gid_prefix);
				if (pSMLid) *pSMLid = port_attr.sm_lid;
				if (pSMSL) *pSMSL = port_attr.sm_sl;
				if (pPortState) *pPortState = port_attr.state;
			} else {
				OMGT_OUTPUT_INFO(port, "Given Port guid (0x%016"PRIx64") not matched for hfi port %u guid (0x%016"PRIx64")\n",
					portGuid, j, ntoh64(port_attr.port_guid));
			}
			umad_release_port(&port_attr);
		}
		umad_release_ca(&Ca);
	}

	if (found == 1) {
		return FSUCCESS;
	} else {
		return FNOT_FOUND;
	}
} // FSTATUS omgt_get_hfi_from_portguid

/* translate ca/port number into a port Guid. Also return other useful structs
 * if non-null pointer passed-in.
 * Warning: Endian conversion not done
 *
 * INPUTS:
 *  ca       - system wide CA number 1-n
 *  port     - 1-n, 
 *  *pCaName - ca name for specified port
 *  port     - if provided, logging parameters will be extracted from this port.  
 *
 *  -h and -p options for assorted utilities (opareport, opasaquery, etc) should
 *  behave as follows:
 *    -h 0      = 1st active port in system  (p defaults to 0 or -1)
 *    -h 0 -p 0 = 1st active port in system (same as above)
 *    -h x      = 1st active port on HFI x (p defaults to 0 or -1)
 *    -h x -p 0 = 1st active port on HFI x (same as above)
 *
 *    -h 0 -p y = port y within system (inrespective of which ports are active)
 *    -h x -p y = HFI x, port y
 *
 * OUTPUTS:
 *  *pCaGuid - ca guid for specified port
 *  *pPortGuid - port guid for specified port (Warning: Endian conversion not done)
 *  *pCaAttributes - attributes for CA,
 *                  If PortAttributesList not null, caller must MemoryDeallocate pCaAttributes->PortAttributesList
 *  *ppPortAtributes - attributes for port, if ppPortAtributes not null, caller must MemoryDeallocate ppPortAtributes
 *  *pCaCount - number of CA in system
 *  *pPortCount - number of ports in system or CA (depends on ca input)
 *  *pRetCaName - pointer to CA name to return.
 *  *pRetPortNum - pointer to port number that was selected (useful in wildcard searches)
 *
 * RETURNS:
 *  FNOT_FOUND - *pCaCount and *pPortCount still output
 *              if ca == 0, *pPortCount = number of ports in system
 *              if ca < *pCaCount, *pPortCount = number of ports in CA
 *                                  otherwise *pPortCount will be 0
 *
 */
FSTATUS omgt_get_portguid(uint32 ca, uint32 port, char *pCaName, struct omgt_port *omgtport,
	EUI64 *pCaGuid, EUI64 *pPortGuid, IB_CA_ATTRIBUTES *pCaAttributes,
	IB_PORT_ATTRIBUTES **ppPortAttributes, uint32 *pCaCount, uint32 *pPortCount,
	char *pRetCaName, int *pRetPortNum, uint64 *pRetGIDPrefix)
{

    FSTATUS     fstatus = FSUCCESS;
    char        ca_names[MAX_NUM_CAS][UMAD_CA_NAME_LEN];
    umad_ca_t   umad_ca_attribs;
    umad_port_t umad_port_attribs;
    int         caCount ;
    int         found = 0;
    int         i,j,k;
	int total_ports = 0;

    OMGT_OUTPUT_INFO(omgtport, "omgt_get_portguid(%u,%u,...)... \n",ca,port );

    // Ensure proper defaults (-1 is equivalent to 0)
    if (port==-1)
        port = 0;
    if (ca==-1)
        ca = 0;

    // Default the input parameters now just in case we exit with an error.
    if (pCaCount) *pCaCount = 0;
    if (pPortCount) *pPortCount = 0;

    // get ca count and names
    OMGT_OUTPUT_INFO(omgtport, "omgt_get_portguid calling omgt_get_hfi_names()... \n" );
	if (omgt_get_hfi_names((void *)ca_names, MAX_NUM_CAS, &caCount) != OMGT_STATUS_SUCCESS || caCount <= 0) {
       OMGT_OUTPUT_ERROR(omgtport,"No hfi names found, no port GUID to find.\n" );
       return FNOT_FOUND;
    }
    OMGT_OUTPUT_INFO(omgtport, "returned omgt_get_hfi_names (%s,)...caCount=%u \n", ca_names[0], caCount);

    if (caCount > MAX_NUM_CAS) {
        OMGT_OUTPUT_ERROR(omgtport, "OFED'S omgt_get_hfi_names returned more CA's than max allowed."
                       "Cas:%d, Max:%d\n",caCount, MAX_NUM_CAS);
    }

    if (pCaCount) 
         *pCaCount = (uint32)caCount;

    // Set default number if CA requested by name...
    // Check bounds of ca requested if requested by number....
    if ((pCaName) && (*pCaName!='\0') ) {
        ca = MAX_NUM_CAS;
    }
    else if (ca > caCount )
    {
        OMGT_OUTPUT_INFO(omgtport, "Ca %u not found. Max Cas are %u\n", ca, caCount);
        return FNOT_FOUND;
    }

    // CA's and ports are numbered 1 to n where n is max num of FIs in system / ports on CA
    for (k=1, i=1; i<=caCount; i++) {

        // if ca name is supplied, determine if this ca name matches the requested
        // and set the ca number.
        if ( (pCaName) && (strncmp(pCaName, ca_names[i-1], UMAD_CA_NAME_LEN)==0) )
                ca = i;

        // Is this the CA we want (value of 0 means wildcard CA (used for first ACTIVE port))
        if ((ca==i) || (ca==0)) {

            // Get CA Info.
            OMGT_OUTPUT_INFO(omgtport,"omgt_get_portguid calling umad_get_ca(%s,)... \n", ca_names[i-1]);
            fstatus = umad_get_ca(ca_names[i-1], &umad_ca_attribs);
            OMGT_OUTPUT_INFO(omgtport,"returned omgt_get_portguid umad_get_ca(%s,) fstatus=%u \n", ca_names[i-1], fstatus);

            if (fstatus != FSUCCESS) {
                OMGT_OUTPUT_ERROR(omgtport,"Cannot query CA %s: %d\n",ca_names[i-1], fstatus);
                return fstatus;
            }
            OMGT_OUTPUT_INFO(omgtport,"returned omgt_get_portguid umad_get_ca(%s,), numports=%u \n", ca_names[i-1], umad_ca_attribs.numports);

            // Initialize port count
            if (ca != 0 && pPortCount) *pPortCount = umad_ca_attribs.numports;

            // If this request for a specific CA - update the ca structs.
            if (ca==i) {
                if (pCaGuid) *pCaGuid = umad_ca_attribs.node_guid;
                if (pRetCaName) strcpy (pRetCaName,ca_names[i-1]);

                if (pCaAttributes)
                {
                    OMGT_OUTPUT_INFO(omgtport,"omgt_get_portguid setting pCaAttributes...\n");

                    fstatus = convert_umad_ca_attribs_to_iba( &umad_ca_attribs, pCaAttributes );
                    if (FSUCCESS != fstatus) {
							  umad_release_ca(&umad_ca_attribs);
                              return fstatus;
					}
                } 

                // check bounds of port requested.
                if (port > umad_ca_attribs.numports) {
                    OMGT_OUTPUT_INFO(omgtport,"Port %u does not exist for Ca %u(%s). Max ports are %u.\n",
                                      port, ca, ca_names[i-1], umad_ca_attribs.numports);
          			umad_release_ca(&umad_ca_attribs);
                    return (FNOT_FOUND);
                }
            }


            // Update the running system count of ports
            total_ports += umad_ca_attribs.numports;
			if (found)	// already found, just counting total_ports
				continue;

            // Evaluate the ports on this CA...
            for (j=1; j <= umad_ca_attribs.numports; j++, k++) {

                // Is this a specific requested port
                if ( 
                      ((ca==0) && (port==k))  || // Specific System port request
                      ((ca==i) && (port==j))  || // Specific HFI / Port request
                      (port==0)                  // Wildcard - first ACTIVE port (in system or HFI)
                    )
                {
                    OMGT_OUTPUT_INFO(omgtport, "Calling umad_get_port (%s,%u)... \n", ca_names[i-1], j);
                    fstatus = umad_get_port( ca_names[i-1], j, &umad_port_attribs);
                    OMGT_OUTPUT_INFO(omgtport, "returned umad_get_port (%s,%u)...Status=%u \n", ca_names[i-1], j, fstatus);
                    if (FSUCCESS != fstatus)
                    {
                        OMGT_OUTPUT_ERROR(omgtport,"returned umad_get_port (%s,%u)...Status=%u \n",ca_names[i-1], j, fstatus);
						umad_release_ca(&umad_ca_attribs);
                        return fstatus;
                    }

                    // If this is a request for an active wildcard port -
                    // Check if we are active ... continue to next port if we are not active.
                    if ((port==0) && (umad_port_attribs.state!=PortStateActive))
					{
 						 umad_release_port(&umad_port_attribs);
						 continue;
					}

                    // Otherwise - we are either specifying THIS port OR
                    // this is the first ACTIVE port (on system or HFI)
                    // Initialized the pointer values.
                    found = 1;

                    // Wildcard HFI. Found first active port.  Initialize CA stuff.
                    if (ca==0) {
                        if (pCaGuid) *pCaGuid = umad_ca_attribs.node_guid;
                        if (pRetCaName) strcpy (pRetCaName,ca_names[i-1]);

                        if (pCaAttributes)
                        {
                            OMGT_OUTPUT_INFO(omgtport, "omgt_get_portguid setting pCaAttributes...\n");

                            fstatus = convert_umad_ca_attribs_to_iba( &umad_ca_attribs, pCaAttributes );
                            if (FSUCCESS != fstatus)
						    {
				 						 umad_release_port(&umad_port_attribs);
				 						 umad_release_ca(&umad_ca_attribs);
                                         return fstatus;
							}
                        } 
                    }

                    // Port Stuff
                    if (pPortGuid) *pPortGuid = ntoh64(umad_port_attribs.port_guid);
                    if (pRetPortNum) *pRetPortNum = j;
                    if (pRetGIDPrefix) * pRetGIDPrefix = ntoh64(umad_port_attribs.gid_prefix);
                    if (ppPortAttributes)
                    {
                        OMGT_OUTPUT_INFO(omgtport, "omgt_get_portguid setting ppPortAttributes...\n");

                        *ppPortAttributes = (IB_PORT_ATTRIBUTES*)
                            MemoryAllocateAndClear((ROUNDUPP2(sizeof(IB_PORT_ATTRIBUTES), 8)+sizeof(IB_GID)),
													 FALSE, OMGT_MEMORY_TAG);

                        if (*ppPortAttributes == NULL)
						{
			 						umad_release_port(&umad_port_attribs);
              		 				umad_release_ca(&umad_ca_attribs);
                                    return (FINSUFFICIENT_MEMORY);
						}
						(*ppPortAttributes)->GIDTable = (IB_GID*) ((char *)(*ppPortAttributes) +
																	 ROUNDUPP2(sizeof(IB_PORT_ATTRIBUTES), 8));
                        fstatus = convert_umad_port_attribs_to_iba( &umad_port_attribs, *ppPortAttributes );
                        if (FSUCCESS != fstatus)
					    {
			 						umad_release_port(&umad_port_attribs);
              		 				umad_release_ca(&umad_ca_attribs);
                                    return fstatus;
						}
                    }   

					umad_release_port(&umad_port_attribs);
                    break;
                }
            }

			umad_release_ca(&umad_ca_attribs);

            // If the query was for specific CA, we are done.
            if (ca==i) break;
        }
    }

    if (ca==0 && pPortCount) {
		*pPortCount = total_ports;
        OMGT_OUTPUT_INFO(omgtport, "total_ports = %u\n", total_ports);
	}

    // We found what we are looking for.
    if (found == 1) 
         return (FSUCCESS);

    // Error cases.  We did not find the port.
    // Remaining possibilites are:
    // No active port in system
    // No active port on HFI requested
    // System port number out of range.

    if (port==0) 
        if (ca==0) 
            OMGT_OUTPUT_INFO(omgtport, "No active ports found on any CA.\n");
        else
            OMGT_OUTPUT_INFO(omgtport, "No active port found on CA %d(%s).\n", ca, ca_names[ca-1]);
    else 
        if (ca==0) {
            OMGT_OUTPUT_INFO(omgtport, "System Port %d does not exist. Max system port is %d.\n",port, k-1);
	} else
            OMGT_OUTPUT_INFO(omgtport, "Port not found for parameters port:%d ca:%d.\n",port, ca);

    return (FNOT_FOUND);
}  // FSTATUS omgt_get_portguid

/* Find the ca indicated by the passed in CA guid, then allocate 
 * and fill in the ca attributes.
 *
 * INPUTS:
 *  caGuid - guid of the desired ca
 *
 * OUTPUTS:
 *  *pCaAttributes - attributes for CA,
 *                  If PortAttributesList not null, caller must
 *                  MemoryDeallocate pCaAttributes->PortAttributesList
 *
 * RETURNS:
 *  FNOT_FOUND 
 *
 */

FSTATUS omgt_query_ca_by_guid_alloc(IN  EUI64 CaGuid,OUT IB_CA_ATTRIBUTES *CaAttributes )
{
   FSTATUS fstatus = FSUCCESS;
   int i, j, count;
   char ca_names[MAX_NUM_CAS][UMAD_CA_NAME_LEN];
   umad_ca_t ca;
   umad_port_t uport;
   IB_PORT_ATTRIBUTES *cur, *prev;

   // get ca count and names
   if (omgt_get_hfi_names((void *)ca_names, MAX_NUM_CAS, &count) != OMGT_STATUS_SUCCESS || count <= 0) {
      return FNOT_FOUND;
   }


   for (i=0;i<count; ++i)
   {


      fstatus = umad_get_ca(ca_names[i], &ca);
      if (fstatus != FSUCCESS) {
         break;
      }  


      if (ca.node_guid == CaGuid)
      {
          convert_umad_ca_attribs_to_iba (&ca,CaAttributes);
          if (ca.numports >0) {
             CaAttributes->PortAttributesListSize =
					 (ROUNDUPP2(sizeof(IB_PORT_ATTRIBUTES), 8) + sizeof(IB_GID))*ca.numports;
             CaAttributes->PortAttributesList = MemoryAllocateAndClear(CaAttributes->PortAttributesListSize,
                                                                       FALSE, OMGT_MEMORY_TAG);
             if (CaAttributes->PortAttributesList == NULL)
             {
				umad_release_ca(&ca);
                return FINSUFFICIENT_MEMORY;
             }
			 cur = CaAttributes->PortAttributesList;
			 prev = NULL;
             for (j=0; j< ca.numports;j++) {
		 		 cur->GIDTable = (IB_GID*) ((char *)cur +
											 ROUNDUPP2(sizeof(IB_PORT_ATTRIBUTES), 8));
                 cur->Next = (IB_PORT_ATTRIBUTES *)((char *)cur->GIDTable + sizeof(IB_GID));
				 prev = cur;
				 cur = cur->Next;
                 fstatus = umad_get_port( ca.ca_name, j+1, &uport);
                 if (fstatus != FSUCCESS) {
                     continue;  //skip to next port
                 }
                 convert_umad_port_attribs_to_iba(&uport, prev);
				 umad_release_port(&uport);
             }
             prev->Next = NULL;
          }
		  umad_release_ca(&ca);
          break;
      }
	  umad_release_ca(&ca);
   }  

   return fstatus;
} // FSTATUS omgt_query_ca_by_guid_alloc


/* Find the ca indicated by the passed in CA guid, then 
 * fill in the ca attributes.
 *                           
 * Note: It is assumed that the CaAttributes is already populated with 
 * allocated memory for pCaAttributes->PortAttributesList
 *                           
 * INPUTS:
 *  caGuid - guid of the desired ca
 *
 * OUTPUTS:
 *  *pCaAttributes - attributes for CA,
 *                  caller responsible for allocating/deallocating memory for
 *                  pCaAttributes->PortAttributesList
 *
 * RETURNS:
 *  FNOT_FOUND 
 *
 */

FSTATUS omgt_query_ca_by_guid(IN EUI64 CaGuid, OUT IB_CA_ATTRIBUTES *CaAttributes )
{
   FSTATUS fstatus = FSUCCESS;
   int i, j, count;
   char ca_names[MAX_NUM_CAS][UMAD_CA_NAME_LEN];
   umad_ca_t ca;
   umad_port_t uport;
   IB_PORT_ATTRIBUTES *cur, *prev;

   // get ca count and names
   if (omgt_get_hfi_names((void *)ca_names, MAX_NUM_CAS, &count) != OMGT_STATUS_SUCCESS || count <= 0) {
      return FNOT_FOUND;
   }


   for (i=0;i<count; ++i)
   {


      fstatus = umad_get_ca(ca_names[i], &ca);
      if (fstatus != FSUCCESS) {
         break;
      }  


      if (ca.node_guid == CaGuid)
      {

         // ensure that the buffer is large enough to hold the ports.
         if ( ((CaAttributes->PortAttributesListSize)/(ROUNDUPP2(sizeof(IB_PORT_ATTRIBUTES), 8)+sizeof(IB_GID)))
				 < ca.numports ) {
			umad_release_ca(&ca);
            return FINSUFFICIENT_MEMORY;
         }

          convert_umad_ca_attribs_to_iba (&ca, CaAttributes);
          if (ca.numports >0) {

             // ensure that list is valid
             if (CaAttributes->PortAttributesList == NULL)
             {
				umad_release_ca(&ca);
                return FINVALID_PARAMETER;
             }
			 cur = CaAttributes->PortAttributesList;
			 prev = NULL;
             for (j=0; j< ca.numports;j++) {
		 		 cur->GIDTable = (IB_GID*) ((char *)cur +
											 ROUNDUPP2(sizeof(IB_PORT_ATTRIBUTES), 8));
                 cur->Next = (IB_PORT_ATTRIBUTES *)((char *)cur->GIDTable + sizeof(IB_GID));
				 prev = cur;
				 cur = cur->Next;
                 fstatus = umad_get_port( ca.ca_name, j+1, &uport);
                 if (fstatus != FSUCCESS) {
                     continue;  //skip to next port
                 }
                 convert_umad_port_attribs_to_iba(&uport, prev);
				 umad_release_port(&uport);
             } // for (j=0; j< ca.numports;j++) 
             prev->Next = NULL;
          } // if (ca.numports >0) 
		  umad_release_ca(&ca);
          break;
      }  // if (ca.node_guid == CaGuid)
	  umad_release_ca(&ca);
   }  // for (i=0;i<count; ++i)

   return fstatus;
} // FSTATUS omgt_query_ca_by_guid

/**
 * Convert openib/umad port attribs to ibaccess port attributes
 * Warning: Only the common attributes are being copied.
 *
 * Note: pPortAttributes->GIDTable must be pointing to valid memory to get the GIDTable
 *
 * @param pumad_port_attribs (expect pointer to type umad_port_t)
 * @param pPortAttributes
 *
 * @return FSTATUS
 */
FSTATUS convert_umad_port_attribs_to_iba(IN  void * p, OUT IB_PORT_ATTRIBUTES  *pPortAttributes)
{
    FSTATUS     fstatus = FSUCCESS;
    umad_port_t *pumad_port_attribs = p;


    if (!pPortAttributes || !pumad_port_attribs)
    {
      return FINVALID_PARAMETER;
    }

    pPortAttributes->GUID = ntoh64(pumad_port_attribs->port_guid);
    pPortAttributes->Address.BaseLID = pumad_port_attribs->base_lid;
    pPortAttributes->Address.LMC = pumad_port_attribs->lmc;

    pPortAttributes->SMAddress.LID= pumad_port_attribs->sm_lid;
    pPortAttributes->SMAddress.ServiceLevel = pumad_port_attribs->sm_sl;
    pPortAttributes->PortState = pumad_port_attribs->state;
    // pPortAttributes->PortState = pumad_port_attribs->phys_state;
    // pPortAttributes->PortState = pumad_port_attribs->rate;

	if (pPortAttributes->GIDTable) {
	   pPortAttributes->GIDTable[0].Type.Global.SubnetPrefix = ntoh64(pumad_port_attribs->gid_prefix);
	   pPortAttributes->GIDTable[0].Type.Global.InterfaceID = pPortAttributes->GUID;
	}

    // what about other attributes?  TBD

   return fstatus;

}   // convert_umad_port_attribs_to_iba()


/**
 * Convert openib/umad ca attribs to ibaccess ca attributes.
 * Warning:   Only very few (common) attributes are being copied.
 *
 * @param pumad_ca_attribs (expects pointer to type umad_ca_t)
 * @param pCaAttributes
 *
 * @return FSTATUS
 */
FSTATUS convert_umad_ca_attribs_to_iba(IN void *p, OUT IB_CA_ATTRIBUTES *pCaAttributes)
{
    FSTATUS    fstatus = FSUCCESS;
    umad_ca_t *pumad_ca_attribs = p;

    if (!pCaAttributes || !pumad_ca_attribs)
    {

      return FINVALID_PARAMETER;

    }

    pCaAttributes->GUID = pumad_ca_attribs->node_guid;
    pCaAttributes->SystemImageGuid = pumad_ca_attribs->system_guid;
    pCaAttributes->Ports = pumad_ca_attribs->numports;

    // what about other attributes?  TBD

   return fstatus;

}   // convert_umad_ca_attribs_to_iba()


////////////////////////////////////////////////////////
#endif

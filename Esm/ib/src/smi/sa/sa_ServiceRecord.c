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

** END_ICS_COPYRIGHT5   ****************************************/

/* [ICS VERSION STRING: unknown] */

//===========================================================================//
//										 //
// FILE NAME									 //
//    sa_ServiceRecord.c							 //
//										 //
// DESCRIPTION									 //
//    This file contains the routines to process the SA requests for		 //
//    records of the ServiceRecord type.					 //
//										 //
// DATA STRUCTURES								 //
//    None									 //
//										 //
// FUNCTIONS									 //
//    sa_ServiceRecord								 //
//										 //
// DEPENDENCIES									 //
//    ib_mad.h									 //
//    ib_status.h								 //
//										 //
//										 //
//===========================================================================//

#include "os_g.h"
#include "ib_mad.h"
#include "ib_sa.h"
#include "ib_status.h"
#include "cs_g.h"
#include "mai_g.h"
#include "sm_counters.h"
#include "sm_l.h"
#include "sa_l.h"
#include "sm_dbsync.h"
#include "stl_print.h"


Status_t	sa_ServiceRecord_GetTable(Mai_t *, uint32_t *);
Status_t	sa_IbServiceRecord_GetTable(Mai_t *, uint32_t *);
Status_t	sa_ServiceRecord_DoDelete(uint32_t *records, ServiceRecKey_t *srkeyp, uint8 *serviceName);
Status_t	sa_ServiceRecord_Delete(Mai_t *maip, uint32_t *records);
Status_t	sa_IbServiceRecord_Delete(Mai_t *maip, uint32_t *records);
Status_t	sa_ServiceRecord_DoAdd(uint16_t slid, STL_SERVICE_RECORD *serviceRecordp,
                       uint32_t *records, int havepkey);
Status_t	sa_ServiceRecord_Add(Mai_t *maip, uint32_t *records);
Status_t	sa_IbServiceRecord_Add(Mai_t *maip, uint32_t *records);

void dumpServices(void);

/*****************************************************************************/

/*
 * make 64 bit Hash from service record key
 */
static uint64_t
sa_ServiceRecHashFromKey(void *ky)
{
    uint64_t hkey=0, guid=0;
    ServiceRecKeyp k = (ServiceRecKeyp)ky;

    /* OR in the lower part of guid into upper part of service id */
    guid = (ntoh64(k->serviceGid.Type.Global.InterfaceID) & 
		0x00000000FFFFFFFF) << 32;
    hkey  = ((uint64_t)k->serviceId);
    hkey |= guid;
    return hkey;
}


/*
 * subscriber map compare function
 * return 1 if the 2 entries matche, 0 otherwise
 */
static int32_t
sa_ServiceRecCompare(void *key1, void *key2) {
    ServiceRecKeyp srkey1 = (ServiceRecKeyp) key1;
    ServiceRecKeyp srkey2 = (ServiceRecKeyp) key2;

    if ((0 == memcmp(&srkey1->serviceGid, &srkey2->serviceGid, sizeof(IB_GID))) &&
        srkey1->serviceId == srkey2->serviceId &&
        srkey1->servicep_key == srkey2->servicep_key) {
        return 1;  /* we have a match */
    } else {
        return 0;  /* no match found */
    }
}


/*
 * service record hash table initialization
 */
Status_t sa_ServiceRecInit(void) {
	Status_t	status=VSTATUS_OK;
	memset(&saServiceRecords, 0, sizeof(ServiceRecTable_t));
	if ((status = vs_lock_init(&saServiceRecords.serviceRecLock, VLOCK_FREE, 
		VLOCK_THREAD)) != VSTATUS_OK) {
		IB_FATAL_ERROR("can't initialize sa lock");
	} else if (NULL == (saServiceRecords.serviceRecMap = 
		cs_create_hashtable("sa_ServiceRec", 16, sa_ServiceRecHashFromKey, 
		sa_ServiceRecCompare, CS_HASH_KEY_ALLOCATED))) {
		status = VSTATUS_NOMEM;
		IB_FATAL_ERROR("sa_main: Can't allocate subscriber hash table");
	}
	return status;
}

/*
 * service record hash table delete
 */
void sa_ServiceRecDelete(void) {
	if (!saServiceRecords.serviceRecMap || vs_lock(&saServiceRecords.serviceRecLock) != VSTATUS_OK) return;
    (void) cs_hashtable_destroy(saServiceRecords.serviceRecMap, TRUE);  /* free everything */
	(void)vs_unlock(&saServiceRecords.serviceRecLock);
    (void)vs_lock_delete(&saServiceRecords.serviceRecLock);
    memset((void *)&saServiceRecords, 0, sizeof(ServiceRecTable_t));
}

/*
 * service record hash table clear
 */
void sa_ServiceRecClear(void) {
	if (!saServiceRecords.serviceRecMap || vs_lock(&saServiceRecords.serviceRecLock) != VSTATUS_OK) return;
    /* easier to destroy and recreate */
    (void) cs_hashtable_destroy(saServiceRecords.serviceRecMap, TRUE);  /* free everything */
    if (NULL == (saServiceRecords.serviceRecMap = 
                 cs_create_hashtable("sa_ServiceRec", 16, sa_ServiceRecHashFromKey, 
                                     sa_ServiceRecCompare, CS_HASH_KEY_ALLOCATED))) {
        IB_FATAL_ERROR("sa_ServiceRecClear: Can't reallocate service record hash table");
    }
	(void)vs_unlock(&saServiceRecords.serviceRecLock);
}

/*****************************************************************************/

Status_t
sa_ServiceRecord(Mai_t *maip, sa_cntxt_t *sa_cntxt) {
	uint32_t	records;
	uint16_t	attribOffset;
	uint16_t	rec_sz;

	IB_ENTER("sa_ServiceRecord", maip, 0, 0, 0);

//
//	Assume failure.
//
	records = 0;

	// Check Base and Class Version
	if (maip->base.bversion == IB_BASE_VERSION && maip->base.cversion == SA_MAD_CVERSION) {
		rec_sz = sizeof(IB_SERVICE_RECORD);
	} else {
		// Generate an error response and return.
		maip->base.status = MAD_STATUS_BAD_CLASS;
		IB_LOG_WARN_FMT(__func__, "invalid Base and/or Class Versions: Base %u, Class %u",
			maip->base.bversion, maip->base.cversion);
		(void)sa_send_reply(maip, sa_cntxt);
		IB_EXIT(__func__, VSTATUS_OK);
		return VSTATUS_OK;
	}

//
//	Check the method.  If this is a template lookup, then call the regular
//	GetTable(*) template lookup routine.  For ServiceRecord_t calls, we
//	really need to use OpaServiceRecord_t for timings.
//
	switch (maip->base.method) {
	case SA_CM_GET:        // both are searches controlled by componentmask bits
	case SA_CM_GETTABLE:
		if (maip->base.method == SA_CM_GET) {
			INCREMENT_COUNTER(smCounterSaRxGetServiceRecord);
		} else {
			INCREMENT_COUNTER(smCounterSaRxGetTblServiceRecord);
		}
		if (maip->base.cversion == STL_SA_CLASS_VERSION) {
			(void)sa_ServiceRecord_GetTable(maip, &records);
		} else {
			(void)sa_IbServiceRecord_GetTable(maip, &records);
		}
		break;
	case SA_CM_SET:
		INCREMENT_COUNTER(smCounterSaRxSetServiceRecord);
		if (maip->base.cversion == STL_SA_CLASS_VERSION) {
			(void)sa_ServiceRecord_Add(maip, &records);
		} else {
			(void)sa_IbServiceRecord_Add(maip, &records);
		}
		break;
	case SA_CM_DELETE:
		INCREMENT_COUNTER(smCounterSaRxDeleteServiceRecord);
		if (maip->base.cversion == STL_SA_CLASS_VERSION) {
			(void)sa_ServiceRecord_Delete(maip, &records);
		} else {
			(void)sa_IbServiceRecord_Delete(maip, &records);
		}
		break;
	default:
		maip->base.status = MAD_STATUS_BAD_METHOD;
		(void)sa_send_reply(maip, sa_cntxt);
		IB_LOG_WARN("sa_ServiceRecord: invalid METHOD:", maip->base.method);
		IB_EXIT("sa_ServiceRecord", VSTATUS_OK);
		return VSTATUS_OK;
		break;
	}

//
//	Determine reply status
//
	if (maip->base.status != MAD_STATUS_OK) {
		records = 0;
	} else if (records == 0) {
		maip->base.status = MAD_STATUS_SA_NO_RECORDS;
	} else if ((maip->base.method == SA_CM_GET) && (records != 1)) {
		IB_LOG_WARN("sa_ServiceRecord: too many records for SA_CM_GET:", records);
		records = 0;
		maip->base.status = MAD_STATUS_SA_TOO_MANY_RECS;
	}

//
//	Call the user exit for authorization.
//
	(void)servicerecord_userexit(maip);

	attribOffset = rec_sz + Calculate_Padding(rec_sz);
	sa_cntxt->attribLen = attribOffset;

	sa_cntxt_data(sa_cntxt, sa_data, records * attribOffset);
	sa_send_reply(maip, sa_cntxt);

	IB_EXIT("sa_ServiceRecord", VSTATUS_OK);
	return(VSTATUS_OK);
}

/* this function only returns records in STL format */
Status_t
sa_ServiceRecord_GetTable(Mai_t *maip, uint32_t *records) {
	STL_SERVICE_RECORD	*srp;
	uint32_t		temp;
	uint32_t		pad_len;
	uint64_t		now;
	STL_SA_MAD		samad;
	Status_t		status;
	OpaServiceRecord_t	*osrp;
    ServiceRecKeyp  srkeyp;
    CS_HashTableItr_t itr;
	Port_t*			reqPortp=NULL;
	Node_t*			reqNodep;
    
	IB_ENTER("sa_ServiceRecord_GetTable", maip, *records, 0, 0);

	*records = 0;
	pad_len = Calculate_Padding(sizeof(STL_SERVICE_RECORD));

//
//  Verify the size of the data received for the request
//
	if ( maip->datasize-sizeof(STL_SA_MAD_HEADER) < sizeof(STL_SERVICE_RECORD) ) {
		IB_LOG_ERROR_FMT(__func__,
			"invalid MAD length; size of STL_SERVICE_RECORD[%"PRISZT"], datasize[%d]",
			sizeof(STL_SERVICE_RECORD), (int)(maip->datasize-sizeof(STL_SA_MAD_HEADER)));
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
		IB_EXIT("sa_ServiceRecord_GetTable", MAD_STATUS_SA_REQ_INVALID);
		return (MAD_STATUS_SA_REQ_INVALID);
	}

	BSWAPCOPY_STL_SA_MAD((STL_SA_MAD*)maip->data, &samad, sizeof(STL_SERVICE_RECORD));

	(void)vs_time_get(&now);

    /* Create the template mask for the lookup */
	status = sa_create_template_mask(maip->base.aid, samad.header.mask);
	if (status != VSTATUS_OK) {
		IB_EXIT("sa_ServiceRecord_GetTable", status);
		return(status);
	}

    /* Look for this record being a duplicate of another ServiceRecord */
	srp = (STL_SERVICE_RECORD*)sa_data;
	if (vs_lock(&saServiceRecords.serviceRecLock) != VSTATUS_OK) return VSTATUS_BAD;
    if (cs_hashtable_count(saServiceRecords.serviceRecMap) > 0)
    {
		cs_hashtable_iterator(saServiceRecords.serviceRecMap, &itr);
        do {
            srkeyp = cs_hashtable_iterator_key(&itr);
            osrp = cs_hashtable_iterator_value(&itr);

			BSWAPCOPY_STL_SERVICE_RECORD(&osrp->serviceRecord, srp);
            status = sa_template_test_noinc(samad.data, (uint8_t*)srp, 
				sizeof(STL_SERVICE_RECORD));
            if (status == VSTATUS_OK) {
                if ((status = sa_check_len((uint8_t*)srp, 
					sizeof(STL_SERVICE_RECORD), pad_len)) != VSTATUS_OK) {
                    maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
                    IB_LOG_ERROR_FMT( "sa_ServiceRecord_GetTable",
                           "Reached size limit at %d records", *records);
                    break;
                }

				// Lease is given in "seconds from now" or "indefinite".
                if (osrp->expireTime != VTIMER_ETERNITY) {
                    temp = (uint32_t)((osrp->expireTime - now) / 1000000);
                } else {
                    temp = 0xffffffff;
                }
                srp->ServiceLease = ntoh32(temp);

                if (sm_smInfo.SM_Key && samad.header.smKey != sm_smInfo.SM_Key) {
					/* IBTA 1.2.1 C15-0.2-1.3 - if not trusted and pkey is defined on create, check req pkey */
					if (osrp->pkeyDefined) {
						if (!reqPortp) {
							reqPortp = sm_find_node_and_port_lid(&old_topology, maip->addrInfo.slid, 
								&reqNodep);
						}
						if (!sm_valid_port(reqPortp) ||
							reqPortp->state <= IB_PORT_DOWN ||
							!smValidatePortPKey(srkeyp->servicep_key, reqPortp)) {
							IB_LOG_WARN_FMT( "sa_ServiceRecord_GetTable", 
								"Filter serviced record ID="FMT_U64" from lid 0x%.4X "
								"due to pkey mismatch from request port",
								osrp->serviceRecord.RID.ServiceID, maip->addrInfo.slid);
							continue;
						}
					}

					/* IBTA 1.2 C15-0.2.2 - do not return real serviceKey if not trusted request */
                    memset(srp->ServiceKey, 0, SERVICE_RECORD_KEY_COUNT);
                }
				
				srp->RID.Reserved = 0;
				srp->Reserved = 0;
                /* put in outbut buffer */
                sa_increment_and_pad((uint8_t**)&srp, sizeof(STL_SERVICE_RECORD), 
					pad_len, records);
            }
        } while (cs_hashtable_iterator_advance(&itr));
    }
	(void)vs_unlock(&saServiceRecords.serviceRecLock);
	if (saDebugPerf) IB_LOG_INFINI_INFO("sa_serviceRecord_GetTable: Number of service records found is ", *records); 

	IB_EXIT("sa_ServiceRecord_GetTable", status);
	return(status);
}


Status_t
sa_IbServiceRecord_GetTable(Mai_t *maip, uint32_t *records) {
	IB_SERVICE_RECORD	*ibsrp;
	STL_SERVICE_RECORD	query;
	Status_t			status=VSTATUS_OK;
	uint32_t			temp;
	uint32_t			pad_len;
	uint64_t			now;
	IB_SA_MAD			samad;
	OpaServiceRecord_t	*osrp;
    ServiceRecKeyp		srkeyp;
    CS_HashTableItr_t	itr;
	Port_t*				reqPortp=NULL;
	Node_t*				reqNodep;
    
	IB_ENTER("sa_IbServiceRecord_GetTable", maip, *records, 0, 0);

	*records = 0;
	pad_len = Calculate_Padding(sizeof(IB_SERVICE_RECORD));
	
	BSWAPCOPY_IB_SA_MAD((IB_SA_MAD*)maip->data, &samad,
		sizeof(IB_SERVICE_RECORD));

	// The query is in IB format, the database is in STL format
	// Convert to a STL query in host byte order
	ibsrp = (IB_SERVICE_RECORD*)&samad.Data;
	memcpy(&query.RID.ServiceGID.Raw, &ibsrp->RID.ServiceGID.Raw, sizeof(query.RID.ServiceGID.Raw));
	query.RID.ServiceID = ibsrp->RID.ServiceID;
	query.RID.ServiceLID = 0; // field does not exist in IB
	query.RID.ServiceP_Key = ibsrp->RID.ServiceP_Key;
	query.ServiceLease = ibsrp->ServiceLease;
	memcpy(&query.ServiceKey,&ibsrp->ServiceKey,sizeof(query.ServiceKey));
	memcpy(&query.ServiceName,&ibsrp->ServiceName,sizeof(query.ServiceName));
	memcpy(&query.ServiceData8,&ibsrp->ServiceData8,sizeof(query.ServiceData8));
	memcpy(&query.ServiceData16,&ibsrp->ServiceData16,sizeof(query.ServiceData16));
	memcpy(&query.ServiceData32,&ibsrp->ServiceData32,sizeof(query.ServiceData32));
	memcpy(&query.ServiceData64,&ibsrp->ServiceData64,sizeof(query.ServiceData64));
	BSWAP_STL_SERVICE_RECORD(&query);

	(void)vs_time_get(&now);

	/* Look for this record being a duplicate of another ServiceRecord */
	ibsrp = (IB_SERVICE_RECORD*)sa_data;
	if (vs_lock(&saServiceRecords.serviceRecLock) != VSTATUS_OK) {
		return VSTATUS_BAD;
	} else if (cs_hashtable_count(saServiceRecords.serviceRecMap) > 0) {
		STL_SERVICE_RECORD *answer;

		cs_hashtable_iterator(saServiceRecords.serviceRecMap, &itr);
		do {
			srkeyp = cs_hashtable_iterator_key(&itr);
			osrp = cs_hashtable_iterator_value(&itr);
			answer = &osrp->serviceRecord;

			if (samad.SaHdr.ComponentMask & SR_COMPONENTMASK_ID &&
				query.RID.ServiceID != answer->RID.ServiceID) {
				continue;
			} else if (samad.SaHdr.ComponentMask & SR_COMPONENTMASK_PKEY &&
				query.RID.ServiceP_Key != answer->RID.ServiceP_Key) {
				continue;
			} else if (samad.SaHdr.ComponentMask & SR_COMPONENTMASK_GID &&
				memcmp(query.RID.ServiceGID.Raw, 
					answer->RID.ServiceGID.Raw, 
					sizeof(IB_GID))) {
				continue;
			} else if (samad.SaHdr.ComponentMask & SR_COMPONENTMASK_LEASE &&
				query.ServiceLease != answer->ServiceLease) {
				continue;
			} else if (samad.SaHdr.ComponentMask & SR_COMPONENTMASK_KEY &&
				memcmp(query.ServiceKey, answer->ServiceKey, sizeof(query.ServiceKey))) {
				continue;
			} else if (samad.SaHdr.ComponentMask & SR_COMPONENTMASK_NAME &&
				memcmp(query.ServiceName, answer->ServiceName, sizeof(query.ServiceName))) {
				continue;
			}
			if ((status = sa_check_len((uint8_t*)ibsrp, 
				sizeof(IB_SERVICE_RECORD), pad_len)) != VSTATUS_OK) {
				maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
				IB_LOG_ERROR_FMT( "sa_IbServiceRecord_GetTable",
					   "Reached size limit at %d records", *records);
				   break;
			   }

			// Lease is given in "seconds from now" or "indefinite".
			if (osrp->expireTime != VTIMER_ETERNITY) {
				temp = (uint32_t)((osrp->expireTime - now) / 1000000);
			} else {
				temp = 0xffffffff;
			}

			//Map back to IB, and to network byte order
			memcpy(ibsrp->RID.ServiceGID.Raw, answer->RID.ServiceGID.Raw, sizeof(ibsrp->RID.ServiceGID.Raw));
			ibsrp->RID.ServiceID = answer->RID.ServiceID;
			// Field does not exist in IB: = hton32(answer->RID.ServiceLID);
			ibsrp->RID.ServiceP_Key = answer->RID.ServiceP_Key;
			ibsrp->ServiceLease = temp;
			memcpy(ibsrp->ServiceKey,answer->ServiceKey,sizeof(ibsrp->ServiceKey));
			memcpy(ibsrp->ServiceName,answer->ServiceName,sizeof(ibsrp->ServiceName));
			memcpy(ibsrp->ServiceData8,answer->ServiceData8,sizeof(ibsrp->ServiceData8));
			memcpy(ibsrp->ServiceData16,answer->ServiceData16,sizeof(ibsrp->ServiceData16));
			memcpy(ibsrp->ServiceData32,answer->ServiceData32,sizeof(ibsrp->ServiceData32));
			memcpy(ibsrp->ServiceData64,answer->ServiceData64,sizeof(ibsrp->ServiceData64));
			BSWAP_IB_SERVICE_RECORD(ibsrp);

			if (sm_smInfo.SM_Key && samad.SaHdr.SmKey != sm_smInfo.SM_Key) {
				/* IBTA 1.2.1 C15-0.2-1.3 - if not trusted and pkey is defined on create, check req pkey */
				if (osrp->pkeyDefined) {
					reqPortp = sm_find_node_and_port_lid(&old_topology, 
						maip->addrInfo.slid, 
						&reqNodep);

					if (!sm_valid_port(reqPortp) ||
						reqPortp->state <= IB_PORT_DOWN ||
						!smValidatePortPKey(srkeyp->servicep_key, reqPortp)) {
						IB_LOG_WARN_FMT( "sa_IbServiceRecord_GetTable", 
							"Filter serviced record ID="FMT_U64" from lid "
							"0x%.4X due to pkey mismatch from request port",
							osrp->serviceRecord.RID.ServiceID, 
							maip->addrInfo.slid);
						continue;
					}
				}

				/* IBTA 1.2 C15-0.2.2 - do not return real serviceKey if not trusted request */
				memset(ibsrp->ServiceKey, 0, SERVICE_RECORD_KEY_COUNT);
			}
			/* put in outbut buffer */
			sa_increment_and_pad((uint8_t**)&ibsrp, sizeof(IB_SERVICE_RECORD), 
				pad_len, records);
		} while (cs_hashtable_iterator_advance(&itr));
	}
	(void)vs_unlock(&saServiceRecords.serviceRecLock);
	if (saDebugPerf) {
		IB_LOG_INFINI_INFO("sa_IbServiceRecord_GetTable: "
			"Number of service records found is ", *records); 
	}

	IB_EXIT("sa_IbServiceRecord_GetTable", status);
	return(status);
}


Status_t
sa_ServiceRecord_Delete(Mai_t *maip, uint32_t *records) {
	STL_SA_MAD			samad;
	STL_SERVICE_RECORD	serviceRecord;
    ServiceRecKey_t		srkey;
	Status_t			status;

	IB_ENTER("sa_ServiceRecord_Delete", maip, *records, 0, 0);

//
//  Verify the size of the data received for the request
//
	if ( maip->datasize-sizeof(STL_SA_MAD_HEADER) < sizeof(STL_SERVICE_RECORD) ) {
		IB_LOG_ERROR_FMT(__func__,
			"invalid MAD length; size of STL_SERVICE_RECORD[%"PRISZT"], datasize[%d]",
			sizeof(STL_SERVICE_RECORD), (int)(maip->datasize-sizeof(STL_SA_MAD_HEADER)));
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
		IB_EXIT("sa_ServiceRecord_Delete", MAD_STATUS_SA_REQ_INVALID);
		return (MAD_STATUS_SA_REQ_INVALID);
	}

	BSWAPCOPY_STL_SA_MAD((STL_SA_MAD*)maip->data, &samad, sizeof(STL_SERVICE_RECORD));
	BSWAPCOPY_STL_SERVICE_RECORD((STL_SERVICE_RECORD*)&samad.data, &serviceRecord);
    /* fill in a service record key for searching hash table */
    memcpy(&srkey.serviceGid, &serviceRecord.RID.ServiceGID, sizeof(IB_GID));
    srkey.serviceId = serviceRecord.RID.ServiceID;
    srkey.servicep_key = serviceRecord.RID.ServiceP_Key;

	status =  sa_ServiceRecord_DoDelete(records, &srkey, serviceRecord.ServiceName);
	if (*records == 0)
		maip->base.status = MAD_STATUS_SA_NO_RECORDS;
	IB_EXIT("sa_ServiceRecord_Delete", status);
	return status;
}

Status_t
sa_ServiceRecord_DoDelete(uint32_t *records, ServiceRecKey_t *srkeyp, uint8 *serviceName) {
	OpaServiceRecordp	osrp;

    /*
     * Since lease period changes between a Get(*) and Set(*), can't do straight 
     * comparison of the structs.  We just compare serviceGid, name, and serviceId.
     */

	*records = 0;

    /* lock out service record hash table */
	if (vs_lock(&saServiceRecords.serviceRecLock) != VSTATUS_OK) return VSTATUS_BAD;
    if (NULL == (osrp = (OpaServiceRecordp)cs_hashtable_remove(saServiceRecords.serviceRecMap, srkeyp))) {
		IB_LOG_VERBOSE_FMT( "sa_serviceRecord_DoDelete",
			"Could not find service record ID="FMT_U64" for GID="FMT_GID", Name=%.63s", 
			srkeyp->serviceId, 
			ntoh64(srkeyp->serviceGid.AsReg64s.L),
			ntoh64(srkeyp->serviceGid.AsReg64s.H), 
			serviceName);
    } else {
        *records = 1;  /* let caller know how many deleted */
		if (saDebugPerf) {
			IB_LOG_INFINI_INFO_FMT( "sa_serviceRecord_DoDelete",
			"Deleted service record ID="FMT_U64" for GID="FMT_GID", Name=%s", 
			srkeyp->serviceId, 
			ntoh64(osrp->serviceRecord.RID.ServiceGID.AsReg64s.L),
			ntoh64(osrp->serviceRecord.RID.ServiceGID.AsReg64s.H), 
			osrp->serviceRecord.ServiceName);
		}
        /* sync the service record deletion to standby SMs if necessary */
        (void)sm_dbsync_syncService(DBSYNC_TYPE_DELETE, osrp);
        /* free the actual ServiceRecord - remove only frees the key */
        free(osrp);
    }
    (void)vs_unlock(&saServiceRecords.serviceRecLock);

	IB_EXIT("sa_ServiceRecord_DoDelete", VSTATUS_OK);
	return(VSTATUS_OK);
}

Status_t
sa_IbServiceRecord_Delete(Mai_t *maip, uint32_t *records) {
	IB_SA_MAD			samad;
	IB_SERVICE_RECORD	serviceRecord;
    ServiceRecKey_t		srkey;
	Status_t			status;

	IB_ENTER("sa_IbServiceRecord_Delete", maip, *records, 0, 0);

//
//  Verify the size of the data received for the request
//
	if ( maip->datasize-sizeof(SA_MAD_HDR) < sizeof(IB_SERVICE_RECORD) ) {
		IB_LOG_ERROR_FMT(__func__,
			"invalid MAD length; size of IB_SERVICE_RECORD[%"PRISZT"], datasize[%d]",
			sizeof(IB_SERVICE_RECORD), (int)(maip->datasize-sizeof(SA_MAD_HDR)));
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
		IB_EXIT("sa_IbServiceRecord_Delete", MAD_STATUS_SA_REQ_INVALID);
		return (MAD_STATUS_SA_REQ_INVALID);
	}

	BSWAPCOPY_IB_SA_MAD((IB_SA_MAD*)maip->data, &samad, sizeof(IB_SERVICE_RECORD));
	serviceRecord = *(IB_SERVICE_RECORD*)&samad.Data;
	BSWAP_IB_SERVICE_RECORD(&serviceRecord);

    /* fill in a service record key for searching hash table */
    memcpy(&srkey.serviceGid, &serviceRecord.RID.ServiceGID, sizeof(IB_GID));
    srkey.serviceId = serviceRecord.RID.ServiceID;
    srkey.servicep_key = serviceRecord.RID.ServiceP_Key;

	status =  sa_ServiceRecord_DoDelete(records, &srkey, serviceRecord.ServiceName);
	if (*records == 0)
		maip->base.status = MAD_STATUS_SA_NO_RECORDS;
	IB_EXIT("sa_IbServiceRecord_Delete", status);
	return status;
}

Status_t
sa_ServiceRecord_Add(Mai_t *maip, uint32_t *records) {
	STL_SA_MAD			samad;
	STL_SERVICE_RECORD	serviceRecord;
	Status_t		status;

	IB_ENTER("sa_ServiceRecord_Add", maip, *records, 0, 0);

//
//  Verify the size of the data received for the request
//
	if ( maip->datasize-sizeof(STL_SA_MAD_HEADER) < sizeof(STL_SERVICE_RECORD) ) {
		IB_LOG_ERROR_FMT(__func__,
			"invalid MAD length; size of STL_SERVICE_RECORD[%"PRISZT"], datasize[%d]",
			sizeof(STL_SERVICE_RECORD), (int)(maip->datasize-sizeof(STL_SA_MAD_HEADER)));
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
		IB_EXIT("sa_ServiceRecord_Add", MAD_STATUS_SA_REQ_INVALID);
		return (MAD_STATUS_SA_REQ_INVALID);
	}

	BSWAPCOPY_STL_SA_MAD((STL_SA_MAD*)maip->data, &samad, sizeof(STL_SERVICE_RECORD));
	BSWAPCOPY_STL_SERVICE_RECORD((STL_SERVICE_RECORD*)samad.data, &serviceRecord);
	status = sa_ServiceRecord_DoAdd(maip->addrInfo.slid, &serviceRecord,
							 records, 
							 0 != (samad.header.mask & STL_SERVICE_RECORD_COMP_SERVICEPKEY));
	if (*records == 1)
		(void)memcpy((void *)sa_data, samad.data, sizeof(STL_SERVICE_RECORD));
				
	IB_EXIT("sa_ServiceRecord_Add", status);
	return(status);
}

Status_t
sa_ServiceRecord_DoAdd(uint16_t slid, STL_SERVICE_RECORD *serviceRecordp,
                       uint32_t *records, int havepkey) {
	uint32_t		timer;
	uint64_t		now;
    ServiceRecKeyp  srkeyp;
	OpaServiceRecordp osrp;
	Port_t*			servicePortp;
	Port_t*			reqPortp;
	Node_t*			reqNodep;

	timer = serviceRecordp->ServiceLease;
	(void)vs_time_get(&now);
    *records = 0;
	
	// protect outselves
	serviceRecordp->ServiceName[sizeof(serviceRecordp->ServiceName)-1] = '\0';

    /* 
     * see if this entry already exist in the hash table
     * if it does, replace the service record data with the incoming
     * if it does not, create a new one and add to the hash table
     */

    /* allocate a service record key for searching hash table */
    srkeyp = (ServiceRecKeyp) malloc(sizeof(ServiceRecKey_t));
    if (srkeyp == NULL) {
        IB_FATAL_ERROR("sa_ServiceRecord_Add: Can't allocate service record key");
        return VSTATUS_NOMEM;
    }
    memcpy(&srkeyp->serviceGid, &serviceRecordp->RID.ServiceGID, sizeof(IB_GID));
    srkeyp->servicep_key = serviceRecordp->RID.ServiceP_Key;
    srkeyp->serviceId = serviceRecordp->RID.ServiceID;

	if (havepkey) {
		/* C15-02-1.3 */
		if (PKEY_VALUE(srkeyp->servicep_key) == 0) {
			IB_LOG_WARN_FMT( "sa_ServiceRecord_Add", 
				"Failed to ADD serviced record ID="FMT_U64" from lid 0x%.4X due to invalid pkey",
				serviceRecordp->RID.ServiceID, slid);
			free(srkeyp);
			return(VSTATUS_BAD);
		}

		/* C15-0.2-1.4 */
		reqPortp = sm_find_node_and_port_lid(&old_topology, slid, &reqNodep);
		if (!sm_valid_port(reqPortp) ||
			reqPortp->state <= IB_PORT_DOWN ||
			!smValidatePortPKey(srkeyp->servicep_key, reqPortp)) {
			IB_LOG_WARN_FMT( "sa_ServiceRecord_Add", 
				"Failed to ADD serviced record ID="FMT_U64" from lid 0x%.4X due to pkey mismatch from request port",
				serviceRecordp->RID.ServiceID, slid);
			free(srkeyp);
			return(VSTATUS_BAD);
		}
		servicePortp = sm_find_port_guid(&old_topology, serviceRecordp->RID.ServiceGID.Type.Global.InterfaceID);
		if (!sm_valid_port(servicePortp) ||
			servicePortp->state <= IB_PORT_DOWN ||
			!smValidatePortPKey(srkeyp->servicep_key, servicePortp)) {
			IB_LOG_WARN_FMT( "sa_ServiceRecord_Add", 
				"Failed to ADD serviced record ID="FMT_U64" from lid 0x%.4X due to pkey mismatch from service port",
				serviceRecordp->RID.ServiceID, slid);
			free(srkeyp);
			return(VSTATUS_BAD);
		}
	}

    /* lock out service record hash table */
	if (vs_lock(&saServiceRecords.serviceRecLock) != VSTATUS_OK) {
        free(srkeyp);
        return VSTATUS_NOMEM;
    }
    if (NULL == (osrp = (OpaServiceRecordp)cs_hashtable_search(saServiceRecords.serviceRecMap, srkeyp))) {
        /* allocate a service record for adding to hash table */
        if ((osrp = (OpaServiceRecordp) malloc(sizeof(OpaServiceRecord_t))) == NULL) {
            free(srkeyp);
            (void)vs_unlock(&saServiceRecords.serviceRecLock);
            IB_FATAL_ERROR("sa_ServiceRecord_Add: Can't allocate Service Record hash entry");
            IB_EXIT("sa_ServiceRecord_Add - memory allocation failure", VSTATUS_NOMEM);
            return VSTATUS_NOMEM;
        }
        osrp->serviceRecord = *serviceRecordp;
        osrp->pkeyDefined = havepkey ? 1 : 0;

        if (timer != 0xffffffff) {
            osrp->expireTime = now + (1000000 * (uint64_t)timer);
        } else {
            osrp->expireTime = VTIMER_ETERNITY;
        }
        if (!cs_hashtable_insert(saServiceRecords.serviceRecMap, srkeyp, osrp)) {
            (void)vs_unlock(&saServiceRecords.serviceRecLock);
            IB_LOG_ERROR_FMT( "sa_ServiceRecord_Add", 
                   "Failed to ADD serviced record ID="FMT_U64", serviceName[%s] from lid 0x%.4X to service record hashtable",
                   osrp->serviceRecord.RID.ServiceID, osrp->serviceRecord.ServiceName, slid);
            free(srkeyp);
            free(osrp);
            IB_EXIT("sa_InformInfo_Subscribe - hashtable insert failure", VSTATUS_BAD);
            return(VSTATUS_BAD);
        }
        *records = 1;  /* let caller know record added */
        if (saDebugPerf) {
            IB_LOG_INFINI_INFO_FMT( "sa_ServiceRecord_Add",
				"Added service record for Gid "FMT_GID", ID="FMT_U64", serviceName[%s]", 
				STLGIDPRINTARGS2(osrp->serviceRecord.RID.ServiceGID.Raw),
				osrp->serviceRecord.RID.ServiceID, 
				osrp->serviceRecord.ServiceName);
        }
    } else {
        /* entry existed already - replace value (serviceRecord) with new input and free allocated key */
        osrp->serviceRecord = *serviceRecordp;
        osrp->pkeyDefined = havepkey ? 1 : 0;
        free(srkeyp);
		/* Update the expiration timer since this is now a duplicate record */
		if (timer != 0xffffffff) {
			osrp->expireTime = now + (1000000 * (uint64_t)timer);
		} else {
			osrp->expireTime = VTIMER_ETERNITY;
		}
		if (saDebugPerf) {
            IB_LOG_INFINI_INFO_FMT( "sa_ServiceRecord_Add",
                   "Updated service record for ID="FMT_U64", serviceName[%s]", 
                   osrp->serviceRecord.RID.ServiceID, osrp->serviceRecord.ServiceName);
        }
		*records = 1;  /* let caller know how many added */
    }
    /* sync the service record change to standby SMs if necessary */
    (void)sm_dbsync_syncService(DBSYNC_TYPE_UPDATE, osrp);
	(void)vs_unlock(&saServiceRecords.serviceRecLock);

	IB_EXIT("sa_ServiceRecord_Add", VSTATUS_OK);
	return(VSTATUS_OK);
}

Status_t
sa_IbServiceRecord_Add(Mai_t *maip, uint32_t *records) {
	IB_SA_MAD			samad;
	IB_SERVICE_RECORD	*ibsrp;
	STL_SERVICE_RECORD	serviceRecord;
	Status_t		status;

	IB_ENTER("sa_IbServiceRecord_Add", maip, *records, 0, 0);

//
//  Verify the size of the data received for the request
//
	if ( maip->datasize-sizeof(SA_MAD_HDR) < sizeof(IB_SERVICE_RECORD) ) {
		IB_LOG_ERROR_FMT(__func__,
			"invalid MAD length; size of IB_SERVICE_RECORD[%"PRISZT"], datasize[%d]",
			sizeof(IB_SERVICE_RECORD), (int)(maip->datasize-sizeof(SA_MAD_HDR)));
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
		IB_EXIT("sa_IbServiceRecord_Add", MAD_STATUS_SA_REQ_INVALID);
		return (MAD_STATUS_SA_REQ_INVALID);
	}

	BSWAPCOPY_IB_SA_MAD((IB_SA_MAD*)maip->data, &samad, sizeof(IB_SERVICE_RECORD));
	// The service record is in IB format, the database is in STL format
	// Convert to a STL service record in host byte order
	ibsrp = (IB_SERVICE_RECORD*)&samad.Data;
	memcpy(&serviceRecord.RID.ServiceGID.Raw, &ibsrp->RID.ServiceGID.Raw, sizeof(serviceRecord.RID.ServiceGID.Raw));
	serviceRecord.RID.ServiceID = ibsrp->RID.ServiceID;
	serviceRecord.RID.ServiceLID = 0; // field does not exist in IB
	serviceRecord.RID.ServiceP_Key = ibsrp->RID.ServiceP_Key;
	serviceRecord.ServiceLease = ibsrp->ServiceLease;
	memcpy(&serviceRecord.ServiceKey,&ibsrp->ServiceKey,sizeof(serviceRecord.ServiceKey));
	memcpy(&serviceRecord.ServiceName,&ibsrp->ServiceName,sizeof(serviceRecord.ServiceName));
	memcpy(&serviceRecord.ServiceData8,&ibsrp->ServiceData8,sizeof(serviceRecord.ServiceData8));
	memcpy(&serviceRecord.ServiceData16,&ibsrp->ServiceData16,sizeof(serviceRecord.ServiceData16));
	memcpy(&serviceRecord.ServiceData32,&ibsrp->ServiceData32,sizeof(serviceRecord.ServiceData32));
	memcpy(&serviceRecord.ServiceData64,&ibsrp->ServiceData64,sizeof(serviceRecord.ServiceData64));
	BSWAP_STL_SERVICE_RECORD(&serviceRecord);

	status = sa_ServiceRecord_DoAdd(maip->addrInfo.slid, &serviceRecord,
							 records,
							 0 != (samad.SaHdr.ComponentMask & SR_COMPONENTMASK_PKEY));
	if (*records == 1)
		(void)memcpy((void *)sa_data, samad.Data, sizeof(IB_SERVICE_RECORD));
				
	IB_EXIT("sa_IbServiceRecord_Add", status);
	return(status);
}

/*
 *	clear the records whose timers have expired.
 */
Status_t
sa_ServiceRecord_Age(uint32_t *records) {
	uint32_t		moreEntries;
	uint64_t		now;
	OpaServiceRecord_t	*osrp;
    CS_HashTableItr_t itr;

	IB_ENTER("sa_ServiceRecord_Age", *records, 0, 0, 0);

	(void)vs_time_get(&now);
    *records=0;
	if (!saServiceRecords.serviceRecMap || vs_lock(&saServiceRecords.serviceRecLock) != VSTATUS_OK) 
        return VSTATUS_BAD;
    if (cs_hashtable_count(saServiceRecords.serviceRecMap) > 0)
    {
		cs_hashtable_iterator(saServiceRecords.serviceRecMap, &itr);
        do {
            osrp = cs_hashtable_iterator_value(&itr);
            if (osrp->expireTime < now) {
                /* remove record from hash table */
                moreEntries = cs_hashtable_iterator_remove(&itr);
                ++(*records);
                /* sync the service record deletion to standby SMs if necessary */
                (void)sm_dbsync_syncService(DBSYNC_TYPE_DELETE, osrp);
				free(osrp);
            } else
                moreEntries = cs_hashtable_iterator_advance(&itr);
        } while (moreEntries);
    }
	(void)vs_unlock(&saServiceRecords.serviceRecLock);

	IB_EXIT("sa_ServiceRecord_Age", VSTATUS_OK);
	return(VSTATUS_OK);
}

#ifdef __VXWORKS__

void dumpServices(void)
{
	OpaServiceRecordp  osrp;
    CS_HashTableItr_t   itr;
    uint32_t            numservrecs=0;
	uint32_t i = 0;
    PrintDest_t dest;

	if (topology_passcount < 1)
	{
		sysPrintf("\nSM is currently in the %s state, count = %d\n\n", sm_getStateText(sm_state), (int)sm_smInfo.ActCount);
		return;
	}
	if (!saServiceRecords.serviceRecMap || vs_lock(&saServiceRecords.serviceRecLock)) {
		sysPrintf("Fabric Manager is not running!\n");
		return;
	}
    numservrecs = cs_hashtable_count(saServiceRecords.serviceRecMap);
    if (numservrecs > 0)
    {
		cs_hashtable_iterator(saServiceRecords.serviceRecMap, &itr);
        //sysPrintf("******************************************************************\n");
        //sysPrintf("                  There are %d Service Records  \n", (int)numservrecs);
        PrintDestInitFile(&dest, stdout);
        //sysPrintf("******************************************************************\n");

        do {
            osrp = cs_hashtable_iterator_value(&itr);
			if (i++) PrintSeparator(&dest);
            (void)PrintStlServiceRecord(&dest, 0, &osrp->serviceRecord);
        } while (cs_hashtable_iterator_advance(&itr));
    }
    //sysPrintf("******************************************************************\n");
    //sysPrintf("                  There are %d Service Records  \n", (int)numservrecs);
    //sysPrintf("******************************************************************\n");
	(void)vs_unlock(&saServiceRecords.serviceRecLock);
}
#endif

STL_SERVICE_RECORD*
getNextService(uint64_t *serviceId, IB_GID *serviceGid, uint16_t *servicep_key, STL_SERVICE_RECORD *pSrp)
{
	int nextOneIsIt=0;
	STL_SERVICE_RECORD *srp = NULL;
    ServiceRecKey_t  srkey;
    ServiceRecKeyp   srkeyp;
	OpaServiceRecordp osrp;
    CS_HashTableItr_t itr;;

	if (!saServiceRecords.serviceRecMap || vs_lock(&saServiceRecords.serviceRecLock)) {
		sysPrintf("Fabric Manager is not running!\n");
		return NULL;
	}
    /* fill in a service record key for searching hash table */
    memcpy(srkey.serviceGid.Raw, serviceGid->Raw, sizeof(IB_GID));
    srkey.serviceId = *serviceId;
    srkey.servicep_key = *servicep_key;
	if (!srkey.serviceId && !srkey.servicep_key) {
		/* return first entry if no key was passed in */
		nextOneIsIt = 1;
	}
    if (cs_hashtable_count(saServiceRecords.serviceRecMap) > 0)
    {
		cs_hashtable_iterator(saServiceRecords.serviceRecMap, &itr);
        do {
            srkeyp = cs_hashtable_iterator_key(&itr);
            osrp = cs_hashtable_iterator_value(&itr);
            if (nextOneIsIt) {
                memcpy(pSrp, &osrp->serviceRecord, sizeof(STL_SERVICE_RECORD));
                srp = pSrp;
                break;
            } else {
                /* see if this entry matches */
                if (srkey.serviceId == srkeyp->serviceId &&
					srkey.servicep_key == srkeyp->servicep_key &&
					!memcmp(&srkey.serviceGid, &srkeyp->serviceGid, sizeof(srkeyp->serviceGid))) {
					nextOneIsIt = 1;
				}
            }
        } while (cs_hashtable_iterator_advance(&itr));
    }
	(void)vs_unlock(&saServiceRecords.serviceRecLock);
    return srp;
}



STL_SERVICE_RECORD*
getService(uint64_t *serviceId, IB_GID *serviceGid, uint16_t *servicep_key, STL_SERVICE_RECORD *pSrp)
{
	STL_SERVICE_RECORD *srp = NULL;
    ServiceRecKey_t  srkey;
	OpaServiceRecordp osrp;

	if (!saServiceRecords.serviceRecMap || vs_lock(&saServiceRecords.serviceRecLock)) {
		sysPrintf("Fabric Manager is not running!\n");
		return NULL;
	}
    /* fill in a service record key for searching hash table */
    memcpy(srkey.serviceGid.Raw, serviceGid->Raw, sizeof(IB_GID));
    srkey.serviceId = *serviceId;
    srkey.servicep_key = *servicep_key;

    /* lock out service record hash table */
    if ((osrp = (OpaServiceRecordp)cs_hashtable_search(saServiceRecords.serviceRecMap, &srkey))) {
        memcpy(pSrp, &osrp->serviceRecord, sizeof(STL_SERVICE_RECORD));
        srp = pSrp;
    }
	(void)vs_unlock(&saServiceRecords.serviceRecLock);
    return srp;
}

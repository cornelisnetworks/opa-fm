/* BEGIN_ICS_COPYRIGHT4 ****************************************

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

** END_ICS_COPYRIGHT4   ****************************************/

#ifdef IB_DEBUG
#define ATOMIC_TEST 1	// perform tests of Atomics
#endif

#include "precomp.h"


//
// Forward declarations
//

FSTATUS
vcaResourceInit(
	IN DEVICE_INFO	*DevInfo
	);

FSTATUS
vcaResourceFree(
	IN DEVICE_INFO	*DevInfo
	);



#ifdef ALLOC_PRAGMA
#pragma alloc_text(PAGE, vcaResourceInit)
#pragma alloc_text(PAGE, vcaResourceFree)
#endif


//
// Globals
//

VCA_GLOBAL	VcaGlobal =
{
	NULL,
	{}
};

#ifdef ICS_LOGGING
_IB_DBG_PARAM_BLOCK( (_DBG_LVL_FATAL | _DBG_LVL_ERROR), \
	_DBG_BREAK_DISABLE, VCA_MEM_TAG, "Vca", MOD_VCA, Vca);
#else
_IB_DBG_PARAM_BLOCK( (_DBG_LVL_FATAL | _DBG_LVL_ERROR), \
	_DBG_BREAK_DISABLE, VCA_MEM_TAG, "Vca");
#endif
IBT_COMPONENT_INFO	ComponentInfo[IbtMaxComponents];


//
// The components of the IBT to be loaded. The structure is initialized in the
// platform specific code.
//

extern IBT_LOAD ComponentLoad[];


//
// Dummy callback routines for VCA internally allocated resources
//

void
vcaCompletionCallback(
	IN void *CaContext,
	IN void *CqContext
	)
{
}

void
vcaAsyncEventCallback(
	IN void *CaContext,
	IN IB_EVENT_RECORD *EventRecord
	)
{
}


//
// Perform initialization of the VCA for a new Channel Adapter
//

FSTATUS
vcaInitCA(
	IN DEVICE_INFO	*DevInfo
	)
{
	int32	i;
	FSTATUS	status = FSUCCESS;

	_DBG_ENTER_EXT(_DBG_LVL_PNP, vcaInitCA, == PASSIVE);

	status = vcaResourceInit(DevInfo);
	if (FSUCCESS != status)
		goto failresource;

	status = vcaAddCA(DevInfo);
	if (FSUCCESS != status)
		goto failadd;

	for (i = 0; i<IbtMaxComponents; i++)
	{
		if ( ComponentInfo[i].AddDevice != NULL )
		{
			status = (*ComponentInfo[i].AddDevice)(DevInfo->VpInterface.CaGUID,
				&DevInfo->ComponentCAContexts[i]);
			if ( status != FSUCCESS )
				goto failcomp;
		}
	}

done:
	_DBG_LEAVE_EXT(_DBG_LVL_PNP);

	return status;

failcomp:
	/* i= component which failed */
	// in reverse order of Init
	for (--i; i >= 0; i--)
	{
		if ( ComponentInfo[i].RemoveDevice != NULL )
		{
			(void)(ComponentInfo[i].RemoveDevice)(DevInfo->VpInterface.CaGUID,
				DevInfo->ComponentCAContexts[i]);
		}
	}
	vcaRemoveCA(DevInfo);
failadd:
	vcaResourceFree(DevInfo);
failresource:
	goto done;
}


//
// 
// Unwind from vcaInitCA's initialization of the VCA for a Channel Adapter
//

FSTATUS
vcaFreeCA(
	IN DEVICE_INFO	*DevInfo
	)
{
	int	i;
	FSTATUS	status = FSUCCESS;

	_DBG_ENTER_EXT(_DBG_LVL_PNP, vcaFreeCA, == PASSIVE);

	// in reverse order of Init
	for (i = IbtMaxComponents-1; i >= 0; i--)
	{
		if ( ComponentInfo[i].RemoveDevice != NULL )
		{
			status = (ComponentInfo[i].RemoveDevice)(DevInfo->VpInterface.CaGUID,
				DevInfo->ComponentCAContexts[i]);
			if (FSUCCESS != status)
				break;
		}
	}

	// Free internal resources allocated on device and remove from the global
	// device list.
	vcaRemoveCA(DevInfo);
	vcaResourceFree(DevInfo);

	_DBG_LEAVE_EXT(_DBG_LVL_PNP);

    return status;
}


//
//	Fill out fields in DEVICE_INFO
//

// this is called before adding the DevInfo to the global list
// it completes initialization of DevInfo Ca handle, and attributes
// it also ensures the CA has no port guids which duplicate existing port guids
FSTATUS
vcaResourceInit(
	IN DEVICE_INFO	*DevInfo
	)
{
	FSTATUS status = FSUCCESS;
	IB_PORT_ATTRIBUTES *portAttributes;

	_DBG_ENTER_EXT(_DBG_LVL_PNP, vcaResourceInit, == PASSIVE);

	status = vcaOpenCa(DevInfo, vcaCompletionCallback, vcaAsyncEventCallback,
				&DevInfo, &DevInfo->VcaCaHandle);
	if ( status != FSUCCESS )
	{
		_DBG_ERROR(("failed to open CA\n"));
		goto failopen;
	}
	status = SetCAInternal(DevInfo->VcaCaHandle);
	if ( status != FSUCCESS )
	{
		_DBG_ERROR(("failed to set CA as Internal\n"));
		goto failinternal;
	}

	DevInfo->CaAttributes.PortAttributesListSize = 0;
	DevInfo->CaAttributes.PortAttributesList = NULL;
	status = iba_query_ca(DevInfo->VcaCaHandle, &DevInfo->CaAttributes, NULL);
	if ( status != FSUCCESS )
	{
		_DBG_ERROR(("failed to query CA\n"));
		goto failquery;
	}
	if ( 0 == DevInfo->CaAttributes.Ports)
	{
		_DBG_ERROR(("CA reports No Ports\n"));
		goto failquery;
	}
	if (0 == DevInfo->CaAttributes.PortAttributesListSize)
	{
		_DBG_ERROR(("CA reports No Port Attributes\n"));
		goto failquery;
	}
	DevInfo->CaAttributes.PortAttributesList =
						(IB_PORT_ATTRIBUTES*)MemoryAllocate2AndClear(
								DevInfo->CaAttributes.PortAttributesListSize,
								IBA_MEM_FLAG_PREMPTABLE, VCA_MEM_TAG);
	if (! DevInfo->CaAttributes.PortAttributesList)
	{
		_DBG_ERROR(("failed to allocate memory for CA port attributes\n"));
		status = FINSUFFICIENT_MEMORY;
		goto failportalloc;
	}
	status = iba_query_ca(DevInfo->VcaCaHandle, &DevInfo->CaAttributes, NULL);
	if ( status != FSUCCESS )
	{
		_DBG_ERROR(("failed to query CA for port attributes\n"));
		goto failqueryports;
	}

	// make sure all the port GUIDs are unique
	for ( portAttributes = DevInfo->CaAttributes.PortAttributesList;
				portAttributes != NULL;
				portAttributes = portAttributes->Next
				)
	{
		EUI64 caGuid;

		if (FSUCCESS == iba_get_ca_for_port(portAttributes->GUID, &caGuid))
		{
			_DBG_ERROR(("Attempt to add CA 0x%016"PRIx64" with duplicate Port GUID: 0x%016"PRIx64"\n",
				DevInfo->VpInterface.CaGUID, portAttributes->GUID));
			status = FDUPLICATE;
			goto dupport;
		}
	}

done:
	_DBG_LEAVE_EXT(_DBG_LVL_PNP);
	return status;

dupport:
failqueryports:
	MemoryDeallocate(DevInfo->CaAttributes.PortAttributesList);
failportalloc:
failquery:
	DevInfo->CaAttributes.PortAttributesList = NULL;
	DevInfo->CaAttributes.PortAttributesListSize = 0;
failinternal:
	(void)iba_close_ca(DevInfo->VcaCaHandle);
failopen:
	goto done;
}


//
// Free fields in DEVICE_INFO initialized by vcaResourceInit
//

FSTATUS
vcaResourceFree(
	IN DEVICE_INFO	*DevInfo
	)
{
	FSTATUS status = FSUCCESS;

	_DBG_ENTER_EXT(_DBG_LVL_PNP, vcaResourceFree, == PASSIVE);

	ASSERT(DevInfo->CaAttributes.PortAttributesList);
	MemoryDeallocate(DevInfo->CaAttributes.PortAttributesList);

	status = iba_close_ca(DevInfo->VcaCaHandle);
	if ( status != FSUCCESS )
	{
		_DBG_ERROR(("failed to close CA\n"));
	}

	_DBG_LEAVE_EXT(_DBG_LVL_PNP);

	return status;
}


//
//
//

FSTATUS
vcaLoad(EUI64 SystemImageGUID)
{
	int		i;
	FSTATUS	status = FSUCCESS;

	_TRC_REGISTER();

	//
	// Initialize the lock on the device list
	//

	SpinLockInitState(&VcaGlobal.Lock);
	SpinLockInit(&VcaGlobal.Lock);
	VcaGlobal.SystemImageGUID = SystemImageGUID;

	// Call load functions for each component of the IB Transport
	MemoryClear(ComponentInfo, sizeof(IBT_COMPONENT_INFO) * IbtMaxComponents );

	for (i = 0; i < IbtMaxComponents; i++)
	{
		if ( ComponentLoad[i] != NULL )
		{
			status = (*ComponentLoad[i])(&ComponentInfo[i]);
			
			if ( status != FSUCCESS )
			{
				break;
			}
		}
	}

	if (status != FSUCCESS)
	{
		// Unload in reverse order of load
		for (i = IbtMaxComponents-1; i >= 0; i--)
		{
			if (ComponentInfo[i].Unload != NULL)
			{
				(*ComponentInfo[i].Unload)();
			}
		}
		SpinLockDestroy(&VcaGlobal.Lock);
	}

#ifdef ATOMIC_TEST
	// simple test of Atomic implementation
	AtomicRunTest();
#endif
	return status;
}


void
vcaUnload()
{
	int i;

	_TRC_UNREGISTER();

	// Unload in reverse order of load
	for (i = IbtMaxComponents-1; i >= 0; i--)
	{
		if (ComponentInfo[i].Unload != NULL)
		{
			(*ComponentInfo[i].Unload)();
		}
	}
	SpinLockDestroy(&VcaGlobal.Lock);
}


FSTATUS
vcaAddCA(IN DEVICE_INFO *DevInfo)
{
	DEVICE_INFO *p;
	FSTATUS status = FSUCCESS;

	_DBG_ENTER_EXT(_DBG_LVL_PNP, vcaAddCA, == PASSIVE);

	if (! DevInfo->VpInterface.PollAndRearmCQ)
			DevInfo->VpInterface.PollAndRearmCQ = default_poll_and_rearm_cq;

	// Insert to tail of list
	SpinLockAcquire(&VcaGlobal.Lock);
	DevInfo->Next = NULL;
	if ((p = VcaGlobal.DeviceList) == NULL) {
		VcaGlobal.DeviceList = DevInfo;
	} else {
		while (TRUE) {
			if (p->VpInterface.CaGUID == DevInfo->VpInterface.CaGUID)
			{
				_DBG_ERROR(("Attempt to add CA with duplicate CA GUID: 0x%016"PRIx64"\n", DevInfo->VpInterface.CaGUID));
				status = FDUPLICATE;
				break;
			}
			if (p->Next == NULL)
				break;
			p = p->Next;
		}
		if (FSUCCESS == status)
		{
			ASSERT(p->Next == NULL);
			p->Next = DevInfo;
		}
	}
	SpinLockRelease(&VcaGlobal.Lock);

	_DBG_LEAVE_EXT(_DBG_LVL_PNP);
	return status;
}


void
vcaRemoveCA(IN DEVICE_INFO *DevInfo)
{
	DEVICE_INFO *curDevInfo;
	DEVICE_INFO *prevDevInfo;

	_DBG_ENTER_EXT(_DBG_LVL_PNP, vcaRemoveCA, == PASSIVE);

	SpinLockAcquire(&VcaGlobal.Lock);

	curDevInfo = VcaGlobal.DeviceList;
	prevDevInfo = NULL;

	while ( curDevInfo != NULL )
	{
		if ( curDevInfo == DevInfo )
		{
			// Found CA to remove
			if ( prevDevInfo )
			{
				// CA is not at head of list, remove from within list
				prevDevInfo->Next = curDevInfo->Next;
			} else {
				// CA head of list, remove from head
				VcaGlobal.DeviceList = curDevInfo->Next;
			}
			break;
		}

		// Traverse to next CA in list
		prevDevInfo = curDevInfo;
		curDevInfo = curDevInfo->Next;
	}

	SpinLockRelease(&VcaGlobal.Lock);

	_DBG_LEAVE_EXT(_DBG_LVL_PNP);
}

/* initialize the SystemImage Guid as needed.
 * returns the SystemImageGuid, if the system image guid was not
 * previously initialized, it uses the CaGuid
 * This routine is intended for use within Verbs Providers
 */
EUI64
VcaSystemImageGuid(IN EUI64 CaGuid)
{
	_DBG_ENTER_LVL(_DBG_LVL_PNP, VcaSystemImageGUID);

	SpinLockAcquire(&VcaGlobal.Lock);

	if (! VcaGlobal.SystemImageGUID)
		VcaGlobal.SystemImageGUID = CaGuid;
	SpinLockRelease(&VcaGlobal.Lock);

	/* safe outside lock since once set, won't change */
	return VcaGlobal.SystemImageGUID;
}

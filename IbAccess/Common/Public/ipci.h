/* BEGIN_ICS_COPYRIGHT3 ****************************************

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

** END_ICS_COPYRIGHT3   ****************************************/

/* [ICS VERSION STRING: unknown] */

#ifndef _IBA_PUBLIC_IPCI_H_
#define _IBA_PUBLIC_IPCI_H_

#include "iba/public/datatypes.h"

#if defined(VXWORKS)

#ifdef __cplusplus
extern "C" {
#endif

/* Enable and disables PciDmaMap debug */
#ifndef PCIDMAMAP_DBG	/* allow to be set from CFLAGS */
#define PCIDMAMAP_DBG	0					/* 0=no, 1=validate, 2=validate+log */
#endif

struct _PCI_DEVICE;
struct _PCI_DMA_FUNCS;

	/* return TRUE if interrupt has been handled */
typedef boolean (*INTR_HANDLER)(IN struct _PCI_DEVICE *pDev, IN void *Context);

#define IBA_PCI_TYPE0_ADDRESSES			6
#define IBA_PCI_BRIDGE_ADDRESSES		2

/* control fields in BaseAddress Registers */
#define IBA_PCI_BAR_CNTL_MASK			0xF
#define IBA_PCI_BAR_CNTL_TYPE_MASK		0x6
#define IBA_PCI_BAR_CNTL_IO_SPACE		0x1
#define IBA_PCI_BAR_CNTL_TYPE_32BIT		0x0	/* PCI */
#define IBA_PCI_BAR_CNTL_TYPE_20BIT		0x2	/* obsolete */
#define IBA_PCI_BAR_CNTL_TYPE_64BIT		0x4	/* PCI-X requires */
#define IBA_PCI_BAR_CNTL_PREFETCHABLE	0x8

typedef struct _IBA_PCI_COMMON_CONFIG {
	uint16		VendorID;
	uint16		DeviceID;
	uint16		Command;
	uint16		Status;
	uint8		RevisionID;
	uint8		ProgIf;
	uint8		SubClass;
	uint8		BaseClass;
	uint8		CacheLineSize;
	uint8		LatencyTimer;
	uint8		HeaderType;
	uint8		BIST;
	union  {
		struct _pci_header_type0 {
			uint32	BaseAddresses[IBA_PCI_TYPE0_ADDRESSES];
			uint32	CIS;
			uint16	SubVendorId;
			uint16	SubSystemId;
			uint32	RomBaseAddress;
			uint8	CapabilitiesPtr;
			uint8	Reserved1[3];
			uint32	Reserved2;
			uint8	InterruptLine;
			uint8	InterruptPin;
			uint8	MinimumGrant;
			uint8	MaximumLatency;
		} type0;
		struct _pci_header_bridge {
			uint32	BaseAddresses[IBA_PCI_BRIDGE_ADDRESSES];
			uint8	PrimaryBusNumber;
			uint8	SecondaryBusNumber;
			uint8	SubordinateBusNumber;
			uint8	SecondaryLatencyTimer;
			uint8	IOBase;
			uint8	IOLimit;
			uint16	SecondaryStatus;
			uint16	MemoryBase;
			uint16	MemoryLimit;
			uint16	PrefetchableMemoryBase_l;
			uint16	PrefetchableMemoryLimit_l;
			uint32	PrefetchableMemoryBase_h;
			uint32	PrefetchableMemoryLimit_h;
			uint16	IOBase_h;
			uint16	IOLimit_h;
			uint8	CapabilitiesPtr;
			uint8	Reserved1[3];
			uint32	RomBaseAddress;
			uint8	InterruptLine;
			uint8	InterruptPin;
			uint16	BridgeControl;
		} bridge;
	} u;
	uint8	DeviceSpecific[192];	/* 48 uint32's */
} IBA_PCI_COMMON_CONFIG;

#define IBA_PCI_COMMON_HDR_LENGTH \
							(offsetof(IBA_PCI_COMMON_CONFIG, DeviceSpecific))

#ifdef __cplusplus
}
#endif

#endif /* defined(VXWORKS) */

#ifdef VXWORKS
#include "iba/public/ipci_osd.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Helper macros for reading and writing PCI memory
 * Note these macros are for convenience, the memory may also be directly
 * accessed
 */

#define SUPPORT_ATOMIC_64BITS

/* use mmx for 64bit write for ia32 user space */
#if defined(SUPPORT_ATOMIC_64BITS_MMX)
#define DEV_WRITE64(Address, Value) \
				mmx_write_64bits(Value,Address)
#elif defined(SUPPORT_ATOMIC_64BITS_PPC)
#define DEV_WRITE64(Address, Value) \
				ppc_write_64bits(Value,Address)
#else
#define DEV_WRITE64(Address, Value) \
				*(volatile uint64 *)(Address) = (Value)
#endif

#define DEV_WRITE32(Address, Value) \
				*(volatile uint32 *)(Address) = (Value)
#define DEV_WRITE16(Address, Value) \
				*(volatile uint16 *)(Address) = (Value)
#define DEV_WRITE8(Address, Value) \
				*(volatile uint8 *)(Address) = (Value)

/* read 64bit using mmx for ia32 user space*/
#if  defined(SUPPORT_ATOMIC_64BITS_MMX)
#define DEV_READ64(Address, pValue) \
			*((uint64 *)pValue) = mmx_read_64bits((volatile uint64 *)Address)
#elif  defined(SUPPORT_ATOMIC_64BITS_PPC)
#define DEV_READ64(Address, pValue) \
			*((uint64 *)pValue) = ppc_read_64bits((volatile uint64 *)Address)
#else
#define DEV_READ64(Address, pValue) \
				*((uint64 *)pValue) = *(volatile uint64 *)(Address)
#endif

#define DEV_READ32(Address, pValue) \
				*((uint32 *)pValue) = *(volatile uint32 *)(Address)
#define DEV_READ16(Address, pValue) \
				*((uint16 *)pValue) = *(volatile uint16 *)(Address)
#define DEV_READ8(Address, pValue) \
				*((uint8 *)pValue) = *(volatile uint8 *)(Address)

#define DEV_WRITE64_TO_BE(Address, Value) \
		*(volatile uint64 *)(Address) = hton64(Value)
#define DEV_WRITE32_TO_BE(Address, Value) \
		*(volatile uint32 *)(Address) = hton32(Value)
#define DEV_WRITE16_TO_BE(Address, Value) \
		*(volatile uint16 *)(Address) = hton16(Value)

#define DEV_READ64_FROM_BE(Address, pValue) \
		do { \
			*((uint64 *)pValue) = *(volatile uint64 *)(Address) ;\
			*pValue = ntoh64(*pValue); \
		} while (0)
#define DEV_READ32_FROM_BE(Address, pValue) \
		do { \
			*((uint32 *)pValue) = *(volatile uint32 *)(Address) ;\
			*pValue = ntoh32(*pValue); \
		} while (0)
#define DEV_READ16_FROM_BE(Address, pValue) \
		do { \
			*((uint16 *)pValue) = *(volatile uint16 *)(Address) ;\
			*pValue = ntoh16(*pValue); \
		} while (0)


#if defined(VXWORKS)
/* These functions implement create/destroy of PCI_DEVICE */
extern void PciInitState(PCI_DEVICE *pDev);

/*PciInit is supplied for Operating systems which provide a plug and play
 * callback based PCI enumeration, in which case the OS specific arguments
 * will be similar to the plag and play callback arguments for the OS
 *extern FSTATUS PciInit( PCI_DEVICE *pDev,
 *		OS specific arguments to describe device
 *		)
 */

/*PciFindDevice is supplied for Operating systems which do a driver initiated
 *PCI enumeration (such as VxWorks)
 * pDev - where to save information about device found
 * vendorId - PCI vendor ID to search for
 * deviceId - PCI device ID to search for
 * pLastDev - previous device returned during this enumeration loop
 *			(pass NULL for 1st interation of loop)
 *
 * returns:
 *	FSUCCESS - a device was found, *pDev is initialized to refer to the device
 *	others - no device found (or no more devices), *pDev undefined
 *		FNOT_FOUND - no more devices of given vendor/device
 *		FINSUFFICIENT_MEMORY - unable to allocate internal memory
 *		others - error reading/accessing device
 */
extern FSTATUS PciFindDevice(IN uint16 vendorId, IN uint16 deviceId,
							IN PCI_DEVICE *pLastDev, OUT PCI_DEVICE *pDev);

/*PciGetBus locates the parent Bus of a given PCI Device
 * pDev - Device whose parent we want to know
 * pBusDev - where to save information about parent bus
 *
 * returns:
 *	FSUCCESS - found parent
 *	FNOT_FOUND - no parent
 */
extern FSTATUS PciGetBus(IN PCI_DEVICE *pDev, OUT PCI_DEVICE *pBusDev);

extern void PciDestroy(IN PCI_DEVICE *pDev);
/*static _inline void PciSetContext(IN PCI_DEVICE *pDev, IN void *context); */
/*static _inline void *PciGetContext(IN PCI_DEVICE *pDev); */

/* Get Basic PCI Device Information (location in system) */
static _inline uint32 PciGetBusNumber(IN PCI_DEVICE *pDev);
static _inline uint32 PciGetDeviceNumber(IN PCI_DEVICE *pDev);
static _inline uint32 PciGetFuncNumber(IN PCI_DEVICE *pDev);

/* Get Basic PCI Device Information (comes from PCI Config registers) */
static _inline uint16 PciGetVendorId(IN PCI_DEVICE *pDev);
static _inline uint16 PciGetDeviceId(IN PCI_DEVICE *pDev);
static _inline uint8 PciGetRevisionId(IN PCI_DEVICE *pDev);
static _inline uint16 PciGetSubsystemVendorId(IN PCI_DEVICE *pDev);
static _inline uint16 PciGetSubsystemId(IN PCI_DEVICE *pDev);
static _inline uint8 PciGetInterruptLine(IN PCI_DEVICE *pDev);
static _inline uint8 PciGetInterruptPin(IN PCI_DEVICE *pDev);

/* PCI Device Memory and BARs
 * depending on device, there could be 6 32 bit BARs or 3 64 bit BARs
 * however in either case we still use values of 0-5 for barNumber below
 * Beware on VxWorks, PciGetMemoryLength always returns 0
 */
extern uint64 PciGetMemoryLength(IN PCI_DEVICE *pDev, IN uint8 barNumber);
extern uint64 PciGetMemoryPciPhysAddr(IN PCI_DEVICE *pDev, IN uint8 barNumber);
extern uint64 PciGetMemoryCpuPhysAddr(IN PCI_DEVICE *pDev, IN uint8 barNumber);

/* Mapping PCI memory to CPU Kernel Virtual
 * These functions work with an opaque handle to a memory map object
 */
extern FSTATUS PciMapUncachedMemory(IN PCI_DEVICE *pDev,
									IN uint8 barNumber,
									OUT PCI_MAP_HANDLE *pMap,
									OUT void** virt OPTIONAL);
extern FSTATUS PciMapUncachedMemoryPartial(IN PCI_DEVICE *pDev,
										IN uint8 barNumber,
										IN uint64 offset,
										IN uint64 length,
										OUT PCI_MAP_HANDLE *pMap,
										OUT void** virt OPTIONAL);
extern void* PciMapGetVirtual(IN PCI_MAP_HANDLE pMap);
extern void PciUnmapMemory(IN PCI_MAP_HANDLE pMap);

/* enable bus master operation by device */
extern FSTATUS PciSetBusMaster(IN PCI_DEVICE *pDev, int bus_width);
/* enable memory mapped access to device */
extern FSTATUS PciSetMemoryEnable(IN PCI_DEVICE *pDev);

/* PCI MSI (Message Signalled Interrupts) support
 * At present only 1 MSI interrupt is supported per device
 * The use of MSI by the driver is optional
 * In the future MSI-X may be implemented and allow >1 MSI interrupt per device
 */
/* Enable use of MSI, must call before PciSetIntrHandler */
extern FSTATUS PciEnableMsi(IN PCI_DEVICE *pDev);
/* Disable use of MSI, must call after PciClearIntrHandler
 * Noop if not enabled
 */
extern void PciDisableMsi(IN PCI_DEVICE *pDev);

/* PCI Device Interrupts */
/* Set Interrupt handler and enable interrupts */
extern FSTATUS PciSetIntrHandler(IN PCI_DEVICE *pDev, IN const char* Name,
							IN INTR_HANDLER Handler, IN void* Context);
/* Clear Interrupt Handler and disable interrupts */
extern void PciClearIntrHandler(IN PCI_DEVICE *pDev);

/*
 * The following functions implement read/write to PCI config space
 * to perform device specific configuration.
 *
 * PCI_DEVICE - is an opaque structure that makes sense only for that OS
 * or platform that implements that abstraction.
 *
 * Offset - the offset into the space where the operation should be performed.
 *
 * Width  - The width of the IO to perform. It could be one of 1,2,4 bytes
 *          in size. The memory provided to these function should be aligned
 *          to the byte width to which the operation is being performed.
 * 
 * pVal   - Pointer to the location where the data should be read into, or 
 *          the value that should be written to the pci device.
 */
extern boolean 
PciReadConfig(
	IN	PCI_DEVICE	*pDev,
	IN	uint32		Offset,
	IN	uint32		Width,
	OUT void		*pVal
	);
extern boolean 
PciWriteConfig(
	IN	PCI_DEVICE	*pDev,
	IN	uint32		Offset,
	IN	uint32		Width,
	IN	void		*pVal
	);

/*
 * The following function finds a device based on the Bus_Number/Slot_Number
 * pair and returns a pointer to the PCI_DEVICE structure. This structure
 * will be used by the ReadConfig and WriteConfig functions listed above.
 *
 * PCI_DEVICE - is an opaque structure that makes sense only for that OS or
 * platform that implements the abstraction.
 * 
 * BusNumber: number of PCI bus on which desired PCI device resides
 * Slot: PCI slot in which the desired PCI device residesdevice resides
 *
 * Return Value: 
 * 	FSUCCESS - If a device at BusNumber and SlotNumber if found.
 *	FNOT_FOUND - If a device cannot be found.
 */

extern FSTATUS
PciGetDevice(
	IN OUT PCI_DEVICE 	*pDev,
	IN uint16		BusNumber,
	IN uint8		SlotNumber 
	);

#if defined(HAVEIOMMU)
/*
 * The following functions create, manage and release pools of memory
 * that are guaranteed to be accessible to PCI devices.
 */
extern void* PciPoolCreate(
	IN const char *name,
	IN PCI_DEVICE *pDev,
	IN size_t size,
	IN size_t align);

extern void PciPoolDestroy(IN void* pool);

/* 
 * Returns the virtual address of the dma-able memory or NULL.
 * The dma address of the dma-able memory is returned in dmaaddr.
 */
extern void* PciPoolAlloc(
	IN void* pool, 
	OUT uint64 *dmaaddr);

/* Free the dma-able memory. NOTE THAT YOU PASS BOTH THE VIRTUAL
 * AND PHYSICAL (DMA) ADDRESSES OF THE MEMORY.
 */
extern void PciPoolFree(
		IN void* pool,
		IN void* vaddr,
		IN uint64 dmaddr);
#endif

// defined in OS specific header
//#define PCI_DMA_BIDIR DMA_BIDIRECTIONAL
//#define PCI_DMA_TODEV DMA_TODEVICE
//#define PCI_DMA_FROMDEV DMA_FROMDEVICE
//...

/*
 * The following functions manage mapping memory for access by PCI devices.
 * For each function, a function type is declared which can be used by
 * PciSetDmaFuncs to specify device specific overrides which will be called
 * by these functions.
 */

/* map a kernel virtual address, returns Bus Physical address to be
 * used by device for IO
 * Virtual memory specified must be kernel virtual and physically contiguous
 */
typedef FSTATUS (PCI_DMA_MAP_VIRT_FUNC)(
	IN struct _PCI_DEVICE *pDev,
	IN void *addr,
	IN uint64 size,
	IN int dir,
	OUT uint64 *dma_address,
	OUT uint64 *map_handle	/* opaque handle for use in Unmap below */
	);
extern PCI_DMA_MAP_VIRT_FUNC PciDmaMapVirt;

/* undoes a previous PciDmaMapVirt, must be called with map_handle = handle
 * previously returned by PciDmaMapVirt
 */
typedef void (PCI_DMA_UNMAP_VIRT_FUNC)(
	IN struct _PCI_DEVICE *pDev,
	IN uint64 size,
	IN int dir,
	IN uint64 map_handle
	);
extern PCI_DMA_UNMAP_VIRT_FUNC PciDmaUnmapVirt;

/* map a set of physically contiguous pages.
 * physadd need not be page aligned, however some OS's may require this not
 * span multiple pages
 */
typedef FSTATUS (PCI_DMA_MAP_PHYS_FUNC)(
	IN struct _PCI_DEVICE *pDev,
	IN uint64 physaddr,
	IN uint64 size,
	IN int dir,
	OUT uint64 *dma_address,
	OUT uint64 *map_handle
	);
extern PCI_DMA_MAP_PHYS_FUNC PciDmaMapPhys;

/* undoes a previous PciDmaMapPhys, must be called with map_handle = handle
 * previously returned by PciDmaMapPhys
 */
typedef void (PCI_DMA_UNMAP_PHYS_FUNC)(
	IN struct _PCI_DEVICE *pDev,
	IN uint64 size,
	IN int dir,
	IN uint64 map_handle
	);
extern PCI_DMA_UNMAP_PHYS_FUNC PciDmaUnmapPhys;

/* map a scatter gather list of segments
 * scatter gather list format is OS specific as is typically supplied to
 * typical drivers on the given OS.  On some OSs an offset is also supplied by
 * the OS for typical drivers.
 * an OS specific ITERATOR is returned for use in walking the resulting
 * list of DMA addresses/lengths.
 * Note that on some OSs, the number of entries in the resulting iterator
 * may be different than sg_len
 */
typedef FSTATUS (PCI_DMA_MAP_SG_FUNC)(
	IN struct _PCI_DEVICE *pDev,
	IN PCI_DMA_SG_LIST *sg_list,
	IN uint32 sg_len,
	IN uint32 offset,
	IN int dir,
	OUT PCI_DMA_SG_ITERATOR *iterator,
	OUT uint64 *map_handle
	);
extern PCI_DMA_MAP_SG_FUNC PciDmaMapSg;

/* return the next mapped SG entry's address and length and advance
 * returns:
 * 	FCOMPLETED - end of list, no address/length available
 * 	FSUCCESS - address/length provided and advanced
 */
typedef FSTATUS (PCI_DMA_SG_NEXT_FUNC)(
	IN struct _PCI_DEVICE *pDev,
	IN PCI_DMA_SG_LIST *sg_list,
	IN PCI_DMA_SG_ITERATOR *iterator,
	OUT uint64 *dma_address,
	OUT uint64 *dma_length
	);
extern PCI_DMA_SG_NEXT_FUNC PciDmaSgNext;

/* return the number of unprocessed SG entries
 * If called immediately after PciDmaMapSg, this is total mapped areas
 * If called after one or more PciDmaSgNext calls, this is remaining areas
 */
extern uint32 PciDmaSgCountLeft(
	IN PCI_DMA_SG_LIST *sg_list,
	IN PCI_DMA_SG_ITERATOR *iterator
	);

/* undoes a previous PciDmaMapSg, must be called with map_handle = handle
 * previously returned by PciDmaMapSg
 */
typedef void (PCI_DMA_UNMAP_SG_FUNC)(
	IN struct _PCI_DEVICE *pDev,
	IN PCI_DMA_SG_LIST *sg_list,
	IN uint32 sg_len,
	IN uint32 offset,
	IN int dir,
	IN uint64 map_handle
	);
extern PCI_DMA_UNMAP_SG_FUNC PciDmaUnmapSg;

/* usage model for scatter/gather DMA:
 * To start IO:
 * 		status = PciDmaMapSg(.....)
 * 		check status, fail if not FSUCCESS
 * 		while (PciDmaSgNext(...) != FCOMPLETED)
 * 			setup transfer to dma_address,length (typically added to WQE)
 *
 * when IO completes:
 * 		PciDmaUnmapSg(...)
 */

/* device specific functions which can be called as part of Pci DMA functions
 * above
 */
typedef struct _PCI_DMA_FUNCS {
	PCI_DMA_MAP_VIRT_FUNC *pMapVirt;
	PCI_DMA_UNMAP_VIRT_FUNC *pUnmapVirt;
	PCI_DMA_MAP_PHYS_FUNC *pMapPhys;
	PCI_DMA_UNMAP_PHYS_FUNC *pUnmapPhys;
	PCI_DMA_MAP_SG_FUNC *pMapSg;
	PCI_DMA_UNMAP_SG_FUNC *pUnmapSg;
	PCI_DMA_SG_NEXT_FUNC *pSgNext;
} PCI_DMA_FUNCS;

/* specify DMA function overrides which should be called by the Pci DMA
 * functions above.  The context supplied may be fetched via PciGetDmaContext
 * it is distinct from the Interrupt context used in interrupt callbacks
 */
extern void PciSetDmaFuncs(
	IN PCI_DEVICE *pDev,
	IN PCI_DMA_FUNCS *funcs,
	IN void *context);
/*static _inline void *PciGetDmaContext(IN PCI_DEVICE *pDev); */

/* This function indicates if the given Device and/or OS requires
 * calls to PciDmaMap functions above.
 * If this returns TRUE, PciDmaMap functions are required.
 * If this returns FALSE the calls above are benign but not necessary.
 * In which case PciDmaMap functions will simply return CPU physical addresses.
 * When FALSE is returned its up to the caller, who may be able to optimize
 * by avoiding the PciDmaMap functions.
 */
//static _inline boolean PciNeedDmaMap(IN PCI_DEVICE *pDev);

#endif /* defined(VXWORKS) */

#ifdef __cplusplus
}	/* extern "C" */
#endif

#endif /* _IBA_PUBLIC_IPCI_H_ */

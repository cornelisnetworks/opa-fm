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

#ifndef _IBA_PUBLIC_IBYTESWAP_H_
#define _IBA_PUBLIC_IBYTESWAP_H_

/* 
 * The ibyteswap_osd.h provides the following macros.
 *		__LITTLE_ENDIAN
 *		__BIG_ENDIAN
 *		__BYTE_ORDER
 *
 * If the platform does not provide byte swapping functions, ibyteswap_osd.h
 * will also provide the following macros.
 *		ntoh16, hton16
 *		ntoh32, hton32
 *		ntoh64, hton64
 */
#include "iba/public/datatypes.h"
#include "iba/public/ibyteswap_osd.h"

#ifdef __cplusplus
extern "C"
{
#endif

#undef CPU_LE
#undef CPU_BE
#if defined(__LITTLE_ENDIAN) && (__BYTE_ORDER == __LITTLE_ENDIAN)
#define CPU_LE		1
#define CPU_BE		0
#elif defined(__BIG_ENDIAN) && (__BYTE_ORDER == __BIG_ENDIAN)
#define CPU_LE		0
#define CPU_BE		1
#else
#error "Unknown BYTE_ORDER"
#endif


/* Default implementation of 16 bit swap function. */
#ifndef ntoh16
	#if CPU_LE
		#define ntoh16( x )	( (((x) & 0x00FF) << 8) | (((x) & 0xFF00) >> 8) )
	#else
		#define ntoh16
	#endif
	#define hton16				ntoh16
#endif

/* Default implementation of 32 bit swap function. */
#ifndef ntoh32
	#if CPU_LE
		#define ntoh32( x )	( (((x) & 0x000000FF) << 24) |	\
								(((x) & 0x0000FF00) << 8) |	\
								(((x) & 0x00FF0000) >> 8) |	\
								(((x) & 0xFF000000) >> 24) )
	#else
		#define ntoh32
	#endif
#define hton32				ntoh32
#endif

/* Default implementation of 64 bit swap function. */
#ifndef ntoh64
	#if CPU_LE
		#define ntoh64( x )	( (((x) & 0x00000000000000FF) << 56)  |		\
								(((x) & 0x000000000000FF00) << 40)  |	\
								(((x) & 0x0000000000FF0000) << 24)  |	\
								(((x) & 0x00000000FF000000) << 8 )  |	\
								(((x) & 0x000000FF00000000) >> 8 )  |	\
								(((x) & 0x0000FF0000000000) >> 24)  |	\
								(((x) & 0x00FF000000000000) >> 40)  |	\
								(((x) & 0xFF00000000000000) >> 56) )
	#else
		#define ntoh64
	#endif
#define hton64				ntoh64
#endif


/* 
 * Generic function to swap the byte order of byte arrays of any given size.
 */
#if CPU_LE
static __inline void 
ntoh( 
	IN	void* pSrc, 
	IN	void* pDest, 
	IN	uint8 Size )
{
	uint8	i;
	uint8	*s = (uint8*)pSrc;
	uint8	*d = (uint8*)pDest;
	char	temp;
	if( s == d )
	{
		/* Swap in place if source and destination are the same. */
		for( i = 0; i < Size / 2; i++ )
		{
			temp = d[i];
			d[i] = s[Size - 1 - i];
			s[Size - 1 - i] = temp;
		}
	}
	else
	{
		for( i = 0; i < Size; i++ )
			d[i] = s[Size - 1 - i];
	}
}
#else
#define ntoh(pSrc, pDest, Size) (void)0
#endif


/*
 * The following is suggested use only. Primarily when there are different 
 * types of fields, its easy to generate this description, rather than
 * manually perform the operation. If the specific case is very simple, the 
 * simplest thing is to do direct swap, using the CPU_LE, CPU_BE for
 * conditional compilation for different bit fields, and htonXX macros. 
 * ntohXX are provided for consistency, in reality they both do the same thing.
 */
typedef struct _swap_desc {
	uint8	size;	/* Size of the field. */
	uint16	offset;	/* Offset from the start of struct. */
} SWAP_DESC;


/*
 * Usage example:
 * typedef struct __MyStruct {
 * 		uint8	myint8_1;
 *		uint8	myint8_2;
 *		uint16	myint16_1;
 *		uint32	myint32_1;
 *	} MYSTRUCT;
 *
 *	SWAP_DESC_START (MYSTRUCT)
 *		SWAP_DESC_FIELD(MYSTRUCT, myint16_1)
 *		SWAP_DESC_FIELD(MYSTRUCT, myint32_1)
 *  SWAP_DESC_END
 *
 * send_buf()
 * {
 *     .....
 *	  
 *     databuf is the buffer that points to a structure of type MYSTRUCT.
 *     SwapStruct(SWAP_DESC_NAME(MYSTRUCT, destbuf, srcbuf);
 *     .....
 */
#define SWAP_DESC_START(name)	SWAP_DESC	DEF_##name[] = {
#define SWAP_DESC_FIELD(name, field)	\
	{									\
		sizeof(((name *) 0)->field),	\
		offsetof(name, field)		\
	},
#define SWAP_DESC_END	{0,0}};

#define SWAP_DESC_NAME(name)	DEF_##name

static __inline void 
SwapStruct( 
	IN	SWAP_DESC *desc, 
	OUT void *pDest, 
	IN	void *pSrc)
{
	int		i = 0;
	void	*src, *dest;

	for( i=0; desc[i].size > 0; i++ ) 
	{
		src = (uchar*)pSrc + desc[i].offset;
		dest = (uchar*)pDest + desc[i].offset;
		switch( desc[i].size ) 
		{
		case 2:
			*(uint16*)dest = hton16( *(uint16*)src );
			break;
		case 4: 
			*(uint32*)dest = hton32( *(uint32*)src );
			break;
		case 8:
			*(uint64*)dest = hton64( *(uint64*)src );
			break;
		}
	}
}

/*!
fetch a byte from a 32 bit value

@param value - 32 bit value
@param byte - byte position (for BE U32 0=MSB, 3=LSB, for LE U32 0=LSB, 3=MSB)

@return - host byte order 8 bit value
*/
static __inline uint8 GetU8FromU32(uint32 value, int byte)
{
	return ((uint8*)&value)[byte];
}

/*!
fetch a byte from a 32 bit BigEndian Byte Order value

@param value - 32 bit big endian value
@param byte - BE byte position (0=MSB, 3=LSB)

@return - host byte order 8 bit value
*/
static __inline uint8 GetU8FromBeU32(uint32 value, int byte)
{
	return ((uint8*)&value)[byte];
}

/*!
fetch a byte from a 32 bit LittleEndian Byte Order value

@param value - 32 bit little endian value
@param byte - LE byte position (0=MSB, 3=LSB)

@return - host byte order 8 bit value
*/
static __inline uint8 GetU8FromLeU32(uint32 value, int byte)
{
	return ((uint8*)&value)[3-byte];
}

/*!
fetch a 16 bit BigEndian Byte Order field

@param addr - address of Big Endian item to fetch
	may be poorly aligned

@return - host byte order 16 bit value
*/
static __inline uint16 GetBeU16(const uint8* addr) 
{
	return ((uint16)(addr[0])<<8) | addr[1];
}

/*!
fetch a 16 bit BigEndian Byte Order array

@param dest - address to store host order array in (must be u16 aligned)
@param src - address of Big Endian array to fetch (must be u16 aligned)
@param byte - byte count for array (note bytes not u16 count)
*/
static __inline void FetchBeU16Array(uint16* dest, const uint16 *src, uint32 bytes)
{
	do {
		*dest++ = ntoh16(*src);
		src++;
	} while (bytes -= 2);
}


/*!
store a 16 bit BigEndian Byte Order field

@param addr - address to store Big Endian value at
	may be poorly aligned
@param value - host byte order 16 bit value to store
*/
static __inline void StoreBeU16(uint8* addr, uint16 value) 
{
	addr[0] = value >> 8;
	addr[1] = value & 0xffu;
}

/*!
store a 16 bit BigEndian Byte Order array

@param dest - address to store Big Endian array in (must be u16 aligned)
@param src - address of host order array to fetch (must be u16 aligned)
@param byte - byte count for array (note bytes not u16 count)
*/
static __inline void StoreBeU16Array(uint16* dest, const uint16 *src, uint32 bytes)
{
	do {
		*dest++ = hton16(*src);
		src++;
	} while (bytes -= 2);
}

/*!
fetch a 24 bit BigEndian Byte Order field

@param addr - address of Big Endian item to fetch
	may be poorly aligned

@return - host byte order 24 bit value
*/
static __inline uint32 GetBeU24(const uint8* addr) 
{
	return ((uint32)(addr[0]) << 16) | GetBeU16(&addr[1]);
}

/*!
store a 24 bit BigEndian Byte Order field

@param addr - address to Big Endian store value at
	may be poorly aligned
@param value - host byte order 24 bit value to store
*/
static __inline void StoreBeU24(uint8* addr, uint32 value) 
{
	addr[0] = value >> 16;
	StoreBeU16(&addr[1], value & 0xffffu);
}

/*!
fetch a 32 bit BigEndian Byte Order field

@param addr - address of Big Endian item to fetch
	may be poorly aligned

@return - host byte order 32 bit value
*/
static __inline uint32 GetBeU32(const uint8* addr) 
{
	return ((uint32)GetBeU16(addr) << 16) | GetBeU16(&addr[2]);
}

/*!
fetch a 32 bit BigEndian Byte Order array

@param dest - address to store host order array in (must be u32 aligned)
@param src - address of Big Endian array to fetch (must be u32 aligned)
@param byte - byte count for array (note bytes not u32 count)
*/
static __inline void FetchBeU32Array(uint32* dest, const uint32 *src, uint32 bytes)
{
	do {
		*dest++ = ntoh32(*src);
		src++;
	} while (bytes -= 4);
}

/*!
store a 32 bit BigEndian Byte Order field

@param addr - address to store Big Endian value at
	may be poorly aligned
@param value - host byte order 32 bit value to store
*/
static __inline void StoreBeU32(uint8* addr, uint32 value) 
{
	StoreBeU16(addr, value >> 16);
	StoreBeU16(&addr[2], value & 0xffffu);
}

/*!
store a 32 bit BigEndian Byte Order array

@param dest - address to store Big Endian array in (must be u32 aligned)
@param src - address of host order array to fetch (must be u32 aligned)
@param byte - byte count for array (note bytes not u32 count)
*/
static __inline void StoreBeU32Array(uint32* dest, const uint32 *src, uint32 bytes)
{
	do {
		*dest++ = hton32(*src);
		src++;
	} while (bytes -= 4);
}

/*!
fetch a 48 bit BigEndian Byte Order field

@param addr - address of Big Endian item to fetch
	may be poorly aligned

@return - host byte order 48 bit value
*/
static __inline uint64 GetBeU48(const uint8* addr) 
{
	return ((uint64)GetBeU16(addr) << 32) | GetBeU32(&addr[2]);
}

/*!
store a 48 bit BigEndian Byte Order field

@param addr - address to store Big Endian value at
	may be poorly aligned
@param value - host byte order 48 bit value to store
*/
static __inline void StoreBeU48(uint8* addr, uint64 value) 
{
	StoreBeU16(addr, value >> 32);
	StoreBeU32(&addr[2], value & 0xffffffffu);
}

/*!
fetch a 64 bit BigEndian Byte Order field

@param addr - address of item to fetch
	may be poorly aligned

@return - host byte order 64 bit value
*/
static __inline uint64 GetBeU64(const uint8* addr) 
{
	return ((uint64)GetBeU32(addr) << 32) | GetBeU32(&addr[4]);
}

/*!
fetch a 64 bit BigEndian Byte Order array

@param dest - address to store host order array in (must be u64 aligned)
@param src - address of Big Endian array to fetch (must be u64 aligned)
@param byte - byte count for array (note bytes not u64 count)
*/
static __inline void FetchBeU64Array(uint64* dest, const uint64 *src, uint32 bytes)
{
	do {
		*dest++ = ntoh64(*src);
		src++;
	} while (bytes -= 8);
}

/*!
store a 64 bit BigEndian Byte Order field

@param addr - address to store Big Endian value at
	may be poorly aligned
@param value - host byte order 64 bit value to store
*/
static __inline void StoreBeU64(uint8* addr, uint64 value) 
{
	StoreBeU32(addr, value >> 32);
	StoreBeU32(&addr[4], value & 0xffffffffu);
}

/*!
store a 64 bit BigEndian Byte Order array

@param dest - address to store Big Endian array in (must be u64 aligned)
@param src - address of host order array to fetch (must be u64 aligned)
@param byte - byte count for array (note bytes not u64 count)
*/
static __inline void StoreBeU64Array(uint64* dest, const uint64 *src, uint32 bytes)
{
	do {
		*dest++ = hton64(*src);
		src++;
	} while (bytes -= 8);
}

/*!
fetch a 16 bit LittleEndian Byte Order field

@param addr - address of Little Endian item to fetch
	may be poorly aligned

@return - host byte order 16 bit value
*/
static __inline uint16 GetLeU16(const uint8* addr) 
{
	return addr[0] | ((uint16)(addr[1])<<8);
}

/*!
fetch a 16 bit LittleEndian Byte Order array

@param dest - address to store host order array in (must be u16 aligned)
@param src - address of Little Endian array to fetch (must be u16 aligned)
@param byte - byte count for array (note bytes not u16 count)
*/
static __inline void FetchLeU16Array(uint16* dest, const uint16 *src, uint32 bytes)
{
	do {
		*dest++ = GetLeU16((const uint8*)src);	/* TBD - optimize with ltoh */
		src++;
	} while (bytes -= 2);
}

/*!
store a 16 bit LittleEndian Byte Order field

@param addr - address to store Little Endian value at
	may be poorly aligned
@param value - host byte order 16 bit value to store
*/
static __inline void StoreLeU16(uint8* addr, uint16 value) 
{
	addr[0] = value & 0xffu;
	addr[1] = value >> 8;
}

/*!
store a 16 bit LittleEndian Byte Order array

@param dest - address to store Little Endian array in (must be u16 aligned)
@param src - address of host order array to fetch (must be u16 aligned)
@param byte - byte count for array (note bytes not u16 count)
*/
static __inline void StoreLeU16Array(uint16* dest, const uint16 *src, uint32 bytes)
{
	do {
		StoreLeU16((uint8*)dest++, *src);	/* TBD optimize with htol */
		src++;
	} while (bytes -= 2);
}

/*!
fetch a 24 bit LittleEndian Byte Order field

@param addr - address of Little Endian item to fetch
	may be poorly aligned

@return - host byte order 24 bit value
*/
static __inline uint32 GetLeU24(const uint8* addr) 
{
	return GetLeU16(addr) | ((uint32)(addr[2]) << 16);
}

/*!
store a 24 bit LittleEndian Byte Order field

@param addr - address to store Little Endian value at
	may be poorly aligned
@param value - host byte order 24 bit value to store
*/
static __inline void StoreLeU24(uint8* addr, uint32 value) 
{
	StoreLeU16(addr, value & 0xffffu);
	addr[2] = value >> 16;
}

/*!
fetch a 32 bit LittleEndian Byte Order field

@param addr - address of Little Endian item to fetch
	may be poorly aligned

@return - host byte order 32 bit value
*/
static __inline uint32 GetLeU32(const uint8* addr) 
{
	return GetLeU16(addr) | ((uint32)GetLeU16(&addr[2]) << 16);
}

/*!
fetch a 32 bit LittleEndian Byte Order array

@param dest - address to store host order array in (must be u32 aligned)
@param src - address of Little Endian array to fetch (must be u32 aligned)
@param byte - byte count for array (note bytes not u32 count)
*/
static __inline void FetchLeU32Array(uint32* dest, const uint32 *src, uint32 bytes)
{
	do {
		*dest++ = GetLeU32((const uint8*)src);	/* TBD - optimize with ltoh */
		src++;
	} while (bytes -= 4);
}

/*!
store a 32 bit LittleEndian Byte Order field

@param addr - address to store Little Endian value at
	may be poorly aligned
@param value - host byte order 32 bit value to store
*/
static __inline void StoreLeU32(uint8* addr, uint32 value) 
{
	StoreLeU16(addr, value & 0xffffu);
	StoreLeU16(&addr[2], value >> 16);
}

/*!
store a 32 bit LittleEndian Byte Order array

@param dest - address to store Little Endian array in (must be u32 aligned)
@param src - address of host order array to fetch (must be u32 aligned)
@param byte - byte count for array (note bytes not u32 count)
*/
static __inline void StoreLeU32Array(uint32* dest, const uint32 *src, uint32 bytes)
{
	do {
		StoreLeU32((uint8*)dest++, *src);	/* TBD optimize with htol */
		src++;
	} while (bytes -= 4);
}

/*!
fetch a 48 bit LittleEndian Byte Order field

@param addr - address of Little Endian item to fetch
	may be poorly aligned

@return - host byte order 48 bit value
*/
static __inline uint64 GetLeU48(const uint8* addr) 
{
	return GetLeU32(addr) | ((uint64)GetLeU16(&addr[4]) << 32);
}

/*!
store a 48 bit LittleEndian Byte Order field

@param addr - address to store Little Endian value at
	may be poorly aligned
@param value - host byte order 48 bit value to store
*/
static __inline void StoreLeU48(uint8* addr, uint64 value) 
{
	StoreLeU32(addr, value & 0xffffffffu);
	StoreLeU16(&addr[4], value >> 32);
}

/*!
fetch a 64 bit LittleEndian Byte Order field

@param addr - address of Little Endian item to fetch
	may be poorly aligned

@return - host byte order 64 bit value
*/
static __inline uint64 GetLeU64(const uint8* addr) 
{
	return GetLeU32(addr) | ((uint64)GetLeU32(&addr[4]) << 32);
}

/*!
fetch a 64 bit LittleEndian Byte Order array

@param dest - address to store host order array in (must be u64 aligned)
@param src - address of Little Endian array to fetch (must be u64 aligned)
@param byte - byte count for array (note bytes not u64 count)
*/
static __inline void FetchLeU64Array(uint64* dest, const uint64 *src, uint32 bytes)
{
	do {
		*dest++ = GetLeU64((const uint8*)src);	/* TBD - optimize with ltoh */
		src++;
	} while (bytes -= 8);
}

/*!
store a 64 bit LittleEndian Byte Order field

@param addr - address to store Little Endian value at
	may be poorly aligned
@param value - host byte order 64 bit value to store
*/
static __inline void StoreLeU64(uint8* addr, uint64 value) 
{
	StoreLeU32(addr, value & 0xffffffffu);
	StoreLeU32(&addr[4], value >> 32);
}

/*!
store a 64 bit LittleEndian Byte Order array

@param dest - address to store Little Endian array in (must be u64 aligned)
@param src - address of host order array to fetch (must be u64 aligned)
@param byte - byte count for array (note bytes not u64 count)
*/
static __inline void StoreLeU64Array(uint64* dest, const uint64 *src, uint32 bytes)
{
	do {
		StoreLeU64((uint8*)dest++, *src);	/* TBD optimize with htol */
		src++;
	} while (bytes -= 8);
}

/*!
fetch a 16 bit Host Byte Order field

@param addr - address of Little Endian item to fetch
	may be poorly aligned

@return - host byte order 16 bit value
*/
static __inline uint16 GetU16(const uint8* addr) 
{
#if CPU_BE
	return GetBeU16(addr);
#else
	return GetLeU16(addr);
#endif
}

/*!
store a 16 bit Host Byte Order field

@param addr - address to store host byte order value at
	may be poorly aligned
@param value - host byte order 16 bit value to store
*/
static __inline void StoreU16(uint8* addr, uint16 value) 
{
#if CPU_BE
	StoreBeU16(addr, value);
#else
	StoreLeU16(addr, value);
#endif
}

/*!
fetch a 24 bit Host Byte Order field

@param addr - address of host byte order item to fetch
	may be poorly aligned

@return - host byte order 24 bit value
*/
static __inline uint32 GetU24(const uint8* addr) 
{
#if CPU_BE
	return GetBeU24(addr);
#else
	return GetLeU24(addr);
#endif
}

/*!
store a 24 bit Host Byte Order field

@param addr - address to store host byte order value at
	may be poorly aligned
@param value - host byte order 24 bit value to store
*/
static __inline void StoreU24(uint8* addr, uint32 value) 
{
#if CPU_BE
	StoreBeU24(addr, value);
#else
	StoreLeU24(addr, value);
#endif
}

/*!
fetch a 32 bit Host Byte Order field

@param addr - address of host byte order item to fetch
	may be poorly aligned

@return - host byte order 32 bit value
*/
static __inline uint32 GetU32(const uint8* addr) 
{
#if CPU_BE
	return GetBeU32(addr);
#else
	return GetLeU32(addr);
#endif
}

/*!
store a 32 bit Host Byte Order field

@param addr - address to store host byte order value at
	may be poorly aligned
@param value - host byte order 32 bit value to store
*/
static __inline void StoreU32(uint8* addr, uint32 value) 
{
#if CPU_BE
	StoreBeU32(addr, value);
#else
	StoreLeU32(addr, value);
#endif
}

/*!
fetch a 48 bit Host Byte Order field

@param addr - address of host byte order item to fetch
	may be poorly aligned

@return - host byte order 48 bit value
*/
static __inline uint64 GetU48(const uint8* addr) 
{
#if CPU_BE
	return GetBeU48(addr);
#else
	return GetLeU48(addr);
#endif
}

/*!
store a 48 bit Host Byte Order field

@param addr - address to store host byte order value at
	may be poorly aligned
@param value - host byte order 48 bit value to store
*/
static __inline void StoreU48(uint8* addr, uint64 value) 
{
#if CPU_BE
	StoreBeU48(addr, value);
#else
	StoreLeU48(addr, value);
#endif
}

/*!
fetch a 64 bit Host Byte Order field

@param addr - address of host byte order item to fetch
	may be poorly aligned

@return - host byte order 64 bit value
*/
static __inline uint64 GetU64(const uint8* addr) 
{
#if CPU_BE
	return GetBeU64(addr);
#else
	return GetLeU64(addr);
#endif
}

/*!
store a 64 bit Host Byte Order field

@param addr - address to store host byte order value at
	may be poorly aligned
@param value - host byte order 64 bit value to store
*/
static __inline void StoreU64(uint8* addr, uint64 value) 
{
#if CPU_BE
	StoreBeU64(addr, value);
#else
	StoreLeU64(addr, value);
#endif
}

/* These macros make it very easy and clean to declare bit fields in byte order
 * sensitive structures. field arguments are provided in big endian order.
 * This matches the order in which most networking specs (including
 * the IBTA Infiniband specs) are presented.
 * The first argument will use the most significant bits.  The last entry
 * will use the least significant bits.  The implementation is dependent on
 * little endian compilers placing the 1st bit field encountered in the
 * least significant bits while big endian compilers put the 1st bit field
 * encountered in the most significant bits.
 * All bits in the vartype should be consumed by the sizes of the fields.
 * Any unused bits could result in undefined or compiler specific behaviour.
 */
#if CPU_BE
#if CPU_LE
#error "Unknown byte order, CPU_LE and CPU_BE both set"
#endif
    #define IB_BITFIELD2(vartype,field1,field2)  \
        vartype field1; \
        vartype field2; 

    #define IB_BITFIELD3(vartype,field1,field2,field3)  \
        vartype field1; \
        vartype field2; \
        vartype field3; 

    #define IB_BITFIELD4(vartype,field1,field2,field3,field4)  \
        vartype field1; \
        vartype field2; \
        vartype field3; \
        vartype field4;

    #define IB_BITFIELD5(vartype,field1,field2,field3,field4,field5)  \
        vartype field1; \
        vartype field2; \
        vartype field3; \
        vartype field4; \
        vartype field5; 

    #define IB_BITFIELD6(vartype,field1,field2,field3,field4,field5,field6)  \
        vartype field1; \
        vartype field2; \
        vartype field3; \
        vartype field4; \
        vartype field5; \
        vartype field6;

    #define IB_BITFIELD7(vartype,field1,field2,field3,field4,field5,field6,field7)  \
        vartype field1; \
        vartype field2; \
        vartype field3; \
        vartype field4; \
        vartype field5; \
        vartype field6; \
        vartype field7;

    #define IB_BITFIELD8(vartype,field1,field2,field3,field4,field5,field6,field7,field8)  \
        vartype field1; \
        vartype field2; \
        vartype field3; \
        vartype field4; \
        vartype field5; \
        vartype field6; \
        vartype field7; \
        vartype field8; 

   #define IB_BITFIELD9(vartype,field1,field2,field3,field4,field5,field6,field7,field8,field9)  \
        vartype field1; \
        vartype field2; \
        vartype field3; \
        vartype field4; \
        vartype field5; \
        vartype field6; \
        vartype field7; \
        vartype field8; \
        vartype field9;
		
    #define IB_BITFIELD10(vartype,field1,field2,field3,field4,field5,field6,field7,field8,field9,field10)  \
        vartype field1; \
        vartype field2; \
        vartype field3; \
        vartype field4; \
        vartype field5; \
        vartype field6; \
        vartype field7; \
        vartype field8; \
        vartype field9; \
        vartype field10;  	

    #define IB_BITFIELD11(vartype,field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11)  \
        vartype field1; \
        vartype field2; \
        vartype field3; \
        vartype field4; \
        vartype field5; \
        vartype field6; \
        vartype field7; \
        vartype field8; \
        vartype field9; \
        vartype field10;\
        vartype field11;

    #define IB_BITFIELD12(vartype,field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11,field12) \
        vartype field1; \
        vartype field2; \
        vartype field3; \
        vartype field4; \
        vartype field5; \
        vartype field6; \
        vartype field7; \
        vartype field8; \
        vartype field9; \
        vartype field10;\
        vartype field11;\
        vartype field12;

    #define IB_BITFIELD13(vartype,field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11,field12,field13) \
        vartype field1; \
        vartype field2; \
        vartype field3; \
        vartype field4; \
        vartype field5; \
        vartype field6; \
        vartype field7; \
        vartype field8; \
        vartype field9; \
        vartype field10;\
        vartype field11;\
        vartype field12;\
        vartype field13;

    #define IB_BITFIELD14(vartype,field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11,field12,field13,field14) \
        vartype field1; \
        vartype field2; \
        vartype field3; \
        vartype field4; \
        vartype field5; \
        vartype field6; \
        vartype field7; \
        vartype field8; \
        vartype field9; \
        vartype field10;\
        vartype field11;\
        vartype field12;\
        vartype field13;\
        vartype field14;

    #define IB_BITFIELD15(vartype,field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11,field12,field13,field14,field15) \
        vartype field1; \
        vartype field2; \
        vartype field3; \
        vartype field4; \
        vartype field5; \
        vartype field6; \
        vartype field7; \
        vartype field8; \
        vartype field9; \
        vartype field10;\
        vartype field11;\
        vartype field12;\
        vartype field13;\
        vartype field14;\
        vartype field15;

    #define IB_BITFIELD16(vartype,field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11,field12,field13,field14,field15,field16) \
        vartype field1; \
        vartype field2; \
        vartype field3; \
        vartype field4; \
        vartype field5; \
        vartype field6; \
        vartype field7; \
        vartype field8; \
        vartype field9; \
        vartype field10;\
        vartype field11;\
        vartype field12;\
        vartype field13;\
        vartype field14;\
        vartype field15;\
        vartype field16; 

    #define IB_BITFIELD17(vartype,field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11,field12,field13,field14,field15,field16,field17) \
        vartype field1; \
        vartype field2; \
        vartype field3; \
        vartype field4; \
        vartype field5; \
        vartype field6; \
        vartype field7; \
        vartype field8; \
        vartype field9; \
        vartype field10;\
        vartype field11;\
        vartype field12;\
        vartype field13;\
        vartype field14;\
        vartype field15;\
        vartype field16;\
        vartype field17; 

    #define IB_BITFIELD18(vartype,field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11,field12,field13,field14,field15,field16,field17,field18) \
        vartype field1; \
        vartype field2; \
        vartype field3; \
        vartype field4; \
        vartype field5; \
        vartype field6; \
        vartype field7; \
        vartype field8; \
        vartype field9; \
        vartype field10;\
        vartype field11;\
        vartype field12;\
        vartype field13;\
        vartype field14;\
        vartype field15;\
        vartype field16;\
        vartype field17;\
        vartype field18;

    #define IB_BITFIELD19(vartype,field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11,field12,field13,field14,field15,field16,field17,field18,field19) \
        vartype field1; \
        vartype field2; \
        vartype field3; \
        vartype field4; \
        vartype field5; \
        vartype field6; \
        vartype field7; \
        vartype field8; \
        vartype field9; \
        vartype field10;\
        vartype field11;\
        vartype field12;\
        vartype field13;\
        vartype field14;\
        vartype field15;\
        vartype field16;\
        vartype field17;\
        vartype field18;\
        vartype field19;

    #define IB_BITFIELD20(vartype,field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11,field12,field13,field14,field15,field16,field17,field18,field19,field20) \
        vartype field1; \
        vartype field2; \
        vartype field3; \
        vartype field4; \
        vartype field5; \
        vartype field6; \
        vartype field7; \
        vartype field8; \
        vartype field9; \
        vartype field10;\
        vartype field11;\
        vartype field12;\
        vartype field13;\
        vartype field14;\
        vartype field15;\
        vartype field16;\
        vartype field17;\
        vartype field18;\
        vartype field19;\
        vartype field20;

    #define IB_BITFIELD21(vartype,field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11,field12,field13,field14,field15,field16,field17,field18,field19,field20,field21) \
        vartype field1; \
        vartype field2; \
        vartype field3; \
        vartype field4; \
        vartype field5; \
        vartype field6; \
        vartype field7; \
        vartype field8; \
        vartype field9; \
        vartype field10;\
        vartype field11;\
        vartype field12;\
        vartype field13;\
        vartype field14;\
        vartype field15;\
        vartype field16;\
        vartype field17;\
        vartype field18;\
        vartype field19;\
        vartype field20;\
        vartype field21;

    #define IB_BITFIELD22(vartype,field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11,field12,field13,field14,field15,field16,field17,field18,field19,field20,field21,field22) \
        vartype field1; \
        vartype field2; \
        vartype field3; \
        vartype field4; \
        vartype field5; \
        vartype field6; \
        vartype field7; \
        vartype field8; \
        vartype field9; \
        vartype field10;\
        vartype field11;\
        vartype field12;\
        vartype field13;\
        vartype field14;\
        vartype field15;\
        vartype field16;\
        vartype field17;\
        vartype field18;\
        vartype field19;\
        vartype field20;\
        vartype field21;\
        vartype field22;

    #define IB_BITFIELD23(vartype,field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11,field12,field13,field14,field15,field16,field17,field18,field19,field20,field21,field22,field23) \
        vartype field1; \
        vartype field2; \
        vartype field3; \
        vartype field4; \
        vartype field5; \
        vartype field6; \
        vartype field7; \
        vartype field8; \
        vartype field9; \
        vartype field10;\
        vartype field11;\
        vartype field12;\
        vartype field13;\
        vartype field14;\
        vartype field15;\
        vartype field16;\
        vartype field17;\
        vartype field18;\
        vartype field19;\
        vartype field20;\
        vartype field21;\
        vartype field22;\
        vartype field23;

    #define IB_BITFIELD24(vartype,field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11,field12,field13,field14,field15,field16,field17,field18,field19,field20,field21,field22,field23,field24) \
        vartype field1; \
        vartype field2; \
        vartype field3; \
        vartype field4; \
        vartype field5; \
        vartype field6; \
        vartype field7; \
        vartype field8; \
        vartype field9; \
        vartype field10;\
        vartype field11;\
        vartype field12;\
        vartype field13;\
        vartype field14;\
        vartype field15;\
        vartype field16;\
        vartype field17;\
        vartype field18;\
        vartype field19;\
        vartype field20;\
        vartype field21;\
        vartype field22;\
        vartype field23;\
        vartype field24;

    #define IB_BITFIELD25(vartype,field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11,field12,field13,field14,field15,field16,field17,field18,field19,field20,field21,field22,field23,field24,field25) \
        vartype field1; \
        vartype field2; \
        vartype field3; \
        vartype field4; \
        vartype field5; \
        vartype field6; \
        vartype field7; \
        vartype field8; \
        vartype field9; \
        vartype field10;\
        vartype field11;\
        vartype field12;\
        vartype field13;\
        vartype field14;\
        vartype field15;\
        vartype field16;\
        vartype field17;\
        vartype field18;\
        vartype field19;\
        vartype field20;\
        vartype field21;\
        vartype field22;\
        vartype field23;\
        vartype field24;\
        vartype field25;

    #define IB_BITFIELD26(vartype,field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11,field12,field13,field14,field15,field16,field17,field18,field19,field20,field21,field22,field23,field24,field25,field26) \
        vartype field1; \
        vartype field2; \
        vartype field3; \
        vartype field4; \
        vartype field5; \
        vartype field6; \
        vartype field7; \
        vartype field8; \
        vartype field9; \
        vartype field10;\
        vartype field11;\
        vartype field12;\
        vartype field13;\
        vartype field14;\
        vartype field15;\
        vartype field16;\
        vartype field17;\
        vartype field18;\
        vartype field19;\
        vartype field20;\
        vartype field21;\
        vartype field22;\
        vartype field23;\
        vartype field24;\
        vartype field25;\
        vartype field26;

    #define IB_BITFIELD27(vartype,field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11,field12,field13,field14,field15,field16,field17,field18,field19,field20,field21,field22,field23,field24,field25,field26,field27) \
        vartype field1; \
        vartype field2; \
        vartype field3; \
        vartype field4; \
        vartype field5; \
        vartype field6; \
        vartype field7; \
        vartype field8; \
        vartype field9; \
        vartype field10;\
        vartype field11;\
        vartype field12;\
        vartype field13;\
        vartype field14;\
        vartype field15;\
        vartype field16;\
        vartype field17;\
        vartype field18;\
        vartype field19;\
        vartype field20;\
        vartype field21;\
        vartype field22;\
        vartype field23;\
        vartype field24;\
        vartype field25;\
        vartype field26;\
        vartype field27;

    #define IB_BITFIELD28(vartype,field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11,field12,field13,field14,field15,field16,field17,field18,field19,field20,field21,field22,field23,field24,field25,field26,field27,field28) \
        vartype field1; \
        vartype field2; \
        vartype field3; \
        vartype field4; \
        vartype field5; \
        vartype field6; \
        vartype field7; \
        vartype field8; \
        vartype field9; \
        vartype field10;\
        vartype field11;\
        vartype field12;\
        vartype field13;\
        vartype field14;\
        vartype field15;\
        vartype field16;\
        vartype field17;\
        vartype field18;\
        vartype field19;\
        vartype field20;\
        vartype field21;\
        vartype field22;\
        vartype field23;\
        vartype field24;\
        vartype field25;\
        vartype field26;\
        vartype field27;\
        vartype field28;

    #define IB_BITFIELD29(vartype,field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11,field12,field13,field14,field15,field16,field17,field18,field19,field20,field21,field22,field23,field24,field25,field26,field27,field28,field29) \
        vartype field1; \
        vartype field2; \
        vartype field3; \
        vartype field4; \
        vartype field5; \
        vartype field6; \
        vartype field7; \
        vartype field8; \
        vartype field9; \
        vartype field10;\
        vartype field11;\
        vartype field12;\
        vartype field13;\
        vartype field14;\
        vartype field15;\
        vartype field16;\
        vartype field17;\
        vartype field18;\
        vartype field19;\
        vartype field20;\
        vartype field21;\
        vartype field22;\
        vartype field23;\
        vartype field24;\
        vartype field25;\
        vartype field26;\
        vartype field27;\
        vartype field28;\
        vartype field29;

    #define IB_BITFIELD30(vartype,field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11,field12,field13,field14,field15,field16,field17,field18,field19,field20,field21,field22,field23,field24,field25,field26,field27,field28,field29,field30) \
        vartype field1; \
        vartype field2; \
        vartype field3; \
        vartype field4; \
        vartype field5; \
        vartype field6; \
        vartype field7; \
        vartype field8; \
        vartype field9; \
        vartype field10;\
        vartype field11;\
        vartype field12;\
        vartype field13;\
        vartype field14;\
        vartype field15;\
        vartype field16;\
        vartype field17;\
        vartype field18;\
        vartype field19;\
        vartype field20;\
        vartype field21;\
        vartype field22;\
        vartype field23;\
        vartype field24;\
        vartype field25;\
        vartype field26;\
        vartype field27;\
        vartype field28;\
        vartype field29;\
        vartype field30;

    #define IB_BITFIELD31(vartype,field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11,field12,field13,field14,field15,field16,field17,field18,field19,field20,field21,field22,field23,field24,field25,field26,field27,field28,field29,field30,field31) \
        vartype field1; \
        vartype field2; \
        vartype field3; \
        vartype field4; \
        vartype field5; \
        vartype field6; \
        vartype field7; \
        vartype field8; \
        vartype field9; \
        vartype field10;\
        vartype field11;\
        vartype field12;\
        vartype field13;\
        vartype field14;\
        vartype field15;\
        vartype field16;\
        vartype field17;\
        vartype field18;\
        vartype field19;\
        vartype field20;\
        vartype field21;\
        vartype field22;\
        vartype field23;\
        vartype field24;\
        vartype field25;\
        vartype field26;\
        vartype field27;\
        vartype field28;\
        vartype field29;\
        vartype field30;\
        vartype field31;

    #define IB_BITFIELD32(vartype,field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11,field12,field13,field14,field15,field16,field17,field18,field19,field20,field21,field22,field23,field24,field25,field26,field27,field28,field29,field30,field31,field32) \
        vartype field1; \
        vartype field2; \
        vartype field3; \
        vartype field4; \
        vartype field5; \
        vartype field6; \
        vartype field7; \
        vartype field8; \
        vartype field9; \
        vartype field10;\
        vartype field11;\
        vartype field12;\
        vartype field13;\
        vartype field14;\
        vartype field15;\
        vartype field16;\
        vartype field17;\
        vartype field18;\
        vartype field19;\
        vartype field20;\
        vartype field21;\
        vartype field22;\
        vartype field23;\
        vartype field24;\
        vartype field25;\
        vartype field26;\
        vartype field27;\
        vartype field28;\
        vartype field29;\
        vartype field30;\
        vartype field31;\
        vartype field32; 

#elif CPU_LE
    #define IB_BITFIELD2(vartype,field1,field2)  \
        vartype field2; \
        vartype field1; 

    #define IB_BITFIELD3(vartype,field1,field2,field3)  \
        vartype field3; \
        vartype field2; \
        vartype field1; 

    #define IB_BITFIELD4(vartype,field1,field2,field3,field4)  \
        vartype field4; \
        vartype field3; \
        vartype field2; \
        vartype field1; 

    #define IB_BITFIELD5(vartype,field1,field2,field3,field4,field5)  \
        vartype field5; \
        vartype field4; \
        vartype field3; \
        vartype field2; \
        vartype field1; 

    #define IB_BITFIELD6(vartype,field1,field2,field3,field4,field5,field6)  \
        vartype field6; \
        vartype field5; \
        vartype field4; \
        vartype field3; \
        vartype field2; \
        vartype field1;

    #define IB_BITFIELD7(vartype,field1,field2,field3,field4,field5,field6,field7)  \
        vartype field7; \
        vartype field6; \
        vartype field5; \
        vartype field4; \
        vartype field3; \
        vartype field2; \
        vartype field1;

    #define IB_BITFIELD8(vartype,field1,field2,field3,field4,field5,field6,field7,field8)  \
        vartype field8; \
        vartype field7; \
        vartype field6; \
        vartype field5; \
        vartype field4; \
        vartype field3; \
        vartype field2; \
        vartype field1;

   #define IB_BITFIELD9(vartype,field1,field2,field3,field4,field5,field6,field7,field8,field9)  \
        vartype field9; \
        vartype field8; \
        vartype field7; \
        vartype field6; \
        vartype field5; \
        vartype field4; \
        vartype field3; \
        vartype field2; \
        vartype field1;

    #define IB_BITFIELD10(vartype,field1,field2,field3,field4,field5,field6,field7,field8,field9,field10)  \
        vartype field10;\
        vartype field9; \
        vartype field8; \
        vartype field7; \
        vartype field6; \
        vartype field5; \
        vartype field4; \
        vartype field3; \
        vartype field2; \
        vartype field1;

    #define IB_BITFIELD11(vartype,field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11)  \
        vartype field11;\
        vartype field10;\
        vartype field9; \
        vartype field8; \
        vartype field7; \
        vartype field6; \
        vartype field5; \
        vartype field4; \
        vartype field3; \
        vartype field2; \
        vartype field1;

    #define IB_BITFIELD12(vartype,field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11,field12) \
        vartype field12;\
        vartype field11;\
        vartype field10;\
        vartype field9; \
        vartype field8; \
        vartype field7; \
        vartype field6; \
        vartype field5; \
        vartype field4; \
        vartype field3; \
        vartype field2; \
        vartype field1;

    #define IB_BITFIELD13(vartype,field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11,field12,field13) \
        vartype field13;\
        vartype field12;\
        vartype field11;\
        vartype field10;\
        vartype field9; \
        vartype field8; \
        vartype field7; \
        vartype field6; \
        vartype field5; \
        vartype field4; \
        vartype field3; \
        vartype field2; \
        vartype field1;

    #define IB_BITFIELD14(vartype,field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11,field12,field13,field14) \
        vartype field14;\
        vartype field13;\
        vartype field12;\
        vartype field11;\
        vartype field10;\
        vartype field9; \
        vartype field8; \
        vartype field7; \
        vartype field6; \
        vartype field5; \
        vartype field4; \
        vartype field3; \
        vartype field2; \
        vartype field1;

    #define IB_BITFIELD15(vartype,field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11,field12,field13,field14,field15) \
        vartype field15;\
        vartype field14;\
        vartype field13;\
        vartype field12;\
        vartype field11;\
        vartype field10;\
        vartype field9; \
        vartype field8; \
        vartype field7; \
        vartype field6; \
        vartype field5; \
        vartype field4; \
        vartype field3; \
        vartype field2; \
        vartype field1;

    #define IB_BITFIELD16(vartype,field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11,field12,field13,field14,field15,field16) \
        vartype field16;\
        vartype field15;\
        vartype field14;\
        vartype field13;\
        vartype field12;\
        vartype field11;\
        vartype field10;\
        vartype field9; \
        vartype field8; \
        vartype field7; \
        vartype field6; \
        vartype field5; \
        vartype field4; \
        vartype field3; \
        vartype field2; \
        vartype field1;

    #define IB_BITFIELD17(vartype,field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11,field12,field13,field14,field15,field16,field17) \
        vartype field17;\
        vartype field16;\
        vartype field15;\
        vartype field14;\
        vartype field13;\
        vartype field12;\
        vartype field11;\
        vartype field10;\
        vartype field9; \
        vartype field8; \
        vartype field7; \
        vartype field6; \
        vartype field5; \
        vartype field4; \
        vartype field3; \
        vartype field2; \
        vartype field1;

    #define IB_BITFIELD18(vartype,field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11,field12,field13,field14,field15,field16,field17,field18) \
        vartype field18;\
        vartype field17;\
        vartype field16;\
        vartype field15;\
        vartype field14;\
        vartype field13;\
        vartype field12;\
        vartype field11;\
        vartype field10;\
        vartype field9; \
        vartype field8; \
        vartype field7; \
        vartype field6; \
        vartype field5; \
        vartype field4; \
        vartype field3; \
        vartype field2; \
        vartype field1;

    #define IB_BITFIELD19(vartype,field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11,field12,field13,field14,field15,field16,field17,field18,field19) \
        vartype field19;\
        vartype field18;\
        vartype field17;\
        vartype field16;\
        vartype field15;\
        vartype field14;\
        vartype field13;\
        vartype field12;\
        vartype field11;\
        vartype field10;\
        vartype field9; \
        vartype field8; \
        vartype field7; \
        vartype field6; \
        vartype field5; \
        vartype field4; \
        vartype field3; \
        vartype field2; \
        vartype field1;

    #define IB_BITFIELD20(vartype,field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11,field12,field13,field14,field15,field16,field17,field18,field19,field20) \
        vartype field20;\
        vartype field19;\
        vartype field18;\
        vartype field17;\
        vartype field16;\
        vartype field15;\
        vartype field14;\
        vartype field13;\
        vartype field12;\
        vartype field11;\
        vartype field10;\
        vartype field9; \
        vartype field8; \
        vartype field7; \
        vartype field6; \
        vartype field5; \
        vartype field4; \
        vartype field3; \
        vartype field2; \
        vartype field1;

    #define IB_BITFIELD21(vartype,field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11,field12,field13,field14,field15,field16,field17,field18,field19,field20,field21) \
        vartype field21;\
        vartype field20;\
        vartype field19;\
        vartype field18;\
        vartype field17;\
        vartype field16;\
        vartype field15;\
        vartype field14;\
        vartype field13;\
        vartype field12;\
        vartype field11;\
        vartype field10;\
        vartype field9; \
        vartype field8; \
        vartype field7; \
        vartype field6; \
        vartype field5; \
        vartype field4; \
        vartype field3; \
        vartype field2; \
        vartype field1;

    #define IB_BITFIELD22(vartype,field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11,field12,field13,field14,field15,field16,field17,field18,field19,field20,field21,field22) \
        vartype field22;\
        vartype field21;\
        vartype field20;\
        vartype field19;\
        vartype field18;\
        vartype field17;\
        vartype field16;\
        vartype field15;\
        vartype field14;\
        vartype field13;\
        vartype field12;\
        vartype field11;\
        vartype field10;\
        vartype field9; \
        vartype field8; \
        vartype field7; \
        vartype field6; \
        vartype field5; \
        vartype field4; \
        vartype field3; \
        vartype field2; \
        vartype field1;

    #define IB_BITFIELD23(vartype,field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11,field12,field13,field14,field15,field16,field17,field18,field19,field20,field21,field22,field23) \
        vartype field23;\
        vartype field22;\
        vartype field21;\
        vartype field20;\
        vartype field19;\
        vartype field18;\
        vartype field17;\
        vartype field16;\
        vartype field15;\
        vartype field14;\
        vartype field13;\
        vartype field12;\
        vartype field11;\
        vartype field10;\
        vartype field9; \
        vartype field8; \
        vartype field7; \
        vartype field6; \
        vartype field5; \
        vartype field4; \
        vartype field3; \
        vartype field2; \
        vartype field1;

    #define IB_BITFIELD24(vartype,field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11,field12,field13,field14,field15,field16,field17,field18,field19,field20,field21,field22,field23,field24) \
        vartype field24;\
        vartype field23;\
        vartype field22;\
        vartype field21;\
        vartype field20;\
        vartype field19;\
        vartype field18;\
        vartype field17;\
        vartype field16;\
        vartype field15;\
        vartype field14;\
        vartype field13;\
        vartype field12;\
        vartype field11;\
        vartype field10;\
        vartype field9; \
        vartype field8; \
        vartype field7; \
        vartype field6; \
        vartype field5; \
        vartype field4; \
        vartype field3; \
        vartype field2; \
        vartype field1;

    #define IB_BITFIELD25(vartype,field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11,field12,field13,field14,field15,field16,field17,field18,field19,field20,field21,field22,field23,field24,field25) \
        vartype field25;\
        vartype field24;\
        vartype field23;\
        vartype field22;\
        vartype field21;\
        vartype field20;\
        vartype field19;\
        vartype field18;\
        vartype field17;\
        vartype field16;\
        vartype field15;\
        vartype field14;\
        vartype field13;\
        vartype field12;\
        vartype field11;\
        vartype field10;\
        vartype field9; \
        vartype field8; \
        vartype field7; \
        vartype field6; \
        vartype field5; \
        vartype field4; \
        vartype field3; \
        vartype field2; \
        vartype field1;

    #define IB_BITFIELD26(vartype,field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11,field12,field13,field14,field15,field16,field17,field18,field19,field20,field21,field22,field23,field24,field25,field26) \
        vartype field26;\
        vartype field25;\
        vartype field24;\
        vartype field23;\
        vartype field22;\
        vartype field21;\
        vartype field20;\
        vartype field19;\
        vartype field18;\
        vartype field17;\
        vartype field16;\
        vartype field15;\
        vartype field14;\
        vartype field13;\
        vartype field12;\
        vartype field11;\
        vartype field10;\
        vartype field9; \
        vartype field8; \
        vartype field7; \
        vartype field6; \
        vartype field5; \
        vartype field4; \
        vartype field3; \
        vartype field2; \
        vartype field1;

    #define IB_BITFIELD27(vartype,field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11,field12,field13,field14,field15,field16,field17,field18,field19,field20,field21,field22,field23,field24,field25,field26,field27) \
        vartype field27;\
        vartype field26;\
        vartype field25;\
        vartype field24;\
        vartype field23;\
        vartype field22;\
        vartype field21;\
        vartype field20;\
        vartype field19;\
        vartype field18;\
        vartype field17;\
        vartype field16;\
        vartype field15;\
        vartype field14;\
        vartype field13;\
        vartype field12;\
        vartype field11;\
        vartype field10;\
        vartype field9; \
        vartype field8; \
        vartype field7; \
        vartype field6; \
        vartype field5; \
        vartype field4; \
        vartype field3; \
        vartype field2; \
        vartype field1;

    #define IB_BITFIELD28(vartype,field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11,field12,field13,field14,field15,field16,field17,field18,field19,field20,field21,field22,field23,field24,field25,field26,field27,field28) \
        vartype field28;\
        vartype field27;\
        vartype field26;\
        vartype field25;\
        vartype field24;\
        vartype field23;\
        vartype field22;\
        vartype field21;\
        vartype field20;\
        vartype field19;\
        vartype field18;\
        vartype field17;\
        vartype field16;\
        vartype field15;\
        vartype field14;\
        vartype field13;\
        vartype field12;\
        vartype field11;\
        vartype field10;\
        vartype field9; \
        vartype field8; \
        vartype field7; \
        vartype field6; \
        vartype field5; \
        vartype field4; \
        vartype field3; \
        vartype field2; \
        vartype field1;

    #define IB_BITFIELD29(vartype,field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11,field12,field13,field14,field15,field16,field17,field18,field19,field20,field21,field22,field23,field24,field25,field26,field27,field28,field29) \
        vartype field29;\
        vartype field28;\
        vartype field27;\
        vartype field26;\
        vartype field25;\
        vartype field24;\
        vartype field23;\
        vartype field22;\
        vartype field21;\
        vartype field20;\
        vartype field19;\
        vartype field18;\
        vartype field17;\
        vartype field16;\
        vartype field15;\
        vartype field14;\
        vartype field13;\
        vartype field12;\
        vartype field11;\
        vartype field10;\
        vartype field9; \
        vartype field8; \
        vartype field7; \
        vartype field6; \
        vartype field5; \
        vartype field4; \
        vartype field3; \
        vartype field2; \
        vartype field1;

    #define IB_BITFIELD30(vartype,field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11,field12,field13,field14,field15,field16,field17,field18,field19,field20,field21,field22,field23,field24,field25,field26,field27,field28,field29,field30) \
        vartype field30;\
        vartype field29;\
        vartype field28;\
        vartype field27;\
        vartype field26;\
        vartype field25;\
        vartype field24;\
        vartype field23;\
        vartype field22;\
        vartype field21;\
        vartype field20;\
        vartype field19;\
        vartype field18;\
        vartype field17;\
        vartype field16;\
        vartype field15;\
        vartype field14;\
        vartype field13;\
        vartype field12;\
        vartype field11;\
        vartype field10;\
        vartype field9; \
        vartype field8; \
        vartype field7; \
        vartype field6; \
        vartype field5; \
        vartype field4; \
        vartype field3; \
        vartype field2; \
        vartype field1;

    #define IB_BITFIELD31(vartype,field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11,field12,field13,field14,field15,field16,field17,field18,field19,field20,field21,field22,field23,field24,field25,field26,field27,field28,field29,field30,field31) \
        vartype field31;\
        vartype field30;\
        vartype field29;\
        vartype field28;\
        vartype field27;\
        vartype field26;\
        vartype field25;\
        vartype field24;\
        vartype field23;\
        vartype field22;\
        vartype field21;\
        vartype field20;\
        vartype field19;\
        vartype field18;\
        vartype field17;\
        vartype field16;\
        vartype field15;\
        vartype field14;\
        vartype field13;\
        vartype field12;\
        vartype field11;\
        vartype field10;\
        vartype field9; \
        vartype field8; \
        vartype field7; \
        vartype field6; \
        vartype field5; \
        vartype field4; \
        vartype field3; \
        vartype field2; \
        vartype field1;

    #define IB_BITFIELD32(vartype,field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11,field12,field13,field14,field15,field16,field17,field18,field19,field20,field21,field22,field23,field24,field25,field26,field27,field28,field29,field30,field31,field32) \
        vartype field32;\
        vartype field31;\
        vartype field30;\
        vartype field29;\
        vartype field28;\
        vartype field27;\
        vartype field26;\
        vartype field25;\
        vartype field24;\
        vartype field23;\
        vartype field22;\
        vartype field21;\
        vartype field20;\
        vartype field19;\
        vartype field18;\
        vartype field17;\
        vartype field16;\
        vartype field15;\
        vartype field14;\
        vartype field13;\
        vartype field12;\
        vartype field11;\
        vartype field10;\
        vartype field9; \
        vartype field8; \
        vartype field7; \
        vartype field6; \
        vartype field5; \
        vartype field4; \
        vartype field3; \
        vartype field2; \
        vartype field1;
#else
#error "Uknown Byte order, neither CPU_LE nor CPU_BE set"
#endif  /* CPU_LE */

#ifdef __cplusplus
}	/* extern "C" */
#endif

#endif /* _IBA_PUBLIC_IBYTE_SWAP_H_ */

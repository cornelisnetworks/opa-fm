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

#ifndef __IBA_IB_MAD_H__
#define __IBA_IB_MAD_H__

#include "iba/public/datatypes.h"

/* Ths should be removed from here */
#include "iba/stl_types.h"

#include "iba/public/ibyteswap.h"
#include "iba/public/imemory.h"

#if defined (__cplusplus)
extern "C" {
#endif

	
#include "iba/public/ipackon.h"

/* --------------------------------------------------------------------------
 * Defines 
 */

// convert a 0-N bit number to a index and uint8 mask for access to
// multi-byte fields which are stored as uint8[] arrays for long bit masks
// Note for uint16, uint32 bytes, byte swapping has already been done so
// simple (1<<bit) masks can be used and these macros should NOT be used
// size is in bits (must be a multiple of 8)
#define LONG_MASK_BIT_INDEX(bit, size)	((((size)/8)-1)-(bit)/8)// byte 0=MSB,byte 7=LSB
#define LONG_MASK_BIT_MASK(bit)	(1 << ((bit)%8))	// bit 0=LSbit,bit 7=MSbit

#define IB_MAD_BLOCK_SIZE			256

#if !defined(STL_MAD_BLOCK_SIZE)
#define STL_MAD_BLOCK_SIZE			2048
#endif
#define MAD_BLOCK_SIZE				STL_MAD_BLOCK_SIZE

#define IB_BASE_VERSION				1
/* use ib_generalServices.h or ib_sm_types.h to get proper class version */


/*
 * MANAGEMENT CLASS METHODS
 *	Methods define the operations which a management class supports. Each
 *	management class can define its own set of Methods.
 *	The upper bit of the Method field is designated as the response bit (R)
 *	and is used to distinguish three types of methods:
 */

/* Management Class Values */

#define MCLASS_SM_LID_ROUTED		0x01
#define MCLASS_SM_DIRECTED_ROUTE	0x81

#define MCLASS_RESERVED_02			0x02

#define MCLASS_SUBN_ADM				0x03	/* Subnet Administration class  */
											/* All Nodes hosting a  */
											/* Subnet Manager */
#define MCLASS_PERF					0x04	/* Performance Management class  */
#define MCLASS_BM					0x05	/* Baseboard Management class  */
#define MCLASS_DEV_MGT				0x06	/* Device Management class  */
#define MCLASS_COMM_MGT				0x07	/* Communications Management class */
#define MCLASS_SNMP					0x08	/* SNMP Tunneling class (tunneling */
											/* of the SNMP protocol through  */
											/* the IBA fabric) */
/*
 * Vendor Defined classes 0x09-0x0F
 */
#define MCLASS_DEV_CONF_MGT			0x10	/* Device Configuration Management */
											/* class (CFM/IORM) */
#define MCLASS_DTA					0x20	/* IBTA CIWG Device Test Agent */
#define MCLASS_CC					0x21	/* Congestion Control class */
#define MCLASS_VFI_PM		        0x32    /* PM VFI mclass value */


/* --------------------------------------------------------------------------
 * MANAGEMENT CLASS METHODS
 *	Methods define the operations which a management class supports. Each
 *	management class can define its own set of Methods.
 *	The upper bit of the Method field is designated as the response bit (R) 
 *	and is used to distinguish three types of methods:
 *
 *	a) Messages - Methods for which no response is ever generated. The R
 *	bit is not set and a corresponding Method with the R bit set is reserved
 *	and not used.
 *	b) Requests - Methods for which a response may be generated. The R
 *	bit is not set and a corresponding Method with the R bit set is defined
 *	and potentially used to respond.
 *	c) Responses - Methods generated in response to a request. The R bit
 *	is set and a corresponding Method with the R bit not set is defined
 *	and used to trigger the response.
 *
 *	It is anticipated that many classes will use similar Methods. 
 *	In order to provide regularity among management datagrams, 
 *	if any of the Methods needed by a class has the same semantics 
 *	as one of the Methods defined below it shall use that Method name 
 *	and value.
 *
 *	A class is not required to support all of the common methods defined here.
 *	However, the class shall not redefine the values assigned to these
 *	common methods for their class-dependent methods. All class-depen-dent
 *	method values shall start at 0x10 and use the R bit according to the
 *	semantics of types of Methods defined above.
 */


/* Common Management Methods (including R bit) Description */

/* 0x00, 0x80 Reserved. */

#define MMTHD_GET			0x01	/* Request (read) an attribute from a node */
#define MMTHD_SET			0x02	/* Request a set (write) of an attribute  */
									/* in a node  */
#define MMTHD_SEND			0x03	/* Send a datagram. Does not require  */
									/* a response. */
#define MMTHD_RESERVED_04	0x04	/* Reserved. */
#define MMTHD_TRAP			0x05	/* Unsolicited datagram sent from a node  */
									/* indicating an event occured that may  */
									/* be of interest.  */
									/* A trap method shall always use the  */
									/* Notice attribute. */
#define MMTHD_REPORT		0x06	/* Forwarding an event/trap/notice to  */
									/* interested party. */
#define	MMTHD_TRAP_REPRESS	0x07	/* Instruct a Trap sender to stop sending */
									/* a repeated Trap. */
/* 0x08-0x0F - Reserved */
/* 0x10-0x7F - Class-specific methods defined by Class */

#define MMTHD_GET_RESP		0x81	/* The response from an attribute Get or  */
									/* Set request. */
#define MMTHD_RESERVED_82	0x82	/* Reserved. */
#define MMTHD_RESERVED_83	0x83	/* Reserved. */
#define MMTHD_RESERVED_84	0x84	/* Reserved. */
#define MMTHD_RESERVED_85	0x85	/* Reserved. */

#define MMTHD_REPORT_RESP	0x86	/* Response to a Report().  */


/* 0x87-0x8F - Reserved. */
/* 0x90-0xFF - Class-dependent methods. Usage is defined by the class.  */






/* --------------------------------------------------------------------------
 * STATUS FIELD
 *	The Status field is only valid for methods of type Response (R bit set).
 *	It consists of 16 bits, each bit representing a unique status indication. 
 *	The 8 low order bits of the field are used for indications common to 
 *	all classes.
 *
 *	The 8 upper bits of the field can be used for class-dependent indications.
 *	A status field value of zero (no bit set) indicates a successful response.
 *	It is reserved and shall be set to 0 for methods of type Request and 
 *	Message.
 */

/*
 * MAD Status Field Bit Values
 */
#define MAD_STATUS_SUCCESS						0x0000
#define MAD_STATUS_BUSY							0x0001	
#define MAD_STATUS_REDIRECT_REQD				0x0002	

/*
 * bits 2-4 Code for invalid field
 *	0 -no invalid fields
 *	1 -The class version specified is not supported.
 *		(IBTA 1.1 also includes unsupported Base version in this category)
 *	2 -The method specified is not supported
 *	3 -The method/attribute combination is not supported
 *	4-6:Reserved
 *	7 -One or more fields in the attribute contain an invalid value.
 *
 * Defines below are coded for use against AsReg16 of MAD_STATUS
 */
#define MAD_STATUS_UNSUPPORTED_CLASS_VER		0x0004	
#define MAD_STATUS_UNSUPPORTED_VER				MAD_STATUS_UNSUPPORTED_CLASS_VER
#define MAD_STATUS_UNSUPPORTED_METHOD			0x0008	
#define MAD_STATUS_UNSUPPORTED_METHOD_ATTRIB	0x000c
/* 0x0010-0x0018 - Reserved */
#define MAD_STATUS_INVALID_ATTRIB				0x001c
	
/* 0x0020-0x00ff - Reserved */

/* 0x0100-0xff00 class-dependent values. */


typedef union _MAD_STATUS {
	uint16	AsReg16;

	struct { IB_BITFIELD5(uint16,
		ClassSpecific:	8,	/* Class dependent values */
		Reserved1	:	3,
		InvalidField:	3,
		RedirectRqrd:	1,	/* Redirect_required Redirection. not an error */
		Busy		:	1)	/* Temporarily busy. MAD discarded. not an error */
	} PACK_SUFFIX S;
	
} PACK_SUFFIX MAD_STATUS;

/* convert MAD Status to a string
 * The string will only contain the message corresponding to the defined values
 * to be safe, callers should also display the hex value in the event a
 * reserved or class specific value is returned by the manager/agent
 */
IBA_API const char* iba_mad_status_msg(MAD_STATUS madStatus);
IBA_API const char* iba_mad_status_msg2(uint16 madStatus);



/* --------------------------------------------------------------------------
 * MANAGEMENT CLASS ATTRIBUTES
 *	Attributes define the data which a management class manipulates. Each
 *	management class defines its own set of Attributes.
 *	Attributes are composite structures consisting of components typically
 *	representing hardware registers in nodes. Attributes can be read or
 *	written. Each Attribute is assigned a unique Attribute ID.
 *	Some Components within Attributes are identified as reserved. They shall
 *	be set to zero when building a MAD, except when building a response
 *	MAD where they may contain the corresponding contents of the request
 *	MAD. They shall be ignored in received MADs.
 *	An Attribute Modifier is also provided for each Attribute; however, not 
 *	all Attributes utilize this modifier. The use of the Attribute Modifier
 *	is Attribute specific. When the Attribute Modifier is not used it shall 
 *	be set to zero. It is not possible to selectively set a single Component 
 *	within an Attribute. A Get() must be performed to obtain the whole 
 *	attribute, the single Component must be modified in the result and a 
 *	Set() must be performed to write the whole attribute. No atomicity is 
 *	provided in this sequence of operations. 
 *	Some Components within an Attribute may not be settable. In this case,
 *	the respective Components within the Attribute are simply ignored 
 *	when setting.
 *
 *	Each management class provides a section that describe its Attributes
 *	and the Components contained within them, along with their settability,
 *	and how they are mapped into the Data field of the MAD.
 *	A given Attribute shall have the same format for the Get(), Set()
 *	and Get-Resp() Methods if used with those Methods. Its format for other 
 *	Methods may be different, in particular a given Attribute may have a 
 *	different format for SendResp() than it has for Send().
 *
 *	The following common Attributes are defined:
 *	All class-dependent attributes values shall start at 0x0010.
 */

/* Common Attributes */
#define	MCLASS_ATTRIB_ID_RESERVED_0			0x0000	/* Reserved */
#define	MCLASS_ATTRIB_ID_CLASS_PORT_INFO	0x0001
#define	MCLASS_ATTRIB_ID_NOTICE				0x0002 
#define	MCLASS_ATTRIB_ID_INFORM_INFO		0x0003 

/* 0x0004-0x000F - Reserved */
/* 0x0010-0x7FFF - Class-dependant values. */

/* Baseboard Management Attributes IDs */   /* Attribute Modifier values */
#define BM_ATTRIB_MOD_REQUEST		        0x0000
#define BM_ATTRIB_MOD_RESPONSE		        0x0001



/* --------------------------------------------------------------------------
 * MANAGEMENT DATAGRAMS
 *	Management Datagrams (MADs) provide a set of defined packets, which
 *	are used by managers and agents to communicate across a fabric. There
 *	is a format and specified use for each MAD, which corresponds to 
 *	predefined management classes. MADs provide a fundamental mechanism
 *	for in-band management.
 *
 * MANAGEMENT DATAGRAM FORMAT
 *	The MAD is an abstract definition of a base format whose data area usage
 *	shall be defined by a management class. The base format of the MAD to
 *	which all classes will conform is described in the structure MAD.
 *	The MAD shall have a packet payload of 256 bytes. This is to guarantee
 *	that MADs will not get dropped by a node that only supports the minimum
 *	packet size.
 *	Any unused portion of the 256 bytes shall be filled with zeroes when
 *	building a MAD, except when building a response MAD where the unused
 *	portion may contain the corresponding contents of the request MAD.
 */
typedef struct _MAD_COMMON {
	uint8	BaseVersion;			/* Version of management datagram base  */
									/* format.Value is set to 1 */
	uint8	MgmtClass;				/* Class of operation.  */
	uint8	ClassVersion;			/* Version of MAD class-specific format. */
									/* Value is set to 1 except  */
									/* for the Vendor class */
	union {
		uint8	AsReg8;
		struct { IB_BITFIELD2(uint8,
			R		:1,		/* Request/response field,  */
							/* conformant to base class definition */
			Method	:7)		/* Method to perform based on  */
							/* the management class */
		} PACK_SUFFIX s;
	} mr;

	union {
		/* All MADs use this structure */
		struct {
			MAD_STATUS	Status;		/* Code indicating status of method */
			uint16		Reserved1;	/* Reserved. Shall be set to 0 */
		} NS;						/* Normal MAD */

		/* This structure is used only for Directed Route SMP's */
		struct {
			struct { IB_BITFIELD2(uint16,
				D		:1,	/* Direction bit to determine  */
							/* direction of packet */
				Status	:15)/* Code indicating status of method */
			} PACK_SUFFIX s;						
			uint8	HopPointer;		/* used to indicate the current byte  */
									/* of the Initial/Return Path field. */
			uint8	HopCount;		/* used to contain the number of valid  */
									/* bytes in the Initial/Return Path */
		} DR;				/* Directed Route only */
	} u;
	uint64	TransactionID;			/* Transaction specific identifier */
	uint16	AttributeID;			/* Defines objects being operated  */
									/* on by a management class */
	uint16	Reserved2;				/* Reserved. Shall be set to 0 */
	uint32	AttributeModifier;		/* Provides further scope to  */
									/* the Attributes, usage  */
									/* determined by the management class */
} PACK_SUFFIX MAD_COMMON;




/*
 * MAD Base Fields
 */
typedef struct _MAD 
{
	MAD_COMMON	common;
	uint8		Data[MAD_BLOCK_SIZE - sizeof(MAD_COMMON)];
} PACK_SUFFIX MAD;

/* Responses are implicitly expected for most requests
 * however behaviour can be class specific.
 * Note that MMTHD_TRAP_REPRESS does not have R bit set but
 * is technically a response, however MMTHD_REPORT_RESP does
 *
 * For mads which are not a request, TransactionID is owned by client.
 * For requests, the GSI will use low bits of TransactionID for client
 * identification to facilitate directing response to correct client.
 */
#define MAD_IS_REQUEST(mad) \
	(((mad)->common.mr.s.R == 0 \
		&& (mad)->common.mr.s.Method != MMTHD_TRAP_REPRESS \
		&& (mad)->common.mr.s.Method != MMTHD_SEND) \
     || (((mad)->common.MgmtClass == MCLASS_BM) && \
        ( (mad)->common.mr.s.Method == MMTHD_SEND && !((mad)->common.AttributeModifier & BM_ATTRIB_MOD_RESPONSE))))

/* mads for which a GET_RESP should be returned if validation fails */
#define MAD_EXPECT_GET_RESP(mad) \
	((mad)->common.mr.s.R == 0 \
	 && ((mad)->common.mr.s.Method == MMTHD_GET \
		 || (mad)->common.mr.s.Method == MMTHD_SET))

/* could optimize and skip MMTHD_SEND test below since R=1 -> != SEND
 * however we play it safe in case remote node fails to initialize
 * R bit for a SEND message.
 */
#define MAD_IS_RESPONSE(mad) \
	((((mad)->common.mr.s.R \
		  || (mad)->common.mr.s.Method == MMTHD_TRAP_REPRESS) \
	   && (mad)->common.mr.s.Method != MMTHD_SEND) \
     || (((mad)->common.MgmtClass == MCLASS_BM) && \
        ((mad)->common.mr.s.Method == MMTHD_SEND && ((mad)->common.AttributeModifier & BM_ATTRIB_MOD_RESPONSE))))        

/* SENDs are technically a 3rd type, not necessarily a request nor response */
#define MAD_IS_SEND(mad) \
	((mad)->common.mr.s.Method == MMTHD_SEND)

#define MAD_SET_VERSION_INFO(mad, base_ver, mgmt_class, class_ver) {\
	(mad)->common.BaseVersion = base_ver; \
	(mad)->common.MgmtClass = mgmt_class; \
	(mad)->common.ClassVersion = class_ver; \
}

#define MAD_GET_VERSION_INFO(mad, base_ver, mgmt_class, class_ver) {\
	(*base_ver) = (mad)->common.BaseVersion; \
	(*mgmt_class) = (mad)->common.MgmtClass; \
	(*class_ver) = (mad)->common.ClassVersion; \
}

#define MAD_GET_STATUS(mad, status) {\
	(*status) = (mad)->common.u.NS.Status; \
}

#define MAD_SET_TRANSACTION_ID(mad, id) {\
	(mad)->common.TransactionID = id; \
}

#define MAD_GET_TRANSACTION_ID(mad, id) {\
	(*id) = (mad)->common.TransactionID; \
}

#define MAD_SET_ATTRIB_ID(mad, id) {\
	(mad)->common.AttributeID = id; \
}

#define MAD_GET_ATTRIB_ID(mad, id) {\
	(*id) = (mad)->common.AttributeID; \
}
	
#define MAD_SET_ATTRIB_MOD(mad, amod) {\
	(mad)->common.AttributeModifier = amod; \
}

#define MAD_GET_ATTRIB_MOD(mad, amod) {\
	(*amod) = (mad)->common.AttributeModifier; \
}
	
#define MAD_SET_METHOD_TYPE(mad, method) {\
	(mad)->common.mr.s.Method = method; \
}

#define MAD_GET_METHOD_TYPE(mad, method) {\
	(*method) = (mad)->common.mr.s.Method; \
}

/*
 * ClassPortInfo
 */

/* CapMask common bits  */
#define CLASS_PORT_CAPMASK_TRAP		0x0001	/* can generate traps */
#define CLASS_PORT_CAPMASK_NOTICE	0x0002	/* implements Get/Set Notice */

/* mask for class specific bits */
#define CLASS_PORT_CAPMASK_CLASS_SPECIFIC	0xff00

typedef struct _ClassPortInfo{
	uint8				BaseVersion;
	uint8				ClassVersion;
	uint16				CapMask;

	union {
		uint32			AsReg32;

		struct { IB_BITFIELD2(uint32,
			CapMask2:		27,
			RespTimeValue:	5)
		} PACK_SUFFIX s;
	} u1;

	/* Define any re-direction. */
	IB_GID				RedirectGID;
	union {
		uint32			AsReg32;
		struct { IB_BITFIELD3(uint32,
			RedirectTClass:	8,
			RedirectSL:		4,
			RedirectFlowLabel:20)
		} PACK_SUFFIX s;
	} u2;
	IB_LID				RedirectLID;
	uint16				Redirect_P_Key;
	union {
		uint32			AsReg32;
		struct { IB_BITFIELD2(uint32,
			Reserved2:		8,
			RedirectQP:		24)
		} PACK_SUFFIX s;
	} u3;
	uint32				Redirect_Q_Key;

	/* Define any refor traps */
	IB_GID				TrapGID;
	union {
		uint32			AsReg32;
		struct { IB_BITFIELD3(uint32,
			TrapTClass:		8,
			TrapSL:			4,
			TrapFlowLabel:	20)
		} PACK_SUFFIX s;
	} u4;
	IB_LID				TrapLID;
	uint16				Trap_P_Key;
	union {
		uint32			AsReg32;
		struct { IB_BITFIELD2(uint32,
			TrapHopLimit:	8,
			TrapQP:			24)
		} PACK_SUFFIX s;
	} u5;
	uint32			Trap_Q_Key;

} PACK_SUFFIX ClassPortInfo, IB_CLASS_PORT_INFO;

/* Notice */

/* Channel adapters, switches, and routers implementing Notice attributes
 * shall conform to the definition specified.
 */

/*
 * Node type info
 */
typedef enum _NODE_TYPE {
	IBA_NODE_CHANNEL_ADAPTER	= 1,
	IBA_NODE_SWITCH			= 2,
	IBA_NODE_ROUTER			= 3,
} NODE_TYPE;


/*
 * Trap/Notice type
 */
typedef enum _NOTICE_TYPE {
	IB_NOTICE_FATAL		= 0,
	IB_NOTICE_URGENT	= 1,
	IB_NOTICE_SECURITY	= 2,
	IB_NOTICE_SM		= 3,
	IB_NOTICE_INFO		= 4,
	IB_NOTICE_ALL		= 0xFFFF
} IB_NOTICE_TYPE;



/*
 * Notice 
 */
typedef struct _NOTICE {

	union {
		/* Generic Notice attributes */
		struct /*_GENERIC*/ {
			union {
				uint32		AsReg32;
				struct { IB_BITFIELD3(uint32,
					IsGeneric:	 1,		/* 1= generic */
					Type	:	 7,		/* IB_NOTICE_TYPE */
					ProducerType:24)	/* NODE_TYPE */
				} PACK_SUFFIX s;
			} u;
			/* If generic, indicates a class-defined trap number.  */
			/* Number 0xFFFF is reserved. */
			uint16	TrapNumber;		
		} PACK_SUFFIX Generic;

		/* Vendor specific Notice attributes */
		struct /*_VENDOR*/ {
			union {
				uint32		AsReg32;
				struct { IB_BITFIELD3(uint32,
					IsGeneric:	 1,
					Type	:	 7,
					VendorID:	24)		/* Vendor OUI */
				} PACK_SUFFIX s;
			} u;
			/* If not generic, this is Device ID information as assigned by  */
			/* device manufacturer. */
			uint16	DeviceID;
		} PACK_SUFFIX Vendor;
	} u;

	/* Common Notice attributes */
	IB_LID	IssuerLID;							/* LID of requester */

	/* Toggle:
	 *	For Notices, alternates between zero and one after each Notice is
	 *	cleared.
	 *	For Traps, this shall be set to 0.
	 *
	 * Count:
	 *	For Notices, indicates the number of notices queued on this channel
	 *	adapter, switch, or router.
	 *	For Traps, this shall be set to all zeroes.
	 */
	struct { IB_BITFIELD2(uint16,
		Toggle	:1,
		Count	:15)
	} PACK_SUFFIX Stats;

	uint8	Data[54];		
	IB_GID	IssuerGID;

} PACK_SUFFIX IB_NOTICE;

/*
 * Structure of the IB_NOTICE Data for the following traps:
 *         SMA_TRAP_GID_NOW_IN_SERVICE
 *         SMA_TRAP_GID_OUT_OF_SERVICE
 *         SMA_TRAP_ADD_MULTICAST_GROUP
 *         SMA_TRAP_DEL_MULTICAST_GROUP
 */
typedef struct _TRAPS_64_65_66_67_DETAILS
{
	uint8  Reserved[6];
	IB_GID GIDAddress;
	uint8  Padding[32];
} PACK_SUFFIX TRAPS_64_65_66_67_DETAILS, TRAP_64_DETAILS, TRAP_65_DETAILS, TRAP_66_DETAILS, TRAP_67_DETAILS;

/*
 * InformInfo
 */
typedef struct _INFORM_INFO {
	IB_GID	GID;				/* specifies specific GID to subscribe for.  */
								/* Set to zero if not desired. */
	IB_LID	LIDRangeBegin;		/* 0xFFFF encapsulates all LIDs */
	IB_LID	LIDRangeEnd;
	uint16	Reserved;
	uint8	IsGeneric;			/* 1 = forward generic traps */
								/* 0 = vendor specific */
	uint8	Subscribe;			/* 1 = subscribe, 0 = unsubscribe */
	
	uint16	Type;				/* IB_NOTICE_TYPE */

	union {
		struct /*_GENERIC_II*/ {
			uint16	TrapNumber;	
			union {
				uint32		AsReg32;
				struct { IB_BITFIELD3(uint32,
							QPNumber:24,
							Reserved:3,
							RespTimeValue:5)
				} PACK_SUFFIX s;
			} u2;
			union {
				uint32		AsReg32;
				struct { IB_BITFIELD2(uint32,
					Reserved:	 8,
					ProducerType:	24)		/* NODE_TYPE */
				} PACK_SUFFIX s;
			} u; 
		} PACK_SUFFIX Generic;

		struct /*_VENDOR_II*/ {
			uint16	DeviceID;
			union {
				uint32		AsReg32;
				struct { IB_BITFIELD3(uint32,
							QPNumber:24,
							Reserved:3,
							RespTimeValue:5)
				} PACK_SUFFIX s;
			} u2;
			union {
				uint32		AsReg32;
				struct { IB_BITFIELD2(uint32,
					Reserved:	 8,
					VendorID:	24)		/* Vendor OUI */
				} PACK_SUFFIX s;
			} u;
		} PACK_SUFFIX Vendor;
	} u;
			
} PACK_SUFFIX IB_INFORM_INFO;
#include "iba/public/ipackoff.h"



/* --------------------------------------------------------------------------
 * Swap between cpu and wire formats
 */
static __inline
void
BSWAP_MAD_HEADER(
	MAD			*Dest
	 )
{
#if CPU_LE
	Dest->common.u.NS.Status.AsReg16 = ntoh16( Dest->common.u.NS.Status.AsReg16);
	Dest->common.TransactionID = ntoh64(Dest->common.TransactionID);
	Dest->common.AttributeID = ntoh16(Dest->common.AttributeID );
	Dest->common.AttributeModifier = ntoh32(Dest->common.AttributeModifier);
#endif
}


static __inline
void
BSWAP_IB_CLASS_PORT_INFO(
	IB_CLASS_PORT_INFO	*Dest
	)
{
#if CPU_LE
	Dest->CapMask = ntoh16(Dest->CapMask);
	Dest->u1.AsReg32 = ntoh32(Dest->u1.AsReg32);
	BSWAP_IB_GID(&Dest->RedirectGID);
	Dest->u2.AsReg32 = ntoh32(Dest->u2.AsReg32);
	Dest->RedirectLID = ntoh16(Dest->RedirectLID);
	Dest->Redirect_P_Key = ntoh16(Dest->Redirect_P_Key);
	Dest->u3.AsReg32 = ntoh32(Dest->u3.AsReg32);
	Dest->Redirect_Q_Key = ntoh32(Dest->Redirect_Q_Key);
	BSWAP_IB_GID(&Dest->TrapGID);
	Dest->u4.AsReg32 = ntoh32(Dest->u4.AsReg32);
	Dest->TrapLID = ntoh16(Dest->TrapLID);
	Dest->Trap_P_Key = ntoh16(Dest->Trap_P_Key);
	Dest->u5.AsReg32 = ntoh32(Dest->u5.AsReg32);
	Dest->Trap_Q_Key = ntoh32(Dest->Trap_Q_Key);
#endif
}

static __inline
void
BSWAP_IB_NOTICE(
	IB_NOTICE			*Dest
	 )
{
#if CPU_LE
	/* Can do Generic since Vendor has same types, just different field names */
	Dest->u.Generic.u.AsReg32 = ntoh32(Dest->u.Generic.u.AsReg32);
	Dest->u.Generic.TrapNumber = ntoh16(Dest->u.Generic.TrapNumber);
	Dest->IssuerLID = ntoh16(Dest->IssuerLID);
	BSWAP_IB_GID(&Dest->IssuerGID);
#endif
}

/*
 * Swaps the GIDAddress in the IB_NOTICE Data
 */
static __inline
void
BSWAP_TRAPS_64_65_66_67_DETAILS(TRAPS_64_65_66_67_DETAILS *Dest)
{
#if CPU_LE
	BSWAP_IB_GID(&Dest->GIDAddress);
#endif
}

#define BSWAP_TRAP_64_DETAILS(Dest) BSWAP_TRAPS_64_65_66_67_DETAILS(Dest)
#define BSWAP_TRAP_65_DETAILS(Dest) BSWAP_TRAPS_64_65_66_67_DETAILS(Dest)
#define BSWAP_TRAP_66_DETAILS(Dest) BSWAP_TRAPS_64_65_66_67_DETAILS(Dest)
#define BSWAP_TRAP_67_DETAILS(Dest) BSWAP_TRAPS_64_65_66_67_DETAILS(Dest)

static __inline
void
BSWAP_INFORM_INFO(
	IB_INFORM_INFO			*Dest
	 )
{
#if CPU_LE
	BSWAP_IB_GID(&Dest->GID);
	Dest->LIDRangeBegin = ntoh16(Dest->LIDRangeBegin);
	Dest->LIDRangeEnd = ntoh16(Dest->LIDRangeEnd);
	Dest->Type = ntoh16(Dest->Type);
	/* Can do Generic since Vendor has same types, just different field names */
	Dest->u.Generic.TrapNumber = ntoh16(Dest->u.Generic.TrapNumber);
	Dest->u.Generic.u2.AsReg32 = ntoh32(Dest->u.Generic.u2.AsReg32);
	Dest->u.Generic.u.AsReg32 = ntoh32(Dest->u.Generic.u.AsReg32);
#endif
}

#if defined (__cplusplus)
};
#endif


#endif /* __IBA_IB_MAD_H__ */

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

** END_ICS_COPYRIGHT2   ****************************************/

/* [ICS VERSION STRING: unknown] */
//---------------------------------------------------------------------------//
//
// Copyright (c) 2001, VIEO Inc.  All rights reserved.
//
//---------------------------------------------------------------------------//

//===========================================================================//
//									
// FILE NAME								
//    sa_m.c								
//									
// DESCRIPTION								
//    This file contains the offsets for doing component mask comparisons.
//									
//===========================================================================//

//
//	Scratch pad for template queries.  They must be used sequentially.
//
uint8_t		template_mask[4096];
uint16_t	template_type;
uint32_t	template_offset;
uint32_t	template_length;
FieldMask_t	*template_fieldp;

FieldMask_t 	StlNodeRecordFieldMask[] = {
	{     0,    32 },	// RID.LID
	{    32,    32 },	// Reserved
	{    64,     8 },	// BaseVersion
	{    72,     8 },	// ClassVersion
	{    80,     8 },	// NodeType
	{    88,     8 },	// NumPorts
	{    96,    32 },	// Reserved
	{   128,    64 },	// SysGuid
	{   196,    64 },	// NodeGuid
	{   256,    64 },	// PortGuid
	{   320,    16 },	// PartCap
	{   336,    16 },	// DeviceID
	{   352,    32 },	// Revision
	{   384,     8 },	// LocalPortNum
	{   392,    24 },	// VendorID
	{   416,   512 },	// Node Description
	{     0,     0 },
};

FieldMask_t		IbNodeRecordFieldMask[] = { 
	{     0,    16 },	// RID.LID
	{    16,    16 },	// Reserved
	{    32,     8 },	// BaseVersion
	{    40,     8 },	// ClassVersion
	{    48,     8 },	// NodeType
	{    56,     8 },	// NumPorts
	{    64,    64 },	// SystemImageGuid
	{   128,    64 },	// NodeGuid
	{   192,    64 },	// PortGuid
	{   256,    16 },	// PartCap
	{   272,    16 },	// DeviceID
	{   288,    32 },	// Revision
	{   320,     8 },	// LocalPortNum
	{   328,    24 },	// VendorID
	{   352,   512 },	// NodeDesc
	{     0,     0 },
};

FieldMask_t	StlPortInfoRecordFieldMask[] = {
	{     0,    32 },	// EndPortLID
	{    32,     8 },	// PortNum
	{    40,     8 },	// Options
	{  1792,    32 },	// CapabilityMask
	{     0,     0 },	// No other fields are searchable at this time.
};


FieldMask_t	IbPortInfoRecordFieldMask[] = {
	{     0,    16 },
    {    16,     8 },
    {    24,     8 },
	{    32,    64 },
	{    96,    64 },
	{   160,    16 },
	{   176,    16 },
	{   192,    32 },
	{   224,    16 },
	{   240,    16 },
	{   256,     8 },
	{   264,     8 },
	{   272,     8 },
	{   280,     8 },
	{   288,     4 },
	{   292,     4 },
	{   296,     4 },
	{   300,     4 },
	{   304,     2 },
	{   306,     3 },
	{   309,     3 },
	{   312,     4 },
	{   316,     4 },
	{   320,     4 },
	{   324,     4 },
	{   328,     4 },
	{   332,     4 },
	{   336,     8 },
	{   344,     8 },
	{   352,     8 },
	{   360,     4 },
	{   364,     4 },
	{   368,     3 },
	{   371,     5 },
	{   376,     4 },
	{   380,     1 },
	{   381,     1 },
	{   382,     1 },
	{   383,     1 },
	{   384,    16 },
	{   400,    16 },
	{   416,    16 },
	{   432,     8 },
	{   440,     3 },
	{   443,     5 },
	{   448,     3 },
	{   451,     5 },
	{   456,     4 },
	{   460,     4 },
    {   464,     16},
    {   480,     8 },
    {   488,     24},
	{     0,     0 },
};

#if 0
// No longer supported.
FieldMask_t	SLVLTableRecordFieldMask[] = {
	{     0,    16 },
	{    16,     8 },
	{    24,     8 },
	{    32,    32 },
	{    64,     4 },
	{    68,     4 },
	{    72,     4 },
	{    76,     4 },
	{    80,     4 },
	{    84,     4 },
	{    88,     4 },
	{    92,     4 },
	{    96,     4 },
	{   100,     4 },
	{   104,     4 },
	{   108,     4 },
	{   112,     4 },
	{   116,     4 },
	{   120,     4 },
	{   124,     4 },
	{     0,     0 },
};
#endif

FieldMask_t	StlSwitchInfoRecordFieldMask[] = {
	{     0,    32 },	// RID LID
	{    32,    32 },	// Reserved
	{    64,    32 },	// LinearFDBCap
	{    96,    32 },	// RandomFDBCap
	{   128,    32 },	// MulticastFDBCap
	{   160,    32 },	// LinearFDBTop
	{   192,    32 },	// MulticastFDBTop
	{   224,    32 },	// CollectiveCap
	{   256,    32 },	// CollectiveTop
	{   288,    32 },	// Reserved
	{   320,   128 },	// IPAddrPrimary
	{   448,   128 },	// IPAddrSecondary
	{   576,     8 },	// DefaultPort
	{   584,     8 },	// Default MC Primary Port
	{   592,     8 },	// Default MC Not Primary Port
	{   600,     8 },	// STL Lifetimevalue
	{   608,    16 },	// LidsPerPort
	{   624,    16 },	// PartitionEnforcementCap
	{   640,    16 },	// Reserved
	{   656,     8 },	// RoutingMode Supported
	{   664,     8 },	// RoutingMode Enabled
	{   672,     8 },	// bitfield
	{   680,     8 },	// MultiCollectMask
	{   688,     8 },	// AdaptiveRouting
	{   696,    16 },	// SwitchCapabilityMask
	{   712,    16 },	// CollectiveMask
	{   000,    00 },
};

FieldMask_t	StlLFTRecordFieldMask[] = {
	{   000,    32 },	// RID LID
	{    32,    14 },	// Reserved
	{    46,    18 },	// BlockNum
	{   000,    00 },
};

FieldMask_t	StlRFTRecordFieldMask[] = {
	{   000,    32 },	// RID LID
	{    32,    11 },	// Reserved
	{    43,    21 },	// BlockNum
	{   000,    00 },
};

FieldMask_t	StlMFTRecordFieldMask[] = {
	{   000,    32 },	// RID LID
	{    32,     2 },	// Position
	{    34,     9 },	// Reserved
	{    43,    21 },	// BlockNum
	{   000,    00 },
};

FieldMask_t	StlSMInfoRecordFieldMask[] = {
	{   000,    32 },	// RID LID
	{    32,    32 },	// Reserved
	{    64,    64 },	// GUID
	{   128,    64 },	// SM_Key
	{   192,    32 },	// ActCount
	{   224,    32 },	// Elapsed Time
	{   228,     4 },	// Priority
	{   232,     4 },	// ElevatedPriority
	{   236,     4 },	// InitialPriority
	{   240,     4 },	// SM State Current
	{   000,    00 },
};

FieldMask_t	StlInformRecordFieldMask[] = {
	{   000,    32 },	// RID LID
	{    32,    16 },	// Enum
	{    48,    16 },	// Reserved
	{    64,   128 },	// GID
	{   192,    32 },	// LID Range Begin
	{   224,    32 },	// LID Range End
	{   256,     8 },	// IsGeneric
	{   264,     8 },	// Subscribe
	{   272,    16 },	// Type
	{   288,    16 },	// Reserved
	{   304,    16 },	// Trap Number
	{   320,    24 },	// Queue Pair Number
	{   344,     3 },	// Reserved
	{   347,     5 },	// RespTimeValue
	{   352,     8 },	// Reserved
	{   360,    24 },	// Producer Type
	{   000,    00 },
};

FieldMask_t	IbInformRecordFieldMask[] = {
	{     0,   128 },	// SubscriberGID
	{   128,    16 },	// Enum
	{   144,    48 },	// Reserved
	{   192,   128 },	// GID
	{   320,    16 },	// LidRangeBegin
	{   336,    16 },	// LidRangeEnd
	{   352,    16 },	// Reserved
	{   368,     8 },	// IsGeneric
	{   376,     8 },	// Subscribe
	{   384,    16 },	// Type
	{   400,    16 },	// Trap # / Dev ID
	{   416,    24 },	// QPN
	{   440,     3 },	// Reserved
	{   443,     5 },	// RespTimeValue
	{   448,     8 },	// Reserved
	{   456,    24 },	// Producer Type / Vendor ID
	{     0,     0 },
};

FieldMask_t	StlLinkRecordFieldMask[] = {
	{   000,    32 },	// From LID
	{    32,     8 },	// From Port
	{    40,     8 },	// To Port
	{    48,    16 },	// Reserved
	{    64,    32 },	// To LID
};

#if 0
// No longer supported.
FieldMask_t	GuidInfoRecordFieldMask[] = {
	{     0,    16 },
	{    16,     8 },
	{    24,     8 },
	{    32,    32 },
	{    64,    64 },
	{   128,    64 },
	{   192,    64 },
	{   256,    64 },
	{   320,    64 },
	{   384,    64 },
	{   448,    64 },
	{   512,    64 },
	{     0,     0 },
};
#endif 

FieldMask_t	StlServiceRecordFieldMask[] = {
	{   000,    64 },	// Service ID
	{    64,    32 },	// Service LID
	{    96,    16 },	// Service PKEY
	{   112,    16 },	// Reserved
	{   128,   128 },	// Service GID
	{   256,    32 },	// Service Lease
	{   288,    32 },	// Reserved
	{   320,   128 },	// Service Key
	{   448,   512 },	// Service Name
	{   960,     8 }, 	// service 8.1
	{   968,     8 }, 	// service 8.2
	{   976,     8 }, 	// service 8.3
	{   984,     8 }, 	// service 8.4
	{   992,     8 }, 	// service 8.5
	{  1000,     8 }, 	// service 8.6
	{  1008,     8 }, 	// service 8.7
	{  1016,     8 }, 	// service 8.8
	{  1024,     8 }, 	// service 8.9
	{  1032,     8 }, 	// service 8.10
	{  1040,     8 }, 	// service 8.11
	{  1048,     8 }, 	// service 8.12
	{  1056,     8 }, 	// service 8.13
	{  1064,     8 }, 	// service 8.14
	{  1072,     8 }, 	// service 8.15
	{  1080,     8 }, 	// service 8.16
	{  1088,    16 }, 	// service 16.1
	{  1104,    16 }, 	// service 16.2
	{  1120,    16 }, 	// service 16.3
	{  1136,    16 }, 	// service 16.4
	{  1152,    16 }, 	// service 16.5
	{  1168,    16 }, 	// service 16.6
	{  1184,    16 }, 	// service 16.7
	{  1200,    16 }, 	// service 16.8
	{  1216,    32 }, 	// service 32.1
	{  1248,    32 }, 	// service 32.2
	{  1280,    32 }, 	// service 32.3
	{  1312,    32 }, 	// service 32.4
	{  1344,    64 }, 	// service 64.1
	{  1408,    64 }, 	// service 64.2
	{   000,    00 },
};

FieldMask_t IbServiceRecordFieldMask[] = {
	{     0,    64 }, 	// Service ID
	{    64,   128 }, 	// Service GID
	{   192,    16 }, 	// Service PKEY
	{   208,    16 }, 	// Reserved
	{   224,    32 }, 	// Service Lease
	{   256,   128 },	// Service Key
	{   384,   512 },	// Service Name
	{   896,     8 }, /*service 8.1 */
	{   904,     8 }, /*service 8.2 */
	{   912,     8 }, /*service 8.3 */
	{   920,     8 }, /*service 8.4 */
	{   928,     8 }, /*service 8.5 */
	{   936,     8 }, /*service 8.6 */
	{   944,     8 }, /*service 8.7 */
	{   952,     8 }, /*service 8.8 */
	{   960,     8 }, /*service 8.9 */
	{   968,     8 }, /*service 8.10 */
	{   976,     8 }, /*service 8.11 */
	{   984,     8 }, /*service 8.12 */
	{   992,     8 }, /*service 8.13 */
	{  1000,     8 }, /*service 8.14 */
	{  1008,     8 }, /*service 8.15 */
	{  1016,     8 }, /*service 8.16 */
	{  1024,    16 }, /*service 16.1 */
	{  1040,    16 }, /*service 16.2 */
	{  1056,    16 }, /*service 16.3 */
	{  1072,    16 }, /*service 16.4 */
	{  1088,    16 }, /*service 16.5 */
	{  1104,    16 }, /*service 16.6 */
	{  1120,    16 }, /*service 16.7 */
	{  1136,    16 }, /*service 16.8 */
	{  1152,    32 }, /*service 32.1 */
	{  1184,    32 }, /*service 32.2 */
	{  1152,    32 }, /*service 32.3 */
	{  1248,    32 }, /*service 32.4 */
	{  1280,    64 }, /*service 64.1 */
	{  1344,    64 }, /*service 64.2 */
	{     0,     0 },
};

FieldMask_t	StlPKeyTableFieldMask[] = {
	{     0,    32 },	// RID LID
	{    32,    16 },	// RID BlockNum
	{    64,     8 },	// RID PortNum
	{    65,     8 },	// Reserved.
	{     0,     0 },	// Can't select individual blocks of pkey table.
};

FieldMask_t	PathRecordFieldMask[] = {
	{     0,    32 },
	{    32,    32 },
	{    64,   128 },
	{   192,   128 },
	{   320,    16 },
	{   336,    16 },
	{   352,     1 },
	{   353,     3 },
	{   356,    20 },
	{   376,     8 },
	{   384,     8 },
	{   392,     1 },
	{   393,     7 },
	{   400,    16 },
	{   416,    12 },
	{   428,     4 },
	{   432,     2 },
	{   434,     6 },
	{   440,     2 },
	{   442,     6 },
	{   448,     2 },
	{   450,     6 },
	{   456,     8 },
	{   464,    48 },
	{     0,     0 },
};

FieldMask_t	StlVLArbTableRecordFieldMask[] = {
	{     0,    32 },	// LID
	{    32,     8 },	// Output Port Number
	{    40,     8 },	// Block Number
	{    48,    16 },	// Reserved
	{     0,     0 },	// Can't select individual blocks of vlarb table.
};

FieldMask_t	StlMcMemberRecordFieldMask[] = {
	{     0,   128 },	// MGID
	{   128,   128 },	// PORTGID
	{   256,    32 },	// QKey
	{   288,    16 },	// Reserved
	{   304,     2 },	// MTU Selector
	{   306,     6 },	// MTU
	{   312,     8 },	// TClass
	{   320,    16 },	// PKey
	{   336,     2 },	// Rate Selector
	{   338,     6 },	// Rate
	{   344,     2 },	// PLT Selector
	{   346,     6 },	// PLT
	{   352,     5 },	// SL
	{   356,    19 },	// Reserved
	{   376,     8 },	// Hop Limit
	{   384,     4 },	// Scope
	{   388,     4 },	// Join State
	{   392,     1 },	// Proxy Join
	{   393,     7 },	// Reserved
	{   400,    16 },	// Reserved
	{   416,    32 },	// MLID
	{     0,     0 },
};


FieldMask_t	IbMcMemberRecordFieldMask[] = {
	{     0,   128 },	// MGID
	{   128,   128 },	// PORTGID
	{   256,    32 },	// QKey
	{   288,    16 },	// MLID
	{   304,     2 },	// MTU Selector
	{   306,     6 },	// MTU
	{   312,     8 },	// TClass
	{   320,    16 },	// PKey
	{   336,     2 },	// Rate Selector
	{   338,     6 },	// Rate
	{   344,     2 },	// PLT Selector
	{   346,     6 },	// PLT
	{   352,     4 },       // SL
	{   356,    20 },       // Flowlabel
	{   376,     8 },	// Hop Limit
	{   384,     4 },	// Scope
	{   388,     4 },	// Join State
	{   392,     1 },	// Proxy Join
	{   393,     7 }, 	// Reserved
	{   400,    16 },	// Reserved
	{     0,     0 },
};

FieldMask_t	StlTraceRecordFieldMask[] = {
	{     0,    16 },	// IDGeneration
	{    16,     8 },	// Reserved
	{    24,     8 },	// NodeType
	{    32,     8 },	// EntryPort
	{    40,     8 },	// ExitPort
	{    48,    16 },	// Reserved
	{    64,    64 },	// NodeID
	{   128,    64 },	// ChassisID
	{   192,    64 },	// EntryPortID
	{   256,    64 },	// ExitPortID
	{   000,    00 },
};

FieldMask_t	StlVfInfoRecordFieldMask[] = {
	{     0,    16 },	// VFIndex
	{    16,    16 },	// PKey
	{    32,    32 },	// Reserved
	{    64,   512 },	// VFName
	{   576,    64 },	// ServiceID
	{   640,   128 },	// MGID
	{   768,     5 },	// SL
	{   773,     3 },	// Select Flags
	{   776,     6 },	// MTU
	{   782,     2 },	// MTU Specif.
	{   784,     6 },	// Rate
	{   790,     2 },	// Rate Specif.
	{   792,     3 },	// PktLifeTimeInc
	{   795,     4 },	// Reserved
	{   799,     1 },       // PktLifeTime Specif.
	{   800,     8 },	// Option flags
	{   808,     8 },	// Bandwidth Percentage
	{   816,     1 },	// Priority
	{   817,     7 },	// Reserved
	{   824,     8 },	// Routing SLs
	{   832,     1 },	// Reserved
	{   833,     7 },	// Preemption Rank
	{   840,     3 },	// Reserved
	{   843,     5 },	// HOQ Life
	{   848,   176 },	// Reserved
	{     0,     0 },
};

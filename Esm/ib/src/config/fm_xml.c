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

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "if3.h"


#include "ib_sm.h"
#include <stl_pa_priv.h>

#ifndef __VXWORKS__
#include <inttypes.h>
#include <syslog.h> // Needed for getFacility / showFacility
#include <fcntl.h>
#endif
#include <ctype.h>
#include <ib_helper.h>
#include <ixml_ib.h>
#include <cs_g.h>
#include <fm_xml.h>
#define _GNU_SOURCE

#include "fm_md5.h"

#ifdef __VXWORKS__
#include "Ism_Idb.h"
#include "ioLib.h"
#include "bspcommon/h/icsBspUtil.h"
#include "bspcommon/h/sysFlash.h"
#include "bspcommon/h/opafm_cfg.h"
#include "config_compression.h"
#include "ib_types.h"
#include "tms/common/rdHelper.h"
#include <xml.h>

#include "sftp.h"
#include "bspcommon/h/configDirName.h"
extern STATUS rm (const char *fileName);

void updateLastScpRetCode(scpFastFabricRetCode_t retCode) {
	Sftp_updateLastScpRetCode(retCode);
}

extern uint32_t sm_state;
extern size_t g_smPoolSize;
extern Status_t initSmMemoryPool(void);
extern Status_t freeSmMemoryPool(void);
extern Status_t vs_pool_delete (Pool_t * handle);
extern void* getEsmXmlParserMemory(uint32_t size, char* info);
extern void freeEsmXmlParserMemory(void *address, uint32_t size, char* info);
#if defined(__VXWORKS__)
extern void idbSmGetManagersToStart(int * pm, int * fe);
extern STATUS cmuRed_cfgIfMasterSyncSlavePEM(char *pem_filename);
extern STATUS cmuRed_cfgIfMasterSyncSlave(char *remote);
#endif

static int debug_log = 0;
#define SCP_LOG( fmt, args... ) \
    do{  \
        if( debug_log ) { \
            sysPrintf( "[%08x] scp: " fmt "\n", taskIdSelf(), ## args ); \
        } \
    } while(0)

uint32_t scpInProcess = 0;
#endif // __VXWORKS__

#ifndef stringize
#define stringize(x) #x
#endif
#ifndef add_quotes
#define add_quotes(x) stringize(x)
#endif

#define PORTS_ALLOC_UNIT 8

#ifndef __VXWORKS__
extern int kmAdvancedFeatureVerification(char* keyString);
#define IFS_FM_CFG_NAME		"/etc/opa-fm/opafm.xml"
#define FM_SSL_SECURITY_DIR "/usr/local/ssl/opafm"
#else
#define FM_SSL_SECURITY_DIR DIR_BASE_NAME
#endif

static FMXmlCompositeConfig_t 	*configp = NULL;
static DGConfig_t			 	*dgp;
static AppConfig_t			 	*app;
static VFConfig_t				*vfp;
static SMXmlConfig_t 			*smp;
static SMMcastDefGrp_t 			*mdgp;
static PmPortGroupXmlConfig_t	*pgp;

static SMMcastDefGrp_t mdgEmpty;

static uint32_t instance;
static uint32_t common;
static uint32_t fm_instance;
static uint32_t end_instance;
static uint32_t full_parse;
static uint32_t embedded_call;
static uint32_t vfInstance;
static uint32_t PmPgInstance;
static uint32_t PmPgMonitorInstance;
static uint32_t appInstance;
static uint32_t groupInstance;
static uint32_t fullMemInstance;
static uint32_t limitedMemInstance;
static uint32_t systemImageInstance;
static uint32_t nodeGuidInstance;
static uint32_t portGuidInstance;
static uint32_t nodeDescInstance;
static uint32_t includedGroupInstance;
static uint32_t serviceIdInstance;
static uint32_t serviceIdRangeInstance;
static uint32_t serviceIdMaskedInstance;
static uint32_t mgidInstance;
static uint32_t mgidRangeInstance;
static uint32_t mgidMaskedInstance;
static uint32_t dgMgidInstance;
static uint32_t dgMgidRangeInstance;
static uint32_t dgMgidMaskedInstance;
static uint32_t includedAppInstance;
static uint32_t defaultGroupInstance;
static uint32_t mlidSharedInstance;
static uint32_t feTrapSubInstance;


static XmlNode_t *last_node_description;
static RegExp_t  *last_reg_expr;
static XmlIncGroup_t *last_included_group;

static uint8_t xml_vf_debug;
static uint8_t xml_sm_debug;
static uint8_t xml_fe_debug;
static uint8_t xml_pm_debug;
static uint8_t xml_parse_debug;

static uint32_t memory_limit;

#ifdef XML_MEMORY
static uint8_t xml_memory_debug = 1;
#else
static uint8_t xml_memory_debug = 0;
#endif

// Forward declared routines.
static VFConfig_t* getVfObject(void);
static DGConfig_t* getGroupObject(void);
static int8_t cloneGroup(DGConfig_t *source, DGConfig_t *dest, int freeObjectBeforeCopy);
static AppConfig_t *dupApplicationObject(AppConfig_t *obj);
static FSTATUS CheckMLIDShareOverlapping(uint32_t fm, FMXmlCompositeConfig_t *config);




// callback functions for allocating/freeing memory
static void* (*get_memory)(uint32_t size, const char* info) = NULL;
static void (*free_memory)(void *address, uint32_t size, const char* info) = NULL;

// memory usage tracking
static uint32_t memory;

// parsing in process
static uint32_t parsingInProcess = 0;

// function declarations - only ones that are required due to function ordering
static void freeGroupObject(DGConfig_t *group, uint8_t full);

#ifndef XML_TEST
void XmlParsePrintError(const char *message)
{
	IB_LOG_ERROR0(Log_StrDup(message));
	fprintf(stderr, "%s\n", message);
}


static void XmlParsePrintWarning(const char *message)
{
	IB_LOG_WARN0(Log_StrDup(message));
	fprintf(stderr, "%s\n", message);
}
#endif

// return buffer that is the uncompressed data from an XML config file
int getXMLConfigData(uint8_t *buffer, uint32_t bufflen, uint32_t *filelen)
{
	uint32_t nextByte = 0;
	uint32_t index = 0;

	if (parsingInProcess || !buffer || !filelen)
		return -1;

	// open the file
#ifdef __VXWORKS__
	if (scpInProcess)
		return -1;
	if(copyFile(IFS_FM_CFG_NAME, IFS_FM_CFG_NAME_UNCOMPRESSED, 1,NULL)) {
		IB_LOG_ERROR("Error decompressing ESM config file! rc:",0x0020);
		return -1;
	}
    FILE *file = fopen( IFS_FM_CFG_NAME_UNCOMPRESSED, "r" );
#else
    FILE *file = fopen( IFS_FM_CFG_NAME, "r" );
#endif

    if (!file) {
		IB_LOG_ERROR("Error opening ESM config file! rc:",0x0020);
		return -1;
	}

    while((nextByte = fgetc(file)) != EOF) {
		buffer[index++] = nextByte;
		if (index >= bufflen) {
			*filelen = 0;
			IB_LOG_ERROR("Buffer overrun geting ESM config file! rc:",0x0020);
			fclose(file);
			return -1;
		}
	}
	*filelen = index;

    fclose(file);
#ifdef __VXWORKS__
	remove( IFS_FM_CFG_NAME_UNCOMPRESSED );
#endif

	return 0;
}

// take a buffer that is uncompressed and replace the XML config file
int putXMLConfigData(uint8_t *buffer, uint32_t filelen)
{
	uint32_t index = 0;

	if (parsingInProcess || !buffer || !filelen)
		return -1;

	// open the file
#ifdef __VXWORKS__
	if (scpInProcess)
		return -1;
	scpInProcess = 1;
	remove( IFS_FM_CFG_NAME_UNCOMPRESSED );
	FILE *file = fopen( IFS_FM_CFG_NAME_UNCOMPRESSED, "w");
	if (!file) {
		IB_LOG_ERROR("Error opening ESM config file! rc:",0x0020);
		scpInProcess = 0;
		return -1;
	}
#else
	remove( IFS_FM_CFG_NAME );
	FILE *file = fopen( IFS_FM_CFG_NAME, "w");
	if (!file) {
		IB_LOG_ERROR("Error opening ESM config file! rc:",0x0020);
		return -1;
	}
#endif
	for (index = 0; index < filelen; index++) {
		fputc(buffer[index], file);
	}
	fclose(file);
#ifdef __VXWORKS__
	if (copyCompressXMLConfigFile( IFS_FM_CFG_NAME_UNCOMPRESSED, IFS_FM_CFG_NAME) == 0) {
		IB_LOG_ERROR("Error compressing ESM config file! rc:",0x0020);
		return -1;
	}
	// will be cleared by copyCompressXMLConfigFile() but clear here anyway for symetry
	scpInProcess = 0;
#endif

	return 0;
}

// get memory for XML parsing etc... will get from pool if there is a registered
// memory function otherwise will malloc
static void* getXmlMemory(uint32_t size, const char* info)
{
	void* address;

	// if we will exceed memory limits then reject
#ifdef __VXWORKS__
	if (memory + size > memory_limit)
		return NULL;
#endif

	// if allocation is from XML parser then do not do memory accounting
	if (strcmp(info, "getParserMemory()") != 0)
		memory += size;

	// if we have a valid callback function then use it
	if (get_memory)
		address = get_memory(size, info);
	else
		address = malloc(size);

	if (address)
		memset(address, UNDEFINED_XML8, size);
	else
		fprintf(stderr, "Failed to allocate %u bytes for XML parser", (unsigned)size);
	return address;
}

// free memory from XML parsing etc... will return to pool if there is a registered
// memory function otherwise will call free
static void freeXmlMemory(void *address, uint32_t size, const char* info)
{
	if (!address) {
#ifdef XML_MEMORY
		if (strcmp(info, "freeParserMemory()") != 0)
			fprintf(stdout, "Memory address invalid in freeXmlMemory()\n");
#endif
		return;
	}

	// if allocation is from XML parser then do not do memory accounting
	if (strcmp(info, "freeParserMemory()") != 0)
		memory -= size;

	// if we have a valid callback function then use it
	if (free_memory) {
		free_memory(address, size, info);
		return;
	}
	free(address);
}

// calculate the max size of an XML and VF configuration
uint32_t xml_compute_pool_size(uint8_t full)
{

	// if full parse consider all FM instances
	if (full)
		return (XML_PARSE_MEMORY_LIMIT * (MAX_INSTANCES + 1));

	// otherwise just a single FM instance
	return XML_PARSE_MEMORY_LIMIT;
}


// Checksum code

static void *cksumBegin(void)
{
	ctx_t *ctx;

	ctx = getXmlMemory(sizeof(ctx_t), "ctx_t cksumBegin()");
	if (!ctx) return NULL;

	fm_md5_start((void *)ctx);

	return (void *)ctx;
}

static void cksumData(void *ctx, void *block, uint32_t length)
{
	if (!ctx || !block || !length) return;

	// MD5 checksum
	fm_md5_update((void *)ctx, block, length);
}

static uint32_t cksumEnd(void *ctx)
{
	uint8_t computedMd5[16];
	uint32_t sum = 0;
	uint32_t *word;
	uint32_t i;

	if (!ctx) return 0;


	// MD5 checksum
	fm_md5_finish((void *)ctx, computedMd5);

	word = (uint32_t*)computedMd5;
	for (i = 0; i < 4; i++) {
		sum += word[i];
	}
	freeXmlMemory(ctx, sizeof(cl_map_item_t), "cl_map_item_t insertNodeDesc()");
	return sum;
}

#define CKSUM_MAX		3
#define CKSUM_OVERALL	(1<<0)
#define CKSUM_DISRUPT	(1<<1)
#define CKSUM_CONSIST	(1<<2)
#define CKSUM_OVERALL_DISRUPT	(CKSUM_OVERALL|CKSUM_DISRUPT)
#define CKSUM_OVERALL_CONSIST	(CKSUM_OVERALL|CKSUM_CONSIST)
#define CKSUM_OVERALL_DISRUPT_CONSIST	(CKSUM_OVERALL|CKSUM_DISRUPT|CKSUM_CONSIST)

#define CKSUM_STR(string, flags)	AddToCksums(#string, string, strlen(string), flags)
#define CKSUM_DATA(data, flags)	AddToCksums(#data, &data, sizeof(data), flags)
#ifndef __VXWORKS__
#define CKSUM_FILE(file, flags) do{                                                             \
                                    int fd = open(file, O_RDONLY);                              \
                                    int filein, len;                                            \
                                    if (fd >= 0) {                                              \
                                        while((len = read(fd, &filein, sizeof(int))) > 0)       \
                                            AddToCksums(#file, &filein, len, flags);            \
                                        close(fd);                                              \
                                    }                                                           \
                                } while(0);
#else
#define CKSUM_FILE(file, flags)
#endif

#ifdef CHECKSUM_DEBUG
#define CKSUM_BEGIN { printf("CKSUM_BEGIN %s\n",__func__); BeginCksums(); }
#else
#define CKSUM_BEGIN { BeginCksums(); }
#endif

/*
 * Generic 'DEFAULT' macro for all integer types.
 */
#define DEFAULT_INT(data, default) { \
	/* Do 'typeof((data))' to cast ~0 to the right size */ \
	if ( data == ((typeof((data)))~0) ) \
		data = default; \
}

#define DEFAULT_AND_CKSUM_INT(data,default,flags) { \
	DEFAULT_INT(data,default); \
	AddToCksums(#data,&data,sizeof(data),flags); }

#define DEFAULT_STR(data, default) { if (!strlen(data)) strncpy(data,default,sizeof(data)); }
#define DEFAULT_AND_CKSUM_STR(data, default, flags) { DEFAULT_STR(data, default); AddToCksums(#data, &data, strlen(data), flags); }

#ifdef CHECKSUM_DEBUG
#define CKSUM_END(overall, disruptive, consistency) { EndCksums(&overall, &disruptive, &consistency); printf("CKSUM_END %s %d, %d, %d\n",__func__,overall,disruptive,consistency); }
#else
#define CKSUM_END(overall, disruptive, consistency) { EndCksums(&overall, &disruptive, &consistency); }
#endif


static ctx_t *cksum_ctx[CKSUM_MAX];

static boolean BeginCksums(void)
{
	uint32_t i;

	for (i = 0; i < CKSUM_MAX; i++) {
		if (!(cksum_ctx[i] = cksumBegin())) {
			while(i > 0) {
				cksumEnd(cksum_ctx[--i]);
			}
			return 0;
		}
	}
	return 1;
}

static void AddToCksums(char *txt, void *block, uint32_t length, uint32_t flags)
{
	uint32_t i;

#ifdef CHECKSUM_DEBUG
	printf("AddToCksums %s:",txt);
	for (i=0; i < length; i++)
		printf("%02x",((unsigned char *)block)[i]);
	printf("\n");
#endif
	for (i = 0; i < CKSUM_MAX; i++) {
		if (flags & (1 << i)) {
			cksumData(cksum_ctx[i], block, length);
		}
	}
}

static void EndCksums(uint32_t *overall_checksum, uint32_t *disruptive_checksum, uint32_t *consistency_checksum)
{
	uint32_t csum;

	csum = cksumEnd(cksum_ctx[0]);
	if (overall_checksum)
		*overall_checksum = csum;
	csum = cksumEnd(cksum_ctx[1]);
	if (disruptive_checksum)
		*disruptive_checksum = csum;
	csum = cksumEnd(cksum_ctx[2]);
	if (consistency_checksum)
		*consistency_checksum = csum;
}

// Map handling routines
static boolean addMap(cl_qmap_t *map, uint64_t key)
{
	cl_map_item_t	*cl_map_item;

	cl_map_item = getXmlMemory(sizeof(cl_map_item_t), "cl_map_item_t addMap()");
	if (!cl_map_item) {
		fprintf(stdout, OUT_OF_MEMORY_RETURN);
		return 0;
	}
	if (cl_qmap_insert(map, key, cl_map_item) != cl_map_item) {
		// Duplicate
		freeXmlMemory(cl_map_item, sizeof(cl_map_item_t), "cl_map_item_t addMap()");
		return 0;
	}
	return 1;
}

static uint32_t cloneMap(cl_qmap_t *dst, cl_qmap_t *src, void *compRoutine, void *dupRoutine)
{
	void *(*dupRoutineCall)(void *) = dupRoutine;
	cl_map_item_t 	*cl_map_item;
	char			*obj;
	uint32_t		cnt = 0;

	cl_qmap_init(dst, compRoutine);
	for_all_qmap_item(src, cl_map_item) {
		if (dupRoutine) {
			obj = dupRoutineCall(XML_QMAP_CHAR_CAST cl_qmap_key(cl_map_item));
			if (obj) cnt += addMap(dst, XML_QMAP_U64_CAST obj);
		} else {
			cnt += addMap(dst, cl_qmap_key(cl_map_item));
		}
	}
	return cnt;
}

static void scrubMap(cl_qmap_t *map, void *freeRoutine)
{
	void (*freeRoutineCall)(void *) = freeRoutine;
	cl_map_item_t 	*cl_map_item;

	for (cl_map_item = cl_qmap_head(map);
		cl_map_item != cl_qmap_end(map);
		cl_map_item = cl_qmap_head(map)) {
			if (freeRoutine)
				freeRoutineCall(XML_QMAP_CHAR_CAST cl_qmap_key(cl_map_item));
			cl_qmap_remove_item(map, cl_map_item);
			freeXmlMemory(cl_map_item, sizeof(cl_map_item_t), "cl_map_item_t scrubMap()");
	}
}

int compareName(IN const uint64 name1, IN  const uint64 name2)
{
	if (!name1 || !name2)
		return 0;

	return strncmp(XML_QMAP_VOID_CAST name1, XML_QMAP_VOID_CAST name2, MAX_VFABRIC_NAME);
}

static void *getName(void)
{
	char *name;

	name = getXmlMemory(MAX_VFABRIC_NAME, "MAX_VFABRIC_NAME dupName()");
	if (!name) {
		fprintf(stdout, OUT_OF_MEMORY_RETURN);
		return NULL;
	}
	return name;
}

static void *dupName(void *name)
{
	char *new_name;

	new_name = getName();
	if (!new_name) {
		fprintf(stdout, OUT_OF_MEMORY_RETURN);
		return NULL;
	}
	strncpy(new_name, name, MAX_VFABRIC_NAME);
	return new_name;
}

static void freeName(void *name)
{
	if (!name) return;
	freeXmlMemory(name, MAX_VFABRIC_NAME, "MAX_VFABRIC_NAME freeName()");
}

// To be equal, the entire VFAppSid_t must match
static int compareAppSid(IN const uint64 sid1, IN  const uint64 sid2)
{
	if (!sid1 || !sid2)
		return 0;

	return memcmp(XML_QMAP_VOID_CAST sid1, XML_QMAP_VOID_CAST sid2, sizeof(VFAppSid_t));
}

static VFAppSid_t *getAppSid(void)
{
	VFAppSid_t *sid;

	sid = getXmlMemory(sizeof(VFAppSid_t), "VFAppSid_t getAppSid()");
	if (sid) memset(sid, 0, sizeof(VFAppSid_t));
	return sid;
}

static void freeAppSid(void *sid)
{
	if (!sid) return;
	freeXmlMemory(sid, sizeof(VFAppSid_t), "VFAppSid_t freeAppSid()");
}

// To be equal, the entire VFAppMgid_t must match
static int compareAppMgid(IN const uint64 mgid1, IN  const uint64 mgid2)
{
	if (!mgid1 || !mgid2)
		return 0;

	return memcmp(XML_QMAP_VOID_CAST mgid1, XML_QMAP_VOID_CAST mgid2, sizeof(VFAppMgid_t));
}

static VFAppMgid_t *getAppMgid(void)
{
	VFAppMgid_t *mgid;

	mgid = getXmlMemory(sizeof(VFAppMgid_t), "VFAppMgid_t getAppMgid()");
	if (mgid) memset(mgid, 0, sizeof(VFAppMgid_t));
	return mgid;
}

static void freeAppMgid(void *mgid)
{
	if (!mgid) return;
	freeXmlMemory(mgid, sizeof(VFAppMgid_t), "VFAppMgid_t freeAppMgid()");
}

// Allocate default Multicast Group object
static VFDg_t *getDMCG(void)
{
	VFDg_t *dmcg;

	dmcg = getXmlMemory(sizeof(VFDg_t), "VFDg_t getDMCG()");
	if (dmcg) {
		memset(dmcg, 0, sizeof(VFDg_t));
		cl_qmap_init(&dmcg->mgidMap, compareAppMgid);
	}
	return dmcg;
}

static void freeDMCG(void *ctx)
{
	VFDg_t *dmcg = ctx;

	if (!dmcg) return;
	scrubMap(&dmcg->mgidMap, freeAppMgid);
	freeXmlMemory(dmcg, sizeof(VFAppSid_t), "VFAppSid_t freeAppSid()");
}

static void freeAppConfigMap(AppXmlConfig_t *app_config)
{
	void freeApplicationObject(AppConfig_t *app);

	scrubMap(&app_config->appMap, freeApplicationObject);
    app_config->appMapSize = 0;
}

// For invalid options
static void InvalidOptionParserEnd(IXmlParserState_t *state, const IXML_FIELD *field,
	void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	IXmlParserPrintWarning(state, "Tag has no affect in this context.", field->tag);
}

// parse U32 and indicate field was encountered
// this can be useful when there are two ways to specify the same capability
// and it must be known which way was encountered
// LogMask vs logLevel is one such good example
void ParamU32XmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint32_t value;

	if (IXmlParseUint32(state, content, len, &value)) {
		FmParamU32_t *p = (FmParamU32_t *)IXmlParserGetField(field, object);
		p->value = value;
		p->valid = 1;
	}
}

// Hfi must be converted from 1 relative in Xml to 0 relative in config
void HfiXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint32_t value;

	if (xml_parse_debug)
		fprintf(stdout, "DecrementU32XmlParserEnd %s instance %u common %u\n", field->tag, (unsigned int)instance, (unsigned int)common);

	if (IXmlParseUint32(state, content, len, &value)) {
		if (value == 0) {
			IXmlParserPrintError(state, "Invalid %s tag value, cannot be zero", field->tag);
			return;
		}
		uint32_t *p = (uint32_t *)IXmlParserGetField(field, object);
		*p = value - 1;
	}
}

// Handle U32 percentage value (0-100) without trailing % symbol
void PercentageXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint32_t value;

	if (xml_parse_debug)
		fprintf(stdout, "PercentageXmlParserEnd %s instance %u common %u\n", field->tag, (unsigned int)instance, (unsigned int)common);

	if (IXmlParseUint32(state, content, len, &value)) {
		if (value > 100) {
			IXmlParserPrintError(state, "Invalid %s tag value, must be 0-100", field->tag);
			return;
		}
		uint32_t *p = (uint32_t *)IXmlParserGetField(field, object);
		*p = value;
	}
}

void DedicatedVLMemMultiXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint32_t value;

	if (xml_parse_debug)
		fprintf(stdout, "DedicatedVLMemMultiXmlParserEnd %s instance %u common %u\n", field->tag, (unsigned int)instance, (unsigned int)common);

	if (IXmlParseUint32(state, content, len, &value)) {
		if (value > 255) {
			IXmlParserPrintError(state, "Invalid %s tag value, must be 0-255", field->tag);
			return;
		}
		uint32_t *p = (uint32_t *)IXmlParserGetField(field, object);
		*p = value;
	}
}

// Validate pkey
void PKeyParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint32_t *p = (uint32_t *)IXmlParserGetField(field, object);
	uint32_t pkey;
	char *sym;

	if (xml_parse_debug)
		fprintf(stdout, "PKeyParserEnd %s instance %u common %u\n", field->tag, (unsigned int)instance, (unsigned int)common);

	if (!valid) {
		fprintf(stderr, "Error processing XML %s tag\n",field->tag);
		return;
	}

	if (!content) {
		IXmlParserPrintError(state, "Invalid %s tag value, cannot be empty", field->tag);
		return;
	}

	pkey = strtol(content, &sym, 16);
	if (*sym) {
		IXmlParserPrintError(state, "Invalid %s tag value, must be a hexadecimal value", field->tag);
	}

	if (pkey == 0 || pkey > 0x7fff) {
		IXmlParserPrintError(state, "Invalid %s tag value, must be in the range 0x0001-0x7fff\n", field->tag);
		return;
	}
	*p = pkey;
}

// Validate 8B pkey
void PKey8BParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint32_t *p = (uint32_t *)IXmlParserGetField(field, object);
	uint32_t pkey;
	char *sym;

	if (xml_parse_debug)
		fprintf(stdout, "PKey8BParserEnd %s instance %u common %u\n", field->tag, (unsigned int)instance, (unsigned int)common);

	if (!valid) {
		fprintf(stderr, "Error processing XML %s tag\n",field->tag);
		return;
	}

	if (!content) {
		IXmlParserPrintError(state, "Invalid %s tag value, cannot be empty", field->tag);
		return;
	}

	pkey = strtol(content, &sym, 16);
	if (*sym) {
		IXmlParserPrintError(state, "Invalid %s tag value, must be a hexadecimal value", field->tag);
	}

	if (pkey != 0x0) {
		if (pkey > 0x7FFE) {
			IXmlParserPrintError(state, "Invalid %s tag value, must be in the range 0x0001-0x7ffe, or 0x0\n", field->tag);
			return;
		}
		// SysAdmin specifies a limited member pkey.
		// Make the pkey a full member before programming any ports.
		pkey |= 0x8000;
	}
	*p = pkey;
}

// Validate 10B pkey
void PKey10BParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint32_t *p = (uint32_t *)IXmlParserGetField(field, object);
	uint32_t pkey;
	char *sym;

	if (xml_parse_debug)
		fprintf(stdout, "PKey10BParserEnd %s instance %u common %u\n", field->tag, (unsigned int)instance, (unsigned int)common);

	if (!valid) {
		fprintf(stderr, "Error processing XML %s tag\n",field->tag);
		return;
	}

	if (!content) {
		IXmlParserPrintError(state, "Invalid %s tag value, cannot be empty", field->tag);
		return;
	}

	pkey = strtol(content, &sym, 16);
	if (*sym) {
		IXmlParserPrintError(state, "Invalid %s tag value, must be a hexadecimal value", field->tag);
	}

	if (pkey != 0x0) {
		if (pkey > 0x7FFF) {
			IXmlParserPrintError(state, "Invalid %s tag value, must be in the range 0x0001-0x7fff to enable, or 0x0 to disable\n", field->tag);
			return;
		}

		// Clear the low order 4-bits to determine the base value.
		pkey &= 0x7FF0;

		// SysAdmin specifies a limited member pkey.
		// Make the pkey a full member before programming any ports.
		pkey |= 0x8000;
	}
	*p = pkey;
}


void MinSupportedVLsParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	IXmlParserPrintWarning(state, "%s tag is deprecated. Minimum number of required VLs is calculated from configuration.", field->tag);
}




// Handle U8 percentage value (0-100) with trailing % symbol
void PercentU8XmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint8_t *p = (uint8_t *)IXmlParserGetField(field, object);
	uint32_t percent;
	char *sym;

	if (xml_parse_debug)
		fprintf(stdout, "PercentU8XmlParserEnd %s instance %u common %u\n", field->tag, (unsigned int)instance, (unsigned int)common);

	if (!valid) {
		fprintf(stderr, "Error processing XML %s tag\n",field->tag);
		return;
	}
	if (!content) {
		IXmlParserPrintError(state, "Invalid %s tag value, cannot be empty", field->tag);
		return;
	}

	percent = strtol(content, &sym, 10);
	if (*sym != '%') {
		IXmlParserPrintError(state, "Invalid %s tag value, must be in %% example: 100%%", field->tag);
		return;
	} else {
		if (percent > 100) {
			IXmlParserPrintError(state, "Invalid %s tag value, must be 0-100%%", field->tag);
			return;
		}
		*p = percent;
	}
}

void BasisU8XmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint8_t *p = (uint8_t *)IXmlParserGetField(field, object);

	if (xml_parse_debug)
		fprintf(stdout, "BasisU8XmlParserEnd %s instance %u common %u\n", field->tag, (unsigned int)instance, (unsigned int)common);

	if (!valid) {
		fprintf(stderr, "Error processing XML %s tag\n",field->tag);
		return;
	}
	if (!content) {
		IXmlParserPrintError(state, "Invald %s tag value, cannot be empty", field->tag);
		return;
    }

    // congestion controls
    if (strcasecmp(content, "qp") == 0) {
        *p = 0;
    } else if (strcasecmp(content, "sl") == 0) {
        *p = 1;
    } else {
		IXmlParserPrintError(state, "Invalid %s tag value, must be either qp or sl", field->tag);
		return;
    }
}

static void LidStrategyXmlParserEnd(IXmlParserState_t *state,
	const IXML_FIELD *field, void *object, void *parent,
	XML_Char *content, unsigned int len, boolean valid)
{
	uint32_t *p = (uint32_t *)IXmlParserGetField(field, object);

	if (!valid) {
		fprintf(stderr, "Error processing XML %s tag\n",field->tag);
		return;
	}
	if (!content) {
		IXmlParserPrintError(state, "Invalid %s tag value, cannot be empty", field->tag);
		return;
	}

	if (!strcasecmp(content, "serial"))
		*p = LID_STRATEGY_SERIAL;
	else if (!strcasecmp(content, "topology"))
		*p = LID_STRATEGY_TOPOLOGY;
	else {
		IXmlParserPrintError(state, "LidStrategy must be one of "
			"(serial,topology)"
		);
		return;
	}
}

void MtuU8XmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint8_t *p = (uint8_t *)IXmlParserGetField(field, object);

	if (xml_parse_debug)
		fprintf(stdout, "MtuU8XmlParserEnd %s instance %u common %u\n", field->tag, (unsigned int)instance, (unsigned int)common);

	if (!valid) {
		fprintf(stderr, "Error processing XML %s tag\n",field->tag);
		return;
	}
	if (!content) {
		IXmlParserPrintError(state, "Invalid %s tag value, cannot be empty", field->tag);
		return;
	}

	/* we restrict vFabric and Multicast group MTU to >= 2K MTU.  This avoids
	 * potential issues with support for 1500 byte UDP payloads as well
	 * as ensuring all vFabrics are capable of sending 2K MADs
	 */
	if (!strcasecmp(content, "Unlimited")) *p = UNDEFINED_XML8;
	else if (!strcasecmp(content, "2048")) *p = GetMtuFromBytes(2048);
	else if (!strcasecmp(content, "4096")) *p = GetMtuFromBytes(4096);
	else if (!strcasecmp(content, "8192")) *p = GetMtuFromBytes(8192);
	else if (!strcasecmp(content, "10240")) *p = GetMtuFromBytes(10240);
	else {
			IXmlParserPrintError(state, "%s must be (2048, 4096, 8192, 10240, or Unlimited)",
				field->tag);
			return;
	}
}

void RateU8XmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint8_t *p = (uint8_t *)IXmlParserGetField(field, object);

	if (xml_parse_debug)
		fprintf(stdout, "RateU8XmlParserEnd %s instance %u common %u\n", field->tag, (unsigned int)instance, (unsigned int)common);

	if (!valid) {
		fprintf(stderr, "Error processing XML %s tag\n",field->tag);
		return;
	}

	if (!content) {
		IXmlParserPrintError(state, "Invalid %s tag value, cannot be empty", field->tag);
		return;
	}

	if (strcasecmp(content, "Unlimited") == 0) {
		*p = UNDEFINED_XML8;
	} else {
		// convert to int
		IXmlParseRateMult_Str(state, content, p);
	}
}

static int checkVFSID(VF_t *vf, uint64_t serviceId)
{
	cl_map_item_t *cl_map_item;
	VFAppSid_t *a;

	for_all_qmap_ptr(&vf->apps.sidMap, cl_map_item, a) {
		if ((!a->service_id_last &&
			(serviceId == a->service_id)) ||
			((a->service_id <= serviceId) &&
			(a->service_id_last >= serviceId)) ||
			((serviceId & a->service_id_mask) ==
			(a->service_id & a->service_id_mask)))
				return TRUE;
	}
	return FALSE;
}

// fill in log_masks.  log_level and syslog_mode decide the value to
// be used for any unspecified log_masks
static void set_log_masks(uint32_t log_level, uint32_t syslog_mode, FmParamU32_t log_masks[VIEO_LAST_MOD_ID+1])
{
	uint32_t modid;
	uint32_t new_log_masks[VIEO_LAST_MOD_ID+1];

	cs_log_set_log_masks(log_level, syslog_mode, new_log_masks);
	for (modid=0; modid <= VIEO_LAST_MOD_ID; ++modid) {
		// test for UNDEFINED_XML8 just in case
		if (! log_masks[modid].valid || log_masks[modid].valid == UNDEFINED_XML8) {
			log_masks[modid].value = new_log_masks[modid];
		} else {
			// set VS_LOG_FATAL just to be safe, no use dying in silence
			log_masks[modid].value |= VS_LOG_FATAL;
		}
	}
}

boolean isValidMtuSetting(VFConfig_t* vfp) {
	int numApplications = vfp->number_of_applications;
	int appIdx;
	boolean isValid = TRUE;
	for (appIdx = 0; appIdx < numApplications; appIdx++) {
		char* appName = &vfp->application[appIdx].application[0];
		if ( (strcasecmp(appName, "SA") == 0) ||
			 (strcasecmp(appName, "PA") == 0) ||
			 (strcasecmp(appName, "PM") == 0)


			 ) {

			if (vfp->max_mtu_int < IB_MTU_2048) {
				isValid = FALSE;
				break;
			}
		}
	}
	return isValid;
}

// Clear FM config
void fmClearConfig(FMXmlConfig_t *fmp)
{
	if (!fmp)
		return;

	memset(fmp->CoreDumpLimit, 0, sizeof(fmp->CoreDumpLimit));
	memset(fmp->CoreDumpDir, 0, sizeof(fmp->CoreDumpDir));
	memset(fmp->log_file, 0, sizeof(fmp->log_file));
	memset(fmp->fm_name, 0, sizeof(fmp->fm_name));
	memset(fmp->syslog_facility, 0, sizeof(fmp->syslog_facility));
	memset(fmp->log_masks, 0, sizeof(fmp->log_masks));
	memset(fmp->SslSecurityDir, 0, sizeof(fmp->SslSecurityDir));
	memset(fmp->SslSecurityFmCertificate, 0, sizeof(fmp->SslSecurityFmCertificate));
	memset(fmp->SslSecurityFmPrivateKey, 0, sizeof(fmp->SslSecurityFmPrivateKey));
	memset(fmp->SslSecurityFmCaCertificate, 0, sizeof(fmp->SslSecurityFmCaCertificate));
	memset(fmp->SslSecurityFmDHParameters, 0, sizeof(fmp->SslSecurityFmDHParameters));
	memset(fmp->SslSecurityFmCaCRL, 0, sizeof(fmp->SslSecurityFmCaCRL));
}

// initialize FM defaults
boolean fmInitConfig(FMXmlConfig_t *fmp, uint32_t instance)
{
	if (!fmp)
		return 0;

	// FM values are not checksummed.
	DEFAULT_INT(fmp->start, 0); // Instance defaults to disabled
	DEFAULT_INT(fmp->hca, 0);
	DEFAULT_INT(fmp->port, 1);
	DEFAULT_INT(fmp->startup_retries, 5);
	DEFAULT_INT(fmp->startup_stable_wait, 10);
	DEFAULT_INT(fmp->subnet_size, DEFAULT_SUBNET_SIZE);
	if (fmp->subnet_size > MAX_SUBNET_SIZE) {
        IB_LOG_INFO_FMT(__func__, "FM subnet size is being adjusted from %u to %u", fmp->subnet_size, MAX_SUBNET_SIZE);
        fmp->subnet_size = MAX_SUBNET_SIZE;
    }
    if (fmp->subnet_size < MIN_SUPPORTED_ENDPORTS) {
        IB_LOG_WARN_FMT(__func__, "FM subnet size of %d is too small, setting to %d", fmp->subnet_size, MIN_SUPPORTED_ENDPORTS);
        fmp->subnet_size = MIN_SUPPORTED_ENDPORTS;
    }

	DEFAULT_INT(fmp->debug, 0);
	DEFAULT_INT(fmp->debug_rmpp, 0);
	DEFAULT_INT(fmp->priority, 0);
	DEFAULT_INT(fmp->elevated_priority, 0);
	DEFAULT_INT(fmp->log_level, 1);
	DEFAULT_INT(fmp->syslog_mode, 0);
	// after parsing is done, fill in unspecified log_masks based on log_level
	set_log_masks(fmp->log_level, fmp->syslog_mode, fmp->log_masks);
	DEFAULT_INT(fmp->config_consistency_check_level, DEFAULT_CCC_LEVEL);
	DEFAULT_INT(fmp->subnet_prefix, 0);
	DEFAULT_INT(fmp->port_guid, 0);
	DEFAULT_STR(fmp->CoreDumpLimit, "0");
	DEFAULT_STR(fmp->CoreDumpDir, "/var/crash/opafm");
	DEFAULT_STR(fmp->syslog_facility, "local6");
    // FIXME: cjking - Temporary patch, default values for OpenSSL Security Support are to be determined
	DEFAULT_INT(fmp->SslSecurityEnabled, 0);
	DEFAULT_STR(fmp->SslSecurityDir, FM_SSL_SECURITY_DIR);
	DEFAULT_STR(fmp->SslSecurityFmCertificate, "fm_cert.pem");
	DEFAULT_STR(fmp->SslSecurityFmPrivateKey, "fm_key.pem");
	DEFAULT_STR(fmp->SslSecurityFmCaCertificate, "fm_ca_cert.pem");
	DEFAULT_INT(fmp->SslSecurityFmCertChainDepth, 1);
	DEFAULT_STR(fmp->SslSecurityFmDHParameters, "fm_dh_parms.pem");
	DEFAULT_INT(fmp->SslSecurityFmCaCRLEnabled, 0);
	DEFAULT_STR(fmp->SslSecurityFmCaCRL, "fm_ca_crl.pem");
	return 1;
}

// Clear PM config
void pmClearConfig(PMXmlConfig_t *pmp)
{
	if (!pmp)
		return;

	memset(pmp->log_file, 0, sizeof(pmp->log_file));
	memset(pmp->syslog_facility, 0, sizeof(pmp->syslog_facility));
	memset(pmp->log_masks, 0, sizeof(pmp->log_masks));
	memset(pmp->name, 0, sizeof(pmp->name));
	memset(pmp->shortTermHistory.StorageLocation, 0, sizeof(pmp->shortTermHistory.StorageLocation));

	pmp->number_of_pm_groups = 0;
	memset(pmp->pm_portgroups, 0, sizeof(pmp->pm_portgroups));

	pmp->overall_checksum = pmp->disruptive_checksum = pmp->consistency_checksum = 0;
}

// initialize PM defaults
boolean pmInitConfig(PMXmlConfig_t *pmp, uint32_t instance)
{
	int i;

#ifdef __VXWORKS__
	int startPM=1;
	int startFE=1;
#endif

	if (!pmp)
		return 0;

#ifdef __VXWORKS__
	idbSmGetManagersToStart(&startPM, &startFE);
	if (!startPM)
		pmp->start = 0;
#endif

	DEFAULT_INT(pmp->start, 1); // PM defaults to enabled
	if (!pmp->start) { // if Pm is diabled, set the checksums to 0 and return
		pmp->overall_checksum = 0;
		pmp->disruptive_checksum = 0;
		pmp->consistency_checksum = 0;
		return 1;
	}

	CKSUM_BEGIN;

	// if the pm is enabled, do CKSUM checks
	CKSUM_DATA(pmp->start, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(pmp->hca, 0, CKSUM_OVERALL_DISRUPT);
	DEFAULT_AND_CKSUM_INT(pmp->port, 1, CKSUM_OVERALL_DISRUPT);
	DEFAULT_AND_CKSUM_INT(pmp->port_guid, 0, CKSUM_OVERALL_DISRUPT);

	// Currently, dynamic changes to pm log_level are supported.
	DEFAULT_AND_CKSUM_INT(pmp->log_level, 1, CKSUM_OVERALL);
	// Dynamic changes to syslog_mode are not (yet) supported.
	DEFAULT_AND_CKSUM_INT(pmp->syslog_mode, 0, CKSUM_OVERALL_DISRUPT);
	// Dynamic manual changes to log_masks are not (yet) supported.
	// Therefore, add the log_masks to the overall and disruptive
	// checksums BEFORE updating them based on defaults,
	// log_level, and syslog_mode.
	CKSUM_DATA(pmp->log_masks, CKSUM_OVERALL_DISRUPT);
	// after parsing is done, fill in unspecified log_masks based on log_level
	set_log_masks(pmp->log_level, pmp->syslog_mode, pmp->log_masks);
	DEFAULT_AND_CKSUM_INT(pmp->config_consistency_check_level, DEFAULT_CCC_LEVEL, CKSUM_OVERALL_DISRUPT_CONSIST);

	DEFAULT_AND_CKSUM_INT(pmp->priority, 0, CKSUM_OVERALL_DISRUPT);
	DEFAULT_AND_CKSUM_INT(pmp->elevated_priority, 0, CKSUM_OVERALL_DISRUPT);
	DEFAULT_AND_CKSUM_INT(pmp->timer, 60, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(pmp->sweep_interval, 10, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(pmp->ErrorClear, 7, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(pmp->ClearDataXfer, 0, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(pmp->Clear64bit, 0, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(pmp->Clear32bit, 1, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(pmp->Clear8bit, 1, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(pmp->process_hfi_counters, 1, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(pmp->process_vl_counters, 1, CKSUM_OVERALL_DISRUPT_CONSIST);

	DEFAULT_AND_CKSUM_INT(pmp->MaxRetries, PM_DEFAULT_MAX_ATTEMPTS, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(pmp->RcvWaitInterval, PM_DEFAULT_RESP_TIMEOUT, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(pmp->MinRcvWaitInterval, PM_DEFAULT_MIN_RESP_TIMEOUT, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(pmp->SweepErrorsLogThreshold, PM_DEFAULT_SWEEP_ERRORS_LOG_THRESHOLD, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(pmp->MaxParallelNodes, PM_DEFAULT_MAX_PARALLEL_NODES, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(pmp->PmaBatchSize, PM_DEFAULT_PMA_BATCH_SIZE, CKSUM_OVERALL_DISRUPT_CONSIST);

	DEFAULT_AND_CKSUM_INT(pmp->freeze_frame_lease, PM_DEFAULT_FF_LEASE, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(pmp->max_clients, PM_DEFAULT_PA_MAX_CLIENTS, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(pmp->freeze_frame_images, PM_DEFAULT_FF_IMAGES, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(pmp->total_images, MAX(pmp->freeze_frame_images + 2, PM_DEFAULT_TOTAL_IMAGES), CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_INT(pmp->image_update_interval, pmp->sweep_interval < 2 ? (pmp->sweep_interval + 1) / 2 : pmp->sweep_interval / 2);

	DEFAULT_AND_CKSUM_INT(pmp->thresholds.Integrity, 100, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(pmp->thresholds.Congestion, 100, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(pmp->thresholds.SmaCongestion, 100, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(pmp->thresholds.Bubble, 100, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(pmp->thresholds.Security, 10, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(pmp->thresholds.Routing, 100, CKSUM_OVERALL_DISRUPT_CONSIST);

	DEFAULT_AND_CKSUM_INT(pmp->thresholdsExceededMsgLimit.Integrity, 10, CKSUM_OVERALL_DISRUPT);
	DEFAULT_AND_CKSUM_INT(pmp->thresholdsExceededMsgLimit.Congestion, 0, CKSUM_OVERALL_DISRUPT);
	DEFAULT_AND_CKSUM_INT(pmp->thresholdsExceededMsgLimit.SmaCongestion, 0, CKSUM_OVERALL_DISRUPT);
	DEFAULT_AND_CKSUM_INT(pmp->thresholdsExceededMsgLimit.Bubble, 0, CKSUM_OVERALL_DISRUPT);
	DEFAULT_AND_CKSUM_INT(pmp->thresholdsExceededMsgLimit.Security, 10, CKSUM_OVERALL_DISRUPT);
	DEFAULT_AND_CKSUM_INT(pmp->thresholdsExceededMsgLimit.Routing, 10, CKSUM_OVERALL_DISRUPT);

	DEFAULT_AND_CKSUM_INT(pmp->integrityWeights.LocalLinkIntegrityErrors, 0, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(pmp->integrityWeights.PortRcvErrors, 100, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(pmp->integrityWeights.ExcessiveBufferOverruns, 100, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(pmp->integrityWeights.LinkErrorRecovery, 0, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(pmp->integrityWeights.LinkDowned, 25, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(pmp->integrityWeights.UncorrectableErrors, 100, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(pmp->integrityWeights.FMConfigErrors, 100, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(pmp->integrityWeights.LinkQualityIndicator, 40, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(pmp->integrityWeights.LinkWidthDowngrade, 100, CKSUM_OVERALL_DISRUPT_CONSIST);

	DEFAULT_AND_CKSUM_INT(pmp->congestionWeights.PortXmitWait, 10, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(pmp->congestionWeights.SwPortCongestion, 100, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(pmp->congestionWeights.PortRcvFECN, 5, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(pmp->congestionWeights.PortRcvBECN, 1, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(pmp->congestionWeights.PortXmitTimeCong, 25, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(pmp->congestionWeights.PortMarkFECN, 25, CKSUM_OVERALL_DISRUPT_CONSIST);

	DEFAULT_AND_CKSUM_INT(pmp->resolution.LocalLinkIntegrity, 8000000, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(pmp->resolution.LinkErrorRecovery, 100000, CKSUM_OVERALL_DISRUPT_CONSIST);

	for (i = 0; i < pmp->number_of_pm_groups; i++) {
		if (pmp->pm_portgroups[i].Enabled) {
			CKSUM_STR(pmp->pm_portgroups[i].Name, CKSUM_OVERALL_DISRUPT_CONSIST);
			CKSUM_DATA(pmp->pm_portgroups[i].Monitors, CKSUM_OVERALL_DISRUPT_CONSIST);
		}
	}

	DEFAULT_AND_CKSUM_INT(pmp->debug, 0, CKSUM_OVERALL_DISRUPT);
	DEFAULT_AND_CKSUM_INT(pmp->debug_rmpp, 0, CKSUM_OVERALL_DISRUPT);

	DEFAULT_INT(pmp->subnet_size, DEFAULT_SUBNET_SIZE);
	if (pmp->subnet_size > MAX_SUBNET_SIZE) {
        IB_LOG_INFO_FMT(__func__, "PM subnet size is being adjusted from %u to %u", pmp->subnet_size, MAX_SUBNET_SIZE);
        pmp->subnet_size = MAX_SUBNET_SIZE;
    }
    if (pmp->subnet_size < MIN_SUPPORTED_ENDPORTS) {
        IB_LOG_WARN_FMT(__func__, "PM subnet size of %d is too small, setting to %d", pmp->subnet_size, MIN_SUPPORTED_ENDPORTS);
        pmp->subnet_size = MIN_SUPPORTED_ENDPORTS;
    }
	CKSUM_DATA(pmp->subnet_size, CKSUM_OVERALL_DISRUPT_CONSIST);

	DEFAULT_AND_CKSUM_INT(pmp->shortTermHistory.enable, 1, CKSUM_OVERALL_DISRUPT_CONSIST);
	if (pmp->shortTermHistory.enable) {
		DEFAULT_AND_CKSUM_INT(pmp->shortTermHistory.imagesPerComposite, 3, CKSUM_OVERALL_DISRUPT_CONSIST);
		DEFAULT_AND_CKSUM_INT(pmp->shortTermHistory.maxDiskSpace, 1024, CKSUM_OVERALL_DISRUPT_CONSIST);
		DEFAULT_AND_CKSUM_STR(pmp->shortTermHistory.StorageLocation, "/var/lib/opa-fm", CKSUM_OVERALL_DISRUPT);
		DEFAULT_AND_CKSUM_INT(pmp->shortTermHistory.totalHistory, 24, CKSUM_OVERALL_DISRUPT_CONSIST);
		DEFAULT_AND_CKSUM_INT(pmp->shortTermHistory.compressionDivisions, 1, CKSUM_OVERALL_DISRUPT_CONSIST);
	}

	DEFAULT_INT(pmp->SslSecurityEnabled, 0);
	DEFAULT_STR(pmp->SslSecurityDir, FM_SSL_SECURITY_DIR);
	DEFAULT_STR(pmp->SslSecurityFmCertificate, "fm_cert.pem");
	DEFAULT_STR(pmp->SslSecurityFmPrivateKey, "fm_key.pem");
	DEFAULT_STR(pmp->SslSecurityFmCaCertificate, "fm_ca_cert.pem");
	DEFAULT_INT(pmp->SslSecurityFmCertChainDepth, 1);
	DEFAULT_STR(pmp->SslSecurityFmDHParameters, "fm_dh_parms.pem");
	DEFAULT_STR(pmp->SslSecurityFmCaCRL, "fm_ca_crl.pem");

	if (pmp->image_update_interval >= pmp->sweep_interval) {
		IB_LOG_WARN_FMT(__func__, "Image Update Interval (%d) cannot be >= than the Sweep Interval (%d)", pmp->image_update_interval, pmp->sweep_interval);
		pmp->image_update_interval = (pmp->sweep_interval < 2 ? (pmp->sweep_interval + 1) / 2 : pmp->sweep_interval / 2);
		IB_LOG_WARN_FMT(__func__, "Setting Image Update Interval to 1/2 Sweep Interval (%d) ", pmp->image_update_interval);
	}
	CKSUM_DATA(pmp->image_update_interval, CKSUM_OVERALL_DISRUPT_CONSIST);

	CKSUM_END(pmp->overall_checksum, pmp->disruptive_checksum, pmp->consistency_checksum);

	if (xml_parse_debug)
		fprintf(stdout, "Pm instance %u checksum overall %u disruptive %u consistency %u\n", (unsigned int)instance,
			(unsigned int)pmp->overall_checksum, (unsigned int)pmp->disruptive_checksum, (unsigned int)pmp->consistency_checksum);
	return 1;
}

// show the PM XML config
void pmShowConfig(PMXmlConfig_t *pmp)
{
	int i,j;
#ifndef __VXWORKS__
	uint32_t    modid;
#endif

	if (!pmp)
		return;

	printf("XML - hca %u\n", (unsigned int)pmp->hca);
	printf("XML - port %u\n", (unsigned int)pmp->port);
	printf("XML - port_guid %u\n", (unsigned int)pmp->port_guid);

	printf("XML - name %s\n", pmp->name);
	printf("XML - log_file %s\n", pmp->log_file);
#ifndef __VXWORKS__
	printf("XML - syslog_facility %s\n", pmp->syslog_facility);
	for (modid=0; modid<= VIEO_LAST_MOD_ID; ++modid)
		printf("XML - log_mask[%u] 0x%x\n", modid, (unsigned int)pmp->log_masks[modid].value);
#endif
	printf("XML - SslSecurityEnabled %u\n", pmp->SslSecurityEnabled);
	printf("XML - SslSecurityDir %s\n", pmp->SslSecurityDir);
	printf("XML - SslSecurityFmCertificate %s\n", pmp->SslSecurityFmCertificate);
	printf("XML - SslSecurityFmPrivateKey %s\n", pmp->SslSecurityFmPrivateKey);
	printf("XML - SslSecurityFmCaCertificate %s\n", pmp->SslSecurityFmCaCertificate);
	printf("XML - SslSecurityFmCertChainDepth %u\n", pmp->SslSecurityFmCertChainDepth);
	printf("XML - SslSecurityFmDHParameters %s\n", pmp->SslSecurityFmDHParameters);
	printf("XML - SslSecurityFmCaCRLEnabled %u\n", pmp->SslSecurityFmCaCRLEnabled);
	printf("XML - SslSecurityFmCaCRL %s\n", pmp->SslSecurityFmCaCRL);

    printf("XML - log_level %u\n", (unsigned int)pmp->log_level);
	printf("XML - syslog_mode %u\n", (unsigned int)pmp->syslog_mode);
	printf("XML - config_consistency_check_level %u\n", (unsigned int)pmp->config_consistency_check_level);
	printf("XML - priority %u\n", (unsigned int)pmp->priority);
	printf("XML - elevated_priority %u\n", (unsigned int)pmp->elevated_priority);
	printf("XML - timer %u\n", (unsigned int)pmp->timer);
	printf("XML - sweep_interval %u\n", (unsigned int)pmp->sweep_interval);
	printf("XML - ErrorClear %u\n", (unsigned int)pmp->ErrorClear);
	printf("XML - ClearDataXfer %u\n", (unsigned int)pmp->ClearDataXfer);
	printf("XML - Clear64bit %u\n", (unsigned int)pmp->Clear64bit);
	printf("XML - Clear32bit %u\n", (unsigned int)pmp->Clear32bit);
	printf("XML - Clear8bit %u\n", (unsigned int)pmp->Clear8bit);
	printf("XML - process_hfi_counters %u\n", (unsigned int)pmp->process_hfi_counters);
	printf("XML - process_vl_counters %u\n", (unsigned int)pmp->process_vl_counters);

	printf("XML - MaxRetries %u\n", (unsigned int)pmp->MaxRetries);
	printf("XML - RcvWaitInterval %u\n", (unsigned int)pmp->RcvWaitInterval);
	printf("XML - MinRcvWaitInterval %u\n", (unsigned int)pmp->MinRcvWaitInterval);
	printf("XML - SweepErrorsLogThreshold %u\n", (unsigned int)pmp->SweepErrorsLogThreshold);
	printf("XML - MaxParallelNodes %u\n", (unsigned int)pmp->MaxParallelNodes);

	printf("XML - freeze_frame_lease %u\n", (unsigned int)pmp->freeze_frame_lease);
	printf("XML - max_clients %u\n", (unsigned int)pmp->max_clients);
	printf("XML - freeze_frame_images %u\n", (unsigned int)pmp->freeze_frame_images);
	printf("XML - total_images %u\n", (unsigned int)pmp->total_images);
	printf("XML - image_update_interval %u\n", (unsigned int)pmp->image_update_interval);

	printf("XML - thresholds.Integrity %u\n", (unsigned int)pmp->thresholds.Integrity);
	printf("XML - thresholds.Congestion %u\n", (unsigned int)pmp->thresholds.Congestion);
	printf("XML - thresholds.SmaCongestion %u\n", (unsigned int)pmp->thresholds.SmaCongestion);
	printf("XML - thresholds.Bubble %u\n", (unsigned int)pmp->thresholds.Bubble);
	printf("XML - thresholds.Security %u\n", (unsigned int)pmp->thresholds.Security);
	printf("XML - thresholds.Routing %u\n", (unsigned int)pmp->thresholds.Routing);

	printf("XML - thresholdsExceededMsgLimit.Integrity %u\n", (unsigned int)pmp->thresholdsExceededMsgLimit.Integrity);
	printf("XML - thresholdsExceededMsgLimit.Congestion %u\n", (unsigned int)pmp->thresholdsExceededMsgLimit.Congestion);
	printf("XML - thresholdsExceededMsgLimit.SmaCongestion %u\n", (unsigned int)pmp->thresholdsExceededMsgLimit.SmaCongestion);
	printf("XML - thresholdsExceededMsgLimit.Bubble %u\n", (unsigned int)pmp->thresholdsExceededMsgLimit.Bubble);
	printf("XML - thresholdsExceededMsgLimit.Security %u\n", (unsigned int)pmp->thresholdsExceededMsgLimit.Security);
	printf("XML - thresholdsExceededMsgLimit.Routing %u\n", (unsigned int)pmp->thresholdsExceededMsgLimit.Routing);

	printf("XML - integrityWeights.LocalLinkIntegrityErrors %u\n", (unsigned int)pmp->integrityWeights.LocalLinkIntegrityErrors);
	printf("XML - integrityWeights.RcvErrors %u\n", (unsigned int)pmp->integrityWeights.PortRcvErrors);
	printf("XML - integrityWeights.ExcessiveBufferOverruns %u\n", (unsigned int)pmp->integrityWeights.ExcessiveBufferOverruns);
	printf("XML - integrityWeights.LinkErrorRecovery %u\n", (unsigned int)pmp->integrityWeights.LinkErrorRecovery);
	printf("XML - integrityWeights.LinkDowned %u\n", (unsigned int)pmp->integrityWeights.LinkDowned);
	printf("XML - integrityWeights.UncorrectableErrors %u\n", (unsigned int)pmp->integrityWeights.UncorrectableErrors);
	printf("XML - integrityWeights.FMConfigErrors %u\n", (unsigned int)pmp->integrityWeights.FMConfigErrors);
	printf("XML - integrityWeights.LinkQualityIndicator %u\n", (unsigned int)pmp->integrityWeights.LinkQualityIndicator);
	printf("XML - integrityWeights.LinkWidthDowngrade %u\n", (unsigned int)pmp->integrityWeights.LinkWidthDowngrade);

	printf("XML - congestionWeights.XmitWaitPct %u\n", (unsigned int)pmp->congestionWeights.PortXmitWait);
	printf("XML - congestionWeights.CongDiscards %u\n", (unsigned int)pmp->congestionWeights.SwPortCongestion);
	printf("XML - congestionWeights.RcvFECNPct %u\n", (unsigned int)pmp->congestionWeights.PortRcvFECN);
	printf("XML - congestionWeights.RcvBECNPct %u\n", (unsigned int)pmp->congestionWeights.PortRcvBECN);
	printf("XML - congestionWeights.XmitTimeCongPct %u\n", (unsigned int)pmp->congestionWeights.PortXmitTimeCong);
	printf("XML - congestionWeights.MarkFECNPct %u\n", (unsigned int)pmp->congestionWeights.PortMarkFECN);

	printf("XML - resolution.LocalLinkIntegrity %u\n",				(unsigned int)pmp->resolution.LocalLinkIntegrity);
	printf("XML - resolution.LinkErrorRecovery %u\n",				(unsigned int)pmp->resolution.LinkErrorRecovery);

	printf("XML - number_of_pm_groups %u\n", (unsigned int)pmp->number_of_pm_groups);
	for (i = 0; i < pmp->number_of_pm_groups; i++) {
		printf("XML - pm_portgroups[%d].Enabled %u\n", i, pmp->pm_portgroups[i].Enabled);
		if (pmp->pm_portgroups[i].Enabled) {
			printf("XML - pm_portgroups[%d].PmPortGroupName %.*s\n", i, (int)sizeof(pmp->pm_portgroups[i].Name), pmp->pm_portgroups[i].Name);
			for (j=0; j < STL_PM_MAX_DG_PER_PMPG; j++) {
				if (sizeof(pmp->pm_portgroups[i].Monitors[j].monitor)) {
					printf("XML - pm_portgroups[%d].Monitors[%d].monitor %.*s\n", i, j, (int)sizeof(pmp->pm_portgroups[i].Monitors[j].monitor), pmp->pm_portgroups[i].Monitors[j].monitor);
				}
			}
		}
	}
/*
	printf("XML - log_level %u\n", (unsigned int)pmp->log_level);
*/
}

// Clear FE config
void feClearConfig(FEXmlConfig_t *fep)
{
	if (!fep)
		return;

	memset(fep->CoreDumpLimit, 0, sizeof(fep->CoreDumpLimit));
	memset(fep->CoreDumpDir, 0, sizeof(fep->CoreDumpDir));
	memset(fep->log_file, 0, sizeof(fep->log_file));
	memset(fep->name, 0, sizeof(fep->name));
	memset(fep->syslog_facility, 0, sizeof(fep->syslog_facility));
	memset(fep->log_masks, 0, sizeof(fep->log_masks));
}

// initialize FE defaults
boolean feInitConfig(FEXmlConfig_t *fep, uint32_t instance)
{
#ifdef __VXWORKS__
	int startPM=1;
	int startFE=1;
#endif

    if (!fep)
        return 0;

#ifdef __VXWORKS__
	idbSmGetManagersToStart(&startPM, &startFE);
	if (!startFE)
		fep->start = 0;
#endif

	DEFAULT_INT(fep->start, 0); // FE defaults to disabled

	DEFAULT_INT(fep->hca, 0);
	DEFAULT_INT(fep->port, 1);
	DEFAULT_INT(fep->port_guid, 0);

	DEFAULT_INT(fep->startup_retries, 5);
	DEFAULT_INT(fep->startup_stable_wait, 10);

	// These are now processed when "fill" at end of parsing whole file
	DEFAULT_INT(fep->login, 0);

	DEFAULT_INT(fep->subnet_size, DEFAULT_SUBNET_SIZE);
	if (fep->subnet_size > MAX_SUBNET_SIZE) {
        IB_LOG_INFO_FMT(__func__, "FE subnet size is being adjusted from %u to %u", fep->subnet_size, MAX_SUBNET_SIZE);
        fep->subnet_size = MAX_SUBNET_SIZE;
    }
    if (fep->subnet_size < MIN_SUPPORTED_ENDPORTS) {
        IB_LOG_WARN_FMT(__func__, "FE subnet size of %d is too small, setting to %d", fep->subnet_size, MIN_SUPPORTED_ENDPORTS);
        fep->subnet_size = MIN_SUPPORTED_ENDPORTS;
    }

	DEFAULT_INT(fep->debug, 0);
	DEFAULT_INT(fep->debug_rmpp, 0);
	DEFAULT_INT(fep->log_level, 1);
	DEFAULT_INT(fep->syslog_mode, 0);
	DEFAULT_INT(fep->listen, FE_LISTEN_PORT);
	DEFAULT_INT(fep->window, FE_WIN_SIZE);
	set_log_masks(fep->log_level, fep->syslog_mode, fep->log_masks);
	DEFAULT_STR(fep->CoreDumpLimit, "0");
	DEFAULT_STR(fep->CoreDumpDir, "/var/crash/opafm");
	DEFAULT_STR(fep->syslog_facility, "local6");
	DEFAULT_INT(fep->manager_check_rate, 60000000);
	DEFAULT_INT(fep->SslSecurityEnabled, 0);
	if (fep->SslSecurityEnabled) {
		DEFAULT_STR(fep->SslSecurityDir, FM_SSL_SECURITY_DIR);
		DEFAULT_STR(fep->SslSecurityFmCertificate, "fm_cert.pem");
		DEFAULT_STR(fep->SslSecurityFmPrivateKey, "fm_key.pem");
		DEFAULT_STR(fep->SslSecurityFmCaCertificate, "fm_ca_cert.pem");
		DEFAULT_INT(fep->SslSecurityFmCertChainDepth, 1);
		DEFAULT_STR(fep->SslSecurityFmDHParameters, "fm_dh_parms.pem");
		DEFAULT_INT(fep->SslSecurityFmCaCRLEnabled, 0);
		DEFAULT_STR(fep->SslSecurityFmCaCRL, "fm_ca_crl.pem");
	}
	
	return 1;
}

// show the FE XML config
void feShowConfig(FEXmlConfig_t *fep)
{
#ifndef __VXWORKS__
	uint32_t    modid;
#endif

	if (!fep)
		return;

	printf("XML - hca %u\n", (unsigned int)fep->hca);
	printf("XML - port %u\n", (unsigned int)fep->port);
	printf("XML - port_guid %u\n", (unsigned int)fep->port_guid);
	printf("XML - name %s\n", fep->name);
	printf("XML - log_file %s\n", fep->log_file);
	printf("XML - syslog_mode %u\n", (unsigned int)fep->syslog_mode);
#ifndef __VXWORKS__
	printf("XML - syslog_facility %s\n", fep->syslog_facility);
	printf("XML - CoreDumpLimit %s\n", fep->CoreDumpLimit);
	printf("XML - CoreDumpDir %s\n", fep->CoreDumpDir);
	printf("XML - log_level %u\n", (unsigned int)fep->log_level);
	for (modid = 0; modid<= VIEO_LAST_MOD_ID; ++modid)
    	printf("XML - log_mask[%u] 0x%x\n", modid, (unsigned int)fep->log_masks[modid].value);
#endif
	printf("XML - SslSecurityEnabled %u\n", (unsigned int)fep->SslSecurityEnabled);
	printf("XML - SslSecurityDir %s\n", fep->SslSecurityDir);
	printf("XML - SslSecurityFmCertificate %s\n", fep->SslSecurityFmCertificate);
	printf("XML - SslSecurityFmPrivateKey %s\n", fep->SslSecurityFmPrivateKey);
	printf("XML - SslSecurityFmCaCertificate %s\n", fep->SslSecurityFmCaCertificate);
	printf("XML - SslSecurityFmCertChainDepth %u\n", (unsigned int)fep->SslSecurityFmCertChainDepth);
	printf("XML - SslSecurityFmDHParameters %s\n", fep->SslSecurityFmDHParameters);
	printf("XML - SslSecurityFmCaCRLEnabled %u\n", (unsigned int)fep->SslSecurityFmCaCRLEnabled);
	printf("XML - SslSecurityFmCaCRL %s\n", fep->SslSecurityFmCaCRL);

	printf("XML - listen %u\n", (unsigned int)fep->listen);
	printf("XML - login %u\n", (unsigned int)fep->login);
	printf("XML - window %u\n", (unsigned int)fep->window);
	printf("XML - debug %u\n", (unsigned int)fep->debug);
	printf("XML - debug_rmpp %u\n", (unsigned int)fep->debug_rmpp);
	printf("XML - subnet_size %u\n", (unsigned int)fep->subnet_size);
}


void smFreeHypercubeRouting(SMXmlConfig_t *smp)
{
	SmSPRoutingCtrl_t *SPRoutingCtrl, *nextSPRoutingCtrl;

	memset(&smp->hypercubeRouting.routeLast.member, 0,
			sizeof(smp->hypercubeRouting.routeLast.member));

	SPRoutingCtrl = smp->hypercubeRouting.enhancedRoutingCtrl;
	if (SPRoutingCtrl == (void *)~0ul) {
		SPRoutingCtrl = NULL;
	}
	while (SPRoutingCtrl) {
		int portsSize = ROUNDUP(SPRoutingCtrl->portCount, PORTS_ALLOC_UNIT);

		if (portsSize) {
			freeXmlMemory(SPRoutingCtrl->ports, portsSize*sizeof(*SPRoutingCtrl->ports),
						  "SmSPRoutingPort_t *ports");
		}
		nextSPRoutingCtrl = SPRoutingCtrl->next;
		freeXmlMemory(SPRoutingCtrl, sizeof(SmSPRoutingCtrl_t), "SmSPRoutingCtrl_t *SPRoutingCtrl");
		SPRoutingCtrl = nextSPRoutingCtrl;
	}
	smp->hypercubeRouting.enhancedRoutingCtrl = NULL;
}

// Free SM configs
void smFreeConfig(SMXmlConfig_t *smp)
{
	smFreeHypercubeRouting(smp);
	freeXmlMemory(smp, sizeof(SMXmlConfig_t), "SMXmlConfig_t smFreeConfig()");
}

// Copy SM configs
boolean smCopyConfig(SMXmlConfig_t *dst, SMXmlConfig_t *src)
{
	SmSPRoutingCtrl_t *SPRoutingCtrl;
	SmSPRoutingCtrl_t **prev;

	if (!src || !dst)
		return 0;

	// Nothing to do if src and dst are the same.
	if (src == dst) return 1;

	// Copy everything first, then handle the qmap
	*dst = *src;

	// Make a copy of the SPRoutingCtrl
	SPRoutingCtrl = src->hypercubeRouting.enhancedRoutingCtrl;
	prev = &dst->hypercubeRouting.enhancedRoutingCtrl;
	if (SPRoutingCtrl == (void *)~0ul) {
		SPRoutingCtrl = NULL;
	}
	while (SPRoutingCtrl) {
		int portsSize = ROUNDUP(SPRoutingCtrl->portCount, PORTS_ALLOC_UNIT);
		SmSPRoutingCtrl_t *dstSPRoutingCtrl;

		dstSPRoutingCtrl = getXmlMemory(sizeof(*dstSPRoutingCtrl), "SmSPRoutingCtrl_t *dstSPRoutingCtrl");
		if (!dstSPRoutingCtrl) {
			fprintf(stderr, "%s: Memory limit exceeded for dstSPRoutingCtrl\n",
					__FUNCTION__);
			return FALSE;
		}
		memcpy(dstSPRoutingCtrl, SPRoutingCtrl, sizeof(*dstSPRoutingCtrl));
		if (portsSize) {
			SmSPRoutingPort_t *ports;

			ports = getXmlMemory(portsSize*sizeof(*ports), "SmSPRoutingPort_t *ports");
			if (!ports) {
				freeXmlMemory(dstSPRoutingCtrl, sizeof(SmSPRoutingCtrl_t), "SmSPRoutingCtrl_t *dstSPRoutingCtrl");
				fprintf(stderr, "%s: Memory limit exceeded for ports\n",
						__FUNCTION__);
				return FALSE;
			}
			memcpy(ports, SPRoutingCtrl->ports, portsSize*sizeof(*ports));
			dstSPRoutingCtrl->ports = ports;
		}
		*prev = dstSPRoutingCtrl;
		prev = &dstSPRoutingCtrl->next;

		SPRoutingCtrl = SPRoutingCtrl->next;
	}

	return 1;
}

// Clear SM configs
void smClearConfig(SMXmlConfig_t *smp)
{
	int i;

	if (!smp)
		return;

	memset(smp->dumpCounters, 0, sizeof(smp->dumpCounters));
	memset(smp->log_file, 0, sizeof(smp->log_file));
	memset(smp->syslog_facility, 0, sizeof(smp->syslog_facility));
	memset(smp->log_masks, 0, sizeof(smp->log_masks));
	memset(smp->name, 0, sizeof(smp->name));
	memset(smp->routing_algorithm, 0, sizeof(smp->routing_algorithm));
	memset(smp->preDefTopo.topologyFilename, 0, sizeof(smp->preDefTopo.topologyFilename));
	memset(&smp->ftreeRouting.coreSwitches, 0, sizeof(smp->ftreeRouting.coreSwitches));
	memset(&smp->ftreeRouting.routeLast, 0, sizeof(smp->ftreeRouting.routeLast));

	smp->dgRouting.dgCount = 0;
	for (i = 0; i < MAX_DGROUTING_ORDER; i++) {
		memset(&smp->dgRouting.dg[i], 0, sizeof(smp->dgRouting.dg[i]));
	}

	smFreeHypercubeRouting(smp);
}

// initialize SM defaults
boolean smInitConfig(SMXmlConfig_t *smp, SMDPLXmlConfig_t *dplp, SMMcastConfig_t *mcp, SmMcastMlidShare_t *mlsp, DGXmlConfig_t *dgp, uint32_t instance)
{
	DGConfig_t *dg;
	uint32_t i;
	SmSPRoutingCtrl_t *SPRoutingCtrl;

	if (!smp || !dplp || !mcp || !mlsp || !dgp)
		return 0;

	DEFAULT_INT(smp->start, 1); // Sm defaults to enabled
	if (!smp->start) { // if Sm is diabled, set the checksums to 0 and return
		smp->overall_checksum = 0;
		smp->disruptive_checksum = 0;
		smp->consistency_checksum = 0;
		return 1;
	}

	CKSUM_BEGIN;

	// if the sm is enabled, do CKSUM checks
	CKSUM_DATA(smp->start, CKSUM_OVERALL_DISRUPT);
	DEFAULT_AND_CKSUM_INT(smp->hca, 0, CKSUM_OVERALL_DISRUPT);
	DEFAULT_AND_CKSUM_INT(smp->port, 1, CKSUM_OVERALL_DISRUPT);
	DEFAULT_AND_CKSUM_INT(smp->port_guid, 0, CKSUM_OVERALL_DISRUPT);

	DEFAULT_AND_CKSUM_INT(smp->startup_retries, 5, CKSUM_OVERALL);
	DEFAULT_AND_CKSUM_INT(smp->startup_stable_wait, 10, CKSUM_OVERALL);

	DEFAULT_AND_CKSUM_INT(smp->sm_key, 0x0ull, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->mkey, 0x0ull, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->timer, 300, CKSUM_OVERALL_DISRUPT_CONSIST);

	DEFAULT_INT(smp->IgnoreTraps, 0);
	if (!smp->IgnoreTraps) {
		DEFAULT_INT(smp->trap_hold_down, 1);
		if (smp->trap_hold_down > smp->timer) {
			IB_LOG_WARN_FMT(__func__,"TrapHoldDown exceeds SweepInterval. Ignoring Traps.");
			smp->IgnoreTraps = 1;
		} else {
			CKSUM_DATA(smp->trap_hold_down, CKSUM_OVERALL_DISRUPT_CONSIST);
		}
	}
	CKSUM_DATA(smp->IgnoreTraps, CKSUM_OVERALL_DISRUPT_CONSIST);

	DEFAULT_AND_CKSUM_INT(smp->max_retries, MAD_RETRIES, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->rcv_wait_msec, MAD_RCV_WAIT_MSEC, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->min_rcv_wait_msec, MAD_MIN_RCV_WAIT_MSEC, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->master_ping_interval, SM_CHECK_MASTER_INTERVAL, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->master_ping_max_fail, SM_CHECK_MASTER_MAX_COUNT, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->topo_errors_threshold, 8, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->topo_abandon_threshold, 2, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->switch_lifetime_n2, 13, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->hoqlife_n2, 8, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->vl15FlowControlDisable, 1, CKSUM_OVERALL_DISRUPT_CONSIST); // by default, VL15 flow ctrl disabled
	DEFAULT_AND_CKSUM_INT(smp->vl15_credit_rate, 18, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->sa_resp_time_n2, 19, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->sa_packet_lifetime_n2, 16, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->vlstall, 7, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->db_sync_interval, 15, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->trap_threshold, SM_TRAP_THRESHOLD_DEFAULT, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->trap_threshold_min_count, SM_TRAP_THRESHOLD_COUNT_DEFAULT, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->trap_log_suppress_trigger_interval, SM_TRAP_LOG_SUPPRESS_TRIGGER_INTERVAL, CKSUM_OVERALL_DISRUPT);
	DEFAULT_AND_CKSUM_INT(smp->mc_dos_threshold, 0, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->mc_dos_action, 0, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->mc_dos_interval, 60, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->node_appearance_msg_thresh, 0, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->spine_first_routing, 0, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->shortestPathBalanced, 1, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->lid, 0x0, CKSUM_OVERALL_DISRUPT);
	DEFAULT_AND_CKSUM_INT(smp->P_Key_8B, 0, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->P_Key_10B, 0, CKSUM_OVERALL_DISRUPT_CONSIST);

	DEFAULT_INT(smp->lmc, 0x0);
	DEFAULT_INT(smp->lmc_e0, 0x0);
	if (smp->lmc > 7) {
        IB_LOG_WARN_FMT(__func__, "'Lmc' option of %d is not valid; defaulting to 0", smp->lmc);
        smp->lmc = 0;
    }
    if (smp->lmc_e0 > smp->lmc) {
        IB_LOG_WARN_FMT(__func__, "'LmcE0' option of must be <= Lmc; defaulting to %d", smp->lmc);
        smp->lmc_e0 = smp->lmc;
    }
	CKSUM_DATA(smp->lmc, CKSUM_OVERALL_DISRUPT_CONSIST);
	CKSUM_DATA(smp->lmc_e0, CKSUM_OVERALL_DISRUPT_CONSIST);

	DEFAULT_INT(smp->subnet_size, DEFAULT_SUBNET_SIZE);
	if (smp->subnet_size > MAX_SUBNET_SIZE) {
        IB_LOG_INFO_FMT(__func__, "SM subnet size is being adjusted from %u to %u", smp->subnet_size, MAX_SUBNET_SIZE);
        smp->subnet_size = MAX_SUBNET_SIZE;
    }
    if (smp->subnet_size < MIN_SUPPORTED_ENDPORTS) {
        IB_LOG_WARN_FMT(__func__, "SM subnet size of %d is too small, setting to %d", smp->subnet_size, MIN_SUPPORTED_ENDPORTS);
        smp->subnet_size = MIN_SUPPORTED_ENDPORTS;
    }
	CKSUM_DATA(smp->subnet_size, CKSUM_OVERALL_DISRUPT_CONSIST);

	DEFAULT_AND_CKSUM_STR(smp->syslog_facility, "local6", CKSUM_OVERALL_DISRUPT);
	DEFAULT_AND_CKSUM_STR(smp->routing_algorithm, "shortestpath", CKSUM_OVERALL_DISRUPT_CONSIST);

	DEFAULT_INT(smp->SslSecurityEnabled, 0);
	DEFAULT_STR(smp->SslSecurityDir, FM_SSL_SECURITY_DIR);
	DEFAULT_STR(smp->SslSecurityFmCertificate, "fm_cert.pem");
	DEFAULT_STR(smp->SslSecurityFmPrivateKey, "fm_key.pem");
	DEFAULT_STR(smp->SslSecurityFmCaCertificate, "fm_ca_cert.pem");
	DEFAULT_INT(smp->SslSecurityFmCertChainDepth, 1);
	DEFAULT_STR(smp->SslSecurityFmDHParameters, "fm_dh_parms.pem");
	DEFAULT_INT(smp->SslSecurityFmCaCRLEnabled, 0);
	DEFAULT_STR(smp->SslSecurityFmCaCRL, "fm_ca_crl.pem");

	DEFAULT_AND_CKSUM_INT(smp->debug, 0, CKSUM_OVERALL_DISRUPT);
	DEFAULT_AND_CKSUM_INT(smp->debug_rmpp, 0, CKSUM_OVERALL_DISRUPT);
	DEFAULT_AND_CKSUM_INT(smp->priority, 0, CKSUM_OVERALL_DISRUPT);
	DEFAULT_AND_CKSUM_INT(smp->elevated_priority, 0, CKSUM_OVERALL_DISRUPT);
	// Currently, dynamic changes to sm log_level are supported.
	DEFAULT_AND_CKSUM_INT(smp->log_level, 1, CKSUM_OVERALL);
	// Dynamic changes to syslog_mode are not (yet) supported.
	DEFAULT_AND_CKSUM_INT(smp->syslog_mode, 0, CKSUM_OVERALL_DISRUPT);
	// Dynamic manual changes to log_masks are not (yet) supported.
	// Therefore, add the log_masks to the overall and disruptive
	// checksums BEFORE updating them based on defaults,
	// log_level, and syslog_mode.
	CKSUM_DATA(smp->log_masks, CKSUM_OVERALL_DISRUPT);
	// after parsing is done, fill in unspecified log_masks based on log_level
	set_log_masks(smp->log_level, smp->syslog_mode, smp->log_masks);
	DEFAULT_AND_CKSUM_INT(smp->config_consistency_check_level, DEFAULT_CCC_LEVEL, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->path_selection, PATH_MODE_MINIMAL, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->queryValidation, 0, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->enforceVFPathRecs, 1, CKSUM_OVERALL_DISRUPT_CONSIST);

	DEFAULT_INT(smp->sma_batch_size, 2);
	DEFAULT_INT(smp->max_parallel_reqs, 3);
	DEFAULT_INT(smp->max_supported_lid, SM_DEFAULT_MAX_LID);
    if (smp->sma_batch_size == 0)
        smp->sma_batch_size = 1;
    if (smp->max_parallel_reqs == 0)
        smp->max_parallel_reqs = 1;
	if (smp->max_supported_lid == 0)
        smp->max_supported_lid = SM_DEFAULT_MAX_LID;
	smp->max_supported_lid |= 0x3FF; // Rounded up to 1K boundary.
	CKSUM_DATA(smp->sma_batch_size, CKSUM_OVERALL_DISRUPT_CONSIST);
	CKSUM_DATA(smp->max_parallel_reqs, CKSUM_OVERALL_DISRUPT_CONSIST);
	CKSUM_DATA(smp->max_supported_lid, CKSUM_OVERALL_DISRUPT_CONSIST);


	DEFAULT_AND_CKSUM_INT(smp->check_mft_responses, 1, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->sm_debug_perf, 0, CKSUM_OVERALL_DISRUPT);
	DEFAULT_AND_CKSUM_INT(smp->sa_debug_perf, 0, CKSUM_OVERALL_DISRUPT);
	DEFAULT_AND_CKSUM_INT(smp->sm_debug_vf, 0, CKSUM_OVERALL_DISRUPT);
	DEFAULT_AND_CKSUM_INT(smp->sm_debug_routing, 0, CKSUM_OVERALL_DISRUPT);
	DEFAULT_AND_CKSUM_INT(smp->sm_debug_lid_assign, 0, CKSUM_OVERALL_DISRUPT);
	DEFAULT_AND_CKSUM_INT(smp->debug_jm, 0, CKSUM_OVERALL_DISRUPT);
	DEFAULT_AND_CKSUM_INT(smp->sa_rmpp_checksum, 0, CKSUM_OVERALL_DISRUPT);
	DEFAULT_AND_CKSUM_INT(smp->loop_test_on, 0, CKSUM_OVERALL_DISRUPT);
	DEFAULT_AND_CKSUM_INT(smp->loop_test_fast_mode, 0, CKSUM_OVERALL_DISRUPT);
	DEFAULT_AND_CKSUM_INT(smp->loop_test_packets, 0, CKSUM_OVERALL_DISRUPT);
	DEFAULT_AND_CKSUM_INT(smp->non_resp_tsec, NONRESP_TIMEOUT, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->non_resp_max_count, NONRESP_MAXRETRY, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->monitor_standby_enable, 1, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->dynamic_port_alloc, 1, CKSUM_OVERALL_DISRUPT);
	DEFAULT_AND_CKSUM_INT(smp->loopback_mode, 0, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->force_rebalance, 0, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->use_cached_node_data, 0, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->sma_spoofing_check, 1, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->hfi_link_policy.link_max_downgrade, 1, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->hfi_link_policy.width_policy.enabled, 1, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->hfi_link_policy.width_policy.policy, 0, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->hfi_link_policy.speed_policy.enabled, 0, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->hfi_link_policy.speed_policy.policy, 0, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->isl_link_policy.link_max_downgrade, 1, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->isl_link_policy.width_policy.enabled, 1, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->isl_link_policy.width_policy.policy, 0, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->isl_link_policy.speed_policy.enabled, 0, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->isl_link_policy.speed_policy.policy, 0, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->preemption.small_packet, SM_PREEMPT_SMALL_PACKET_DEF, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->preemption.large_packet, SM_PREEMPT_LARGE_PACKET_DEF, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->preemption.preempt_limit, SM_PREEMPT_LIMIT_DEF, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->congestion.enable, 0, CKSUM_OVERALL_DISRUPT_CONSIST);
	smp->use_cached_hfi_node_data = 0;
	if (smp->use_cached_node_data) {
		smp->use_cached_hfi_node_data = 1;
	}
	if (smp->congestion.enable) {
		DEFAULT_AND_CKSUM_INT(smp->congestion.debug, 0, CKSUM_OVERALL_DISRUPT);
		DEFAULT_AND_CKSUM_INT(smp->congestion.sw.victim_marking_enable, 0, CKSUM_OVERALL_DISRUPT_CONSIST);
		DEFAULT_AND_CKSUM_INT(smp->congestion.sw.threshold, 8, CKSUM_OVERALL_DISRUPT_CONSIST);
		DEFAULT_INT(smp->congestion.sw.packet_size, 0);
		if (smp->congestion.sw.packet_size > 162) {
			IB_LOG_WARN_FMT(__func__, "FM CC SwitchCongestionSetting:Packet Size Limit %d exceeds max value setting to 162", smp->congestion.sw.packet_size);
			smp->congestion.sw.packet_size = 162;
		}
		CKSUM_DATA(smp->congestion.sw.packet_size, CKSUM_OVERALL_DISRUPT_CONSIST);
		DEFAULT_AND_CKSUM_INT(smp->congestion.sw.cs_threshold, 0, CKSUM_OVERALL_DISRUPT_CONSIST);
		DEFAULT_AND_CKSUM_INT(smp->congestion.sw.cs_return_delay, 0, CKSUM_OVERALL_DISRUPT_CONSIST);
		DEFAULT_AND_CKSUM_INT(smp->congestion.sw.marking_rate, 0, CKSUM_OVERALL_DISRUPT_CONSIST);
		DEFAULT_AND_CKSUM_INT(smp->congestion.ca.sl_based, 0, CKSUM_OVERALL_DISRUPT_CONSIST);
		DEFAULT_AND_CKSUM_INT(smp->congestion.ca.increase, 5, CKSUM_OVERALL_DISRUPT_CONSIST);
		DEFAULT_AND_CKSUM_INT(smp->congestion.ca.timer, 10, CKSUM_OVERALL_DISRUPT_CONSIST);
		DEFAULT_AND_CKSUM_INT(smp->congestion.ca.threshold, 8, CKSUM_OVERALL_DISRUPT_CONSIST);
		DEFAULT_AND_CKSUM_INT(smp->congestion.ca.min, 0, CKSUM_OVERALL_DISRUPT_CONSIST);
		DEFAULT_INT(smp->congestion.ca.limit, 127);

		// limit is max index == max_entries-1
		//Enforcing implementation limit for congestion rather than architectural limit (CONGESTION_CONTROL_TABLE_ENTRIES_PER_MAD-1).
		if (smp->congestion.ca.limit > CONGESTION_CONTROL_IMPLEMENTATION_LIMIT ) {
			IB_LOG_WARN_FMT(__func__, "FM CC SwitchCongestionSetting:CCTI Limit %d exceeds max value, setting to %d", smp->congestion.ca.limit,
						CONGESTION_CONTROL_IMPLEMENTATION_LIMIT);
			smp->congestion.ca.limit = (uint16_t)(CONGESTION_CONTROL_IMPLEMENTATION_LIMIT);
		}
		if (smp->congestion.ca.limit == 0) {
			IB_LOG_WARN_FMT(__func__, "FM CC SwitchCongestionSetting:CCTI Lower limit cannot be 0, setting to 1");
			smp->congestion.ca.limit =	(uint16_t)(1);
		}

		CKSUM_DATA(smp->congestion.ca.limit, CKSUM_OVERALL_DISRUPT_CONSIST);
		DEFAULT_AND_CKSUM_INT(smp->congestion.ca.desired_max_delay, 8000, CKSUM_OVERALL_DISRUPT_CONSIST);
	}

	DEFAULT_AND_CKSUM_INT(smp->adaptiveRouting.enable, 0, CKSUM_OVERALL_DISRUPT_CONSIST);
	if (smp->adaptiveRouting.enable) {
		DEFAULT_AND_CKSUM_INT(smp->adaptiveRouting.debug, 0, CKSUM_OVERALL_DISRUPT);
		DEFAULT_AND_CKSUM_INT(smp->adaptiveRouting.lostRouteOnly, 0, CKSUM_OVERALL_DISRUPT_CONSIST);
		DEFAULT_AND_CKSUM_INT(smp->adaptiveRouting.algorithm, 2, CKSUM_OVERALL_DISRUPT_CONSIST);
		DEFAULT_AND_CKSUM_INT(smp->adaptiveRouting.arFrequency, 4, CKSUM_OVERALL_DISRUPT_CONSIST);
		DEFAULT_AND_CKSUM_INT(smp->adaptiveRouting.threshold, 3, CKSUM_OVERALL_DISRUPT_CONSIST);
	}

	DEFAULT_AND_CKSUM_INT(smp->ftreeRouting.passthru, 0, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->ftreeRouting.debug, 0, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->ftreeRouting.tierCount, 0, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->ftreeRouting.fis_on_same_tier, 0, CKSUM_OVERALL_DISRUPT_CONSIST);


	for (i = 0; i < smp->dgRouting.dgCount; i++) {
		CKSUM_STR(smp->dgRouting.dg[i].member, CKSUM_OVERALL_DISRUPT_CONSIST);
	}

	DEFAULT_AND_CKSUM_INT(smp->hypercubeRouting.debug, 0, CKSUM_OVERALL_DISRUPT_CONSIST);
	CKSUM_STR(smp->hypercubeRouting.routeLast.member, CKSUM_OVERALL_DISRUPT_CONSIST);

	SPRoutingCtrl = smp->hypercubeRouting.enhancedRoutingCtrl;
	if (SPRoutingCtrl == (void *)~0ul) {
		SPRoutingCtrl = NULL;
		smp->hypercubeRouting.enhancedRoutingCtrl = NULL;
	}

	while (SPRoutingCtrl) {
		SmSPRoutingPort_t *ports = SPRoutingCtrl->ports;
		int portCount = SPRoutingCtrl->portCount;

		CKSUM_STR(SPRoutingCtrl->switches.member, CKSUM_OVERALL_DISRUPT_CONSIST);
		for (i = 0; i < portCount; i++, ports++) {
			CKSUM_DATA(ports->pport, CKSUM_OVERALL_DISRUPT_CONSIST);
			CKSUM_DATA(ports->vport, CKSUM_OVERALL_DISRUPT_CONSIST);
			CKSUM_DATA(ports->cost, CKSUM_OVERALL_DISRUPT_CONSIST);
		}
		SPRoutingCtrl = SPRoutingCtrl->next;
	}

	if (!strcasecmp(smp->routing_algorithm, "dor")) {
		// If using dor, checksum the smDorRouting config.
		CKSUM_DATA(smp->smDorRouting.dimensionCount, CKSUM_OVERALL_DISRUPT_CONSIST);
		CKSUM_DATA(smp->smDorRouting.numToroidal, CKSUM_OVERALL_DISRUPT_CONSIST);
		CKSUM_DATA(smp->smDorRouting.dimension, CKSUM_OVERALL_DISRUPT_CONSIST);
		CKSUM_DATA(smp->smDorRouting.escapeVLs, CKSUM_OVERALL_DISRUPT_CONSIST);
		CKSUM_DATA(smp->smDorRouting.faultRegions, CKSUM_OVERALL_DISRUPT_CONSIST);
		CKSUM_STR(smp->ftreeRouting.routeLast.member, CKSUM_OVERALL_DISRUPT_CONSIST);
		CKSUM_DATA(smp->smDorRouting.debug, CKSUM_OVERALL_DISRUPT);
		CKSUM_DATA(smp->smDorRouting.warn_threshold, CKSUM_OVERALL_DISRUPT);
		CKSUM_DATA(smp->smDorRouting.routingSCs, CKSUM_OVERALL_DISRUPT_CONSIST);
		CKSUM_DATA(smp->smDorRouting.topology, CKSUM_OVERALL_DISRUPT_CONSIST);
		CKSUM_DATA(smp->smDorRouting.overlayMCast, CKSUM_OVERALL_DISRUPT_CONSIST);
	}

	if (!strcasecmp(smp->routing_algorithm, "fattree")) {
		//If using fattree, checksum the fattree configuration
		CKSUM_DATA(smp->ftreeRouting.debug, CKSUM_OVERALL_DISRUPT);
		CKSUM_DATA(smp->ftreeRouting.tierCount, CKSUM_OVERALL_DISRUPT);
		CKSUM_DATA(smp->ftreeRouting.fis_on_same_tier, CKSUM_OVERALL_DISRUPT);
		CKSUM_STR(smp->ftreeRouting.coreSwitches.member, CKSUM_OVERALL_DISRUPT_CONSIST);
		CKSUM_STR(smp->ftreeRouting.routeLast.member, CKSUM_OVERALL_DISRUPT_CONSIST);
	}

	DEFAULT_AND_CKSUM_INT(smp->appliances.enable, 0, CKSUM_OVERALL_DISRUPT_CONSIST);
	if (smp->appliances.enable)
		for (i = 0; i < MAX_SM_APPLIANCES; i++)
			DEFAULT_AND_CKSUM_INT(smp->appliances.guids[i], 0, CKSUM_OVERALL_DISRUPT_CONSIST);

	DEFAULT_INT(smp->preDefTopo.enabled, 0)
#ifdef __VXWORKS__
	if(smp->preDefTopo.enabled) {
		smp->preDefTopo.enabled = 0;
		IB_LOG_ERROR0("Pre Defined Topology: (Disabled) Not supported on embedded platforms.");
	}
#endif
	// If no pre-defined topology filename was provided, disable the feature
	if (!strlen(smp->preDefTopo.topologyFilename)) {
		if (smp->preDefTopo.enabled)
			IB_LOG_ERROR0("Pre Defined Topology: (Disabled) Empty topology file path.");
		smp->preDefTopo.enabled = 0;
	}

	DEFAULT_AND_CKSUM_INT(smp->preDefTopo.enabled, 0, CKSUM_OVERALL_DISRUPT_CONSIST);
	if(smp->preDefTopo.enabled) {
		CKSUM_STR(smp->preDefTopo.topologyFilename, CKSUM_OVERALL_DISRUPT);
		CKSUM_FILE(smp->preDefTopo.topologyFilename, CKSUM_OVERALL_DISRUPT_CONSIST);
		DEFAULT_AND_CKSUM_INT(smp->preDefTopo.logMessageThreshold, 0, CKSUM_OVERALL_DISRUPT_CONSIST);
		DEFAULT_AND_CKSUM_INT(smp->preDefTopo.fieldEnforcement.nodeDesc, FIELD_ENF_LEVEL_DISABLED, CKSUM_OVERALL_DISRUPT_CONSIST);
		DEFAULT_AND_CKSUM_INT(smp->preDefTopo.fieldEnforcement.nodeGuid, FIELD_ENF_LEVEL_DISABLED, CKSUM_OVERALL_DISRUPT_CONSIST);
		DEFAULT_AND_CKSUM_INT(smp->preDefTopo.fieldEnforcement.portGuid, FIELD_ENF_LEVEL_DISABLED, CKSUM_OVERALL_DISRUPT_CONSIST);
		DEFAULT_AND_CKSUM_INT(smp->preDefTopo.fieldEnforcement.undefinedLink, FIELD_ENF_LEVEL_DISABLED, CKSUM_OVERALL_DISRUPT_CONSIST);
	}

	DEFAULT_AND_CKSUM_INT(smp->lid_strategy, LID_STRATEGY_SERIAL, CKSUM_OVERALL_DISRUPT_CONSIST);
	if (smp->lid_strategy == LID_STRATEGY_TOPOLOGY && !smp->preDefTopo.enabled) {
		IB_LOG_ERROR_FMT(__func__,"Terminating FM: Topology-LID strategy requires Pre Defined Topology enabled.");
		return 0;
	}

	if (smp->lid_strategy == LID_STRATEGY_TOPOLOGY && smp->lid != 0) {
		IB_LOG_WARN_FMT(__func__, "LID strategy is Topology but user-specified"
			" SM LID is non-zero. Ignoring user-specified SM LID.");
		smp->lid = 0;
	}

#ifdef __VXWORKS__
	uint8_t testLmc = smp->lmc_e0; // Assume SM on enhanced SP0
	const char *lmcStr = "LmcE0";
#else
	uint8_t testLmc = smp->lmc; // Assume SM on HFI
	const char *lmcStr = "Lmc";
#endif

	// If user-defined SM LID is given, must be divisible by LMC
	if (smp->lid_strategy == LID_STRATEGY_SERIAL && smp->lid != 0 && smp->lid % (1 << testLmc) != 0) {
		IB_LOG_ERROR_FMT(__func__, "Terminating: User-specified SM LID"
			" must be divisible by %d (%s is %d)", (1 << testLmc), lmcStr, testLmc);
		return 0;
	}

	DEFAULT_AND_CKSUM_INT(dplp->dp_lifetime[0], 0x01, CKSUM_OVERALL_DISRUPT_CONSIST);
	for (i = 1; i < DYNAMIC_PACKET_LIFETIME_ARRAY_SIZE; i++)
		DEFAULT_AND_CKSUM_INT(dplp->dp_lifetime[i], 0x0, CKSUM_OVERALL_DISRUPT_CONSIST);

	DEFAULT_AND_CKSUM_INT(mcp->disable_mcast_check, 0x0, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(mcp->enable_pruning, 0, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(mcp->mcast_mlid_table_cap, DEFAULT_SW_MLID_TABLE_CAP, CKSUM_OVERALL_DISRUPT_CONSIST);
	CKSUM_STR(mcp->mcroot_select_algorithm, CKSUM_OVERALL_DISRUPT_CONSIST);
	CKSUM_STR(mcp->mcroot_min_cost_improvement, CKSUM_OVERALL_DISRUPT_CONSIST);

	for (i = 0; i < MAX_SUPPORTED_MCAST_GRP_CLASSES_XML; i++) {
		CKSUM_DATA(mlsp->mcastMlid[i].enable, CKSUM_OVERALL_DISRUPT_CONSIST);
		CKSUM_DATA(mlsp->mcastMlid[i].mcastGrpMGidLimitMax, CKSUM_OVERALL_DISRUPT_CONSIST);
		CKSUM_DATA(mlsp->mcastMlid[i].mcastGrpMGidLimitMaskConvert, CKSUM_OVERALL_DISRUPT_CONSIST);
		CKSUM_DATA(mlsp->mcastMlid[i].mcastGrpMGidLimitValueConvert, CKSUM_OVERALL_DISRUPT_CONSIST);
	}

	DEFAULT_AND_CKSUM_INT(smp->minSharedVLMem, 0, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->dedicatedVLMemMulti, 1, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->cableInfoPolicy, CIP_LINK, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->timerScalingEnable, 0, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->min_supported_vls, 8, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->max_fixed_vls, 8, CKSUM_OVERALL_DISRUPT_CONSIST);

	CKSUM_DATA(smp->wireDepthOverride, CKSUM_OVERALL_DISRUPT_CONSIST);
	CKSUM_DATA(smp->replayDepthOverride, CKSUM_OVERALL_DISRUPT_CONSIST);

	// If NoReplyIfBusy is set to 1, a MAD_STATUS_BUSY will not be returned to the SA requester.
	DEFAULT_AND_CKSUM_INT(smp->NoReplyIfBusy, 0, CKSUM_OVERALL_DISRUPT_CONSIST);

	if ((smp->lft_multi_block == UNDEFINED_XML32) || (smp->lft_multi_block > (STL_MAX_PAYLOAD_SMP_DR/MAX_LFT_ELEMENTS_BLOCK)))
		smp->lft_multi_block = STL_MAX_PAYLOAD_SMP_DR/MAX_LFT_ELEMENTS_BLOCK;
	CKSUM_DATA(smp->lft_multi_block, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->use_aggregates, 1, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->sc_multi_block, STL_NUM_SCSC_BLOCKS_PER_DRSMP, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->optimized_portinfo, 1, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->optimized_buffer_control, 1, CKSUM_OVERALL_DISRUPT_CONSIST);

	DEFAULT_AND_CKSUM_INT(smp->forceAttributeRewrite, 0, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->skipAttributeWrite, 0, CKSUM_OVERALL_DISRUPT_CONSIST);

	if (smp->defaultPortErrorAction == UNDEFINED_XML32) {
		// Making defaultPortErrorAction a _PortErrorAction would make more sense
		// but I can't find guarantees IXML_FIELD(SMXmlConfig_t, defPortErrorAction)
		// would coincide with the .AsReg32 field of _PortErrorAction
		union _PortErrorAction defPea = { 0 };
		defPea.s.FmConfigErrorBadHeadDist = 1;
		defPea.s.FmConfigErrorBadTailDist = 1;
		defPea.s.FmConfigErrorBadCtrlDist = 1;
		defPea.s.FmConfigErrorUnsupportedVLMarker = 1;
		defPea.s.PortRcvErrorBadVLMarker = 1;
		smp->defaultPortErrorAction = defPea.AsReg32;
	}
	CKSUM_DATA(smp->defaultPortErrorAction, CKSUM_OVERALL_DISRUPT_CONSIST);

	DEFAULT_AND_CKSUM_INT(smp->switchCascadeActivateEnable, 0, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->neighborNormalRetries, 2, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->terminateAfter, 0, CKSUM_OVERALL_DISRUPT_CONSIST);
	DEFAULT_AND_CKSUM_INT(smp->portBounceLogLimit, PORT_BOUNCE_LOG_NO_LIMIT, CKSUM_OVERALL_DISRUPT_CONSIST);
	// FIXME: cjking - Temporary patch for FPGA related PR-124905
	DEFAULT_AND_CKSUM_INT(smp->neighborFWAuthenEnable, 0, CKSUM_OVERALL_DISRUPT_CONSIST);

	DEFAULT_AND_CKSUM_INT(smp->cumulative_timeout_limit, 300 * VTIMER_1S, CKSUM_OVERALL_DISRUPT_CONSIST);

	CKSUM_STR(smp->dumpCounters, CKSUM_OVERALL_DISRUPT);

	// Add all of the DGs to the SM checksums
	// Because the dgs are in an array, order matters!
	// The checksum will change if the order in the xml file changes.
	// If they were in a map, then order wouldn't matter.
	// NOTE: DGs are a good candidate to make non-disruptive.
	for(i = 0 ; i < dgp->number_of_dgs; i++) {
		cl_map_item_t *guidqp;
		XmlNode_t *nodep;
		XmlIncGroup_t *groupp;

		dg = dgp->dg[i];

		// Because these are linked lists, order matters!
		// The checksum will change if the order in the xml file changes.
		// If they were maps, then order wouldn't matter.
		CKSUM_DATA(dg->name, CKSUM_OVERALL_DISRUPT_CONSIST);
		for_all_qmap_item(&dg->system_image_guid, guidqp) {
			uint64_t guid = cl_qmap_key(guidqp);
			CKSUM_DATA(guid, CKSUM_OVERALL_DISRUPT_CONSIST);
		}
		for_all_qmap_item(&dg->node_guid, guidqp) {
			uint64_t guid = cl_qmap_key(guidqp);
			CKSUM_DATA(guid, CKSUM_OVERALL_DISRUPT_CONSIST);
		}
		for_all_qmap_item(&dg->port_guid, guidqp) {
			uint64_t guid = cl_qmap_key(guidqp);
			CKSUM_DATA(guid, CKSUM_OVERALL_DISRUPT_CONSIST);
		}
		for (nodep = dg->node_description; nodep; nodep = nodep->next) {
			CKSUM_DATA(nodep->node, CKSUM_OVERALL_DISRUPT_CONSIST);
		}
		for (groupp = dg->included_group; groupp; groupp = groupp->next) {
			CKSUM_DATA(groupp->group, CKSUM_OVERALL_DISRUPT_CONSIST);
		}
		CKSUM_DATA(dg->select_all, CKSUM_OVERALL_DISRUPT_CONSIST);
		CKSUM_DATA(dg->select_self, CKSUM_OVERALL_DISRUPT_CONSIST);
		CKSUM_DATA(dg->select_hfi_direct_connect, CKSUM_OVERALL_DISRUPT_CONSIST);
		CKSUM_DATA(dg->select_swe0, CKSUM_OVERALL_DISRUPT_CONSIST);
		CKSUM_DATA(dg->select_all_mgmt_allowed, CKSUM_OVERALL_DISRUPT_CONSIST);
		CKSUM_DATA(dg->select_all_tfis, CKSUM_OVERALL_DISRUPT_CONSIST);
		CKSUM_DATA(dg->node_type_fi, CKSUM_OVERALL_DISRUPT_CONSIST);
		CKSUM_DATA(dg->node_type_sw, CKSUM_OVERALL_DISRUPT_CONSIST);
	}

	CKSUM_DATA(smp->subnet_prefix, CKSUM_OVERALL_DISRUPT_CONSIST);

	CKSUM_END(smp->overall_checksum, smp->disruptive_checksum, smp->consistency_checksum);

	// Verify interdependencies
	if (smp->lid >= smp->max_supported_lid) {
		IB_LOG_ERROR_FMT(__func__, "Value LID (0x%x) needs to be less than MaximumLID (0x%x)\n",smp->lid,smp->max_supported_lid);
		return 0;
	}
	if (smp->max_supported_lid > 0x00EFFFFF) {
		// LID range from 0xF00000-0xFFFFFF are reserved for multicast operation in HW.
		IB_LOG_ERROR_FMT(__func__, "MaximumLID (0x%x) setting is out of range.\n", smp->max_supported_lid);
		return 0;
	}

	if (xml_parse_debug)
		fprintf(stdout, "Sm instance %u checksum overall %u disruptive %u consistency %u\n", (unsigned int)instance,
			(unsigned int)smp->overall_checksum, (unsigned int)smp->disruptive_checksum, (unsigned int)smp->consistency_checksum);
	return 1;
}

// show the SM XML config
void smShowConfig(SMXmlConfig_t *smp, SMDPLXmlConfig_t *dplp, SMMcastConfig_t *mcp, SmMcastMlidShare_t *mlsp)
{
	uint32_t i;
	char     str[32];
	SmSPRoutingCtrl_t *SPRoutingCtrl;

	if (!smp || !dplp || !mcp || !mlsp)
		return;

	printf("XML - hca %u\n", (unsigned int)smp->hca);
	printf("XML - port %u\n", (unsigned int)smp->port);
	printf("XML - port_guid %u\n", (unsigned int)smp->port_guid);
	printf("XML - name %s\n", smp->name);
	printf("XML - sm_key 0x%16.16llx\n", (long long unsigned int)smp->sm_key);
	printf("XML - mkey 0x%16.16llx\n", (long long unsigned int)smp->mkey);
	printf("XML - timer %llu\n", (long long unsigned int)smp->timer);
	printf("XML - IgnoreTraps %u\n", (unsigned int)smp->IgnoreTraps);
	printf("XML - trap_hold_down %u\n", (unsigned int)smp->trap_hold_down);
	printf("XML - max_retries %u\n", (unsigned int)smp->max_retries);
	printf("XML - rcv_wait_msec %u\n", (unsigned int)smp->rcv_wait_msec);
	printf("XML - min_rcv_wait_msec %u\n", (unsigned int)smp->min_rcv_wait_msec);
	printf("XML - master_ping_interval %u\n", (unsigned int)smp->master_ping_interval);
	printf("XML - master_ping_max_fail %u\n", (unsigned int)smp->master_ping_max_fail);
	printf("XML - topo_errors_threshold %u\n", (unsigned int)smp->topo_errors_threshold);
	printf("XML - topo_abandon_threshold %u\n", (unsigned int)smp->topo_abandon_threshold);
	printf("XML - switch_lifetime_n2 %u\n", (unsigned int)smp->switch_lifetime_n2);
	printf("XML - hoqlife_n2 %u\n", (unsigned int)smp->hoqlife_n2);
	printf("XML - sa_resp_time_n2 %u\n", (unsigned int)smp->sa_resp_time_n2);
	printf("XML - sa_packet_lifetime_n2 %u\n", (unsigned int)smp->sa_packet_lifetime_n2);
	printf("XML - vlstall %u\n", (unsigned int)smp->vlstall);
	printf("XML - db_sync_interval %u\n", (unsigned int)smp->db_sync_interval);
	printf("XML - trap_threshold %u\n", (unsigned int)smp->trap_threshold);
	printf("XML - trap_threshold_min_count %u\n", (unsigned int)smp->trap_threshold_min_count);
	printf("XML - trap_log_suppress_trigger_interval %u\n", (unsigned int)smp->trap_log_suppress_trigger_interval);
	printf("XML - mc_dos_threshold %u\n", (unsigned int)smp->mc_dos_threshold);
	printf("XML - mc_dos_action %u\n", (unsigned int)smp->mc_dos_action);
	printf("XML - mc_dos_interval %u\n", (unsigned int)smp->mc_dos_interval);
	printf("XML - node_appearance_msg_thresh %u\n", (unsigned int)smp->node_appearance_msg_thresh);
	printf("XML - spine_first_routing %u\n", (unsigned int)smp->spine_first_routing);
	printf("XML - shortestPathBalanced %u\n", (unsigned int)smp->shortestPathBalanced);
	printf("XML - lid 0x%x\n", (unsigned int)smp->lid);
	printf("XML - lmc 0x%x\n", (unsigned int)smp->lmc);
	printf("XML - lmc_e0 0x%x\n", (unsigned int)smp->lmc_e0);
	printf("XML - subnet_size %u\n", (unsigned int)smp->subnet_size);
#ifndef __VXWORKS__
	printf("XML - syslog_facility %s\n", smp->syslog_facility);
	for (i = 0; i <= VIEO_LAST_MOD_ID; ++i)
		printf("XML - log_mask[%u] 0x%x\n", i, (unsigned int)smp->log_masks[i].value);
#endif
	printf("XML - SslSecurityEnabled %u\n", (unsigned int)smp->SslSecurityEnabled);
	printf("XML - SslSecurityDir %s\n", smp->SslSecurityDir);
	printf("XML - SslSecurityFmCertificate %s\n", smp->SslSecurityFmCertificate);
	printf("XML - SslSecurityFmPrivateKey %s\n", smp->SslSecurityFmPrivateKey);
	printf("XML - SslSecurityFmCaCertificate %s\n", smp->SslSecurityFmCaCertificate);
	printf("XML - SslSecurityFmCertChainDepth %u\n", (unsigned int)smp->SslSecurityFmCertChainDepth);
	printf("XML - SslSecurityFmDHParameters %s\n", smp->SslSecurityFmDHParameters);
	printf("XML - SslSecurityFmCaCRLEnabled %u\n", (unsigned int)smp->SslSecurityFmCaCRLEnabled);
	printf("XML - SslSecurityFmCaCRL %s\n", smp->SslSecurityFmCaCRL);

	printf("XML - log_file %s\n", smp->log_file);
	printf("XML - debug %u\n", (unsigned int)smp->debug);
	printf("XML - debug_rmpp %u\n", (unsigned int)smp->debug_rmpp);
	printf("XML - priority %u\n", (unsigned int)smp->priority);
	printf("XML - elevated_priority %u\n", (unsigned int)smp->elevated_priority);
	printf("XML - log_level %u\n", (unsigned int)smp->log_level);
	printf("XML - syslog_mode %u\n", (unsigned int)smp->syslog_mode);
	printf("XML - config_consistency_check_level %u\n", (unsigned int)smp->config_consistency_check_level);
	printf("XML - routing_algorithm %s\n", smp->routing_algorithm);
	printf("XML - path_selection %u\n", (unsigned int)smp->path_selection);
	printf("XML - queryValidation %u\n", (unsigned int)smp->queryValidation);
	printf("XML - enforceVFPathRecs %u\n", (unsigned int)smp->enforceVFPathRecs);
	printf("XML - sma_batch_size %u\n", (unsigned int)smp->sma_batch_size);
	printf("XML - max_parallel_reqs %u\n", (unsigned int)smp->max_parallel_reqs);
	printf("XML - check_mft_responses %u\n", (unsigned int)smp->check_mft_responses);
	printf("XML - sm_debug_perf %u\n", (unsigned int)smp->sm_debug_perf);
	printf("XML - sa_debug_perf %u\n", (unsigned int)smp->sa_debug_perf);
	printf("XML - sm_debug_vf %u\n", (unsigned int)smp->sm_debug_vf);
	printf("XML - sm_debug_routing %u\n", (unsigned int)smp->sm_debug_routing);
	printf("XML - sm_debug_lid_assign %u\n", (unsigned int)smp->sm_debug_lid_assign);
	printf("XML - debug_jm %u\n", (unsigned int)smp->debug_jm);
	printf("XML - sa_rmpp_checksum %u\n", (unsigned int)smp->sa_rmpp_checksum);
	printf("XML - loop_test_on %u\n", (unsigned int)smp->loop_test_on);
	printf("XML - loop_test_fast_mode %u\n", (unsigned int)smp->loop_test_fast_mode);
	printf("XML - loop_test_packets %u\n", (unsigned int)smp->loop_test_packets);
	printf("XML - lid_strategy %u\n", (unsigned int)smp->lid_strategy);
	printf("XML - non_resp_tsec %u\n", (unsigned int)smp->non_resp_tsec);
	printf("XML - non_resp_max_count %u\n", (unsigned int)smp->non_resp_max_count);
	printf("XML - monitor_standby_enable %u\n", (unsigned int)smp->monitor_standby_enable);
	printf("XML - dynamic_port_alloc %u\n", (unsigned int)smp->dynamic_port_alloc);
	printf("XML - loopback_mode %u\n", (unsigned int)smp->loopback_mode);
	printf("XML - force_rebalance %u\n", (unsigned int)smp->force_rebalance);
	printf("XML - use_cached_node_data %u\n", (unsigned int)smp->use_cached_node_data);
	printf("XML - NoReplyIfBusy %u\n", (unsigned int)smp->NoReplyIfBusy);
	printf("XML - lft_multi_block %u\n", (unsigned int)smp->lft_multi_block);
	printf("XML - use_aggregates %u\n", (unsigned int)smp->use_aggregates);
	printf("XML - sc_multi_block %u\n", (unsigned int)smp->sc_multi_block);
	printf("XML - optimized_portinfo %u\n", (unsigned int)smp->optimized_portinfo);
	printf("XML - optimized_buffer_control %u\n", (unsigned int)smp->optimized_buffer_control);
	printf("XML - sma_spoofing_check %u\n", (unsigned int)smp->sma_spoofing_check);
	printf("XML - minSharedVLMem %u\n", (unsigned int)smp->minSharedVLMem);
	printf("XML - dedicatedVLMemMulti %u\n", (unsigned int)smp->dedicatedVLMemMulti);
	printf("XML - wireDepthOverride %d\n", (unsigned int)smp->wireDepthOverride);
	printf("XML - replayDepthOverride %d\n", (unsigned int)smp->replayDepthOverride);
    printf("XML - timerScalingEnable  %d\n", (unsigned int) smp->timerScalingEnable);
	printf("XML - min_supported_vls %d\n", (unsigned int)smp->min_supported_vls);
	printf("XML - max_fixed_vls %d\n", (unsigned int)smp->max_fixed_vls);
	printf("XML - hfi_link_policy.link_max_downgrade 0x%x\n", (unsigned int)smp->hfi_link_policy.link_max_downgrade);
	printf("XML - hfi_link_policy.link_width.enabled 0x%x\n", (unsigned int)smp->hfi_link_policy.width_policy.enabled);
	printf("XML - hfi_link_policy.link_width.policy 0x%x\n", (unsigned int)smp->hfi_link_policy.width_policy.policy);
	printf("XML - hfi_link_policy.link_speed.enabled 0x%x\n", (unsigned int)smp->hfi_link_policy.speed_policy.enabled);
	printf("XML - hfi_link_policy.link_speed.policy 0x%x\n", (unsigned int)smp->hfi_link_policy.speed_policy.policy);
	printf("XML - isl_link_policy.link_max_downgrade 0x%x\n", (unsigned int)smp->isl_link_policy.link_max_downgrade);
	printf("XML - isl_link_policy.link_width.enabled 0x%x\n", (unsigned int)smp->isl_link_policy.width_policy.enabled);
	printf("XML - isl_link_policy.link_width.policy 0x%x\n", (unsigned int)smp->isl_link_policy.width_policy.policy);
	printf("XML - isl_link_policy.link_speed.enabled 0x%x\n", (unsigned int)smp->isl_link_policy.speed_policy.enabled);
	printf("XML - isl_link_policy.link_speed.policy 0x%x\n", (unsigned int)smp->isl_link_policy.speed_policy.policy);
	printf("XML - preemption.small_packet 0x%x\n", (unsigned int) smp->preemption.small_packet);
	printf("XML - preemption.large_packet 0x%x\n", (unsigned int) smp->preemption.large_packet);
	printf("XML - preemption.preempt_limit 0x%x\n", (unsigned int) smp->preemption.preempt_limit);
	printf("XML - congestion.enable %u\n", (unsigned int)smp->congestion.enable);
	printf("XML - congestion.debug %u\n", (unsigned int)smp->congestion.debug);
	printf("XML - congestion.sw.victim_marking_enable %u\n", (unsigned int)smp->congestion.sw.victim_marking_enable);
	printf("XML - congestion.sw.threshold %u\n", (unsigned int)smp->congestion.sw.threshold);
	printf("XML - congestion.sw.packet_size %u\n", (unsigned int)smp->congestion.sw.packet_size);
	printf("XML - congestion.sw.cs_threshold %u\n", (unsigned int)smp->congestion.sw.cs_threshold);
	printf("XML - congestion.sw.cs_return_delay %u\n", (unsigned int)smp->congestion.sw.cs_return_delay);
	printf("XML - congestion.sw.marking_rate %u\n", (unsigned int)smp->congestion.sw.marking_rate);
	printf("XML - congestion.ca.increase %u\n", (unsigned int)smp->congestion.ca.increase);
	printf("XML - congestion.ca.timer %u\n", (unsigned int)smp->congestion.ca.timer);
	printf("XML - congestion.ca.threshold %u\n", (unsigned int)smp->congestion.ca.threshold);
	printf("XML - congestion.ca.min %u\n", (unsigned int)smp->congestion.ca.min);
	printf("XML - congestion.ca.limit %u\n", (unsigned int)smp->congestion.ca.limit);
	printf("XML - congestion.ca.desired_max_delay %u\n", smp->congestion.ca.desired_max_delay);

	printf("XML - adaptiveRouting.enable %u\n", (unsigned int)smp->adaptiveRouting.enable);
	printf("XML - adaptiveRouting.debug %u\n", (unsigned int)smp->adaptiveRouting.debug);
	printf("XML - adaptiveRouting.lostRouteOnly %u\n", (unsigned int)smp->adaptiveRouting.lostRouteOnly);
	printf("XML - adaptiveRouting.algorithm %u\n", (unsigned int)smp->adaptiveRouting.algorithm);
	printf("XML - adaptiveRouting.arFrequency %u\n", (unsigned int)smp->adaptiveRouting.arFrequency);
	printf("XML - adaptiveRouting.threshold %u\n", (unsigned int)smp->adaptiveRouting.threshold);

	printf("XML - ftreeRouting.passthru %u\n", (unsigned int)smp->ftreeRouting.passthru);
	printf("XML - ftreeRouting.debug %u\n", (unsigned int)smp->ftreeRouting.debug);
	printf("XML - ftreeRouting.tierCount %u\n", (unsigned int)smp->ftreeRouting.tierCount);
	printf("XML - ftreeRouting.fis_on_same_tier %u\n", (unsigned int)smp->ftreeRouting.fis_on_same_tier);
	printf("XML - ftreeRouting.coreSwitches %s\n", smp->ftreeRouting.coreSwitches.member);
	printf("XML - ftreeRouting.routeLast %s\n", smp->ftreeRouting.routeLast.member);

	SPRoutingCtrl = smp->hypercubeRouting.enhancedRoutingCtrl;
	while (SPRoutingCtrl) {
		SmSPRoutingPort_t *ports = SPRoutingCtrl->ports;
		int portCount = SPRoutingCtrl->portCount;

		printf("XML - SPRoutingCtrl.switches = %s, pPort, vPort, cost:", SPRoutingCtrl->switches.member);
		for (i = 0; i < portCount; i++, ports++) {
			printf("XML        %d %d %d\n", ports->pport, ports->vport, ports->cost);
		}
		SPRoutingCtrl = SPRoutingCtrl->next;
	}


	printf("XML - cableInfoPolicy %u\n", (unsigned int)smp->cableInfoPolicy);
	printf("XML - terminateAfter %u\n", (unsigned int)smp->terminateAfter);
	printf("XML - dumpCounters %s\n", smp->dumpCounters);

	printf("XML - sm_mc_config.mcast_mlid_table_cap %u\n", (unsigned int)mcp->mcast_mlid_table_cap);
	printf("XML - sm_mc_config.disable_mcast_check %u\n", (unsigned int)mcp->disable_mcast_check);
	printf("XML - sm_mc_config.enable_pruning %u\n", (unsigned int)mcp->enable_pruning);
	printf("XML - sm_mc_config.mcroot_select_algorithm %s\n", mcp->mcroot_select_algorithm);
	printf("XML - sm_mc_config.mcroot_min_cost_improvement %s\n", mcp->mcroot_min_cost_improvement);

	cs_strlcpy(str, "dynamicPlt", sizeof(str));
	for (i = 0; i < DYNAMIC_PACKET_LIFETIME_ARRAY_SIZE; i++) {
		printf("XML - %s %u\n", str, (unsigned int)dplp->dp_lifetime[i]);
		snprintf(str, 32, "dynamicPlt_%.2u", (unsigned int)(i + 1));
	}

	for (i = 0; i < MAX_SUPPORTED_MCAST_GRP_CLASSES; ++i) {
		snprintf(str, 32, "mcastGrpMGidLimitMask_%u", (unsigned int)i);
		printf("XML - %s %s\n", str, mlsp->mcastMlid[i].mcastGrpMGidLimitMaskConvert.value);
		snprintf(str, 32, "mcastGrpMGidLimitValue_%u", (unsigned int)i);
		printf("XML - %s %s\n", str, mlsp->mcastMlid[i].mcastGrpMGidLimitValueConvert.value);
		snprintf(str, 32, "mcastGrpMGidLimitMax_%u", (unsigned int)i);
		printf("XML - %s %u\n", str, (unsigned int)mlsp->mcastMlid[i].mcastGrpMGidLimitMax);
	}
}

boolean fmCopyConfig(FMXmlConfig_t *dst, FMXmlConfig_t *src)
{
	if (!src || !dst)
		return 0;

	// Nothing to do if src and dst are the same.
	if (src == dst) return 1;

	*dst = *src;
	return 1;
}

boolean smDplCopyConfig(SMDPLXmlConfig_t *dst, SMDPLXmlConfig_t *src)
{
	if (!src || !dst)
		return 0;

	// Nothing to do if src and dst are the same.
	if (src == dst) return 1;

	*dst = *src;
	return 1;
}

boolean smMcCopyConfig(SMMcastConfig_t *dst, SMMcastConfig_t *src)
{
	if (!src || !dst)
		return 0;

	// Nothing to do if src and dst are the same.
	if (src == dst) return 1;

	*dst = *src;
	return 1;
}

boolean smMlsCopyConfig(SmMcastMlidShare_t *dst, SmMcastMlidShare_t *src)
{
	if (!src || !dst)
		return 0;

	// Nothing to do if src and dst are the same.
	if (src == dst) return 1;

	*dst = *src;
	return 1;
}

boolean smMdgCopyConfig(SMMcastDefGrpCfg_t *dst, SMMcastDefGrpCfg_t *src)
{
	if (!src || !dst)
		return 0;

	// Nothing to do if src and dst are the same.
	if (src == dst) return 1;

	*dst = *src;
	return 1;
}

static
void vfInitConfig(VFXmlConfig_t *vf)
{
	memset(vf->vf, 0, sizeof(vf->vf));
	vf->number_of_vfs = 0;
}

static
boolean vfCopyConfig(VFXmlConfig_t *dst, VFXmlConfig_t *src)
{
	int i;

	if (!src || !dst)
		return 0;

	// Nothing to do if src and dst are the same.
	if (src == dst) return 1;

	*dst = *src;
	memset(dst->vf, 0, sizeof(dst->vf));
	dst->number_of_vfs = 0;

	for (i = 0; i < src->number_of_vfs; i++) {
		if (NULL != (dst->vf[i] = getVfObject())) {
			*dst->vf[i] = *src->vf[i];
			dst->number_of_vfs++;
		}
	}
	if (dst->number_of_vfs == src->number_of_vfs)
		return 1;

	return 0;
}

void dgInitConfig(DGXmlConfig_t *dg)
{
	memset(dg->dg, 0, sizeof(dg->dg));
	dg->number_of_dgs = 0;
}

boolean dgCopyConfig(DGXmlConfig_t *dst, DGXmlConfig_t *src)
{
	int i;

	if (!src || !dst)
		return 0;

	// Nothing to do if src and dst are the same.
	if (src == dst) return 1;

	*dst = *src;
	memset(dst->dg, 0, sizeof(dst->dg));
	dst->number_of_dgs = 0;
	for (i = 0; i < src->number_of_dgs; i++) {
		if (NULL == (dst->dg[i] = getGroupObject())) break;
		if (cloneGroup(src->dg[i], dst->dg[i], 0)) break;
		dst->number_of_dgs++;
	}
	if (dst->number_of_dgs == src->number_of_dgs) return 1;
	else return 0;
}

boolean appCopyConfig(AppXmlConfig_t *dst, AppXmlConfig_t *src)
{
	if (!src || !dst)
		return 0;

	// Nothing to do if src and dst are the same.
	if (src == dst) return 1;

	*dst = *src;
	cl_qmap_init(&dst->appMap, compareName);
	dst->appMapSize = cloneMap(&dst->appMap, &src->appMap, compareName, dupApplicationObject);
	return (dst->appMapSize == src->appMapSize);
}

boolean pmCopyConfig(PMXmlConfig_t *dst, PMXmlConfig_t *src)
{
	if (!src || !dst)
		return 0;

	// Nothing to do if src and dst are the same.
	if (src == dst) return 1;

	*dst = *src;
	return 1;
}

boolean feCopyConfig(FEXmlConfig_t *dst, FEXmlConfig_t *src)
{
	if (!src || !dst)
		return 0;

	// Nothing to do if src and dst are the same.
	if (src == dst) return 1;

	*dst = *src;
	return 1;
}


// initialize XML data structures
static
boolean cloneFmInstance(FMXmlInstance_t *dst, FMXmlInstance_t *src)
{
	return fmCopyConfig(&dst->fm_config, &src->fm_config) &&
		smCopyConfig(&dst->sm_config, &src->sm_config) &&
		smDplCopyConfig(&dst->sm_dpl_config, &src->sm_dpl_config) &&
		smMcCopyConfig(&dst->sm_mc_config, &src->sm_mc_config) &&
		smMlsCopyConfig(&dst->sm_mls_config, &src->sm_mls_config) &&
		smMdgCopyConfig(&dst->sm_mdg_config, &src->sm_mdg_config) &&
		vfCopyConfig(&dst->vf_config, &src->vf_config) &&
		dgCopyConfig(&dst->dg_config, &src->dg_config) &&
		appCopyConfig(&dst->app_config, &src->app_config) &&
		pmCopyConfig(&dst->pm_config, &src->pm_config) &&
		feCopyConfig(&dst->fe_config, &src->fe_config)

		;
}

// initialize XML data structures
void xmlInitConfigInstance(FMXmlInstance_t *instance)
{
	uint32_t i;

	// set all fields to 0xFF
	memset(instance, UNDEFINED_XML8, sizeof(FMXmlInstance_t));

	// clear the number of objects
	instance->vf_config.number_of_vfs = 0;
	instance->dg_config.number_of_dgs = 0;

	// clear all pointers to VirtualFabrics
	memset(instance->vf_config.vf, 0, sizeof(instance->vf_config.vf));


	// clear all pointers to VirtualFabric Groups
	memset(instance->dg_config.dg, 0, sizeof(instance->dg_config.dg));

	// init the app map
	cl_qmap_init(&instance->app_config.appMap, compareName);
	instance->app_config.appMapSize = 0;

	// clear all PmPortGroups
	memset(instance->pm_config.pm_portgroups, 0, sizeof(instance->pm_config.pm_portgroups));

	// Init Fm configuration
	fmClearConfig(&instance->fm_config);

	// Init Sm configuration
    smClearConfig(&instance->sm_config);

	memset(instance->sm_mc_config.mcroot_select_algorithm, 0, sizeof(instance->sm_mc_config.mcroot_select_algorithm));
	memset(instance->sm_mc_config.mcroot_min_cost_improvement, 0, sizeof(instance->sm_mc_config.mcroot_min_cost_improvement));

	// init Multicast MLIDShared values
	instance->sm_mls_config.number_of_shared = 0;
	for (i = 0; i < MAX_SUPPORTED_MCAST_GRP_CLASSES_XML; i++) {
		instance->sm_mls_config.mcastMlid[i].enable = 0;
		instance->sm_mls_config.mcastMlid[i].mcastGrpMGidLimitMax = 0;
		sprintf(&instance->sm_mls_config.mcastMlid[i].mcastGrpMGidLimitMaskConvert.value[0], "00000000000000000000000000000000");
		sprintf(&instance->sm_mls_config.mcastMlid[i].mcastGrpMGidLimitValueConvert.value[0], "00000000000000000000000000000000");
	}

	// init Default Group values
	instance->sm_mdg_config.number_of_groups = 0;
	for (i = 0; i < MAX_DEFAULT_GROUPS; i++) {
		mdgp = &instance->sm_mdg_config.group[i];
		memset(mdgp->virtual_fabric, 0, sizeof(mdgp->virtual_fabric));
		memset(mdgp->mgid, 0, sizeof(mdgp->mgid));
		memset(mdgp->mgid_range, 0, sizeof(mdgp->mgid_range));
		memset(mdgp->mgid_masked, 0, sizeof(mdgp->mgid_masked));
	}

	// Init Pm configuration
	pmClearConfig(&instance->pm_config);

	// Init Fe configuration
	feClearConfig(&instance->fe_config);

}

// initialize XML data structures
void xmlInitConfig(void)
{
	// zero instance
	instance = 0;

	common = 1;
	vfInstance = 0;
	appInstance = 0;
	groupInstance = 0;
	fullMemInstance = 0;
	limitedMemInstance = 0;
	systemImageInstance = 0;
	nodeGuidInstance = 0;
	portGuidInstance = 0;
	nodeDescInstance = 0;
	includedGroupInstance = 0;
	serviceIdInstance = 0;
	serviceIdRangeInstance = 0;
	serviceIdMaskedInstance = 0;
	mgidInstance = 0;
	mgidRangeInstance = 0;
	mgidMaskedInstance = 0;
	dgMgidInstance = 0;
	dgMgidRangeInstance = 0;
	dgMgidMaskedInstance = 0;
	includedAppInstance = 0;
	defaultGroupInstance = 0;
	mlidSharedInstance = 0;
	PmPgInstance = 0;
	PmPgMonitorInstance = 0;


	last_node_description = NULL;
	last_included_group = NULL;

#ifdef XML_DEBUG
	xml_vf_debug = 1;
	xml_sm_debug = 1;
	xml_fe_debug = 1;
	xml_pm_debug = 1;
	xml_parse_debug = 1;
#else
	xml_vf_debug = 0;
	xml_sm_debug = 0;
	xml_fe_debug = 0;
	xml_pm_debug = 0;
	xml_parse_debug = 0;
#endif

	if (!configp->fm_instance_common) {
		fprintf(stdout, "XML parse error - configp->fm_instance_common is NULL\n");
		return;
	}

	// clear/set debug info
	memset(&configp->xmlDebug, 0, sizeof(XmlDebug_t));
	configp->xmlDebug.xml_vf_debug = xml_vf_debug;
	configp->xmlDebug.xml_sm_debug = xml_sm_debug;
	configp->xmlDebug.xml_fe_debug = xml_fe_debug;
	configp->xmlDebug.xml_pm_debug = xml_pm_debug;
	configp->xmlDebug.xml_parse_debug = xml_parse_debug;

	xmlInitConfigInstance(configp->fm_instance_common);

	// initialize the empty multicast group structure
	mdgEmpty = configp->fm_instance_common->sm_mdg_config.group[0];
}

static
void releaseFMXmlInstance(FMXmlInstance_t *inst, int freeInst)
{
	uint32_t i;
	if (!inst)
		return;

	for (i = 0; i < MAX_CONFIGURED_VFABRICS; i++) {
		if (!inst->vf_config.vf[i])
			break;
		freeXmlMemory(inst->vf_config.vf[i], sizeof(VFConfig_t), __func__);
		inst->vf_config.vf[i] = NULL;
	}
	for (i = 0; i < MAX_VFABRIC_GROUPS; i++) {
		if (!inst->dg_config.dg[i])
			break;
		freeGroupObject(inst->dg_config.dg[i], /* full */ 1);
		inst->dg_config.dg[i] = NULL;
	}

	freeAppConfigMap(&inst->app_config);


	if (freeInst)
		freeXmlMemory(inst, sizeof(FMXmlInstance_t), __func__);
}

// release all memory in XML Configuration
void releaseXmlConfig(FMXmlCompositeConfig_t *config, uint32_t full)
{
	uint32_t fm;

	if (xml_parse_debug)
		fprintf(stdout, "Releasing XML config in releaseXmlConfig()\n");

	if (!config)
		return;

	// free common XML objects
	if (config->fm_instance_common && full_parse) {
		releaseFMXmlInstance(config->fm_instance_common, 1);
	}
	config->fm_instance_common = NULL;

	// free specific XML objects
	for (fm = fm_instance; fm < end_instance; fm++) {
		if (!config->fm_instance[fm])
			continue;

		releaseFMXmlInstance(config->fm_instance[fm], full);
		if (full)
			config->fm_instance[fm] = NULL;
	}

	if (full) {
		freeXmlMemory(config, sizeof(FMXmlCompositeConfig_t), "FMXmlCompositeConfig_t releaseXmlConfig()");
		if (config == configp)
			configp = NULL;
	}

	// print memory info
	if (xml_memory_debug)
		fprintf(stdout, "Memory level %u full %u after releaseXmlConfig()\n", (unsigned int)memory, (unsigned int)full);
}

// get VF object pointer and initialize data
static VFConfig_t* getVfObject(void)
{
	VFConfig_t *vfp;

	vfp = getXmlMemory(sizeof(VFConfig_t), "VFConfig_t getVfObject()");
	if (vfp == NULL)
		return NULL;

	// set all fields to 0xFF
	memset(vfp, UNDEFINED_XML8, sizeof(VFConfig_t));

	// clear all string values in VirtualFabrics
	memset(vfp->name, 0, sizeof(vfp->name));
	memset(vfp->full_member, 0, sizeof(vfp->full_member));
	memset(vfp->limited_member, 0, sizeof(vfp->limited_member));
	memset(vfp->application, 0, sizeof(vfp->application));

	// make sure it is disabled
	vfp->enable = 0;
	vfp->standby = 0;

	return vfp;
}


// get Group object pointer and initialize data
PmPortGroupXmlConfig_t* getPmPgObject(void)
{
	PmPortGroupXmlConfig_t *pmpgp;
	int i;

	pmpgp = getXmlMemory(sizeof(PmPortGroupXmlConfig_t), "PmPortGroupXmlConfig_t getPmPgObject()");
	if (pmpgp == NULL)
		return NULL;

	// set all fields to NULL
	memset(pmpgp, 0, sizeof(PmPortGroupXmlConfig_t));
	for(i =0; i< STL_PM_MAX_DG_PER_PMPG; i++)
		 pmpgp->Monitors[i].dg_Index = UNDEFINED_XML16;

	return pmpgp;
}

// get Group object pointer and initialize data
static DGConfig_t* getGroupObject(void)
{
	DGConfig_t *dgp;

	dgp = getXmlMemory(sizeof(DGConfig_t), "DGConfig_t getGroupObject()");
	if (dgp == NULL)
		return NULL;

	// set all fields to 0xFF
	memset(dgp, UNDEFINED_XML8, sizeof(DGConfig_t));

	// clear all string values in VirtulFabric Groups
	memset(dgp->name, 0, sizeof(dgp->name));

	// clear device pointers
	cl_qmap_init(&dgp->system_image_guid, NULL);
	cl_qmap_init(&dgp->node_guid, NULL);
	cl_qmap_init(&dgp->port_guid, NULL);
	dgp->node_description = NULL;
	dgp->reg_expr = NULL;
	dgp->included_group = NULL;

	// default flags
	dgp->select_all = 0;
	dgp->select_self = 0;
	dgp->select_hfi_direct_connect = 0;
	dgp->select_swe0 = 0;
	dgp->select_all_mgmt_allowed = 0;
	dgp->select_all_tfis = 0;
	dgp->node_type_fi = 0;
	dgp->node_type_sw = 0;

	return dgp;
}

// free Group object
static void freeGroupObject(DGConfig_t *group, uint8_t full)
{
	XmlNode_t *node;
	XmlNode_t *lastNode;
	RegExp_t  *regEx;
	RegExp_t  *lastRegEx;
	XmlIncGroup_t *incGroup;
	XmlIncGroup_t *lastIncGroup;

	scrubMap(&group->system_image_guid, NULL);
	scrubMap(&group->node_guid, NULL);
	scrubMap(&group->port_guid, NULL);

	node = group->node_description;
	while(node != NULL) {
		lastNode = node->next;
		freeXmlMemory(node, sizeof(XmlNode_t), "XmlNode_t freeGroup()");
		node = lastNode;
	}
	regEx = group->reg_expr;
	while(regEx != NULL) {
		lastRegEx = regEx->next;
		freeXmlMemory(regEx, sizeof(RegExp_t), "RegExp_t freeGroup()");
		regEx = lastRegEx;
	}
	incGroup = group->included_group;
	while(incGroup != NULL) {
		lastIncGroup = incGroup->next;
		freeXmlMemory(incGroup, sizeof(XmlIncGroup_t), "XmlIncGroup_t freeGroup()");
		incGroup = lastIncGroup;
	}

	// if full free then free group object too - otherwise clear
	if (full)
		freeXmlMemory(group, sizeof(DGConfig_t), "DGConfig_t freeGroup()");
	else
		memset(group, 0, sizeof(DGConfig_t));
}

// get Application object pointer and initialize data
AppConfig_t* getApplicationObject(void)
{
	AppConfig_t *app;

	app = getXmlMemory(sizeof(AppConfig_t), "AppConfig_t getApplicationObject()");
	if (app == NULL)
		return NULL;

	// set all fields to 0xFF
	memset(app, UNDEFINED_XML8, sizeof(AppConfig_t));

	cl_qmap_init(&app->serviceIdMap, NULL);
	app->serviceIdMapSize = 0;

	cl_qmap_init(&app->serviceIdRangeMap, compareName);
	app->serviceIdRangeMapSize = 0;

	cl_qmap_init(&app->serviceIdMaskedMap, compareName);
	app->serviceIdMaskedMapSize = 0;

	// clear all string values in VirtulFabric Applications
	memset(app->name, 0, sizeof(app->name));
	memset(app->mgid, 0, sizeof(app->mgid));
	memset(app->mgid_range, 0, sizeof(app->mgid_range));
	memset(app->mgid_masked, 0, sizeof(app->mgid_masked));
	memset(app->included_app, 0, sizeof(app->included_app));

	// default flags
	app->select_sa = 0;
	app->select_unmatched_sid = 0;
	app->select_unmatched_mgid = 0;
	app->select_pm = 0;


	return app;
}

// get Application object pointer and initialize data
void freeApplicationObject(AppConfig_t *app)
{
	scrubMap(&app->serviceIdMap, NULL);
	app->serviceIdMapSize = 0;

	scrubMap(&app->serviceIdRangeMap, freeName);
	app->serviceIdRangeMapSize = 0;

	scrubMap(&app->serviceIdMaskedMap, freeName);
	app->serviceIdMaskedMapSize = 0;


	freeXmlMemory(app, sizeof(AppConfig_t), "AppConfig_t freeApplicationObject()");
}

// Copy Application object
void cloneApplicationObject(AppConfig_t *dst, AppConfig_t *src)
{
	if (!src || !dst || (src == dst)) return;

	// Copy everything first, then handle the qmap
	*dst = *src;
	dst->serviceIdMapSize = cloneMap(&dst->serviceIdMap, &src->serviceIdMap, NULL, NULL);
	dst->serviceIdRangeMapSize = cloneMap(&dst->serviceIdRangeMap, &src->serviceIdRangeMap, compareName, dupName);
	dst->serviceIdMaskedMapSize = cloneMap(&dst->serviceIdMaskedMap, &src->serviceIdMaskedMap, compareName, dupName);
	return;
}

// Duplicate Application object
static AppConfig_t *dupApplicationObject(AppConfig_t *obj)
{
	AppConfig_t *new_obj;

	new_obj = getApplicationObject();
	if (!new_obj) return NULL;

	cloneApplicationObject(new_obj, obj);
	return new_obj;
}

// copy of Group Device Nodes
static int8_t cloneGroupDeviceNodes(XmlNode_t *source, XmlNode_t **dest)
{
	XmlNode_t *lastNode = NULL;
	XmlNode_t *newNode = NULL;
	XmlNode_t *nodePtr;

	if (source == NULL || dest == NULL)
		return 0;

	nodePtr = source;

	while(nodePtr != NULL) {
		newNode = getXmlMemory(sizeof(XmlNode_t), "XmlNode_t cloneGroupDeviceNodes()");
		if (newNode == NULL)
			return -1;
		cs_strlcpy(newNode->node, nodePtr->node, MAX_VFABRIC_NAME);
		newNode->next = NULL;
		if (lastNode == NULL)
			*dest = newNode;
		else
			lastNode->next = newNode;
		lastNode = newNode;
		nodePtr = nodePtr->next;
	}
	return 0;
}

int8_t cloneGroupRegExpr(RegExp_t *source, RegExp_t **dest)
{
	RegExp_t *lastRegExpr = NULL;
	RegExp_t *newRegExpr = NULL;
	RegExp_t *regExprPtr;

	if (source == NULL || dest == NULL)
		return 0;

	regExprPtr = source;

	while(regExprPtr != NULL) {
		newRegExpr = getXmlMemory(sizeof(RegExp_t), "RegExp_t cloneGroupRegExpr()");
		if (newRegExpr == NULL)
			return -1;

		memcpy(newRegExpr, regExprPtr, sizeof(RegExp_t));

		newRegExpr->next = NULL;
		if (lastRegExpr == NULL)
			*dest = newRegExpr;
		else
			lastRegExpr->next = newRegExpr;
		lastRegExpr = newRegExpr;
		regExprPtr = regExprPtr->next;
	}
	return 0;
}


// copy of Group Included Groups
int8_t cloneGroupIncGroups(XmlIncGroup_t *source, XmlIncGroup_t **dest)
{
	XmlIncGroup_t *lastIncGroup = NULL;
	XmlIncGroup_t *newIncGroup = NULL;
	XmlIncGroup_t *incGroupPtr;

	if (source == NULL || dest == NULL)
		return 0;

	incGroupPtr = source;

	while(incGroupPtr != NULL) {
		newIncGroup = getXmlMemory(sizeof(XmlIncGroup_t), "XmlIncGroup_t cloneGroupIncGroups()");
		if (newIncGroup == NULL)
			return -1;
		cs_strlcpy(newIncGroup->group, incGroupPtr->group, MAX_VFABRIC_NAME);
		newIncGroup->next = NULL;
		if (lastIncGroup == NULL)
			*dest = newIncGroup;
		else
			lastIncGroup->next = newIncGroup;
		lastIncGroup = newIncGroup;
		incGroupPtr = incGroupPtr->next;
	}
	return 0;
}

// copy Group
static int8_t cloneGroup(DGConfig_t *source, DGConfig_t *dest, int freeObjectBeforeCopy)
{
	if (freeObjectBeforeCopy) {
		// first free devices from destination group if there are any
		freeGroupObject(dest, /* full */ 0);
	}

	// copy other info but null the pointers
	*dest = *source;

	dest->node_description = NULL;
	dest->reg_expr = NULL;
	dest->included_group = NULL;

	dest->number_of_system_image_guids=cloneMap(&dest->system_image_guid, &source->system_image_guid, NULL, NULL);
	dest->number_of_node_guids=cloneMap(&dest->node_guid, &source->node_guid, NULL, NULL);
	dest->number_of_port_guids=cloneMap(&dest->port_guid, &source->port_guid, NULL, NULL);

	if (cloneGroupDeviceNodes(source->node_description, &dest->node_description) < 0)
		return -1;
	if (cloneGroupRegExpr(source->reg_expr, &dest->reg_expr) < 0)
		return -1;
	if (cloneGroupIncGroups(source->included_group, &dest->included_group) < 0)
		return -1;

	return 0;
}

// copy APP DG and VF Group Info
int8_t copyDgVfInfo(FMXmlInstance_t *instance, DGXmlConfig_t *dg, VFXmlConfig_t *vf)
{
	vfInitConfig(vf);
	dgInitConfig(dg);

	if (vfCopyConfig(vf, &instance->vf_config)
	&& dgCopyConfig(dg, &instance->dg_config)) return 0;
	else return 1;
}

// verify and convert an SID Range or Masked value into an VFAppSid_t structure
int verifyAndConvertSidCompoundString(char *sidString, uint8_t range, VFAppSid_t *sidMapping)
{
	uint64 sidLower;
	uint64 sidUpper;
	char *operator = NULL;

	if (!sidString || !sidMapping) {
		if (xml_parse_debug)
			fprintf(stdout, "NULL pointer sent to verifyAndConvertSidCompoundString()\n");
		return -1;
	}

	if (FSUCCESS != StringToUint64(&sidLower, sidString, &operator, 16, TRUE)) {
		if (xml_parse_debug)
			fprintf(stdout, "Illegal ServiceID length found in hex string %s in verifyAndConvertSidCompoundString\n", sidString);
		return -1;
	}

	if (range) {
		if (*operator != '-') {
			if (xml_parse_debug)
				fprintf(stdout, "Illegal range operator found in hex string %s in verifyAndConvertSidCompoundString\n", sidString);
			return -1;
		}
	} else {
		if (*operator != '*') {
			if (xml_parse_debug)
				fprintf(stdout, "Illegal mask operator found in hex string %s in verifyAndConvertSidCompoundString\n", sidString);
			return -1;
		}
	}

	if (FSUCCESS != StringToUint64(&sidUpper, operator+1, NULL, 16, TRUE)) {
		if (xml_parse_debug)
			fprintf(stdout, "Illegal ServiceID length found in hex string %s in verifyAndConvertSidCompoundString\n", sidString);
		return -1;
	}

#ifdef XML_DEBUG
/*
		fprintf(stdout, "sidLower 0x%llx sidUpper 0x%llx\n", sidLower, sidUpper);
*/
#endif

	sidMapping->service_id = sidLower;

	if (range) {
		sidMapping->service_id_last = sidUpper;
		sidMapping->service_id_mask = UNDEFINED_XML64;
	} else {
		sidMapping->service_id_mask = sidUpper;
		sidMapping->service_id_last = 0;
	}
	return 0;
}

// verify and convert an MGID value into an VFAppMgid_t structure
int verifyAndConvertMGidString(char *mgidString, VFAppMgid_t *mgidMapping)
{
	uint64 mgid[2];

	if (!mgidString || !mgidMapping) {
		if (xml_parse_debug)
			fprintf(stdout, "NULL pointer sent to verifyAndConvertMGidString()\n");
		return -1;
	}

	if (FSUCCESS != StringToGid(&mgid[0], &mgid[1], mgidString, NULL, TRUE)) {
		if (xml_parse_debug) {
			fprintf(stdout, "Illegal MGID found in hex string %s\n", mgidString);
		}
		return -1;
	}

#ifdef XML_DEBUG
/*
		fprintf(stdout, "MGID input string %s mgid[0] 0x%llx mgid[1] 0x%llx\n", mgidString, mgid[0], mgid[1]);
*/
#endif

	mgidMapping->mgid[0] = mgid[0];
	mgidMapping->mgid[1] = mgid[1];
	mgidMapping->mgid_last[0] = 0;
	mgidMapping->mgid_last[1] = 0;
	mgidMapping->mgid_mask[0] = UNDEFINED_XML64;
	mgidMapping->mgid_mask[1] = UNDEFINED_XML64;

	return 0;
}

// verify and convert an MGID Range or Masked value into an VFAppMgid_t structure
int verifyAndConvertMGidCompoundString(char *mgidString, uint8_t range, VFAppMgid_t *mgidMapping)
{
	uint64 mgidLower[2];
	uint64 mgidUpper[2];
	char *operator = NULL;

	if (!mgidString || !mgidMapping) {
		if (xml_parse_debug)
			fprintf(stdout, "NULL pointer sent to verifyAndConvertMGidCompoundString()\n");
		return -1;
	}

	if (FSUCCESS != StringToGid(&mgidLower[0], &mgidLower[1], mgidString, &operator, TRUE)) {
		if (xml_parse_debug)
			fprintf(stdout, "Illegal MGID found in hex string %s in verifyAndConvertMGidCompoundString\n", mgidString);
		return -1;
	}

	if (range) {
		if (*operator != '-') {
			if (xml_parse_debug)
				fprintf(stdout, "Illegal range operator found in hex string %s in verifyAndConvertMGidCompoundString\n", mgidString);
			return -1;
		}
	} else {
		if (*operator != '*') {
			if (xml_parse_debug)
				fprintf(stdout, "Illegal mask operator found in hex string %s in verifyAndConvertMGidCompoundString\n", mgidString);
			return -1;
		}
	}
	if (FSUCCESS != StringToGid(&mgidUpper[0], &mgidUpper[1], operator+1, NULL, TRUE)) {
		if (xml_parse_debug)
			fprintf(stdout, "Illegal MGID found in hex string %s in verifyAndConvertMGidCompoundString\n", mgidString);
		return -1;
	}

#ifdef XML_DEBUG
/*
		fprintf(stdout, "mgidLower[0] 0x%llx mgidLower[1] 0x%llx mgidUpper[0] 0x%llx mgidUpper[1] 0x%llx\n",
			mgidLower[0], mgidLower[1], mgidUpper[0], mgidUpper[1]);
*/
#endif

	mgidMapping->mgid[0] = mgidLower[0];
	mgidMapping->mgid[1] = mgidLower[1];

	if (range) {
		mgidMapping->mgid_last[0] = mgidUpper[0];
		mgidMapping->mgid_last[1] = mgidUpper[1];
		mgidMapping->mgid_mask[0] = UNDEFINED_XML64;
		mgidMapping->mgid_mask[1] = UNDEFINED_XML64;
	} else {
		mgidMapping->mgid_mask[0] = mgidUpper[0];
		mgidMapping->mgid_mask[1] = mgidUpper[1];
		mgidMapping->mgid_last[0] = 0;
		mgidMapping->mgid_last[1] = 0;
	}
	return 0;
}

void scrubVF(VF_t *vfp)
{
	VFDg_t						*dgp;

	// free Application objects
	scrubMap(&vfp->apps.sidMap, freeAppSid);
	scrubMap(&vfp->apps.mgidMap, freeAppMgid);

	// free Member objects
	scrubMap(&vfp->full_members.sysGuidMap, NULL);
	scrubMap(&vfp->full_members.nodeGuidMap, NULL);
	scrubMap(&vfp->full_members.portGuidMap, NULL);
	scrubMap(&vfp->full_members.nodeDescMap, freeName);

	// free LimitedMember objects
	scrubMap(&vfp->limited_members.sysGuidMap, NULL);
	scrubMap(&vfp->limited_members.nodeGuidMap, NULL);
	scrubMap(&vfp->limited_members.portGuidMap, NULL);
	scrubMap(&vfp->limited_members.nodeDescMap, freeName);

	// free default group and associated MGID list
	while (vfp->default_group) {
		dgp = vfp->default_group->next_default_group;
		freeDMCG(vfp->default_group);
		vfp->default_group = dgp;
	}
}

// release all memory in Virtual Fabric Configuration in firmware
void releaseVirtualFabricsConfig(VirtualFabrics_t *vfsip)
{
	VF_t						*vfp;
	uint32_t 					i;

	if (xml_parse_debug)
		fprintf(stdout, "Releasing Virtual Fabrics config in releaseVirtualFabricsConfig()\n");

	if (!vfsip)
		return;

	for (i = 0; i < vfsip->number_of_vfs_all && i < MAX_ENABLED_VFABRICS; i++) {
		vfp = &vfsip->v_fabric_all[i];

		scrubVF(vfp);
	}

	// finally free the parent object
	freeXmlMemory(vfsip, sizeof(VirtualFabrics_t),
		"VirtualFabrics_t releaseVirtualFabricsConfig()");

	// print memory info
	if (xml_memory_debug)
		fprintf(stdout, "Memory level %u after releaseVirtualFabricsConfig()\n", (unsigned int)memory);
}

// create a checksum for one Virtual Fabric
void checksumOneVirtualFabricsConfig(VF_t *vfp, SMXmlConfig_t *smp)
{
	VFAppSid_t					*sidp;
	VFAppMgid_t					*mgidp;
	VFDg_t						*dgp;
	uint64_t					guid;
	char						*name;
	cl_map_item_t				*cl_map_item;

	if (!vfp)
		return;

	CKSUM_DATA(vfp->name, CKSUM_OVERALL_DISRUPT_CONSIST);
	CKSUM_DATA(vfp->index, CKSUM_OVERALL_DISRUPT_CONSIST);
	CKSUM_DATA(vfp->pkey, CKSUM_OVERALL_DISRUPT_CONSIST);
	CKSUM_DATA(vfp->standby, CKSUM_OVERALL_CONSIST);
	CKSUM_DATA(vfp->security, CKSUM_OVERALL_DISRUPT_CONSIST);
	CKSUM_DATA(vfp->max_mtu_int, CKSUM_OVERALL_DISRUPT_CONSIST);
	CKSUM_DATA(vfp->max_mtu_specified, CKSUM_OVERALL_DISRUPT_CONSIST);
	CKSUM_DATA(vfp->max_rate_int, CKSUM_OVERALL_DISRUPT_CONSIST);
	CKSUM_DATA(vfp->max_rate_specified, CKSUM_OVERALL_DISRUPT_CONSIST);
	CKSUM_DATA(vfp->qos_enable, CKSUM_OVERALL_DISRUPT_CONSIST);
	if (vfp->qos_enable) {
		CKSUM_DATA(vfp->base_sl, CKSUM_OVERALL_DISRUPT_CONSIST);
		CKSUM_DATA(vfp->requires_resp_sl, CKSUM_OVERALL_DISRUPT_CONSIST);
		CKSUM_DATA(vfp->resp_sl, CKSUM_OVERALL_DISRUPT_CONSIST);

		CKSUM_DATA(vfp->contains_mcast, CKSUM_OVERALL_DISRUPT_CONSIST);
		CKSUM_DATA(vfp->mcast_sl, CKSUM_OVERALL_DISRUPT_CONSIST);

		if (!vfp->standby)
			CKSUM_DATA(vfp->percent_bandwidth, CKSUM_OVERALL_CONSIST);

		CKSUM_DATA(vfp->flowControlDisable, CKSUM_OVERALL_DISRUPT_CONSIST);
		CKSUM_DATA(vfp->priority, CKSUM_OVERALL_DISRUPT_CONSIST);
		CKSUM_DATA(vfp->pkt_lifetime_mult, CKSUM_OVERALL_DISRUPT_CONSIST);
		CKSUM_DATA(vfp->pkt_lifetime_specified, CKSUM_OVERALL_DISRUPT_CONSIST);
	}
	for_all_qmap_ptr(&vfp->apps.sidMap, cl_map_item, sidp) {
   		CKSUM_DATA(*sidp, CKSUM_OVERALL_DISRUPT_CONSIST);
	}
	for_all_qmap_ptr(&vfp->apps.mgidMap, cl_map_item, mgidp) {
   		CKSUM_DATA(*mgidp, CKSUM_OVERALL_DISRUPT_CONSIST);
	}
   	CKSUM_DATA(vfp->apps.select_sa, CKSUM_OVERALL_DISRUPT_CONSIST);
	CKSUM_DATA(vfp->apps.select_unmatched_sid, CKSUM_OVERALL_DISRUPT_CONSIST);
	CKSUM_DATA(vfp->apps.select_unmatched_mgid, CKSUM_OVERALL_DISRUPT_CONSIST);
	CKSUM_DATA(vfp->apps.select_pm, CKSUM_OVERALL_DISRUPT_CONSIST);


	for_all_qmap_u64(&vfp->full_members.sysGuidMap, cl_map_item, guid) {
   		CKSUM_DATA(guid, CKSUM_OVERALL_DISRUPT_CONSIST);
	}
	for_all_qmap_u64(&vfp->full_members.nodeGuidMap, cl_map_item, guid) {
   		CKSUM_DATA(guid, CKSUM_OVERALL_DISRUPT_CONSIST);
	}
	for_all_qmap_u64(&vfp->full_members.portGuidMap, cl_map_item, guid) {
   		CKSUM_DATA(guid, CKSUM_OVERALL_DISRUPT_CONSIST);
	}
	for_all_qmap_ptr(&vfp->full_members.nodeDescMap, cl_map_item, name) {
   		CKSUM_STR(name, CKSUM_OVERALL_DISRUPT_CONSIST);
	}

	CKSUM_DATA(vfp->full_members.select_all, CKSUM_OVERALL_DISRUPT_CONSIST);
	CKSUM_DATA(vfp->full_members.select_self, CKSUM_OVERALL_DISRUPT_CONSIST);
	CKSUM_DATA(vfp->full_members.select_hfi_direct_connect, CKSUM_OVERALL_DISRUPT_CONSIST);
	CKSUM_DATA(vfp->full_members.select_swe0, CKSUM_OVERALL_DISRUPT_CONSIST);
	CKSUM_DATA(vfp->full_members.select_all_mgmt_allowed, CKSUM_OVERALL_DISRUPT_CONSIST);
	CKSUM_DATA(vfp->full_members.select_all_tfis, CKSUM_OVERALL_DISRUPT_CONSIST);
	CKSUM_DATA(vfp->full_members.node_type_fi, CKSUM_OVERALL_DISRUPT_CONSIST);
	CKSUM_DATA(vfp->full_members.node_type_sw, CKSUM_OVERALL_DISRUPT_CONSIST);

	for_all_qmap_u64(&vfp->limited_members.sysGuidMap, cl_map_item, guid) {
   		CKSUM_DATA(guid, CKSUM_OVERALL_DISRUPT_CONSIST);
	}
	for_all_qmap_u64(&vfp->limited_members.nodeGuidMap, cl_map_item, guid) {
   		CKSUM_DATA(guid, CKSUM_OVERALL_DISRUPT_CONSIST);
	}
	for_all_qmap_u64(&vfp->limited_members.portGuidMap, cl_map_item, guid) {
   		CKSUM_DATA(guid, CKSUM_OVERALL_DISRUPT_CONSIST);
	}
	for_all_qmap_ptr(&vfp->limited_members.nodeDescMap, cl_map_item, name) {
   		CKSUM_STR(name, CKSUM_OVERALL_DISRUPT_CONSIST);
	}

	CKSUM_DATA(vfp->limited_members.select_all, CKSUM_OVERALL_DISRUPT_CONSIST);
	CKSUM_DATA(vfp->limited_members.select_self, CKSUM_OVERALL_DISRUPT_CONSIST);
	CKSUM_DATA(vfp->limited_members.select_hfi_direct_connect, CKSUM_OVERALL_DISRUPT_CONSIST);
	CKSUM_DATA(vfp->limited_members.select_swe0, CKSUM_OVERALL_DISRUPT_CONSIST);
	CKSUM_DATA(vfp->limited_members.select_all_mgmt_allowed, CKSUM_OVERALL_DISRUPT_CONSIST);
	CKSUM_DATA(vfp->limited_members.select_all_tfis, CKSUM_OVERALL_DISRUPT_CONSIST);
	CKSUM_DATA(vfp->limited_members.node_type_fi, CKSUM_OVERALL_DISRUPT_CONSIST);
	CKSUM_DATA(vfp->limited_members.node_type_sw, CKSUM_OVERALL_DISRUPT_CONSIST);

	for (dgp = vfp->default_group; dgp; dgp = dgp->next_default_group) {
		CKSUM_DATA(dgp->def_mc_create, CKSUM_OVERALL_DISRUPT_CONSIST);
		CKSUM_DATA(dgp->def_mc_pkey, CKSUM_OVERALL_DISRUPT_CONSIST);
		CKSUM_DATA(dgp->def_mc_mtu_int, CKSUM_OVERALL_DISRUPT_CONSIST);
		CKSUM_DATA(dgp->def_mc_rate_int, CKSUM_OVERALL_DISRUPT_CONSIST);
		CKSUM_DATA(dgp->def_mc_sl, CKSUM_OVERALL_DISRUPT_CONSIST);
		CKSUM_DATA(dgp->def_mc_qkey, CKSUM_OVERALL_DISRUPT_CONSIST);
		CKSUM_DATA(dgp->def_mc_fl, CKSUM_OVERALL_DISRUPT_CONSIST);
		CKSUM_DATA(dgp->def_mc_tc, CKSUM_OVERALL_DISRUPT_CONSIST);
		for_all_qmap_ptr(&dgp->mgidMap, cl_map_item, mgidp) {
   			CKSUM_DATA(*mgidp, CKSUM_OVERALL_DISRUPT_CONSIST);
		}
	}

   	CKSUM_DATA(vfp->hoqlife_vf, CKSUM_OVERALL_DISRUPT_CONSIST);
   	CKSUM_DATA(vfp->preempt_rank, CKSUM_OVERALL_DISRUPT_CONSIST);

	return;
}

// create a total checksum for the Virtual Fabric database
void checksumVirtualFabricsConfig(VirtualFabrics_t *vfsip, SMXmlConfig_t *smp)
{
	VF_t						*vfp;
	uint32_t 					i;

	if (!vfsip)
		return;

	// Checksum each individual Virtual Fabric
	for (i = 0; i < vfsip->number_of_vfs_all; i++) {
		vfp = &vfsip->v_fabric_all[i];

		CKSUM_BEGIN;
		checksumOneVirtualFabricsConfig(vfp, smp);
		CKSUM_END(vfp->overall_checksum, vfp->disruptive_checksum, vfp->consistency_checksum);
	}

	// Checksum all of the Virtual Fabrics together

	CKSUM_BEGIN;

	for (i = 0; i < vfsip->number_of_vfs_all; i++) {
		vfp = &vfsip->v_fabric_all[i];

		checksumOneVirtualFabricsConfig(vfp, smp);
	}

	CKSUM_END(vfsip->overall_checksum, vfsip->disruptive_checksum, vfsip->consistency_checksum);

	return;
}

boolean validateDefaultVirtualFabric(uint32_t fm, FMXmlCompositeConfig_t *config, VFXmlConfig_t *vf, char *error)
{
	boolean default_pkey = 0;
	uint32_t i;

	for (i = 0; i < vf->number_of_vfs; i++) {
		if ((vf->vf[i]->pkey & ~0x8000) == STL_DEFAULT_PKEY) {
			// Default pkey
			default_pkey = 1;
			if (!vf->vf[i]->standby) {
				// found an enabled, active vf with default pkey
				return 1;
			}
		}
	}
	if (default_pkey)
		sprintf(error, "Must have at least one non-standby Virtual Fabric with Mgmt Pkey");
	return 0;
}

// find a pointer to a Group given the name
DGConfig_t* findGroupPointer(FMXmlCompositeConfig_t *config, char *group, uint32_t instance)
{
	DGConfig_t *dgp;
	uint32_t i;

	if (xml_vf_debug)
		fprintf(stdout, "findGroupPointer() group %s\n", group);

	for (i = 0; i < MAX_VFABRIC_GROUPS; i++) {
		dgp = config->fm_instance[instance]->dg_config.dg[i];
		if (dgp == NULL)
			break;
		if (strcmp(group, dgp->name) == 0)
			return dgp;
	}
	return NULL;
}

// find a pointer to a Application given the name
AppConfig_t* findAppPointer(FMXmlCompositeConfig_t *config, char *name, uint32_t instance)
{
	cl_map_item_t *app_item;

	if (xml_vf_debug)
		fprintf(stdout, "findAppPointer() app %s\n", name);

	app_item = cl_qmap_get(&config->fm_instance[instance]->app_config.appMap, XML_QMAP_U64_CAST name);
	if (app_item)
		return XML_QMAP_VOID_CAST cl_qmap_key(app_item);
	else
		return NULL;
}

// add to list of apps all included apps - this is a reccursive routine
int includedApps(FMXmlCompositeConfig_t *config, uint32_t fm, AppConfig_t **list, AppConfig_t *parent, uint32_t *index, char *error)
{
	AppConfig_t *included;

	uint32_t i;
	uint32_t ii;
	int32_t result;
	uint32_t skip = 0;

	if (!parent || !list) {
		fprintf(stdout, "Illegal call to includedApps()\n");
		return -1;
	}

	if (xml_vf_debug)
		fprintf(stdout, "includedApps() parent->name %s included apps %u index %u\n",
			parent->name, (unsigned int)parent->number_of_included_apps, (unsigned int)*index);

	for (i = 0; i < parent->number_of_included_apps; i++)
	{
		// if we are at the end of the list then return
		if (*index >= MAX_VFABRIC_APPS) {
			if (error)
				sprintf(error, "Maximum number of included Applications exceeded");
			return -1;
		}

		// if the parent does not have a name then parse error
		if (strlen(parent->name) == 0) {
			if (error)
				sprintf(error, "Parent Application encountered without a name");
			return -1;
		}

		// if included does not have a name then parse error
		if (strlen(parent->included_app[i].node) == 0) {
			if (error)
				sprintf(error, "Included Application encountered without a name");
			return -1;
		}

		// get a pointer to the included app
		included = findAppPointer(config, parent->included_app[i].node, fm);

		// if this app does not exist it is a parse error
		if (included == NULL) {
			if (error)
				sprintf(error, "Application (%s) does not exist", parent->included_app[i].node);
			return -1;
		}

	if (xml_vf_debug)
		fprintf(stdout, "includedApps() parent->name %s included name %s index %u\n",
			parent->name, parent->included_app[i].node, (unsigned int)*index);

		// make sure this pointer is not already on the list
		for (ii = 0; ii < MAX_VFABRIC_APPS; ii++) {
			if (list + ii == NULL)
				break;
			if (included == *(list + ii)) {
				if (!error)
					fprintf(stdout, "\nApplication (%s) should not be included more than once - ignoring\n", included->name);
				skip = 1;
			}
		}

		if (skip)
			continue;

		// save app pointer on app list
		list[*index] = included;
		(*index)++;

		// recursively call this again for this included app
		result = includedApps(config, fm, list, included, index, error);
		if (result < 0)
			return -1;
	}
	return 0;
}

// add to list of groups all included groups - this is a reccursive routine
int includedGroups(FMXmlCompositeConfig_t *config, uint32_t fm, DGConfig_t **list, DGConfig_t *parent, uint32_t *index, char *error)
{
	DGConfig_t *included;
	XmlIncGroup_t *groupPtr;

	uint32_t i;
	int32_t result;
	uint32_t skip = 0;

	if (!parent || !list) {
		fprintf(stdout, "Illegal call to includedGroups()\n");
		return -1;
	}

	if (xml_vf_debug)
		fprintf(stdout, "includedGroups() parent->name %s included groups %u index %u\n",
			parent->name, (unsigned int)parent->number_of_included_groups, (unsigned int)*index);

	// get pointer to first included group
	groupPtr = parent->included_group;

	while (groupPtr)
	{
		// if we are at the end of the list then return
		if (*index >= MAX_VFABRIC_GROUPS) {
			if (error)
				sprintf(error, "Maximum number of IncludeGroups exceeded");
			return -1;
		}

		// if the parent does not have a name then parse error
		if (strlen(parent->name) == 0) {
			if (error)
				sprintf(error, "Parent DeviceGroup encountered without a name");
			return -1;
		}

		// if included does not have a name then parse error
		if (strlen(groupPtr->group) == 0) {
			if (error)
				sprintf(error, "Included DeviceGroup encountered without a name");
			return -1;
		}

		// get a pointer to the included group
		included = findGroupPointer(config, groupPtr->group, fm);

		// if this group does not exist it is a parse error
		if (included == NULL) {
			if (error)
				sprintf(error, "DeviceGroup (%s) does not exist", groupPtr->group);
			return -1;
		}

		if (xml_vf_debug)
			fprintf(stdout, "includedGroups() parent->name %s included name %s index %u\n",
				parent->name, groupPtr->group, (unsigned int)*index);

		// make sure this pointer is not already on the list
		for (i = 0; i < MAX_VFABRIC_GROUPS; i++) {
			if (list + i == NULL)
				break;
			if (included == *(list + i)) {
				if (!error)
					fprintf(stdout, "\nDeviceGroup (%s) should not be included more than once - ignoring\n", included->name);
				skip = 1;
			}
		}

		if (skip) {
			groupPtr = groupPtr->next;
			continue;
		}

		// save group pointer on group list
		list[*index] = included;
		(*index)++;

		// recursively call this again for this included group
		result = includedGroups(config, fm, list, included, index, error);
		if (result < 0)
			return -1;

		groupPtr = groupPtr->next;
	}
	return 0;
}
// verify VFs are congruent
boolean VerifyCongruentVF(VF_t *v_fp1, VF_t *v_fp2, VFDg_t *mcp)
{
	uint8_t	vf_sl1 = UNDEFINED_XML8;
	uint8_t vf_sl2 = UNDEFINED_XML8;

	//checking PKey
	if ((v_fp1->pkey != UNDEFINED_XML32) && (v_fp2->pkey != UNDEFINED_XML32)
			&& ((v_fp1->pkey & 0x7fff) != (v_fp2->pkey & 0x7fff))){
		return FALSE;
	}
	//check MTU
	if ((GetBytesFromMtu(mcp->def_mc_mtu_int) > GetBytesFromMtu(v_fp1->max_mtu_int)) ||
			(GetBytesFromMtu(mcp->def_mc_mtu_int) > GetBytesFromMtu(v_fp2->max_mtu_int)))
		return FALSE;

	// check rate
	if ((IbStaticRateToMbps(mcp->def_mc_rate_int) > IbStaticRateToMbps(v_fp1->max_rate_int)) ||
		(IbStaticRateToMbps(mcp->def_mc_rate_int) > IbStaticRateToMbps(v_fp2->max_rate_int)))
		return FALSE;

	// Check the SL's are the same
	if ((v_fp1->qos_enable == 0) && (v_fp2->qos_enable ==0))
		return TRUE; // if both VF have QoS =0, the SLs will be equal.
					// otherwise it is not known in advance and the user must
					// specify them explicitly
	if (v_fp1->qos_enable != 0) {
		if (v_fp1->mcast_sl != UNDEFINED_XML8)
				vf_sl1 = v_fp1->mcast_sl;
		else {
			if (v_fp1->base_sl == UNDEFINED_XML8)
				return FALSE;
			else vf_sl1 = v_fp1->base_sl;
		}
	}

	if (v_fp2->qos_enable != 0) {
		if (v_fp2->mcast_sl != UNDEFINED_XML8)
				vf_sl2 = v_fp2->mcast_sl;
		else {
			if (v_fp2->base_sl == UNDEFINED_XML8)
				return FALSE;
			else vf_sl2 = v_fp2->base_sl;
		}
	}

	if (vf_sl1 != vf_sl2)
		return FALSE;
	else
		if ((vf_sl1 == UNDEFINED_XML8) && (vf_sl2 == UNDEFINED_XML8))
			return FALSE;
		else return TRUE;
} 


// check for duplicate MGID's within all DefaultGroups in all VirtualFabrics including itself
void checkDefaultGroupMGIDDuplicates(VirtualFabrics_t *vfsip, char *error, SMXmlConfig_t *smp)
{
	uint32_t vf1, vf2;
	VFDg_t *dg_ref;
	VFDg_t *dg_check;
	VFAppMgid_t *mgid_ref;
	VFAppMgid_t *mgid_check;
	cl_map_item_t *item1, *item2;

	// if no error checking then just return
	if (!error)
		return;

	// go to each MGID and make sure no other MGID is the same
	for (vf1 = 0; vf1 < vfsip->number_of_vfs_all; vf1++) {
		dg_ref = vfsip->v_fabric_all[vf1].default_group;
		while (dg_ref) {
			if (dg_ref->def_mc_create) {
				for_all_qmap_ptr(&dg_ref->mgidMap, item1, mgid_ref) {
					for (vf2=0; vf2 < vfsip->number_of_vfs_all; vf2++) {
						if (vf1 == vf2) {//verify this VF has no duplicate groups.
							dg_check = dg_ref->next_default_group;
							while (dg_check) {
								if (dg_check->def_mc_create) {
									for_all_qmap_ptr(&dg_check->mgidMap, item2, mgid_check) {
										//do not allow duplicates in the same VF
										if (mgid_ref->mgid[0] == mgid_check->mgid[0] && mgid_ref->mgid[1] == mgid_check->mgid[1]) {
											sprintf(error, "Duplicate MGID (0x%016"PRIx64":0x%016"PRIx64") in MulticastGroup definitions matching same VirtualFabric (%s)",
													mgid_ref->mgid[0], mgid_ref->mgid[1], vfsip->v_fabric_all[vf2].name);
											return;
										}
									}
								}
								dg_check = dg_check->next_default_group;
							}
						}
						else {
							dg_check = vfsip->v_fabric_all[vf2].default_group;
							while (dg_check) {
								if (dg_check->def_mc_create) {
									for_all_qmap_ptr(&dg_check->mgidMap, item2, mgid_check) {
										if (mgid_ref == mgid_check) continue;
										if (mgid_ref->mgid[0] == mgid_check->mgid[0] && mgid_ref->mgid[1] == mgid_check->mgid[1]) {
											if (!smp->enforceVFPathRecs) {
												//verify VFs are congruent, if so, allow duplicates, otherwise fail config
												if (!VerifyCongruentVF(&vfsip->v_fabric_all[vf1],&vfsip->v_fabric_all[vf2], dg_ref)) {
													sprintf(error, "Duplicate MGID (0x%016"PRIx64":0x%016"PRIx64") in MulticastGroup definitions for VirtualFabric (%s)",
															mgid_ref->mgid[0], mgid_ref->mgid[1], vfsip->v_fabric_all[vf2].name);
													return;
												}
											}
											else {
												sprintf(error, "Duplicate MGID (0x%016"PRIx64":0x%016"PRIx64") in MulticastGroup definitions for VirtualFabric (%s)",
														mgid_ref->mgid[0], mgid_ref->mgid[1], vfsip->v_fabric_all[vf2].name);
												return;
											}
										}
									}
								}
								dg_check = dg_check->next_default_group;
							}
						}
					}
				}
			}
			dg_ref = dg_ref->next_default_group;
		}
	}
}

// find a pointer to a rendered Virtual Fabric given the name
VF_t* findVfPointer(VirtualFabrics_t* vf_config, char* virtualFabric)
{
	uint32_t i;

	if (vf_config == NULL || virtualFabric == NULL)
		return NULL;

	for (i = 0; i < vf_config->number_of_vfs_all; i++) {
		if (vf_config->v_fabric_all[i].standby) continue;
		if (strcmp(virtualFabric, vf_config->v_fabric_all[i].name) == 0) 
			return &vf_config->v_fabric_all[i];
	}
	return NULL;
}

// build a list of GUID's
static int32_t buildGuidMap(cl_qmap_t *map, cl_qmap_t *guid_list, char *error, uint32_t *entries)
{
	cl_map_item_t *cl_map_item;
	for_all_qmap_item(guid_list, cl_map_item) {
		*entries += addMap(map, cl_qmap_key(cl_map_item));
	}
	return 0;
}

// compare node descriptions
int nodeDescCompare(IN const uint64 desc1, IN  const uint64 desc2)
{
	if (!desc1 || !desc2)
		return 0;

	return strncmp(XML_QMAP_CHAR_CAST desc1, XML_QMAP_CHAR_CAST desc2, MAX_VFABRIC_NAME);
}

// build a list of Node Descriptions
int32_t buildDescList(cl_qmap_t *map, XmlNode_t *desc_list, char *error, uint32_t *entries)
{
	char *node_descr;

	while (desc_list) {
		node_descr = getXmlMemory(MAX_VFABRIC_NAME, "cl_map_item_t buildDescList()");
		if (!node_descr) {
			if (error)
				sprintf(error, OUT_OF_MEMORY);
			fprintf(stdout, OUT_OF_MEMORY_RETURN);
			return -1;
		}
		cs_strlcpy(node_descr, desc_list->node, MAX_VFABRIC_NAME);
		if (addMap(map, XML_QMAP_U64_CAST node_descr)) {
			(*entries)++;
		} else {
			freeXmlMemory(node_descr, MAX_VFABRIC_NAME, "cl_map_item_t buildDescList()");
		}
		desc_list = desc_list->next;
	}
	return 0;
}

static boolean testSlSharing(VF_t *testVf, uint8_t testSl, VF_t *targetVf, uint8_t targetSl, char *type, IXmlParserPrintMessage printError, IXmlParserPrintMessage printWarning)
{
	const char *sharing_error = "Virtual Fabric (%s) shares %s (%d) with Virtual Fabric (%s), but different %s setting.\n";
	const char *reuse_error = "Virtual Fabric (%s) %s (%d) previously used in VirtualFabric (%s) as %s.\n";
	const char *mixeduse_error = "Virtual Fabric (%s) shares %s (%d) with Virtual Fabric (%s), but have different %s.\n";
	char msg[256];
	boolean result = 1;

	if (testSl != UNDEFINED_XML8) {
		if (targetSl != UNDEFINED_XML8 && testSl == targetSl) {
			if (testVf->priority != targetVf->priority) {
				if (printError) {
					snprintf(msg, sizeof(msg),  sharing_error, testVf->name, type, testSl, targetVf->name, "HighPriority");
					(*printError)(msg);
				}
				result = 0;
			}
			if (testVf->preempt_rank != targetVf->preempt_rank) {
				if (printError) {
					snprintf(msg, sizeof(msg),  sharing_error, testVf->name, type, testSl, targetVf->name, "PreemptRank");
					(*printError)(msg);
				}
				result = 0;
			}
			if (testVf->flowControlDisable != targetVf->flowControlDisable) {
				if (printError) {
					snprintf(msg, sizeof(msg),  sharing_error, testVf->name, type, testSl, targetVf->name, "FlowControlDisable");
					(*printError)(msg);
				}
				result = 0;
			}
			if (testVf->hoqlife_vf != targetVf->hoqlife_vf) {
				if (printError) {
					snprintf(msg, sizeof(msg),  sharing_error, testVf->name, type, testSl, targetVf->name, "HoqLife");
					(*printError)(msg);
				}
				result = 0;
			}
			if (testVf->pkt_lifetime_mult != targetVf->pkt_lifetime_mult) {
				if (printError) {
					snprintf(msg, sizeof(msg),  sharing_error, testVf->name, type, testSl, targetVf->name, "PktLifeTimeMult");
					(*printError)(msg);
				}
				result = 0;
			}
		}
		if (targetVf->base_sl != UNDEFINED_XML8 && targetVf->base_sl == testSl) {
			if (testSl != testVf->base_sl) {
				if (printError) {
					snprintf(msg, sizeof(msg), reuse_error, testVf->name, type, testSl, targetVf->name, "BaseSl");
					(*printError)(msg);
				}
				result = 0;
			} else {
				if (testVf->resp_sl != UNDEFINED_XML8 && targetVf->resp_sl != UNDEFINED_XML8 && testVf->resp_sl != targetVf->resp_sl) {
					if (printError) {
						snprintf(msg, sizeof(msg), mixeduse_error, testVf->name, type, testSl, targetVf->name, "RespSL");
						(*printError)(msg);
					}
					result = 0;
				}
				if (testVf->mcast_sl != UNDEFINED_XML8 && targetVf->mcast_sl != UNDEFINED_XML8 && testVf->mcast_sl != targetVf->mcast_sl) {
					if (printError) {
						snprintf(msg, sizeof(msg), mixeduse_error, testVf->name, type, testSl, targetVf->name, "MulticastSL");
						(*printError)(msg);
					}
					result = 0;
				}
			}
		}
		if (targetVf->resp_sl != UNDEFINED_XML8 && targetVf->resp_sl == testSl) {
			if (testSl != testVf->resp_sl) {
				if (printError) {
					snprintf(msg, sizeof(msg), reuse_error, testVf->name, type, testSl, targetVf->name, "RespSl");
					(*printError)(msg);
				}
				result = 0;
			} else {
				if (testVf->base_sl != UNDEFINED_XML8 && targetVf->base_sl != UNDEFINED_XML8 && testVf->base_sl != targetVf->base_sl) {
					if (printError) {
						snprintf(msg, sizeof(msg), mixeduse_error, testVf->name, type, testSl, targetVf->name, "BaseSL");
						(*printError)(msg);
					}
					result = 0;
				}
				if (testVf->mcast_sl != UNDEFINED_XML8 && targetVf->mcast_sl != UNDEFINED_XML8 && testVf->mcast_sl != targetVf->mcast_sl) {
					if (printError) {
						snprintf(msg, sizeof(msg), mixeduse_error, testVf->name, type, testSl, targetVf->name, "MulticastSL");
						(*printError)(msg);
					}
					result = 0;
				}
			}
		}
		if (targetVf->mcast_sl != UNDEFINED_XML8 && targetVf->mcast_sl == testSl) {
			if (testSl != testVf->mcast_sl) {
				if (printError) {
					snprintf(msg, sizeof(msg), reuse_error, testVf->name, type, testSl, targetVf->name, "MulticastSL");
					(*printError)(msg);
				}
				result = 0;
			} else {
				if (testVf->base_sl != UNDEFINED_XML8 && targetVf->base_sl != UNDEFINED_XML8 && testVf->base_sl != targetVf->base_sl) {
					if (printError) {
						snprintf(msg, sizeof(msg), mixeduse_error, testVf->name, type, testSl, targetVf->name, "BaseSL");
						(*printError)(msg);
					}
					result = 0;
				}
				if (testVf->resp_sl != UNDEFINED_XML8 && targetVf->resp_sl != UNDEFINED_XML8 && testVf->resp_sl != targetVf->resp_sl) {
					if (printError) {
						snprintf(msg, sizeof(msg), mixeduse_error, testVf->name, type, testSl, targetVf->name, "RespSL");
						(*printError)(msg);
					}
					result = 0;
				}
			}
		}
	}
	return result;
}

// Test routine to validate SL assignement to VFs
boolean applyVirtualFabricRules(VirtualFabrics_t *vfsip, IXmlParserPrintMessage printError, IXmlParserPrintMessage printWarning)
{
	boolean result = 1;
	char msg[256];
	boolean slsWithBw[STL_MAX_SLS] = { 0 };
	int vf;

	for (vf = 0; vf < vfsip->number_of_vfs_all && vf < MAX_ENABLED_VFABRICS; vf++) {
		const char *ignored = "Virtual Fabric (%s) is %s, configured %s (%d%s) will be ignored.\n";
		VF_t *vfp = &vfsip->v_fabric_all[vf];

		if (!vfp->qos_enable) {
			if (vfp->preempt_rank) {
				if (printWarning) {
					snprintf(msg, sizeof(msg), ignored, vfp->name, "Qos disabled", "PreemptRank", vfp->preempt_rank, "");
					(*printWarning)(msg);
				}
				vfp->preempt_rank = 0;
			}
			if (vfp->flowControlDisable) {
				if (printWarning) {
					snprintf(msg, sizeof(msg), ignored, vfp->name, "Qos disabled", "FlowControlDisable", vfp->flowControlDisable, "");
					(*printWarning)(msg);
				}
				vfp->flowControlDisable = 0;
			}
			if (vfp->priority) {
				if (printWarning) {
					snprintf(msg, sizeof(msg), ignored, vfp->name, "Qos disabled", "HighPriority", vfp->priority, "");
					(*printWarning)(msg);
				}
				vfp->priority = 0;
			}
			if (!vfp->standby && vfp->percent_bandwidth != UNDEFINED_XML8) {
				if (printWarning) {
					snprintf(msg, sizeof(msg), ignored, vfp->name, "Qos disabled", "Bandwidth", vfp->percent_bandwidth, "%");
					(*printWarning)(msg);
				}
				vfp->percent_bandwidth = UNDEFINED_XML8;
			}
			if (vfp->pkt_lifetime_specified) {
				if (printWarning) {
					snprintf(msg, sizeof(msg), ignored, vfp->name, "Qos disabled", "PktLifeTimeMult", vfp->pkt_lifetime_mult, "");
					(*printWarning)(msg);
				}
				vfp->pkt_lifetime_mult = CeilLog2(1);
				vfp->pkt_lifetime_specified = 0;
			}
			if (vfp->base_sl != UNDEFINED_XML8) {
				if (printWarning) {
					snprintf(msg, sizeof(msg), ignored, vfp->name, "Qos disabled", "BaseSL", vfp->base_sl, "");
					(*printWarning)(msg);
				}
				vfp->base_sl = UNDEFINED_XML8;
			}
			if (vfp->resp_sl != UNDEFINED_XML8) {
				if (printWarning) {
					snprintf(msg, sizeof(msg), ignored, vfp->name, "Qos disabled", "RespSL", vfp->resp_sl, "");
					(*printWarning)(msg);
				}
				vfp->resp_sl = UNDEFINED_XML8;
			}
			if (vfp->mcast_sl != UNDEFINED_XML8) {
				if (printWarning) {
					snprintf(msg, sizeof(msg), ignored, vfp->name, "Qos disabled", "MulticastSL", vfp->mcast_sl, "");
					(*printWarning)(msg);
				}
				vfp->mcast_sl = UNDEFINED_XML8;
			}
			if (vfp->hoqlife_specified) {
				if (printWarning) {
					snprintf(msg, sizeof(msg),
							"Virtual Fabric (%s) is Qos disabled, configured HoqLife ignored, using SM value (%d).\n",
							vfp->name, vfp->hoqlife_vf);
					(*printWarning)(msg);
				}
			}
		} else {
			const char *unneeded = "Virtual Fabric (%s) includes a user specified %s (%d), but includes no %s.\n";
			const char *reused = "Virtual Fabric (%s) cannot share SL (%d) for %s and %s, %s.\n";
			const char *missing = "Virtual Fabric (%s) includes %s (%d) but missing required %s.\n";
			int i;

			if (vfp->priority && !vfp->standby && vfp->percent_bandwidth != UNDEFINED_XML8) {
				if (printWarning) {
					snprintf(msg, sizeof(msg), ignored, vfp->name, "High Priority", "Bandwidth", vfp->percent_bandwidth, "%");
					(*printWarning)(msg);
				}
				vfp->percent_bandwidth = UNDEFINED_XML8;
			}
			if (vfp->base_sl != UNDEFINED_XML8) {
				if (vfp->requires_resp_sl) {
					if (vfp->resp_sl == UNDEFINED_XML8) {
						if (printError) {
							snprintf(msg, sizeof(msg), missing, vfp->name, "BaseSL", vfp->base_sl, "RespSL");
							(*printError)(msg);
						}
						result = 0;
					} else if (vfp->resp_sl == vfp->base_sl) {
						if (printError) {
							snprintf(msg, sizeof(msg), reused, vfp->name, vfp->base_sl, "BaseSL", "RespSL", "requires separate RespSL");
							(*printError)(msg);
						}
						result = 0;
					}
				}
			}
			if (vfp->resp_sl != UNDEFINED_XML8) {
				if (vfp->base_sl == UNDEFINED_XML8) {
					if (printError) {
						snprintf(msg, sizeof(msg), missing, vfp->name, "RespSL", vfp->resp_sl, "BaseSL");
						(*printError)(msg);
					}
					result = 0;
				} else {
					if (vfp->resp_sl != vfp->base_sl) {
						if (!vfp->requires_resp_sl) {
							if (printWarning) {
								snprintf(msg, sizeof(msg), unneeded, vfp->name, "RespSL", vfp->resp_sl, "response applications");
								(*printWarning)(msg);
							}
						}
					}
				}
			}
			if (vfp->mcast_sl != UNDEFINED_XML8) {
				if (vfp->base_sl == UNDEFINED_XML8) {
					if (printError) {
						snprintf(msg, sizeof(msg), missing, vfp->name, "MulticastSL", vfp->mcast_sl, "BaseSL");
						(*printError)(msg);
					}
					result = 0;
				} else {
					if (vfp->mcast_sl != vfp->base_sl) {
						if (!vfp->contains_mcast) {
							if (printWarning) {
								snprintf(msg, sizeof(msg), unneeded, vfp->name, "MulticastSL", vfp->mcast_sl, "multicast groups");
								(*printWarning)(msg);
							}
						}
						// MCAST != BASE, but MCAST == RESP
						if (vfp->resp_sl != UNDEFINED_XML8 && vfp->mcast_sl == vfp->resp_sl) {
							if (printError) {
								snprintf(msg, sizeof(msg), reused, vfp->name, vfp->mcast_sl, "RespSL", "MulticastSL", "must be separate SLs");
								(*printError)(msg);
							}
							result = 0;
						}
					}
				}
			}
			// Check this VF against all previous VFs
			for (i = 0; i < vf; i++) {
				VF_t *tmpVf = &vfsip->v_fabric_all[i];

				if (!tmpVf->qos_enable) continue;

				if (!testSlSharing(vfp, vfp->base_sl, tmpVf, tmpVf->base_sl, "BaseSL", printError, printWarning)) {
					result = 0;
				}
				if (!testSlSharing(vfp, vfp->resp_sl, tmpVf, tmpVf->resp_sl, "RespSL", printError, printWarning)) {
					result = 0;
				}
				if (!testSlSharing(vfp, vfp->mcast_sl, tmpVf, tmpVf->mcast_sl, "MulticastSL", printError, printWarning)) {
					result = 0;
				}

			}
		}
		if (!vfp->standby && vfp->qos_enable && !vfp->priority && vfp->percent_bandwidth != 0) {
			// Find all the SLs that are used by Qos, Low Priority, and have a nonzero bandwidth
			if (vfp->base_sl < STL_CONFIGURABLE_SLS) { // note that this implicitly checks for != UNDEFINED.
				slsWithBw[vfp->base_sl] = 1;
			}
			if (vfp->resp_sl < STL_CONFIGURABLE_SLS) { // note that this implicitly checks for != UNDEFINED.
				slsWithBw[vfp->resp_sl] = 1;
			}
			if (vfp->mcast_sl < STL_CONFIGURABLE_SLS) { // note that this implicitly checks for != UNDEFINED.
				slsWithBw[vfp->mcast_sl] = 1;
			}
		}
	}
	for (vf = 0; vf < vfsip->number_of_vfs_all; vf++) {
		const char *no_bw_sl = "Virtual Fabric (%s) specifies 0%% Bandwidth, %s (%d) must be included in a VF with nonzero Bandwidth.\n";
		const char *no_bw_no_sl = "Virtual Fabric (%s) specifies 0%% Bandwidth, must include SLs shared by other VFs.\n";
		VF_t *vfp = &vfsip->v_fabric_all[vf];

		if (!vfp->standby && vfp->qos_enable && !vfp->priority && vfp->percent_bandwidth == 0) {
			boolean hasSls = 0;
			if (vfp->base_sl < STL_CONFIGURABLE_SLS) { // note that this implicitly checks for != UNDEFINED.
				hasSls = 1;
				if (!slsWithBw[vfp->base_sl]) {
					if (printError) {
						snprintf(msg, sizeof(msg), no_bw_sl, vfp->name, "BaseSL", vfp->base_sl);
						(*printError)(msg);
					}
					result = 0;
				}
			}
			if (vfp->resp_sl < STL_CONFIGURABLE_SLS) { // note that this implicitly checks for != UNDEFINED.
				hasSls = 1;
				if (!slsWithBw[vfp->resp_sl]) {
					if (printError) {
						snprintf(msg, sizeof(msg), no_bw_sl, vfp->name, "RespSL", vfp->resp_sl);
						(*printError)(msg);
					}
					result = 0;
				}
			}
			if (vfp->mcast_sl < STL_CONFIGURABLE_SLS) { // note that this implicitly checks for != UNDEFINED.
				hasSls = 1;
				if (!slsWithBw[vfp->mcast_sl]) {
					if (printError) {
						snprintf(msg, sizeof(msg), no_bw_sl, vfp->name, "MulticastSL", vfp->mcast_sl);
						(*printError)(msg);
					}
					result = 0;
				}
			}
			if (!hasSls) {
				if (printError) {
					snprintf(msg, sizeof(msg), no_bw_no_sl, vfp->name);
					(*printError)(msg);
				}
				result = 0;
			}
		}
	}

	return result;
}

// conversion utility  to render XML captured data into a more application readable format
VirtualFabrics_t* renderVirtualFabricsConfig(uint32_t fm, FMXmlCompositeConfig_t *config, SMXmlConfig_t *smp, char *error)
{
	VFConfig_t 					*vfp;

	VirtualFabrics_t			*vfsip;
	VF_t						*vfip;

	DGConfig_t					*group_list[MAX_VFABRIC_GROUPS];
	DGConfig_t	 				*parent_group;
	uint32_t					group_list_index;
	uint32_t					full_member_groups;
	uint32_t					limited_member_groups;
	uint32_t					full_groups_in_list;
	uint32_t					limited_groups_in_list;
	uint32_t					entry;

	AppConfig_t					*app_list[MAX_VFABRIC_APPS];
	AppConfig_t					*parent_app;
	uint32_t					app_list_index;
	uint32_t					apps;
	uint32_t					apps_in_list;

	SMMcastDefGrp_t 			*mdgp;

	uint32_t 					v_fabrics;
	uint32_t					valid_vfs;
	uint32_t					total_bw;

	uint8_t						num_non_qos = 0;
	uint8_t						num_bwundef_qos = 0;

	int32_t						result;
	uint8_t						default_vf_check;

	uint8_t						isPAAssigned = 0;
	uint8_t						isPMAssigned = 0;
	uint8_t						isSAAssigned = 0;


	// if there is no config data for this instance then we are done
	if (config->fm_instance[fm] == NULL)
		return NULL;

	// get memory for the main VF structure
	vfsip = getXmlMemory(sizeof(VirtualFabrics_t), "VirtualFabrics_t renderVirtualFabricsConfig()");
	if (!vfsip)
		return NULL;

	// clear all data
	memset(vfsip, 0, sizeof(VirtualFabrics_t));

	// first build the list of Virtual Fabrics - a VF is valid if it is enabled
	// is not 0 length
	valid_vfs = 0;
	total_bw = 0;


	/*
	 * Account for VLArb peculiarities when processing VF QoS values, in
	 * particular, the fact that VLArb works in 5% increments
	 */

	for (v_fabrics = 0; v_fabrics < MAX_CONFIGURED_VFABRICS; v_fabrics++) {

		vfp = config->fm_instance[fm]->vf_config.vf[v_fabrics];

		// if there are no more Virtual Fabrics then we are done
		if (vfp == NULL)
			break;

		// if the VF is not enabled then continue
		if (!vfp->enable)
			continue;

		// see if the number of enabled VF's will exceed 32
		if (valid_vfs >= MAX_ENABLED_VFABRICS) {
			if (error) {
				sprintf(error, "No more than 32 Virtual Fabrics can be enabled in the configuration");
				goto fail;
			}
			continue;
		}
		// verify that Virtual Fabric has a Name
		if (strlen(vfp->name) == 0) {
			if (error) {
				sprintf(error, "Virtual Fabric must have a name if its Enabled");
				goto fail;
			}
			continue;
		}

		// verify that Virtual Fabric PKey is >0 and <= 15 bits
		if ((vfp->pkey != UNDEFINED_XML32) && (((vfp->pkey == 0) || (vfp->pkey > 0x7fff)))) {
			if (error) {
				sprintf(error, "Virtual Fabric (%s) Invalid PKey: 0x%x, must be in the range 0x0001-0x7fff", vfp->name, vfp->pkey);
				goto fail;
			}
			continue;
		}

		// verify that necessary objects in Virtual Fabric are specified
		if (vfp->number_of_full_members == 0 && vfp->number_of_limited_members == 0) {
			if (error) {
				sprintf(error, "Virtual Fabric (%s) must have either one or more Member or LimitedMember entries defined", vfp->name);
				goto fail;
			}
			continue;
		}
		if (vfp->number_of_applications == 0) {
			if (error) {
				sprintf(error, "Virtual Fabric (%s) must have either one or more Application entries defined", vfp->name);
				goto fail;
			}
			continue;
		}

		default_vf_check = (vfp->pkey & ~0x8000) == STL_DEFAULT_PKEY;

		// get pointer to next internal VF object
		vfip = &vfsip->v_fabric_all[vfsip->number_of_vfs_all];

		// save settings for this VF
		cs_strlcpy(vfip->name, vfp->name, MAX_VFABRIC_NAME);
		vfip->index = v_fabrics;
		vfip->pkey = vfp->pkey;
		vfip->standby = vfp->standby;

		vfip->security = vfp->security;

		vfip->qos_enable = vfp->qos_enable;

		vfip->base_sl = vfp->base_sl;
		vfip->resp_sl = vfp->resp_sl;
		vfip->mcast_sl = vfp->mcast_sl;

		if (vfp->flowControlDisable == UNDEFINED_XML8) {
			// change undefined to keep enabled
			vfp->flowControlDisable = 0;
		}
		vfip->flowControlDisable = vfp->flowControlDisable;
		vfip->percent_bandwidth = vfp->percent_bandwidth;

		if (vfp->priority == UNDEFINED_XML8) {
			// Change undefined to low.
			vfp->priority = 0;
		}
		vfip->priority = vfp->priority;

		if (vfp->pkt_lifetime_mult != UNDEFINED_XML8) {
			vfip->pkt_lifetime_mult = vfp->pkt_lifetime_mult;
			vfip->pkt_lifetime_specified = 1;
		} else {
			vfip->pkt_lifetime_mult = CeilLog2(1);
			vfip->pkt_lifetime_specified = 0;
		}

		if (vfp->max_mtu_int != UNDEFINED_XML8) {
			vfip->max_mtu_int = vfp->max_mtu_int;
			vfip->max_mtu_specified = 1;
		} else {
			vfip->max_mtu_int = STL_MTU_MAX;
			vfip->max_mtu_specified = 0;
		}

		if (vfp->max_rate_int != UNDEFINED_XML8) {
			vfip->max_rate_int = vfp->max_rate_int;
			vfip->max_rate_specified = 1;
		} else {
			vfip->max_rate_int = IB_STATIC_RATE_MAX;
			vfip->max_rate_specified = 0;
		}

		// Inherit default values from SM Instance if not yet defined by VF.
		vfip->hoqlife_specified = (vfp->hoqlife_vf != UNDEFINED_XML32);
		vfip->hoqlife_vf = ((!vfip->hoqlife_specified || !vfp->qos_enable) ? smp->hoqlife_n2 : vfp->hoqlife_vf);

		// Copy the preemption rank -
		vfip->preempt_rank = ((vfp->preempt_rank == UNDEFINED_XML8) ? 0 : vfp->preempt_rank);

/*
        // if absolute bandwidth is valid the use value
        if (strlen(vfp->absolute_bandwidth) > 0) {
            // if Full then set percentage to 100
            if (strcasecmp(vfp->absolute_bandwidth, "Full") == 0) {
                vfip->percent_bandwidth = 100;
                vfip->absolute_bandwidth_gbps = UNDEFINED_XML32;
            }
        }
*/

		// build a list of all applications and included application pointers
		// if an app is encountered more than once disregard but let the user know - this
		// is not really a parse error
		memset(app_list, 0, sizeof(app_list));

		app_list_index = 0;
		for (apps = 0; apps < MAX_VFABRIC_APPS_PER_VF; apps++)
		{
			// if there is no app then we are done with this VF
			if (strlen(vfp->application[apps].application) == 0)
				break;
			app_list[app_list_index] = findAppPointer(config, vfp->application[apps].application, fm);
			if (app_list[app_list_index]) {
				parent_app = app_list[app_list_index];
				app_list_index++;
				result = includedApps(config, fm, app_list, parent_app, &app_list_index, error);
				if (result < 0) {
					if (error) {
						goto fail;
					}
				}
			} else if (error) {
				sprintf(error, "VirtualFabric (%s) does not have any valid Application named (%s) in the configuration",
					vfp->name, vfp->application[apps].application);
				goto fail;
			}

		}
		apps_in_list = app_list_index;

		// if there are no apps to match this enabled VF then this is a parse error
		if (apps_in_list == 0) {
			if (error) {
				sprintf(error, "VirtualFabric (%s) does not have any valid Applications specified in the configuration", vfp->name);
				goto fail;
			}
			continue;
		}

		// aggregate the flags for each parsed app into the internal app structure
		for (app_list_index = 0; app_list_index < apps_in_list; app_list_index++) {
			vfip->apps.select_sa |= app_list[app_list_index]->select_sa;
			vfip->apps.select_unmatched_sid |= app_list[app_list_index]->select_unmatched_sid;
			vfip->apps.select_unmatched_mgid |= app_list[app_list_index]->select_unmatched_mgid;
		}

		for (app_list_index = 0; app_list_index < apps_in_list; app_list_index++)
			vfip->apps.select_pm |= app_list[app_list_index]->select_pm;


		// now that we have the app list create entries in internal database for service ID's
		// and MGID's.
		VFAppSid_t *sid;
		VFAppMgid_t *mgid;

		vfip->apps.sidMapSize = 0;
		cl_qmap_init(&vfip->apps.sidMap, compareAppSid);

		vfip->apps.mgidMapSize = 0;
		cl_qmap_init(&vfip->apps.mgidMap, compareAppMgid);

		for (app_list_index = 0; app_list_index < apps_in_list; app_list_index++)
		{
			cl_map_item_t *cl_map_item;

			// add all of the individual SID's to the list
			for_all_qmap_item(&app_list[app_list_index]->serviceIdMap, cl_map_item) {
				sid = getAppSid();
				if (!sid) {
					if (error)
						sprintf(error, OUT_OF_MEMORY);
					fprintf(stdout, OUT_OF_MEMORY_RETURN);
					goto fail;
				}

				// since this is a unique service ID set SID then set last to 0 to
				// denote no range, and mask to UNDEFINED_XML64 to denote no mask
				sid->service_id = cl_qmap_key(cl_map_item);
				sid->service_id_last = 0;
				sid->service_id_mask = UNDEFINED_XML64;

				if (!addMap(&vfip->apps.sidMap, XML_QMAP_U64_CAST sid)) {
					freeAppSid(sid);
				} else {
					vfip->apps.sidMapSize++;
				}
			}
			// add all of the SID ranges to the list
			char *range;
			for_all_qmap_ptr(&app_list[app_list_index]->serviceIdRangeMap, cl_map_item, range) {
				sid = getAppSid();
				if (!sid) {
					if (error)
						sprintf(error, OUT_OF_MEMORY);
					fprintf(stdout, OUT_OF_MEMORY_RETURN);
					goto fail;
				}

				// since this is a range of service ID's then set it up appropriately
				verifyAndConvertSidCompoundString(range, /* range */ 1, sid);

				if (!addMap(&vfip->apps.sidMap, XML_QMAP_U64_CAST sid)) {
					freeAppSid(sid);
				} else {
					vfip->apps.sidMapSize++;
				}
			}
			// add all of the SID masked values to the list
			char *masked;
			for_all_qmap_ptr(&app_list[app_list_index]->serviceIdMaskedMap, cl_map_item, masked) {
				sid = getAppSid();
				if (!sid) {
					if (error)
						sprintf(error, OUT_OF_MEMORY);
					fprintf(stdout, OUT_OF_MEMORY_RETURN);
					goto fail;
				}

				// since this is a masked service ID then set it up appropriately
				verifyAndConvertSidCompoundString(masked, /* masked */ 0, sid);

				if (!addMap(&vfip->apps.sidMap, XML_QMAP_U64_CAST sid)) {
					freeAppSid(sid);
				} else {
					vfip->apps.sidMapSize++;
				}
			}
			// add all of the individual MGID's to the list
			for (entry = 0; entry < app_list[app_list_index]->number_of_mgids; entry++) {
				mgid = getAppMgid();
				if (!mgid) {
					if (error)
						sprintf(error, OUT_OF_MEMORY);
					fprintf(stdout, OUT_OF_MEMORY_RETURN);
					goto fail;
				}

				// since this is an individual MGID ID then set it up appropriately
				verifyAndConvertMGidString(app_list[app_list_index]->mgid[entry].mgid, mgid);

				if (!addMap(&vfip->apps.mgidMap, XML_QMAP_U64_CAST mgid)) {
					freeAppMgid(mgid);
				} else {
					vfip->apps.mgidMapSize++;
				}
			}
			// add all of the MGID range values to the list
			for (entry = 0; entry < app_list[app_list_index]->number_of_mgid_ranges; entry++) {
				mgid = getAppMgid();
				if (!mgid) {
					if (error)
						sprintf(error, OUT_OF_MEMORY);
					fprintf(stdout, OUT_OF_MEMORY_RETURN);
					goto fail;
				}

				// since this is an MGID range then set it up appropriately
				verifyAndConvertMGidCompoundString(app_list[app_list_index]->mgid_range[entry].range, /* range */ 1, mgid);

				if (!addMap(&vfip->apps.mgidMap, XML_QMAP_U64_CAST mgid)) {
					freeAppMgid(mgid);
				} else {
					vfip->apps.mgidMapSize++;
				}
			}
			// add all of the MGID masked values to the list
			for (entry = 0; entry < app_list[app_list_index]->number_of_mgid_range_maskeds; entry++) {
				mgid = getAppMgid();
				if (!mgid) {
					if (error)
						sprintf(error, OUT_OF_MEMORY);
					fprintf(stdout, OUT_OF_MEMORY_RETURN);
					goto fail;
				}

				// since this is an MGID mask then set it up appropriately
				verifyAndConvertMGidCompoundString(app_list[app_list_index]->mgid_masked[entry].masked, /* masked */ 0, mgid);

				if (!addMap(&vfip->apps.mgidMap, XML_QMAP_U64_CAST mgid)) {
					freeAppMgid(mgid);
				} else {
					vfip->apps.mgidMapSize++;
				}
			}
		}
		//MC groups will be initialized after all the VFs are done, in the meantime
		vfip->number_of_default_groups = 0;

		vfip->contains_mcast = (vfip->apps.mgidMapSize != 0) || vfip->apps.select_unmatched_mgid;

		// Keep track of bandwidth in use but don't check until all
		// VFs have been processed
		if (!vfip->standby) {
			if (!vfip->qos_enable) {
				num_non_qos++;
			} else if (!vfip->priority) {
				if (vfip->percent_bandwidth != UNDEFINED_XML8) {
					total_bw += vfip->percent_bandwidth;
				} else {
					num_bwundef_qos++;
				}
			}
		}

		// If this is a Default VF, make sure that SA, PM, or PA is included.
		if (default_vf_check) {
			if (vfip->apps.select_sa || checkVFSID(vfip, STL_PM_SERVICE_ID) ||
				vfip->apps.select_pm 
				
				) {
				if (!vfp->standby) {
					if (vfip->apps.select_sa) isSAAssigned++;
					if (checkVFSID(vfip, STL_PM_SERVICE_ID)) isPAAssigned++;
					if (vfip->apps.select_pm) isPMAssigned++;


				}
			} else {
				const char errStr[] = "Virtual Fabrics (%s) using the Mgmt Pkey must have <Select>SA</Select>, <Select>PM</Select>, "
								  "or PA Service ID configured in an Application";
				if (error)
					sprintf(error, errStr, vfip->name);
				else fprintf(stdout, errStr, vfip->name);

				goto fail;
			}
		} else if (vfip->apps.select_sa || checkVFSID(vfip, STL_PM_SERVICE_ID) ||
				vfip->apps.select_pm 

				   ) {
				const char errStr[] = "Virtual Fabrics (%s) including <Select>SA</Select>, <Select>PM</Select>, "
								  "or PA Service ID configured in an Application must use Mgmt Pkey";
				if (error)
					sprintf(error, errStr, vfip->name);
				else fprintf(stdout, errStr, vfip->name);

				goto fail;
		}

		// also part of PR112665 - make sure only 1 Active VF has the <Select>SA</Select> in an application
		if (isSAAssigned > 1) {
			const char errStr[] = "Only one Virtual Fabric can have <Select>SA</Select> configured in an Application";
			if (error)
				sprintf(error, errStr);
			else fprintf(stdout, errStr);

			goto fail;
		}

		// DN 441 - make sure only 1 VF has <Select>PM</Select>
		if (isPMAssigned > 1) {
			const char errStr[] = "Only one Virtual Fabric can have <Select>PM</Select> configured in an Application";
			if (error)
				sprintf(error, errStr);
			else fprintf(stdout, errStr);

			goto fail;
		}


		// build a list of all included groups pointers and all of their included group pointers
		// for this VF - of there are any groups included twice they will be ignored and a message
		// will be sent to the console.
		memset(group_list, 0, sizeof(group_list));

		group_list_index = 0;
		for (full_member_groups = 0; full_member_groups < MAX_VFABRIC_MEMBERS_PER_VF;  full_member_groups++)
		{
			// if there is no group then we are done with this VF
			if (strlen(vfp->full_member[full_member_groups].member) == 0)
				break;
			group_list[group_list_index] = findGroupPointer(config, vfp->full_member[full_member_groups].member, fm);
			if (group_list[group_list_index]) {
				parent_group = group_list[group_list_index];
				group_list_index++;
				result = includedGroups(config, fm, group_list, parent_group, &group_list_index, error);
				if (result < 0) {
					if (error) {
						goto fail;
					}
				}
			} else if (error) {
				sprintf(error, "VirtualFabric (%s) does not have any valid DeviceGroup named (%s) in the configuration",
					vfp->name, vfp->full_member[full_member_groups].member);
				goto fail;
			}
		}
		full_groups_in_list = group_list_index;

		// aggregate the flags for each parsed app into the internal group structure
		for (group_list_index = 0; group_list_index < full_groups_in_list; group_list_index++) {
			vfip->full_members.select_all |= group_list[group_list_index]->select_all;
			vfip->full_members.select_self |= group_list[group_list_index]->select_self;
			vfip->full_members.select_hfi_direct_connect |= group_list[group_list_index]->select_hfi_direct_connect;
			vfip->full_members.select_swe0 |= group_list[group_list_index]->select_swe0;
			vfip->full_members.select_all_mgmt_allowed |= group_list[group_list_index]->select_all_mgmt_allowed;
			vfip->full_members.select_all_tfis |= group_list[group_list_index]->select_all_tfis;
		}

		// aggregate node type
		for (group_list_index = 0; group_list_index < full_groups_in_list; group_list_index++) {
			vfip->full_members.node_type_fi |= group_list[group_list_index]->node_type_fi;
			vfip->full_members.node_type_sw |= group_list[group_list_index]->node_type_sw;
		}

		// now that we have the group list create entries for the full member
		cl_qmap_init(&vfip->full_members.sysGuidMap, NULL);
		vfip->full_members.sysGuidMapSize = 0;
		cl_qmap_init(&vfip->full_members.nodeGuidMap, NULL);
		vfip->full_members.nodeGuidMapSize = 0;
		cl_qmap_init(&vfip->full_members.portGuidMap, NULL);
		vfip->full_members.portGuidMapSize = 0;
		cl_qmap_init(&vfip->full_members.nodeDescMap, compareName);
		vfip->full_members.nodeDescMapSize = 0;
		for (group_list_index = 0; group_list_index < full_groups_in_list; group_list_index++) {
			XmlNode_t *desc_list;

			// add all of the SystemImageGUID entries
			if(buildGuidMap(&vfip->full_members.sysGuidMap,
				&group_list[group_list_index]->system_image_guid,
				error, &vfip->full_members.sysGuidMapSize) < 0) {
				goto fail;
			}

			// add all of the NodeGUID entries
			if(buildGuidMap(&vfip->full_members.nodeGuidMap,
				&group_list[group_list_index]->node_guid,
				error, &vfip->full_members.nodeGuidMapSize) < 0) {
				goto fail;
			}

			// add all of the PortGUID entries
			if(buildGuidMap(&vfip->full_members.portGuidMap,
				&group_list[group_list_index]->port_guid,
				error, &vfip->full_members.portGuidMapSize) < 0) {
				goto fail;
			}

			// add all of the NodeDesc entries
			desc_list = group_list[group_list_index]->node_description;
			if(buildDescList(&vfip->full_members.nodeDescMap, desc_list, error, &vfip->full_members.nodeDescMapSize) < 0) {
				goto fail;
			}
		}

		// now get all of the limited members
		memset(group_list, 0, sizeof(group_list));

		// build a list of all included groups pointers and all of there included group pointer
		// for this VF - of there are any groups included twice they will be ignored and a message
		// will be sent to the console.
		group_list_index = 0;
		for (limited_member_groups = 0; limited_member_groups < MAX_VFABRIC_MEMBERS_PER_VF;  limited_member_groups++) {
			// if there is no group then we are done with this VF
			if (strlen(vfp->limited_member[limited_member_groups].member) == 0)
				break;
			group_list[group_list_index] = findGroupPointer(config, vfp->limited_member[limited_member_groups].member, fm);
			if (group_list[group_list_index]) {
				parent_group = group_list[group_list_index];
				group_list_index++;
				result = includedGroups(config, fm, group_list, parent_group, &group_list_index, error);
				if (result < 0) {
					if (error) {
						goto fail;
					}
				}
			} else if (error) {
				sprintf(error, "VirtualFabric (%s) does not have any valid DeviceGroup named (%s) in the configuration",
					vfp->name, vfp->full_member[limited_member_groups].member);
				goto fail;
			}
		}
		limited_groups_in_list = group_list_index;

		// aggregate the flags for each parsed app into the internal group structure
		for (group_list_index = 0; group_list_index < limited_groups_in_list; group_list_index++) {
			vfip->limited_members.select_all |= group_list[group_list_index]->select_all;
			vfip->limited_members.select_self |= group_list[group_list_index]->select_self;
			vfip->limited_members.select_hfi_direct_connect |= group_list[group_list_index]->select_hfi_direct_connect;
			vfip->limited_members.select_swe0 |= group_list[group_list_index]->select_swe0;
			vfip->limited_members.select_all_mgmt_allowed |= group_list[group_list_index]->select_all_mgmt_allowed;
			vfip->limited_members.select_all_tfis |= group_list[group_list_index]->select_all_tfis;
		}

		// make sure that either All or AllMgmtAllowed is included if this is a Default virtual fabric - this rule was added
		// to make sure IBTA works for PR112665
		if (default_vf_check && !vfip->full_members.select_all && !vfip->full_members.select_all_mgmt_allowed) {
			const char errStr[] = "Default Virtual Fabric (%s) must have <Select>All</Select> or <Select>AllMgmtAllowed</Select> configured in a Member";
			if (error)
				sprintf(error, errStr, vfip->name);
			else fprintf(stdout, errStr, vfip->name);
			goto fail;
		}

		// make sure that All is included as a full or limited member if this virtual fabric includes SA or PM- this rule was added
		// to make sure IBTA works for PR112665
		if (vfip->apps.select_sa || vfip->apps.select_pm) {
			if (!vfip->full_members.select_all && !vfip->limited_members.select_all) {
				const char errStr[] = "Virtual Fabric (%s) includes application (%s), must contain a device group which contains "
									  "<Select>ALL</Select> as a Member or LimitedMember";
				if (error)
					sprintf(error, errStr, vfip->name, vfip->apps.select_sa?(vfip->apps.select_pm?"SA and PM":"SA"):"PM");
				else fprintf(stdout, errStr, vfip->name, vfip->apps.select_sa?(vfip->apps.select_pm?"SA and PM":"SA"):"PM");
				goto fail;
			}
		}

		// aggregate node type
		for (group_list_index = 0; group_list_index < limited_groups_in_list; group_list_index++) {
			vfip->limited_members.node_type_fi |= group_list[group_list_index]->node_type_fi;
			vfip->limited_members.node_type_sw |= group_list[group_list_index]->node_type_sw;
		}

		// now that we have the group list create entries for the limited member
		cl_qmap_init(&vfip->limited_members.sysGuidMap, NULL);
		vfip->limited_members.sysGuidMapSize = 0;
		cl_qmap_init(&vfip->limited_members.nodeGuidMap, NULL);
		vfip->limited_members.nodeGuidMapSize = 0;
		cl_qmap_init(&vfip->limited_members.portGuidMap, NULL);
		vfip->limited_members.portGuidMapSize = 0;
		cl_qmap_init(&vfip->limited_members.nodeDescMap, compareName);
		vfip->limited_members.nodeDescMapSize = 0;
		for (group_list_index = 0; group_list_index < limited_groups_in_list; group_list_index++) {
			XmlNode_t *desc_list;

			// add all of the SystemImageGUID entries
			if(buildGuidMap(&vfip->limited_members.sysGuidMap,
				&group_list[group_list_index]->system_image_guid,
				error, &vfip->limited_members.sysGuidMapSize) < 0) {
				goto fail;
			}

			// add all of the NodeGUID entries
			if(buildGuidMap(&vfip->limited_members.nodeGuidMap,
				&group_list[group_list_index]->node_guid,
				error, &vfip->limited_members.nodeGuidMapSize) < 0) {
				goto fail;
			}

			// add all of the PortGUID entries
			if(buildGuidMap(&vfip->limited_members.portGuidMap,
				&group_list[group_list_index]->port_guid,
				error, &vfip->limited_members.portGuidMapSize) < 0) {
				goto fail;
			}

			// add all of the NodeDesc entries
			desc_list = group_list[group_list_index]->node_description;
			if(buildDescList(&vfip->limited_members.nodeDescMap, desc_list, error, &vfip->limited_members.nodeDescMapSize) < 0) {
				goto fail;
			}
		}

		// if there are no full or limited members to match this enabled VF then this is a parse error
		if (full_groups_in_list == 0 && limited_groups_in_list == 0) {
			if (error) {
				sprintf(error, "VirtualFabric %s does not have any valid Members or LimitedMembers specified in the configuration", vfp->name);
				goto fail;
			}
			continue;
		}

		if (!vfip->standby) {
			vfsip->number_of_vfs++;
		}
		vfsip->number_of_vfs_all++;
		valid_vfs++;
	}

	// If there are no VFs the configuration is failed
        if (!valid_vfs) {
		const char errStr[] = "Virtual Fabric configuration is invalid.";
		if (error)
			sprintf(error, errStr);
		else
			fprintf(stdout,"%s\n", errStr);
		goto fail;
	}

///// Match Mc Groups to VFs
	char mcerror[256];
	FSTATUS status=FSUCCESS;
	int default_group;

	uint32_t mg, mgj;
	memset(mcerror, 0, sizeof(mcerror));
	//match explicit MGIDs with VFs
	for (default_group = 0; default_group < config->fm_instance[fm]->sm_mdg_config.number_of_groups; default_group++) {
		mdgp = &config->fm_instance[fm]->sm_mdg_config.group[default_group];
		if (mdgp->def_mc_create == 0)
			continue;
		// first assign VFs to all explicit MC groups
 		if (mdgp->number_of_mgids > 0 && mdgp->number_of_mgids <= MAX_VFABRIC_DG_MGIDS) {
			// check that the MGID is not duplicated within the group section
			for (mg=0; (mg < mdgp->number_of_mgids && mdgp->number_of_mgids <= MAX_VFABRIC_DG_MGIDS); mg++)
				for (mgj=0; (mgj < mdgp->number_of_mgids && mdgp->number_of_mgids <= MAX_VFABRIC_DG_MGIDS); mgj++) {
					if (mg == mgj) continue;
					if (strncmp(mdgp->mgid[mg].mgid,mdgp->mgid[mgj].mgid, sizeof(mdgp->mgid[mg].mgid)) == 0) {
						sprintf(mcerror,"Duplicate MGIDs in the same <MulticastGroup> section.");
						if (error)
							snprintf(error, sizeof (mcerror), "Multicast group matching XML parse error - %s", mcerror);
						else
							fprintf(stdout, "Multicast group matching XML parse error - %s", mcerror);
						goto fail;
					}
				}

			status = MatchExplicitMGIDtoVF(mdgp, vfsip, FALSE /* update_active_vfabrics */, smp->enforceVFPathRecs);
			if (status != FSUCCESS) {
				switch (status) {
					case FUNAVAILABLE:
							sprintf(mcerror,"MC Groups could not be created, not enough memory.");
							break;
					case FINVALID_PARAMETER:
							sprintf(mcerror, "All explicit MGIDs must match some application. ");
							break;
					case FINVALID_STATE:
						sprintf(mcerror, "Explicit MGID matches more than a single VF.");
						break;
					case FNOT_FOUND:
						sprintf(mcerror,"Explicit MGID did not match any enabled VF.");
						break;
					default:
						break;
				}
				if (error)
					snprintf(error, sizeof (mcerror), "Multicast group matching XML parse error - %s", mcerror);
				else
					fprintf(stdout, "Multicast group matching XML parse error - %s", mcerror);
				goto fail;
			}

		}
	}
	//match implicit MGIDs with VFs
	for (default_group = 0; default_group < config->fm_instance[fm]->sm_mdg_config.number_of_groups; default_group++) {
		mdgp = &config->fm_instance[fm]->sm_mdg_config.group[default_group];
		// first assign VFs to all implicit MC groups
		if (mdgp->number_of_mgids == 0) {
			status = MatchImplicitMGIDtoVF(mdgp, vfsip);
			if (status != FSUCCESS) {
				switch (status) {
					case FUNAVAILABLE:
						sprintf(mcerror, "Implicit MC Groups could not be created, not enough memory.");
						break;
					case FINVALID_PARAMETER:
						sprintf(mcerror, "All Implicit MGIDs must match some application.");
						break;
					case FINVALID_STATE:
						sprintf(mcerror, "Implicit MGID matches more than a single VF.");
						break;
					case FNOT_FOUND:
						sprintf(mcerror,"Implicit MC Group did not match any enabled VF.");
						break;
					default:
						break;
				}
				if (error)
					snprintf(error, sizeof(mcerror),"Multicast group matching XML parse error - %s", mcerror);
				else
					fprintf(stdout, "Multicast group matching XML parse error - %s", mcerror);
				goto fail;
			}
		}
	}

	// if there is at least 1 nonQos VF, BW can't be greater than 95%
	// Reserve 1% BW per QoS VF without BW specified
	uint32_t needed_bw_reserve = ((num_non_qos!=0)?5:0) + num_bwundef_qos;

	if (total_bw > 100) {
		if (error)
			sprintf(error, "Total user-specified bandwidth for all active, enabled VFs (%d) cannot exceed 100%%", total_bw);
		else fprintf(stdout, "Total user-specified bandwidth for all active, enabled VFs (%d) cannot exceed 100%%\n", total_bw);
		goto fail;
	}

	if ((total_bw+needed_bw_reserve) > 100) {
		// if there is at least 1 nonQos VF, BW can't be greater than 95%
		// Reserve 1% BW per QoS VF without BW specified
		if (error)
			sprintf(error, "Need %d%% remaining bandwidth for Virtual Fabrics that cannot"
				" or do not specify bandwidth. Available remaining bandwidth is %d%%",
				needed_bw_reserve, 100 - total_bw);
		else fprintf(stdout, "Need %d%% remaining bandwidth for Virtual Fabrics that cannot"
			" or do not specify bandwidth. Available remaining bandwidth is %d%%\n",
			needed_bw_reserve, 100 - total_bw);
		goto fail;
	}

	if (!isSAAssigned) {
		const char errStr[] = "An Active Virtual Fabric must exist with <Select>SA</Select> configured in an Application. ";
		if (error)
			sprintf(error, errStr);
		else fprintf(stdout,"%s\n", errStr);

		goto fail;
	}
	if (!isPMAssigned) {
		const char errStr[] = "An Active Virtual Fabric must exist with <Select>PM</Select> configured in an Application. ";
		if (error)
			sprintf(error, errStr);
		else fprintf(stdout,"%s\n", errStr);

		goto fail;
	}
	if (!isPAAssigned) {
		const char errStr[] = "An Active Default Virtual Fabric must exist with PA Service ID configured in an Application. ";
		if (error)
			sprintf(error, errStr);
		else fprintf(stdout,"%s\n", errStr);

		goto fail;
	}


	if (!error && xml_vf_debug)
		fprintf(stdout, "Number of valid Virtual Fabrics %u\n", (unsigned int)valid_vfs);

	// for this VF check for duplicate MGID's in all default group
	checkDefaultGroupMGIDDuplicates(vfsip, error, smp);

	// calculate Virtual Fabric database checksum
	if (vfsip)
		checksumVirtualFabricsConfig(vfsip, smp);

	if (xml_memory_debug)
		fprintf(stdout, "Memory level %u after renderVirtualFabricsConfig()\n", (unsigned int)memory);

	return vfsip;

fail:
	if (vfsip)
		releaseVirtualFabricsConfig(vfsip);
	return NULL;
}


#ifndef __VXWORKS__
// Syslog Facilities lookup table
static struct log_facility {
    char *name;
    int facility;
} log_facilities[] = {
	{ "auth", LOG_AUTH },
	{ "authpriv", LOG_AUTHPRIV },
	{ "cron", LOG_CRON },
	{ "daemon", LOG_DAEMON },
	{ "ftp", LOG_FTP },
	{ "kern", LOG_KERN },
	{ "local0", LOG_LOCAL0 },
	{ "local1", LOG_LOCAL1 },
	{ "local2", LOG_LOCAL2 },
	{ "local3", LOG_LOCAL3 },
	{ "local4", LOG_LOCAL4 },
	{ "local5", LOG_LOCAL5 },
	{ "local6", LOG_LOCAL6 },
	{ "local7", LOG_LOCAL7 },
	{ "lpr", LOG_LPR },
	{ "mail", LOG_MAIL },
	{ "news", LOG_NEWS },
	{ "syslog", LOG_SYSLOG },
	{ "user", LOG_USER },
	{ "uucp", LOG_UUCP },
	{ NULL, 0 }
};

// Translate Syslog Facility string to a integer value
int getFacility(char* name, uint8_t test)
{
	struct log_facility* p;

	if (!name)
		return -1;

	for (p = log_facilities; p->name != NULL; ++p) {
		if (strcmp(name, p->name) == 0)
			return p->facility;
	}
	// if not testing then return the default if not found
	if (!test)
		return LOG_LOCAL6;
	return -1;
}

// Show possible Facility settings
char* showFacilities(char *facility, size_t size)
{
	struct log_facility* p;
	int res = 0;

	if (!facility)
		return NULL;

	facility[0] = 0;

	for (p = log_facilities; p->name != NULL; ++p) {
		res = snprintf(facility, size, "%s%s", p->name, ((p + 1)->name != NULL) ? ", " : "");
		if(res > 0 && res < size)
			size -= res;
		else
			break;
	}
	return facility;
}
#endif

static IXML_FIELD SmDPLifetimeFields[] = {
	{ tag:"Enable", format:'u', IXML_FIELD_INFO(SMDPLXmlConfig_t, dp_lifetime[0]) },
	{ tag:"Hops01_Int", format:'u', IXML_FIELD_INFO(SMDPLXmlConfig_t, dp_lifetime[1]) },
	{ tag:"Hops01", format:'k', IXML_FIELD_INFO(SMDPLXmlConfig_t, dp_lifetime[1]), end_func:IXmlParserEndTimeoutMult32_Str },
	{ tag:"Hops02_Int", format:'u', IXML_FIELD_INFO(SMDPLXmlConfig_t, dp_lifetime[2]) },
	{ tag:"Hops02", format:'k', IXML_FIELD_INFO(SMDPLXmlConfig_t, dp_lifetime[2]), end_func:IXmlParserEndTimeoutMult32_Str },
	{ tag:"Hops03_Int", format:'u', IXML_FIELD_INFO(SMDPLXmlConfig_t, dp_lifetime[3]) },
	{ tag:"Hops03", format:'k', IXML_FIELD_INFO(SMDPLXmlConfig_t, dp_lifetime[3]), end_func:IXmlParserEndTimeoutMult32_Str },
	{ tag:"Hops04_Int", format:'u', IXML_FIELD_INFO(SMDPLXmlConfig_t, dp_lifetime[4]) },
	{ tag:"Hops04", format:'k', IXML_FIELD_INFO(SMDPLXmlConfig_t, dp_lifetime[4]), end_func:IXmlParserEndTimeoutMult32_Str },
	{ tag:"Hops05_Int", format:'u', IXML_FIELD_INFO(SMDPLXmlConfig_t, dp_lifetime[5]) },
	{ tag:"Hops05", format:'k', IXML_FIELD_INFO(SMDPLXmlConfig_t, dp_lifetime[5]), end_func:IXmlParserEndTimeoutMult32_Str },
	{ tag:"Hops06_Int", format:'u', IXML_FIELD_INFO(SMDPLXmlConfig_t, dp_lifetime[6]) },
	{ tag:"Hops06", format:'k', IXML_FIELD_INFO(SMDPLXmlConfig_t, dp_lifetime[6]), end_func:IXmlParserEndTimeoutMult32_Str },
	{ tag:"Hops07_Int", format:'u', IXML_FIELD_INFO(SMDPLXmlConfig_t, dp_lifetime[7]) },
	{ tag:"Hops07", format:'k', IXML_FIELD_INFO(SMDPLXmlConfig_t, dp_lifetime[7]), end_func:IXmlParserEndTimeoutMult32_Str },
	{ tag:"Hops08_Int", format:'u', IXML_FIELD_INFO(SMDPLXmlConfig_t, dp_lifetime[8]) },
	{ tag:"Hops08", format:'k', IXML_FIELD_INFO(SMDPLXmlConfig_t, dp_lifetime[8]), end_func:IXmlParserEndTimeoutMult32_Str },
	{ tag:"Hops09_Int", format:'u', IXML_FIELD_INFO(SMDPLXmlConfig_t, dp_lifetime[9]) },
	{ tag:"Hops09", format:'k', IXML_FIELD_INFO(SMDPLXmlConfig_t, dp_lifetime[9]), end_func:IXmlParserEndTimeoutMult32_Str },
	{ NULL }
};

// Sm "DynamicPacketLifetime" start tag
static void* SmDPLifetimeXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	SMDPLXmlConfig_t *dplp = getXmlMemory(sizeof(SMDPLXmlConfig_t), "SMDPLXmlConfig_t SmDPLifetimeXmlParserStart()");

	if (xml_parse_debug)
		fprintf(stdout, "SmDPLifetimeXmlParserStart instance %u common %u\n", (unsigned int)instance, (unsigned int)common);

	if (!dplp) {
		PRINT_MEMORY_ERROR;
		return NULL;
	}

	// inherit the values of config items already set
	if (!common && configp->fm_instance[instance])
		*dplp = configp->fm_instance[instance]->sm_dpl_config;
	else
		*dplp = configp->fm_instance_common->sm_dpl_config;

	return dplp;			// will be passed to SmDPLifetimeXmlParserEnd as object
}

// Sm "DynamicPacketLifetime" end tag
static void SmDPLifetimeXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	SMDPLXmlConfig_t *dplp = (SMDPLXmlConfig_t *)IXmlParserGetField(field, object);

	if (xml_parse_debug)
		fprintf(stdout, "SmDPLifetimeXmlParserEnd instance %u common %u\n", (unsigned int)instance, (unsigned int)common);

	if (!valid) {
		fprintf(stderr, "Error processing XML Sm DynamicPacketLifetime tag\n");
	} else {
		// as needed process or validate self consistency of config
	}

	if (common) {
		// save the common settings for this instance
		configp->fm_instance_common->sm_dpl_config = *dplp;
	} else if (configp->fm_instance[instance]) {
		// save the Sm config for this instance
		configp->fm_instance[instance]->sm_dpl_config = *dplp;
	}

	freeXmlMemory(dplp, sizeof(SMDPLXmlConfig_t), "SMDPLXmlConfig_t SmDPLifetimeXmlParserEnd()");
}


// "Sm/Appliances" start tag
static void* SmAppliancesXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	return &((SMXmlConfig_t *)parent)->appliances;
}

static IXML_FIELD SmAppliancesFields[] = {
	{ tag:"Enable", format:'u', IXML_FIELD_INFO(SmAppliancesXmlConfig_t, enable) },
	{ tag:"Appliance01", format:'h', IXML_FIELD_INFO(SmAppliancesXmlConfig_t, guids[0]) },
	{ tag:"Appliance02", format:'h', IXML_FIELD_INFO(SmAppliancesXmlConfig_t, guids[1]) },
	{ tag:"Appliance03", format:'h', IXML_FIELD_INFO(SmAppliancesXmlConfig_t, guids[2]) },
	{ tag:"Appliance04", format:'h', IXML_FIELD_INFO(SmAppliancesXmlConfig_t, guids[3]) },
	{ tag:"Appliance05", format:'h', IXML_FIELD_INFO(SmAppliancesXmlConfig_t, guids[4]) },
	{ NULL }
};

const char*
SmPreDefFieldEnfToText(FieldEnforcementLevel_t fieldEnfLevel) {
	switch(fieldEnfLevel) {
		case(FIELD_ENF_LEVEL_DISABLED): return "Disabled";
		case(FIELD_ENF_LEVEL_WARN): return "Warn";
		case(FIELD_ENF_LEVEL_ENABLED): return "Enabled";
		default: return "Unknown";
	}
}

static void* SmPreDefTopoXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	return &((SMXmlConfig_t*)parent)->preDefTopo;
}

static void* SmPreDefTopoFieldEnfXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	return &((SmPreDefTopoXmlConfig_t*)parent)->fieldEnforcement;
}

static void SmPreDefTopoFieldEnfParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid) {
	FieldEnforcementLevel_t* fel = (FieldEnforcementLevel_t*)IXmlParserGetField(field, object);

	if (0 == strcasecmp(content, "Disabled")) {
		*fel = FIELD_ENF_LEVEL_DISABLED;
	} else if (0 == strcasecmp(content, "Warn")) {
		*fel = FIELD_ENF_LEVEL_WARN;
	} else if (0 == strcasecmp(content, "Enabled")) {
		*fel = FIELD_ENF_LEVEL_ENABLED;
	} else {
		IXmlParserPrintError(state, "FieldEnforcementLevel(s) must be (Disabled, Warn, or Enabled)");
		return;
	}
}

static void SmPreDefTopoXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
        if ((configp->fm_instance_common->sm_config.preDefTopo.fieldEnforcement.undefinedLink == FIELD_ENF_LEVEL_DISABLED) &&
                ((configp->fm_instance_common->sm_config.preDefTopo.fieldEnforcement.nodeGuid > FIELD_ENF_LEVEL_DISABLED) ||
                (configp->fm_instance_common->sm_config.preDefTopo.fieldEnforcement.portGuid > FIELD_ENF_LEVEL_DISABLED) ||
                (configp->fm_instance_common->sm_config.preDefTopo.fieldEnforcement.nodeDesc > FIELD_ENF_LEVEL_DISABLED))){
                IXmlParserPrintWarning(state, "UndefinedLink is disabled. Disabling NodeDesc/NodeGUID/PortGUID.");
                configp->fm_instance_common->sm_config.preDefTopo.fieldEnforcement.nodeGuid = FIELD_ENF_LEVEL_DISABLED;
                configp->fm_instance_common->sm_config.preDefTopo.fieldEnforcement.nodeDesc = FIELD_ENF_LEVEL_DISABLED;
                configp->fm_instance_common->sm_config.preDefTopo.fieldEnforcement.portGuid  = FIELD_ENF_LEVEL_DISABLED;
        }
}

static IXML_FIELD SmPreDefTopoFieldEnfFields[] = {
	{ tag:"NodeGUID", format:'k', end_func:SmPreDefTopoFieldEnfParserEnd, IXML_FIELD_INFO(SmPreDefTopoFieldEnfXmlConfig_t, nodeGuid) },
	{ tag:"NodeDesc", format:'k', end_func:SmPreDefTopoFieldEnfParserEnd, IXML_FIELD_INFO(SmPreDefTopoFieldEnfXmlConfig_t, nodeDesc) },
	{ tag:"PortGUID", format:'k', end_func:SmPreDefTopoFieldEnfParserEnd, IXML_FIELD_INFO(SmPreDefTopoFieldEnfXmlConfig_t, portGuid) },
	{ tag:"UndefinedLink", format:'k', end_func:SmPreDefTopoFieldEnfParserEnd, IXML_FIELD_INFO(SmPreDefTopoFieldEnfXmlConfig_t, undefinedLink) },
	{ NULL }
};

static IXML_FIELD SmPreDefTopoFields[] = {
	{ tag:"Enabled", format:'u', IXML_FIELD_INFO(SmPreDefTopoXmlConfig_t, enabled) },
	{ tag:"Enable", format:'u', IXML_FIELD_INFO(SmPreDefTopoXmlConfig_t, enabled) },
	{ tag:"TopologyFilename", format:'s', IXML_FIELD_INFO(SmPreDefTopoXmlConfig_t, topologyFilename) },
	{ tag:"LogMessageThreshold", format:'u', IXML_FIELD_INFO(SmPreDefTopoXmlConfig_t, logMessageThreshold) },
	{ tag:"FieldEnforcement", format:'k', subfields:SmPreDefTopoFieldEnfFields, start_func:SmPreDefTopoFieldEnfXmlParserStart },
	{ NULL }
};

static void NormalizeGuidStringParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	char *p = (char *)IXmlParserGetField(field, object);
	uint64_t mgid1, mgid2;

	if (xml_parse_debug)
		fprintf(stdout, "NormalizeGuidStringParserEnd instance %u common %u\n", (unsigned int)instance, (unsigned int)common);

	if (!valid){
		fprintf(stderr, "Error processing XML %s tag\n", field->tag);
        return;
    }

	if (FSUCCESS != StringToGid(&mgid1, &mgid2, content, NULL, TRUE)) {
		IXmlParserPrintError(state, "Error processing XML %s tag value, \"%s\"",field->tag,content);
		return;
	}
	sprintf(p, "%16.16llx%16.16llx",
		(long long unsigned int)mgid1, (long long unsigned int)mgid2);
}

static void MLIDShareMGidLimitParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint32_t value;

	if (xml_parse_debug)
		fprintf(stdout, "%s instance %u common %u\n", __func__, (unsigned int)instance, (unsigned int)common);

	if (!IXmlParseUint32(state, content, len, &value)) {
		fprintf(stderr, "Error processing XML %s tag\n", field->tag);
		return;
	}

	if (value == 0) {
		IXmlParserPrintError(state, "Error processing XML %s tag: value (%u) must be greater than 0\n", field->tag, value);
		return;
	}

	*(uint32_t *)IXmlParserGetField(field, object) = value;
}

// fields within Sm Multicast "MLIDShare" tag
static IXML_FIELD SmMcastMlidShareFields[] = {
	{ tag:"Enable", format:'h', IXML_FIELD_INFO(SmMcastMlidShared_t, enable) },
	{ tag:"MGIDMask", format:'K', IXML_FIELD_INFO(SmMcastMlidShared_t, mcastGrpMGidLimitMaskConvert), end_func:NormalizeGuidStringParserEnd },
	{ tag:"MGIDValue", format:'K', IXML_FIELD_INFO(SmMcastMlidShared_t, mcastGrpMGidLimitValueConvert), end_func:NormalizeGuidStringParserEnd },
	{ tag:"MaxMLIDs", format:'U', IXML_FIELD_INFO(SmMcastMlidShared_t, mcastGrpMGidLimitMax), end_func:MLIDShareMGidLimitParserEnd },
	{ NULL }
};

// Sm "MLIDShare" start tag
static void* SmMcastMlidShareXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	SmMcastMlidShared_t *dmsp = getXmlMemory(sizeof(SmMcastMlidShared_t), "SmMcastMlidShared_t SmMcastMlidShareXmlParserStart()");

	if (xml_parse_debug)
		fprintf(stdout, "SmMcastMlidShareXmlParserStart instance %u mlidSharedInstance %u common %u\n",
			(unsigned int)instance, (unsigned int)mlidSharedInstance, (unsigned int)common);

	if (!dmsp) {
		PRINT_MEMORY_ERROR;
		return NULL;
	}

	// inherit the values of config items already set
	if (!common && configp->fm_instance[instance])
		*dmsp = configp->fm_instance[instance]->sm_mls_config.mcastMlid[mlidSharedInstance];
	else
		*dmsp = configp->fm_instance_common->sm_mls_config.mcastMlid[mlidSharedInstance];

	return dmsp;			// will be passed to SmMcastMlidShareXmlParserEnd as object
}

// Sm "MLIDShare" end tag
static void SmMcastMlidShareXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	SmMcastMlidShared_t *dmsp = (SmMcastMlidShared_t*)IXmlParserGetField(field, object);
	uint32_t i;
	char check[MAX_VFABRIC_NAME];

	if (xml_parse_debug)
		fprintf(stdout, "SmMcastMlidShareXmlParserEnd instance %u mlidSharedInstance %u common %u\n",
			(unsigned int)instance, (unsigned int)mlidSharedInstance, (unsigned int)common);

	// check next MLIDShare instance
	if (mlidSharedInstance >= MAX_SUPPORTED_MCAST_GRP_CLASSES_XML) {
		IXmlParserPrintError(state, "Maximum number of MLIDShare instances exceeded");
		freeXmlMemory(dmsp, sizeof(SmMcastMlidShared_t), "SmMcastMlidShared_t SmMcastMlidShareXmlParserEnd()");
		return;
	}

	if (!valid) {
		fprintf(stderr, "Error processing XML Sm MLIDShare tag\n");
	}

	// if enabled then check for duplicate settings
	if (dmsp->enable) {
		for (i = 0; i < mlidSharedInstance; i++) {
			if (common)
				cs_strlcpy(check, configp->fm_instance_common->sm_mls_config.mcastMlid[i].mcastGrpMGidLimitValueConvert.value, sizeof(check));
			else
				cs_strlcpy(check, configp->fm_instance[instance]->sm_mls_config.mcastMlid[i].mcastGrpMGidLimitValueConvert.value, sizeof(check));
			if (strcasecmp(dmsp->mcastGrpMGidLimitValueConvert.value, check) == 0) {
				IXmlParserPrintError(state, "Duplicate MGID %s encountered in MLIDShare tag", dmsp->mcastGrpMGidLimitValueConvert.value);
				freeXmlMemory(dmsp, sizeof(SmMcastMlidShared_t), "SmMcastMlidShared_t SmMcastMlidShareXmlParserEnd()");
		return;
	}
		}
	}

	// only save if enabled
	if (dmsp->enable) {
		if (common) {
			// save the common settings for this instance
			configp->fm_instance_common->sm_mls_config.mcastMlid[mlidSharedInstance] = *dmsp;
		} else if (configp->fm_instance[instance]) {
			// save the Sm config for this instance
			configp->fm_instance[instance]->sm_mls_config.mcastMlid[mlidSharedInstance] = *dmsp;
	}

		// index to next MLIDShare instance
		mlidSharedInstance++;
	}

	freeXmlMemory(dmsp, sizeof(SmMcastMlidShared_t), "SmMcastMlidShared_t SmMcastMlidShareXmlParserEnd()");
}

// "DefaultGroup MGID" end tag
static void VfDgMGidEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	if (xml_parse_debug)
		fprintf(stdout, "VfDgMGidEnd instance %u defaultGroupInstance %u dgMgidInstance %u common %u\n",
			(unsigned int)instance, (unsigned int)defaultGroupInstance, (unsigned int)dgMgidInstance, (unsigned int)common);

	// check for max
	if (dgMgidInstance >= MAX_VFABRIC_DG_MGIDS) {
		IXmlParserPrintError(state, "Maximum number of MulticastGroup MGID instances exceeded");
		return;
	}

	if (!valid) {
		fprintf(stderr, "Error processing XML MGID tag\n");
	} else if (!content || !mdgp || strlen(content) > MAX_VFABRIC_NAME - 1) {
		IXmlParserPrintError(state, "MGID is too long - ignoring");
		return;
	}

	// save away the MGID to the parent structure
	snprintf(mdgp->mgid[dgMgidInstance].mgid, sizeof(mdgp->mgid[dgMgidInstance].mgid), "%s", content);

	// test conversion
	VFAppMgid_t mgidMapping;
	if (verifyAndConvertMGidString(content, &mgidMapping) < 0) {
		IXmlParserPrintError(state, "Bad MGID format %s", content);
		return;
	}

	// index to next MGID instance
	dgMgidInstance++;
}

// fields within Sm Multicast "DefaultGroup" tag
static IXML_FIELD SmMcastDgFields[] = {
	{ tag:"Create", format:'u', IXML_FIELD_INFO(SMMcastDefGrp_t, def_mc_create) },
	{ tag:"VirtualFabric", format:'s', IXML_FIELD_INFO(SMMcastDefGrp_t, virtual_fabric) },
	{ tag:"PKey", format:'h', IXML_FIELD_INFO(SMMcastDefGrp_t, def_mc_pkey),  end_func:PKeyParserEnd},
	{ tag:"MTU_Int", format:'u', IXML_FIELD_INFO(SMMcastDefGrp_t, def_mc_mtu_int) },
	{ tag:"MTU", format:'k', IXML_FIELD_INFO(SMMcastDefGrp_t, def_mc_mtu_int), end_func:MtuU8XmlParserEnd },
	{ tag:"Rate_Int", format:'u', IXML_FIELD_INFO(SMMcastDefGrp_t, def_mc_rate_int) },
	{ tag:"Rate", format:'k', IXML_FIELD_INFO(SMMcastDefGrp_t, def_mc_rate_int), end_func:RateU8XmlParserEnd },
	{ tag:"SL", format:'h', IXML_FIELD_INFO(SMMcastDefGrp_t, def_mc_sl) },
	{ tag:"QKey", format:'h', IXML_FIELD_INFO(SMMcastDefGrp_t, def_mc_qkey) },
	{ tag:"FlowLabel", format:'h', IXML_FIELD_INFO(SMMcastDefGrp_t, def_mc_fl) },
	{ tag:"TClass", format:'h', IXML_FIELD_INFO(SMMcastDefGrp_t, def_mc_tc) },
	{ tag:"MGID", format:'k', end_func:VfDgMGidEnd },
	{ NULL }
};

// Sm "MulticastGroup" start tag
static void* SmMcastDgXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	if (xml_parse_debug)
		fprintf(stdout, "SmMcastDgXmlParserStart instance %u defaultGroupInstance %u common %u\n",
			(unsigned int)instance, (unsigned int)defaultGroupInstance, (unsigned int)common);

	mdgp = getXmlMemory(sizeof(SMMcastDefGrp_t), "SMMcastDefGrp_t SmMcastDgXmlParserStart()");

	if (!mdgp) {
		PRINT_MEMORY_ERROR;
		return NULL;
	}

	*mdgp = mdgEmpty;

	// clear instances
	dgMgidInstance = 0;
	dgMgidRangeInstance = 0;
	dgMgidMaskedInstance = 0;

	return mdgp;			// will be passed to SmMcastDgXmlParserEnd as object
}

// Sm "MulticastGroup" end tag
static void SmMcastDgXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	if (xml_parse_debug)
		fprintf(stdout, "SmMcastDgXmlParserEnd instance %u defaultGroupInstance %u common %u\n",
			(unsigned int)instance, (unsigned int)defaultGroupInstance, (unsigned int)common);

	mdgp = (SMMcastDefGrp_t *)IXmlParserGetField(field, object);

	if (!valid) {
		fprintf(stderr, "Error processing XML Sm MulticastGroup tag\n");
		goto cleanup;
	}

	// if the group is disabled; skip it
	if (mdgp->def_mc_create == 0) {
		if (xml_parse_debug)
			fprintf(stdout, "MulticastGroup ingnored: Create == 0\n");
		goto cleanup;
	}

	if (mdgp->def_mc_create == UNDEFINED_XML32)
		mdgp->def_mc_create=1;

	if (mdgp->def_mc_mtu_int == UNDEFINED_XML8)
		mdgp->def_mc_mtu_int = IB_MTU_2048;

	if (mdgp->def_mc_rate_int == UNDEFINED_XML8)
		mdgp->def_mc_rate_int =IB_STATIC_RATE_25G;

	if (mdgp->def_mc_qkey == UNDEFINED_XML32)
		mdgp->def_mc_qkey=0;
	if (mdgp->def_mc_fl == UNDEFINED_XML32)
		mdgp->def_mc_fl=0;
	if (mdgp->def_mc_tc == UNDEFINED_XML32)
		mdgp->def_mc_tc=0;
	if (mdgp->prejoin_allowed == UNDEFINED_XML32)
		mdgp->prejoin_allowed=0;

	// check for max (AFTER skipping uncreatable groups)
	if (defaultGroupInstance >= MAX_DEFAULT_GROUPS) {
		PRINT_MEMORY_ERROR;
		goto cleanup;
	}

	if (mdgp->def_mc_sl != UNDEFINED_XML8 && mdgp->def_mc_sl >= STL_CONFIGURABLE_SLS) {
		IXmlParserPrintError(state, "MulticastGroup SL must be in range 0-%u.", (STL_CONFIGURABLE_SLS - 1));
		goto cleanup;
	}

	// keep track of the number of elements
	mdgp->number_of_mgids = dgMgidInstance;
	mdgp->number_of_mgid_ranges = dgMgidRangeInstance;
	mdgp->number_of_mgid_range_maskeds = dgMgidMaskedInstance;

	if (common) {
		// save the common settings for this instance
		configp->fm_instance_common->sm_mdg_config.group[defaultGroupInstance] = *mdgp;
	} else if (configp->fm_instance[instance]) {
		// save the Sm config for this instance
		configp->fm_instance[instance]->sm_mdg_config.group[defaultGroupInstance] = *mdgp;
	}

	// index to next DefaultGroup instance
	defaultGroupInstance++;

cleanup:
	freeXmlMemory(mdgp, sizeof(SMMcastDefGrp_t), "SMMcastDefGrp_t SmMcastDgXmlParserEnd()");
}

static IXML_FIELD SmMcastFields[] = {
	{ tag:"DisableStrictCheck", format:'u', IXML_FIELD_INFO(SMMcastConfig_t, disable_mcast_check) },
	{ tag:"EnablePruning", format:'u', IXML_FIELD_INFO(SMMcastConfig_t, enable_pruning) },
	{ tag:"MLIDTableCap", format:'u', IXML_FIELD_INFO(SMMcastConfig_t, mcast_mlid_table_cap) },
	{ tag:"RootSelectionAlgorithm", format:'s', IXML_FIELD_INFO(SMMcastConfig_t, mcroot_select_algorithm) },
	{ tag:"MinCostImprovement", format:'s', IXML_FIELD_INFO(SMMcastConfig_t, mcroot_min_cost_improvement) },
	{ tag:"MLIDShare", format:'k', subfields:SmMcastMlidShareFields, start_func:SmMcastMlidShareXmlParserStart, end_func:SmMcastMlidShareXmlParserEnd },
	{ tag:"MulticastGroup", format:'k', subfields:SmMcastDgFields, start_func:SmMcastDgXmlParserStart, end_func:SmMcastDgXmlParserEnd },
    { NULL }
};

// Sm "Multicast" start tag
static void* SmMcastXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	SMMcastConfig_t *mcp = getXmlMemory(sizeof(SMMcastConfig_t), "SMMcastConfig_t SmMcastXmlParserStart()");

	if (xml_parse_debug)
		fprintf(stdout, "SmMcastXmlParserStart instance %u common %u\n", (unsigned int)instance, (unsigned int)common);

	if (!mcp) {
		PRINT_MEMORY_ERROR;
		return NULL;
	}

	// inherit the values of config items already set
	if (!common && configp->fm_instance[instance])
		*mcp = configp->fm_instance[instance]->sm_mc_config;
	else
		*mcp = configp->fm_instance_common->sm_mc_config;

	if (common) {
		// clear instance
		defaultGroupInstance = 0;
		mlidSharedInstance = 0;
	} else if (configp->fm_instance[instance]) {
		defaultGroupInstance = configp->fm_instance[instance]->sm_mdg_config.number_of_groups;
		mlidSharedInstance = configp->fm_instance[instance]->sm_mls_config.number_of_shared;
	} else {
		defaultGroupInstance = 0;
		mlidSharedInstance = 0;
	}

	return mcp;			// will be passed to SmMcastXmlParserEnd as object
}

// Sm "Multicast" end tag
static void SmMcastXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	SMMcastConfig_t *mcp = (SMMcastConfig_t*)IXmlParserGetField(field, object);

	if (xml_parse_debug)
		fprintf(stdout, "SmMcastXmlParserEnd instance %u common %u\n", (unsigned int)instance, (unsigned int)common);

	if (!valid) {
		fprintf(stderr, "Error processing XML Sm Multicast tag\n");
	} else {
		// as needed process or validate self consistency of config
	}

	if (common) {
		// save the common settings for this instance
		configp->fm_instance_common->sm_mc_config = *mcp;
	} else if (configp->fm_instance[instance]) {
		// save the Sm config for this instance
		configp->fm_instance[instance]->sm_mc_config = *mcp;
	}

	// save the number of default groups and MLIDShared instances
	if (common) {
		configp->fm_instance_common->sm_mdg_config.number_of_groups = defaultGroupInstance;
		configp->fm_instance_common->sm_mls_config.number_of_shared = mlidSharedInstance;
	} else if (configp->fm_instance[instance]) {
		configp->fm_instance[instance]->sm_mdg_config.number_of_groups = defaultGroupInstance;
		configp->fm_instance[instance]->sm_mls_config.number_of_shared = mlidSharedInstance;
	}

	freeXmlMemory(mcp, sizeof(SMMcastConfig_t), "SMMcastConfig_t SmMcastXmlParserEnd()");
}

static void* SmLinkWidthPolicyXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
    return &((SMLinkPolicyXmlConfig_t *)parent)->width_policy;
}

static void* SmLinkSpeedPolicyXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
    return &((SMLinkPolicyXmlConfig_t *)parent)->speed_policy;
}

// fields within "Sm/LinkPolicy" tag

static void SmLinkSpeedPolicyXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
    uint16_t *p = (uint16_t *)IXmlParserGetField(field, object);

	if (xml_parse_debug)
		fprintf(stdout, "SmLinkSpeedQuarantineXmlParserEnd instance %u common %u\n", (unsigned int)instance, (unsigned int)common);

	if (!valid) {
        fprintf(stderr, "Error processing XML Sm %s tag\n", field->tag);
		return;
	}
	if (!content) {
		IXmlParserPrintError(state, "Invalid Sm %s tag value, cannot be empty", field->tag);
		return;
	}

	if (0 == strcasecmp(content, "Supported")) {
        *p=0;
	} else {
		IXmlParserPrintError(state, "Invalid Value for Link Speed Policy: %s\n", content);
		return;
	}
}

static void SmLinkWidthPolicyXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
    uint16_t *p = (uint16_t *)IXmlParserGetField(field, object);

	if (xml_parse_debug)
		fprintf(stdout, "SmLinkWidthQuarantineXmlParserEnd instance %u common %u\n", (unsigned int)instance, (unsigned int)common);

	if (!valid){
        fprintf(stderr, "Error processing XML Sm %s tag\n", field->tag);
        return;
    }
	if (!content) {
		IXmlParserPrintError(state, "Invalid Sm %s tag value, cannot be empty", field->tag);
		return;
	}

	//If content is not "Supported, it must be a valid number.
	if (0 == strcasecmp(content, "Supported")) {
        *p=0;
	} else if (0 == strcasecmp(content, "4x")) {
        *p=STL_LINK_WIDTH_4X;
	} else if (0 == strcasecmp(content, "2x")) {
        *p=STL_LINK_WIDTH_2X;
	} else if (0 == strcasecmp(content, "3x")) {
        *p=STL_LINK_WIDTH_3X;
	} else {
		IXmlParserPrintError(state, "Invalid Value for width Quarantine Policy: %s\n", content);
		return;
	}
}


static IXML_FIELD SmLinkSpeedPolicyFields[] = {
    { tag:"Enable", format:'u', IXML_FIELD_INFO(SMPolicyConfig_t, enabled) },
	{ tag:"Policy", format:'k', IXML_FIELD_INFO(SMPolicyConfig_t, policy), end_func:SmLinkSpeedPolicyXmlParserEnd},
	{ NULL }
};

static IXML_FIELD SmLinkWidthPolicyFields[] = {
    { tag:"Enable", format:'u', IXML_FIELD_INFO(SMPolicyConfig_t, enabled)},
	{ tag:"Policy", format:'k', IXML_FIELD_INFO(SMPolicyConfig_t, policy), end_func:SmLinkWidthPolicyXmlParserEnd},
	{ NULL }
};

static IXML_FIELD SmLinkPolicyFields[] = {
	{ tag:"MaxDroppedLanes", format:'u', IXML_FIELD_INFO(SMLinkPolicyXmlConfig_t, link_max_downgrade)},
	{ tag:"SpeedPolicy", format:'k', subfields:SmLinkSpeedPolicyFields, start_func:SmLinkSpeedPolicyXmlParserStart},
	{ tag:"WidthPolicy", format:'k', subfields:SmLinkWidthPolicyFields, start_func:SmLinkWidthPolicyXmlParserStart},
	{ NULL }
};

// "Sm/HFILink Policy" start tag
static void* SmHFILinkPolicyXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	return &((SMXmlConfig_t *)parent)->hfi_link_policy;
}

// "Sm/ISLLink Policy" start tag
static void* SmISLLinkPolicyXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	return &((SMXmlConfig_t *)parent)->isl_link_policy;
}

static int minMaxNumString(IXmlParserState_t *state, const IXML_FIELD *field, char *str, uint32_t min, uint32_t max, uint32_t inc, boolean allowZero, uint32_t inf)
{
	int value;
	char *c;

	if (!strcasecmp(str,"MIN")) return min;
	else if (!strcasecmp(str,"MAX")) return max;
	else if (inf > 0 && !strcasecmp(str,"INF")) return inf;

	value = strtol(str,&c,10);
	if (*c) {
		IXmlParserPrintError(state, "Invalid Sm %s tag value, \"%s\"", field->tag, str);
		return -1;
	}
	if (value == 0 && allowZero) return 0;
	if (value < min) {
		IXmlParserPrintWarning(state, "%s tag value (%d) less than minimum (%d), using minimum",field->tag,value,min);
		value = min;
	}
	if (value > max) {
		IXmlParserPrintWarning(state, "%s tag value (%d) greater than maximum (%d), using maximum",field->tag,value,max);
		value = max;
	}
	if (value % inc) {
		int rounded = ROUNDUP(value, inc);
		IXmlParserPrintWarning(state, "%s tag value (%d), rounding to multiple of %d, using (%d)",field->tag,value,inc,rounded);
		value = rounded;
	}
	return value;
}

static void PreemptSmallPktParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint32_t *p = (uint32_t *)IXmlParserGetField(field, object);

	if (xml_parse_debug)
		fprintf(stdout, "PreemptSmallPktParserEnd instance %u common %u\n", (unsigned int)instance, (unsigned int)common);

	if (!valid){
		fprintf(stderr, "Error processing XML Sm %s tag\n", field->tag);
		return;
	}
	if (!content) {
		IXmlParserPrintError(state, "Invalid Sm %s tag value, cannot be empty", field->tag);
		return;
	}
	*p = minMaxNumString(state, field, content, SM_PREEMPT_SMALL_PACKET_MIN, SM_PREEMPT_SMALL_PACKET_MAX, SM_PREEMPT_SMALL_PACKET_INC, FALSE, 0);
}

static void PreemptLargePktParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint32_t *p = (uint32_t *)IXmlParserGetField(field, object);

	if (xml_parse_debug)
		fprintf(stdout, "PreemptLargePktParserEnd instance %u common %u\n", (unsigned int)instance, (unsigned int)common);

	if (!valid){
		fprintf(stderr, "Error processing XML Sm %s tag\n", field->tag);
		return;
	}
	if (!content) {
		IXmlParserPrintError(state, "Invalid Sm %s tag value, cannot be empty", field->tag);
		return;
	}
	*p = minMaxNumString(state, field, content, SM_PREEMPT_LARGE_PACKET_MIN, SM_PREEMPT_LARGE_PACKET_MAX, SM_PREEMPT_LARGE_PACKET_INC, FALSE, 0);
}

static void PreemptLimitParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint32_t *p = (uint32_t *)IXmlParserGetField(field, object);

	if (xml_parse_debug)
		fprintf(stdout, "PreemptLimitParserEnd instance %u common %u\n", (unsigned int)instance, (unsigned int)common);

	if (!valid){
		fprintf(stderr, "Error processing XML Sm %s tag\n", field->tag);
		return;
	}
	if (!content) {
		IXmlParserPrintError(state, "Invalid Sm %s tag value, cannot be empty", field->tag);
		return;
	}
	*p = minMaxNumString(state, field, content, SM_PREEMPT_LIMIT_MIN, SM_PREEMPT_LIMIT_MAX, SM_PREEMPT_LIMIT_INC, TRUE, SM_PREEMPT_LIMIT_INF);
}

// fields within "Sm/Preemption" tag
static IXML_FIELD SmPreemptionFields[] = {
	{ tag:"SmallPacket", format:'k', IXML_FIELD_INFO(SMPreemptionXmlConfig_t, small_packet), end_func:PreemptSmallPktParserEnd },
	{ tag:"LargePacket", format:'k', IXML_FIELD_INFO(SMPreemptionXmlConfig_t, large_packet), end_func:PreemptLargePktParserEnd },
	{ tag:"PreemptLimit", format:'k', IXML_FIELD_INFO(SMPreemptionXmlConfig_t, preempt_limit), end_func:PreemptLimitParserEnd },
	{ NULL }
};

// "Sm/Preemption" start tag
static void* SmPreemptionXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	SMPreemptionXmlConfig_t *preemption = &((SMXmlConfig_t *)parent)->preemption;

	// Initialize defaults
	return preemption;
}

// fields within "Sm/Congestion/Sw" tag
static IXML_FIELD SmSwCongestionFields[] = {
	{ tag:"VictimMarkingEnable", format:'u', IXML_FIELD_INFO(SmSwCongestionXmlConfig_t, victim_marking_enable) },
	{ tag:"Threshold", format:'u', IXML_FIELD_INFO(SmSwCongestionXmlConfig_t, threshold) },
	{ tag:"PacketSize", format:'u', IXML_FIELD_INFO(SmSwCongestionXmlConfig_t, packet_size) },
	{ tag:"CsThreshold", format:'u', IXML_FIELD_INFO(SmSwCongestionXmlConfig_t, cs_threshold) },
	{ tag:"CsReturnDelay", format:'u', IXML_FIELD_INFO(SmSwCongestionXmlConfig_t, cs_return_delay) },
	{ tag:"MarkingRate", format:'u', IXML_FIELD_INFO(SmSwCongestionXmlConfig_t, marking_rate) },
	{ NULL }
};

// "Sm/Congestion/Sw" start tag
static void* SmSwCongestionXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	return &((SmCongestionXmlConfig_t *)parent)->sw;
}

// fields within "Sm/Congestion/Ca" tag
static IXML_FIELD SmCaCongestionFields[] = {
	{ tag:"Basis", format:'k', IXML_FIELD_INFO(SmCaCongestionXmlConfig_t, sl_based), end_func:BasisU8XmlParserEnd },
	{ tag:"Increase", format:'u', IXML_FIELD_INFO(SmCaCongestionXmlConfig_t, increase) },
	{ tag:"Timer", format:'u', IXML_FIELD_INFO(SmCaCongestionXmlConfig_t, timer) },
	{ tag:"Threshold", format:'u', IXML_FIELD_INFO(SmCaCongestionXmlConfig_t, threshold) },
	{ tag:"Min", format:'u', IXML_FIELD_INFO(SmCaCongestionXmlConfig_t, min) },
	{ tag:"Limit", format:'u', IXML_FIELD_INFO(SmCaCongestionXmlConfig_t, limit) },
	{ tag:"DesiredMaxDelay", format:'u', IXML_FIELD_INFO(SmCaCongestionXmlConfig_t, desired_max_delay) },
	{ NULL }
};

// "Sm/Congestion/Ca" start tag
static void* SmCaCongestionXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	return &((SmCongestionXmlConfig_t *)parent)->ca;
}

// fields within "Sm/CongestionControl" tag
static IXML_FIELD SmCongestionFields[] = {
	{ tag:"Enable", format:'u', IXML_FIELD_INFO(SmCongestionXmlConfig_t, enable) },
	{ tag:"Debug", format:'u', IXML_FIELD_INFO(SmCongestionXmlConfig_t, debug) },
	{ tag:"Switch", format:'k', subfields:SmSwCongestionFields, start_func:SmSwCongestionXmlParserStart },
	{ tag:"Fi", format:'k', subfields:SmCaCongestionFields, start_func:SmCaCongestionXmlParserStart },
	{ NULL }
};


// "Sm/Congestion" start tag
static void* SmCongestionXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	return &((SMXmlConfig_t *)parent)->congestion;
}


// fields within "Sm/Mesh/Torus " tag
static void SmPortPairEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	int port1, port2;
	SmDimension_t *dim = (SmDimension_t*)parent;

	if (xml_parse_debug)
		fprintf(stdout, "SmPortPairEnd instance %u common %u\n",
			(unsigned int)instance, (unsigned int)common);

	if (!valid) {
		fprintf(stderr, "Error processing XML PortPair tag\n");
		return;
	} else if (!content || !dim || strlen(content) > 8) {
		IXmlParserPrintError(state, "PortPair is too long - ignoring");
		return;
	}

	if (dim->portCount >= MAX_SWITCH_PORTS) {
		IXmlParserPrintError(state, "PortPair configured exceeds max allowed %d", dim->portCount);
		return;
	}

	if (sscanf(content, "%d,%d", &port1, &port2) != 2) {
		IXmlParserPrintError(state, "PortPair %s is formatted incorrectly, expecting: port1,port2", content);
		return;
	}

	if ((port1 > 255) || (port2 > 255)) {
		IXmlParserPrintError(state, "PortPair %s is out of range (>255)", content);
		return;
	}

	dim->portPair[dim->portCount].port1 = port1;
	dim->portPair[dim->portCount].port2 = port2;
	dim->portCount++;
}

static void* SmDimensionStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	SmDorRouting_t  *dor = (SmDorRouting_t*)parent;

	if (xml_parse_debug)
		fprintf(stdout, "SmDimensionStart instance %u common %u\n",
			(unsigned int)instance, (unsigned int)common);

	if (!dor)
		return NULL;

	if (dor->dimensionCount >= MAX_DOR_DIMENSIONS) {
		IXmlParserPrintError(state, "Dimensions configured exceeds max allowed %d", dor->dimensionCount);
		return NULL;
	}

	dor->dimensionCount++;
	return &(dor->dimension[dor->dimensionCount-1]);
}

static void SmDorRouteLastEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	SmDorRouting_t *dor = (SmDorRouting_t*)parent;

	if (!valid) {
		fprintf(stderr, "Error processing XML RouteLast tag\n");
	} else if (!content || !dor || strlen(content) > MAX_VFABRIC_NAME) {
		IXmlParserPrintError(state, "RouteLast group name is too long - ignoring");
		return;
	}
	cs_strlcpy(dor->routeLast.member, content, MAX_VFABRIC_NAME);
}

static void* SmDorRoutingXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	SmDorRouting_t *dor = getXmlMemory(sizeof(SmDorRouting_t), "SmDorRouting_t SmDorRoutingXmlParserStart()");

	if (xml_parse_debug)
		fprintf(stdout, "SmDorRoutingXmlParserStart instance %u common %u\n",
			(unsigned int)instance, (unsigned int)common);

	if (!dor) {
		PRINT_MEMORY_ERROR;
		return NULL;
	}

	memset(dor, 0, sizeof(SmDorRouting_t));

	dor->warn_threshold = DEFAULT_DOR_PORT_PAIR_WARN_THRESHOLD;
	dor->escapeVLs = DEFAULT_ESCAPE_VLS_IN_USE;
	dor->faultRegions = DEFAULT_FAULT_REGIONS_IN_USE;

	return dor;
}

static void SmDorRoutingXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{

	uint8_t portsInUse[256];
    int  nonToroidalEdges = 0;
	int  configFailed = 0;
	int	 port, dim;

	if (xml_parse_debug)
		fprintf(stdout, "SmDorRoutingXmlParserEnd instance %u common %u\n",
			(unsigned int)instance, (unsigned int)common);

	SMXmlConfig_t	*smp = (SMXmlConfig_t*)parent;
	SmDorRouting_t  *dor = (SmDorRouting_t*)IXmlParserGetField(field, object);

	if (!dor) return;

	if (dor->dimensionCount == 0) {
		IXmlParserPrintError(state, "dor algorithm selected but no dimensions specified!");
	}

	memset (portsInUse, 0, sizeof(portsInUse));

	for (dim=0; dim<dor->dimensionCount; dim++) {
		if (dor->dimension[dim].portCount == 0) {
			IXmlParserPrintError(state, "MeshTorusTopology Dimension must be configured with a PortPair.");
			break;
		}

		if (dor->dimension[dim].toroidal) {
			dor->numToroidal++;
		} else {
			nonToroidalEdges = 1;
		}

		if (dor->dimension[dim].length == 0)
			IXmlParserPrintError(state, "The length of the dimension must be specified");

		// Verify no overlap for port pairs
		for (port=0; port<dor->dimension[dim].portCount; port++) {
			if (dor->dimension[dim].toroidal && dor->dimension[dim].portPair[port].port1 ==
				dor->dimension[dim].portPair[port].port2) {
				configFailed = 1;
				IXmlParserPrintError(state, "Port %d is used for both ports in dimension pair, hyperlink cannot be toroidal",
										dor->dimension[dim].portPair[port].port1);
			}
			if (!dor->dimension[dim].toroidal && dor->dimension[dim].length > 2
				&& dor->dimension[dim].portPair[port].port1 ==
				dor->dimension[dim].portPair[port].port2) {
				configFailed = 1;
				IXmlParserPrintError(state, "Port %d is used for both ports in"
					" dimension pair, hyperlink cannot have a dimension length > 2",
					dor->dimension[dim].portPair[port].port1);
			}
			if (portsInUse[dor->dimension[dim].portPair[port].port1] || portsInUse[dor->dimension[dim].portPair[port].port2]) {
				configFailed = 1;
				if (portsInUse[dor->dimension[dim].portPair[port].port1])
					IXmlParserPrintError(state, "Port %d is used in multiple dimension pairs",
											dor->dimension[dim].portPair[port].port1);
				if (portsInUse[dor->dimension[dim].portPair[port].port2])
					IXmlParserPrintError(state, "Port %d is used in multiple dimension pairs",
											dor->dimension[dim].portPair[port].port2);
		 		break;
			}
			portsInUse[dor->dimension[dim].portPair[port].port1] = 1;
			portsInUse[dor->dimension[dim].portPair[port].port2] = 1;
		}
		if (configFailed) break;
	}

	if (dor->numToroidal > MAX_TOROIDAL_DIMENSIONS) {
		IXmlParserPrintError(state, "The number of dimensions that may be configured as toroidal is limited to "add_quotes(MAX_TOROIDAL_DIMENSIONS)".");
	}

	if (dor->numToroidal && nonToroidalEdges) {
		dor->topology = DOR_PARTIAL_TORUS;
	} else if (dor->numToroidal) {
		dor->topology = DOR_TORUS;
	} else {
		dor->topology = DOR_MESH;
	}

	if (dor->topology == DOR_MESH) {
		dor->routingSCs = dor->escapeVLs ? 2 : 1;
	} else {
		dor->routingSCs = dor->escapeVLs ? 4 : 2;
	}

	smp->smDorRouting = *dor;

	freeXmlMemory(dor, sizeof(SmDorRouting_t), "SmDorRouting_t SmDorRoutingXmlParserEnd()");
}

static IXML_FIELD SmDimensionFields[] = {
	{ tag:"Toroidal", format:'u', IXML_FIELD_INFO(SmDimension_t, toroidal) },
	{ tag:"Length", format:'u', IXML_FIELD_INFO(SmDimension_t, length) },
	{ tag:"PortPair", format:'s', end_func:SmPortPairEnd },
	{ NULL }
};

static IXML_FIELD SmDorRoutingFields[] = {
	{ tag:"Debug", format:'u', IXML_FIELD_INFO(SmDorRouting_t, debug) },
	{ tag:"UseEscapeVLs", format:'u', IXML_FIELD_INFO(SmDorRouting_t, escapeVLs) },
	{ tag:"UseFaultRegions", format:'u', IXML_FIELD_INFO(SmDorRouting_t, faultRegions) },
	{ tag:"Dimension", format:'k', subfields:SmDimensionFields, start_func:SmDimensionStart },
	{ tag:"RouteLast", format:'k', end_func:SmDorRouteLastEnd },
	{ tag:"WarnThreshold", format:'u', IXML_FIELD_INFO(SmDorRouting_t, warn_threshold) },
	{ tag:"OverlayMulticast", format:'u', IXML_FIELD_INFO(SmDorRouting_t, overlayMCast) },
	{ NULL }
};

// fields within "Sm/Adaptive Routing" tag
static IXML_FIELD SmAdaptiveRoutingFields[] = {
	{ tag:"Enable", format:'u', IXML_FIELD_INFO(SmAdaptiveRoutingXmlConfig_t, enable) },
	{ tag:"Debug", format:'u', IXML_FIELD_INFO(SmAdaptiveRoutingXmlConfig_t, debug) },
	{ tag:"Algorithm", format:'u', IXML_FIELD_INFO(SmAdaptiveRoutingXmlConfig_t, algorithm) },
	{ tag:"LostRouteOnly", format:'u', IXML_FIELD_INFO(SmAdaptiveRoutingXmlConfig_t, lostRouteOnly) },
	{ tag:"ARFrequency", format:'u', IXML_FIELD_INFO(SmAdaptiveRoutingXmlConfig_t, arFrequency) },
	{ tag:"Threshold", format:'u', IXML_FIELD_INFO(SmAdaptiveRoutingXmlConfig_t, threshold) },
	{ NULL }
};

static void* SmFtreeRoutingXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	SmFtreeRouting_t *p = &((SMXmlConfig_t *)parent)->ftreeRouting;
	return p;
}

static void SmFtreeCoreSwitchEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	SmFtreeRouting_t *ftreeRouting = (SmFtreeRouting_t*)parent;

	if (!valid) {
		fprintf(stderr, "Error processing XML CoreSwitch tag\n");
	} else if (!content || !ftreeRouting || strlen(content) > MAX_VFABRIC_NAME) {
		IXmlParserPrintError(state, "CoreSwitch group name is too long - ignoring");
		return;
	}

	cs_strlcpy(ftreeRouting->coreSwitches.member, content, MAX_VFABRIC_NAME);
}

static void SmFtreeRouteLastEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	SmFtreeRouting_t *ftreeRouting = (SmFtreeRouting_t*)parent;

	if (!valid) {
		fprintf(stderr, "Error processing XML RouteLast tag\n");
	} else if (!content || !ftreeRouting || strlen(content) > MAX_VFABRIC_NAME) {
		IXmlParserPrintError(state, "RouteLast group name is too long - ignoring");
		return;
	}

	cs_strlcpy(ftreeRouting->routeLast.member, content, MAX_VFABRIC_NAME);
}

static void SmFtreeRoutingXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	if (!valid) {
		fprintf(stderr, "Error processing XML Sm FtreeRouting tag\n");
	}
}

static IXML_FIELD SmFtreeRoutingFields[] = {
	{ tag:"Debug", format:'u', IXML_FIELD_INFO(SmFtreeRouting_t, debug) },
	{ tag:"PassThrough", format:'u', IXML_FIELD_INFO(SmFtreeRouting_t, passthru) },
	{ tag:"FIsOnSameTier", format:'u', IXML_FIELD_INFO(SmFtreeRouting_t, fis_on_same_tier) },
	{ tag:"TierCount", format:'u', IXML_FIELD_INFO(SmFtreeRouting_t, tierCount) },
	{ tag:"CoreSwitches", format:'k', end_func:SmFtreeCoreSwitchEnd },
	{ tag:"RouteLast", format:'k', end_func:SmFtreeRouteLastEnd },
	{ NULL }
};


static void* DGRoutingOrderXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	SmDGRouting_t *p = (SmDGRouting_t *)parent;
	p->dgCount = 0;
	return p;
}

static void DGRoutingOrderXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	if (!valid) {
		fprintf(stderr, "Error processing XML DGRouting RoutingOrder tag\n");
	}
}

static void* SmDGRoutingXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	// Initialize structure to NULL.
	SmDGRouting_t *p = &((SMXmlConfig_t *)parent)->dgRouting;
	memset(p,0,sizeof(SmDGRouting_t));

	return p;
}

static void SmDGRoutingXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	if (!valid) {
		fprintf(stderr, "Error processing XML Sm DGRouting tag\n");
	}
}

static void DGRoutingDeviceGroupEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	SmDGRouting_t *p = (SmDGRouting_t *)parent;

	if (!valid) {
		fprintf(stderr, "Error processing XML DGRouting RoutingOrder DeviceGroup tag\n");
		return;
	} else if (p->dgCount >= MAX_DGROUTING_ORDER) {
		IXmlParserPrintError(state, "Error processing XML DGRouting RoutingOrder too many DeviceGroup tags\n");
		return;
	}

	cs_strlcpy(p->dg[p->dgCount++].member, content, MAX_VFABRIC_NAME);
}

static IXML_FIELD DGRoutingOrderFields[] = {
	{ tag:"DeviceGroup", format:'k', end_func:DGRoutingDeviceGroupEnd }
};

static IXML_FIELD SmDGRoutingFields[] = {
	{ tag:"RoutingOrder", format:'k', subfields:DGRoutingOrderFields, start_func:DGRoutingOrderXmlParserStart, end_func:DGRoutingOrderXmlParserEnd },
	{ NULL }
};

static IXML_FIELD XmlSPRoutingPortFields[] = {
	{ tag:"pPort", format:'u', IXML_FIELD_INFO(SmSPRoutingPort_t, pport) },
	{ tag:"vPort", format:'u', IXML_FIELD_INFO(SmSPRoutingPort_t, vport) },
	{ tag:"Cost", format:'u', IXML_FIELD_INFO(SmSPRoutingPort_t, cost) },
	{ NULL }
};

static void XmlSProutingSwitchesEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	SmSPRoutingCtrl_t *routingData = (SmSPRoutingCtrl_t*)parent;

	if (!valid) {
		fprintf(stderr, "Error processing XML Enhanced Hypercube Routing Control Switches tag\n");
	} else if (!content || !routingData || strlen(content) > MAX_VFABRIC_NAME) {
		IXmlParserPrintError(state, "Enhanced Hypercube Routing Control Switches group name is too long - ignoring");
		return;
	}

	cs_strlcpy(routingData->switches.member, content, MAX_VFABRIC_NAME);
}

static void XmlSPRoutingPortParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	SmSPRoutingPort_t *PortData = object;
	uint8_t pport = PortData->pport;
	uint8_t vport = PortData->vport;
	uint16_t cost = PortData->cost;
	SmSPRoutingCtrl_t *SPRoutingCtrl = parent;
	portMap_t *pportMap = SPRoutingCtrl->pportMap;
	portMap_t *vportMap = SPRoutingCtrl->vportMap;
	SmSPRoutingPort_t *ports = SPRoutingCtrl->ports;
	uint16_t portCount = SPRoutingCtrl->portCount;
	uint16_t portsSize = ROUNDUP(portCount, PORTS_ALLOC_UNIT);

	if (xml_parse_debug) {
		fprintf(stdout, "%s %s instance %u common %u\n", __FUNCTION__,
				field->tag, (unsigned int)instance, (unsigned int)common);
	}

	MemoryDeallocate(PortData);
	if (!valid) {
		fprintf(stderr, "Error processing XML %s tag\n", field->tag);
		return;
	}

	if (pport >= MAX_SWITCH_PORTS) {
		IXmlParserPrintError(state, "bad pPort %d", pport);
		return;
	}
	if (vport >= MAX_SWITCH_PORTS) {
		IXmlParserPrintError(state, "bad vPort %d", vport);
		return;
	}

	// check for duplicates

	if (pport) {
		if (portMapTest(pportMap, pport)) {
		  IXmlParserPrintError(state, "pPort %d already specified", pport);
			return;
		}
		portMapSet(pportMap, pport);
	}

	if (vport) {
		if (portMapTest(vportMap, vport)) {
		  IXmlParserPrintError(state, "vPort %d already specified", vport);
			return;
		}
		portMapSet(vportMap, vport);
	}

	if (portCount >= portsSize) {
		portsSize += PORTS_ALLOC_UNIT;
		ports = getXmlMemory(portsSize*sizeof(*ports), "SmSPRoutingPort_t *ports");
		if (!ports) {
			PRINT_MEMORY_ERROR;
			return;
		}
		if (portCount) {
			memcpy(ports, SPRoutingCtrl->ports, portCount*sizeof(*ports));
			freeXmlMemory(SPRoutingCtrl->ports, portCount*sizeof(*ports),
						  "SmSPRoutingPort_t *ports");
		}
		SPRoutingCtrl->ports = ports;
	}
	ports += portCount;
	ports->pport = pport;
	ports->vport = vport;
	ports->cost = cost;
	SPRoutingCtrl->portCount++;
};

static IXML_FIELD XmlSPRoutingCtrlFields[] = {
	{ tag:"Switches", format:'k', end_func:XmlSProutingSwitchesEnd },
	{ tag:"PortData", format:'k', size:sizeof(SmSPRoutingPort_t), subfields:XmlSPRoutingPortFields, start_func:IXmlParserStartStruct, end_func:XmlSPRoutingPortParserEnd },
	{ NULL }
};

static void XmlSPRoutingCtrlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	SmSPRoutingCtrl_t *XmlSPRoutingCtrl = object;
	XMLMember_t switches = XmlSPRoutingCtrl->switches;
	SmHypercubeRouting_t *ehc = (SmHypercubeRouting_t*)parent;
	SmSPRoutingCtrl_t **prev = &ehc->enhancedRoutingCtrl;
	SmSPRoutingCtrl_t *SPRoutingCtrl = ehc->enhancedRoutingCtrl;

	if (xml_parse_debug) {
		fprintf(stdout, "%s %s instance %u common %u\n", __FUNCTION__,
				field->tag, (unsigned int)instance, (unsigned int)common);
	}

	if (!valid) {
		fprintf(stderr, "Error processing XML %s tag\n", field->tag);
		goto done;
	}

	// see if we already have this GUID
	while (SPRoutingCtrl) {
		if (strcmp(SPRoutingCtrl->switches.member, switches.member) == 0) {
			if (strlen(switches.member) == 0) {
				IXmlParserPrintError(state, "Multiple Enhanced Hypercube Routing Control default groups encountered, only one default group allowed.");
			} else {
				IXmlParserPrintError(state, "Duplicate Enhanced Hypercube Routing Control Switches groups (%s) encountered",
								 switches.member);
			}
			goto done;
		}
		prev = &SPRoutingCtrl->next;
		SPRoutingCtrl = *prev;
	}
	SPRoutingCtrl = getXmlMemory(sizeof(*SPRoutingCtrl), "SmSPRoutingCtrl_t *SPRoutingCtrl");
	if (!SPRoutingCtrl) {
		PRINT_MEMORY_ERROR;
		goto done;
	}
	*SPRoutingCtrl = *XmlSPRoutingCtrl;
	*prev = SPRoutingCtrl;

done:
	MemoryDeallocate(XmlSPRoutingCtrl);
}

static void* SmHypercubeRoutingXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	SmHypercubeRouting_t *p = &((SMXmlConfig_t *)parent)->hypercubeRouting;
	return p;
}

static void SmHypercubeRoutingXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	if (!valid) {
		fprintf(stderr, "Error processing XML Sm Enhanced Hypercube tag\n");
	}
}

static void SmHyperRouteLastEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	SmHypercubeRouting_t *p = &((SMXmlConfig_t *)parent)->hypercubeRouting;

	if (!valid) {
		fprintf(stderr, "Error processing XML Hypercube RouteLast tag\n");
	} else if (!content || !p || strlen(content) > MAX_VFABRIC_NAME) {
		IXmlParserPrintError(state, "RouteLast group name is too long - ignoring");
		return;
	}

	cs_strlcpy(p->routeLast.member, content, MAX_VFABRIC_NAME);
}

static IXML_FIELD SmHypercubeRoutingFields[] = {
	{ tag:"Debug", format:'u', IXML_FIELD_INFO(SmHypercubeRouting_t, debug) },
	{ tag:"RouteLast", format:'k', end_func:SmHyperRouteLastEnd },
	{ tag:"EnhancedRoutingCtrl", format:'k', size:sizeof(SmSPRoutingCtrl_t), subfields:XmlSPRoutingCtrlFields, start_func:IXmlParserStartStruct, end_func:XmlSPRoutingCtrlParserEnd },
	{ NULL }
};

// "Sm/AdaptiveRouting" start tag
static void* SmAdaptiveRoutingXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	return &((SMXmlConfig_t *)parent)->adaptiveRouting;
}

// "Sm/AdaptiveRouting" end tag
static void SmAdaptiveRoutingXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	SmAdaptiveRoutingXmlConfig_t *p = (SmAdaptiveRoutingXmlConfig_t*)IXmlParserGetField(field, object);
	if (!valid) {
		fprintf(stderr, "Error processing XML Sm AdaptiveRouting tag\n");
		return;
	}
	// Not sure we should validate the threshold here and only here
	// since there may be other places AR can be enabled
	//
	// AR has 16 congestion levels but the user can only specify 8 of those
	// [0,7] where 0 means 'use SMA/firmware default'
	else if (p->enable && (p->threshold > 7)) {
		if (p->threshold == UNDEFINED_XML8) p->threshold = 0;
		else IXmlParserPrintError(state, "Adaptive Routing threshold must be in the range 0-7");
	}

	if (p->algorithm > 2) {// 2 is the maximum defined right now
		IXmlParserPrintError(state, "Adaptive Routing algorithm must be 0, 1 or 2.");
		p->algorithm = 0;
		return;
	}
}

// "PathSelection" end tag
static void SmPathSelectionParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint32_t *p = (uint32_t *)IXmlParserGetField(field, object);

	if (0 == strcasecmp(content, "minimal")) {
		*p = PATH_MODE_MINIMAL;
	} else if (0 == strcasecmp(content, "pairwise")) {
		*p = PATH_MODE_PAIRWISE;
	} else if (0 == strcasecmp(content, "orderall")) {
		*p = PATH_MODE_ORDERALL;
	} else if (0 == strcasecmp(content, "srcdstall")) {
		*p = PATH_MODE_SRCDSTALL;
	} else {
		IXmlParserPrintError(state, "PathSelection must be (Minimal, Pairwise, Orderall or SrcDstAll)");
		return;
	}
}

static void SmCIPParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint8_t i = CIP_NONE;

	while (isspace(*content)) content++;

	if (0 == strcasecmp(content, "ByPort")) i = CIP_PORT;
	else if (0 == strcasecmp(content, "ByLink")) i = CIP_LINK;

	*(uint8_t *)IXmlParserGetField(field, object) = i;

	return;
}

// Validate a LID against the range 0x0 - (0xf0000000-1).
static void SmUnicastLidXmlParserEnd(IXmlParserState_t *state,
	const IXML_FIELD *field, void *object, void *parent, XML_Char *content,
	unsigned len, boolean valid)
{
	uint64_t long_lid;
	uint32_t *real_lid = (uint32_t *)IXmlParserGetField(field, object);

	if (! IXmlParseUint64(state, content, len, &long_lid)) {
		IXmlParserPrintError(state, "Invalid value");
		return;
	}

	if (long_lid >= STL_LID_MULTICAST_BEGIN) {
		IXmlParserPrintError(state, "Value out of range");
		return;
	}

	*real_lid = (uint32_t)long_lid;
	return;
}

// Validate the sweep interval against the range [0,3-86400]
static void SmSweepIntervalParserEnd(IXmlParserState_t *state,
	const IXML_FIELD *field, void *object, void *parent, XML_Char *content,
	unsigned len, boolean valid)
{
	uint64_t timer;

	if (!IXmlParseUint64(state, content, len, &timer)) {
		IXmlParserPrintError(state, "Invalid value");
		return;
	}

	if (timer > SM_MAX_SWEEPRATE || (timer < SM_MIN_SWEEPRATE && timer != SM_NO_SWEEP)) {
		IXmlParserPrintError(state, "Sm SweepInterval must either be %u or in the range of %u-%u",
			SM_NO_SWEEP, SM_MIN_SWEEPRATE, SM_MAX_SWEEPRATE);
		return;
	}

	*(uint64_t *)IXmlParserGetField(field, object) = timer;
}

static void CumulativeTimeoutLimitParserEnd(IXmlParserState_t *state,
	const IXML_FIELD *field, void *object, void *parent, XML_Char *content,
	unsigned len, boolean valid)
{
	uint64_t value;

	if (!IXmlParseUint64(state, content, len, &value)) {
		IXmlParserPrintError(state, "Invalid value");
		return;
	}

	*(uint64_t *)IXmlParserGetField(field, object) = value * VTIMER_1S;
}

// fields within "Sm" tag
static IXML_FIELD SmFields[] = {
	{ tag:"Start", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, start) },
	{ tag:"SmKey", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, sm_key) },
	{ tag:"MKey", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, mkey) },
	{ tag:"StartupRetries", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, startup_retries) },
	{ tag:"StartupStableWait", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, startup_stable_wait) },
	{ tag:"SweepInterval", format:'k', IXML_FIELD_INFO(SMXmlConfig_t, timer), end_func:SmSweepIntervalParserEnd },
	{ tag:"IgnoreTraps", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, IgnoreTraps) },
	{ tag:"TrapHoldDown", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, trap_hold_down) },
	{ tag:"MaxAttempts", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, max_retries) },
	{ tag:"RespTimeout", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, rcv_wait_msec) },
	{ tag:"MinRespTimeout", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, min_rcv_wait_msec) },
	{ tag:"MasterPingInterval", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, master_ping_interval) },
	{ tag:"MasterPingMaxFail", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, master_ping_max_fail) },
	{ tag:"DbSyncInterval", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, db_sync_interval) },
	{ tag:"SweepErrorsThreshold", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, topo_errors_threshold) },
	{ tag:"SweepAbandonThreshold", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, topo_abandon_threshold) },
	{ tag:"SwitchLifetime_Int", format:'k', IXML_FIELD_INFO(SMXmlConfig_t, switch_lifetime_n2), end_func:IXmlParserEndHoqTimeout_Int },
	{ tag:"SwitchLifetime", format:'k', IXML_FIELD_INFO(SMXmlConfig_t, switch_lifetime_n2), end_func:IXmlParserEndHoqTimeout_Str },
	{ tag:"HoqLife_Int", format:'k', IXML_FIELD_INFO(SMXmlConfig_t, hoqlife_n2), end_func:IXmlParserEndHoqTimeout_Int },
	{ tag:"HoqLife", format:'k', IXML_FIELD_INFO(SMXmlConfig_t, hoqlife_n2), end_func:IXmlParserEndHoqTimeout_Str },
	{ tag:"VL15FlowControlDisable", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, vl15FlowControlDisable) },
	{ tag:"VL15CreditRate", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, vl15_credit_rate) },
	{ tag:"SaRespTime_Int", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, sa_resp_time_n2) },
	{ tag:"SaRespTime", format:'k', IXML_FIELD_INFO(SMXmlConfig_t, sa_resp_time_n2), end_func:IXmlParserEndTimeoutMult32_Str },
	{ tag:"PacketLifetime_Int", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, sa_packet_lifetime_n2) },
	{ tag:"PacketLifetime", format:'k', IXML_FIELD_INFO(SMXmlConfig_t, sa_packet_lifetime_n2), end_func:IXmlParserEndTimeoutMult32_Str },
	{ tag:"VLStallCount", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, vlstall) },
	{ tag:"McDosThreshold", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, mc_dos_threshold) },
	{ tag:"McDosAction", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, mc_dos_action) },
	{ tag:"McDosInterval", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, mc_dos_interval) },
	{ tag:"TrapThreshold", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, trap_threshold) },
	{ tag:"TrapThresholdMinCount", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, trap_threshold_min_count) },
	{ tag:"TrapLogSuppressTriggerInterval", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, trap_log_suppress_trigger_interval) },
	{ tag:"NodeAppearanceMsgThreshold", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, node_appearance_msg_thresh) },
	{ tag:"SpineFirstRouting", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, spine_first_routing) },
	{ tag:"ShortestPathBalanced", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, shortestPathBalanced) },
	{ tag:"PathSelection", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, path_selection), end_func:SmPathSelectionParserEnd },
	{ tag:"QueryValidation", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, queryValidation) },
	{ tag:"EnforceVFPathRecord", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, enforceVFPathRecs) },
	{ tag:"SmaBatchSize", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, sma_batch_size) },
	{ tag:"MaxParallelReqs", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, max_parallel_reqs) },
 	{ tag:"CheckMftResponses", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, check_mft_responses) },
	{ tag:"MonitorStandby", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, monitor_standby_enable) },
	{ tag:"Lmc", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, lmc) },
	{ tag:"LmcE0", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, lmc_e0) },
	{ tag:"SubnetSize", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, subnet_size) },
	{ tag:"Debug", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, debug) },
	{ tag:"RmppDebug", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, debug_rmpp) },
	{ tag:"Priority", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, priority) },
	{ tag:"ElevatedPriority", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, elevated_priority) },
	{ tag:"LogLevel", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, log_level) },
	{ tag:"LogMode", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, syslog_mode) },
	{ tag:"SyslogFacility", format:'s', IXML_FIELD_INFO(SMXmlConfig_t, syslog_facility) },
	{ tag:"SslSecurityEnabled", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, SslSecurityEnabled) },
	{ tag:"SslSecurityEnable", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, SslSecurityEnabled) },
#ifndef __VXWORKS__
	{ tag:"SslSecurityDir", format:'s', IXML_FIELD_INFO(SMXmlConfig_t, SslSecurityDir) },
#endif
	{ tag:"SslSecurityFmCertificate", format:'s', IXML_FIELD_INFO(SMXmlConfig_t, SslSecurityFmCertificate) },
	{ tag:"SslSecurityFmPrivateKey", format:'s', IXML_FIELD_INFO(SMXmlConfig_t, SslSecurityFmPrivateKey) },
	{ tag:"SslSecurityFmCaCertificate", format:'s', IXML_FIELD_INFO(SMXmlConfig_t, SslSecurityFmCaCertificate) },
	{ tag:"SslSecurityFmCertChainDepth", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, SslSecurityFmCertChainDepth) },
	{ tag:"SslSecurityFmDHParameters", format:'s', IXML_FIELD_INFO(SMXmlConfig_t, SslSecurityFmDHParameters) },
	{ tag:"SslSecurityFmCaCRLEnabled", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, SslSecurityFmCaCRLEnabled) },
	{ tag:"SslSecurityFmCaCRLEnable", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, SslSecurityFmCaCRLEnabled) },
	{ tag:"SslSecurityFmCaCRL", format:'s', IXML_FIELD_INFO(SMXmlConfig_t, SslSecurityFmCaCRL) },
	{ tag:"LID", format:'k', IXML_FIELD_INFO(SMXmlConfig_t, lid), end_func:SmUnicastLidXmlParserEnd },
	{ tag:"SmPerfDebug", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, sm_debug_perf) },
	{ tag:"SaPerfDebug", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, sa_debug_perf) },
	{ tag:"SaRmppChecksum", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, sa_rmpp_checksum) },
	{ tag:"LoopTestOn", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, loop_test_on) },
	{ tag:"LoopTestFastMode", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, loop_test_fast_mode) },
	{ tag:"LoopTestPackets", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, loop_test_packets) },
	{ tag:"LidStrategy", format:'k', IXML_FIELD_INFO(SMXmlConfig_t, lid_strategy), end_func:LidStrategyXmlParserEnd },
	{ tag:"CS_LogMask", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, log_masks[VIEO_CS_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"MAI_LogMask", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, log_masks[VIEO_MAI_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"CAL_LogMask", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, log_masks[VIEO_CAL_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"DVR_LogMask", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, log_masks[VIEO_DRIVER_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"IF3_LogMask", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, log_masks[VIEO_IF3_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"SM_LogMask", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, log_masks[VIEO_SM_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"SA_LogMask", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, log_masks[VIEO_SA_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"PM_LogMask", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, log_masks[VIEO_PM_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"PA_LogMask", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, log_masks[VIEO_PA_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"BM_LogMask", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, log_masks[VIEO_BM_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"FE_LogMask", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, log_masks[VIEO_FE_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"APP_LogMask", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, log_masks[VIEO_APP_MOD_ID]), end_func:ParamU32XmlParserEnd },


	{ tag:"LogFile", format:'s', IXML_FIELD_INFO(SMXmlConfig_t, log_file) },
	{ tag:"NonRespTimeout", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, non_resp_tsec) },
	{ tag:"NonRespMaxCount", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, non_resp_max_count) },
	{ tag:"DynamicPortAlloc", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, dynamic_port_alloc) },
	{ tag:"LoopbackMode", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, loopback_mode) },
	{ tag:"ForceRebalance", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, force_rebalance) },
	{ tag:"UseCachedNodeData", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, use_cached_node_data) },
	{ tag:"DynamicPacketLifetime", format:'k', subfields:SmDPLifetimeFields, start_func:SmDPLifetimeXmlParserStart, end_func:SmDPLifetimeXmlParserEnd },
	{ tag:"Multicast", format:'k', subfields:SmMcastFields, start_func:SmMcastXmlParserStart, end_func:SmMcastXmlParserEnd },
	{ tag:"RoutingAlgorithm", format:'s', IXML_FIELD_INFO(SMXmlConfig_t, routing_algorithm) },
	{ tag:"DebugJm", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, debug_jm) },
	{ tag:"HFILinkPolicy", format:'k', subfields:SmLinkPolicyFields, start_func:SmHFILinkPolicyXmlParserStart },
	{ tag:"ISLLinkPolicy", format:'k', subfields:SmLinkPolicyFields, start_func:SmISLLinkPolicyXmlParserStart },
	{ tag:"Preemption", format:'k', subfields:SmPreemptionFields, start_func:SmPreemptionXmlParserStart },
	{ tag:"CongestionControl", format:'k', subfields:SmCongestionFields, start_func:SmCongestionXmlParserStart },
	{ tag:"AdaptiveRouting", format:'k', subfields:SmAdaptiveRoutingFields, start_func:SmAdaptiveRoutingXmlParserStart, end_func:SmAdaptiveRoutingXmlParserEnd },
	{ tag:"FatTreeTopology", format:'k', subfields:SmFtreeRoutingFields, start_func:SmFtreeRoutingXmlParserStart, end_func:SmFtreeRoutingXmlParserEnd },
	{ tag:"DGShortestPathTopology", format:'k', subfields:SmDGRoutingFields, start_func:SmDGRoutingXmlParserStart, end_func:SmDGRoutingXmlParserEnd },
	{ tag:"HypercubeTopology", format:'k', subfields:SmHypercubeRoutingFields, start_func:SmHypercubeRoutingXmlParserStart, end_func:SmHypercubeRoutingXmlParserEnd },
	{ tag:"MeshTorusTopology", format:'k', subfields:SmDorRoutingFields, start_func:SmDorRoutingXmlParserStart, end_func:SmDorRoutingXmlParserEnd },
	{ tag:"DebugVf", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, sm_debug_vf) },
	{ tag:"DebugRouting", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, sm_debug_routing) },
	{ tag:"DebugLidAssign", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, sm_debug_lid_assign) },
	{ tag:"NoReplyIfBusy", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, NoReplyIfBusy) },
	{ tag:"LftMultiblock", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, lft_multi_block) },
	{ tag:"UseAggregateMADs", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, use_aggregates) },
	{ tag:"ScMultiblock", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, sc_multi_block) },
	{ tag:"OptimizedPortInfo", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, optimized_portinfo) },
	{ tag:"OptimizedBufferControl", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, optimized_buffer_control) },
	{ tag:"SmaSpoofingCheck", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, sma_spoofing_check) },
	{ tag:"MinSharedVLMem", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, minSharedVLMem), end_func:PercentageXmlParserEnd },
	{ tag:"DedicatedVLMemMulti", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, dedicatedVLMemMulti), end_func:DedicatedVLMemMultiXmlParserEnd },
	{ tag:"WireDepthOverride", format:'d', IXML_FIELD_INFO(SMXmlConfig_t, wireDepthOverride) },
	{ tag:"ReplayDepthOverride", format:'d', IXML_FIELD_INFO(SMXmlConfig_t, replayDepthOverride) },
	{ tag:"TimerScalingEnable", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, timerScalingEnable) },
	{ tag:"SmAppliances", format:'k', subfields:SmAppliancesFields, start_func:SmAppliancesXmlParserStart },
	{ tag:"CableInfoPolicy", format:'k', end_func:SmCIPParserEnd, IXML_FIELD_INFO(SMXmlConfig_t, cableInfoPolicy) },
	{ tag:"ForceAttributeRewrite", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, forceAttributeRewrite) },
	{ tag:"SkipAttributeWrite", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, skipAttributeWrite) },
	{ tag:"DefaultPortErrorAction", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, defaultPortErrorAction) },
	{ tag:"SwitchCascadeActivateEnable", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, switchCascadeActivateEnable) },
	{ tag:"NeighborNormalRetries", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, neighborNormalRetries) },
	{ tag:"PreDefinedTopology", format:'k', subfields:SmPreDefTopoFields, start_func:SmPreDefTopoXmlParserStart, end_func:SmPreDefTopoXmlParserEnd },
	{ tag:"TerminateAfter", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, terminateAfter) },
	{ tag:"DumpCounters", format:'s', IXML_FIELD_INFO(SMXmlConfig_t, dumpCounters) },
	{ tag:"PortBounceLogLimit", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, portBounceLogLimit) },
	{ tag:"NeighborFWAuthenEnable", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, neighborFWAuthenEnable) },
	{ tag:"MinSupportedVLs", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, min_supported_vls), end_func:MinSupportedVLsParserEnd },
	{ tag:"MaxFixedVLs", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, max_fixed_vls) },
	{ tag:"PKey_8B", format:'h',  IXML_FIELD_INFO(SMXmlConfig_t, P_Key_8B), end_func:PKey8BParserEnd},
	{ tag:"PKey_10B", format:'h', IXML_FIELD_INFO(SMXmlConfig_t, P_Key_10B), end_func:PKey10BParserEnd},
	{ tag:"CumulativeTimeoutLimit", format:'u', IXML_FIELD_INFO(SMXmlConfig_t, cumulative_timeout_limit), end_func:CumulativeTimeoutLimitParserEnd },
	{ NULL }
};

// "Sm" start tag
static void* SmXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	SMXmlConfig_t *smp = getXmlMemory(sizeof(SMXmlConfig_t), "SMXmlConfig_t SmXmlParserStart()");

	if (xml_parse_debug)
		fprintf(stdout, "SmXmlParserStart instance %u common %u\n", (unsigned int)instance, (unsigned int)common);

	if (!smp) {
		PRINT_MEMORY_ERROR;
		return NULL;
	}

	// inherit the values of config items already set
	if (!common && configp->fm_instance[instance])
		smCopyConfig(smp, &configp->fm_instance[instance]->sm_config);
	else
		smCopyConfig(smp, &configp->fm_instance_common->sm_config);

	return smp;			// will be passed to SmXmlParserStartEnd as object
}

// "Sm" end tag
static void SmXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	smp = (SMXmlConfig_t*)IXmlParserGetField(field, object);
#ifndef __VXWORKS__
	char facility[256];
#endif

	if (xml_parse_debug)
		fprintf(stdout, "SmXmlParserEnd instance %u common %u\n", (unsigned int)instance, (unsigned int)common);

	if (!valid) {
		fprintf(stderr, "Error processing XML Sm tag\n");
	} else {
		if (smp->priority != UNDEFINED_XML32 && smp->priority > MAX_PRIORITY) {
			IXmlParserPrintError(state, "Sm Priority must be in the range of 0-15");
			freeXmlMemory(smp, sizeof(SMXmlConfig_t), "SMXmlConfig_t SmXmlParserEnd()");
			return;
		}
		if (smp->elevated_priority != UNDEFINED_XML32 && smp->elevated_priority > MAX_PRIORITY) {
			IXmlParserPrintError(state, "Sm ElevatedPriority must be in the range of 0-15");
			freeXmlMemory(smp, sizeof(SMXmlConfig_t), "SMXmlConfig_t SmXmlParserEnd()");
			return;
		}
		if (smp->vl15_credit_rate != UNDEFINED_XML32 && smp->vl15_credit_rate > MAX_VL15_CREDIT_RATE) {
			IXmlParserPrintError(state, "Sm VL15CreditRate must be in the range of 0-21");
			freeXmlMemory(smp, sizeof(SMXmlConfig_t), "SMXmlConfig_t SmXmlParserEnd()");
			return;
		}

#ifndef __VXWORKS__
		if (strlen(smp->syslog_facility) && getFacility(smp->syslog_facility, /* test */ 1) < 0) {
			IXmlParserPrintError(state, "Sm SyslogFacility must be set to one of the following - %s", showFacilities(facility, sizeof(facility)));
			freeXmlMemory(smp, sizeof(SMXmlConfig_t), "SMXmlConfig_t SmXmlParserEnd()");
			return;
		}
#endif
		if (strcasecmp(smp->routing_algorithm, "dor") == 0) {
			if (smp->smDorRouting.dimensionCount == 0xff) {
				IXmlParserPrintError(state, "Sm routing algorithm requires MeshTorusTopology configuration");
				freeXmlMemory(smp, sizeof(SMXmlConfig_t), "SMXmlConfig_t SmXmlParserEnd()");
				return;
			}
		}

		if (strcasecmp(smp->routing_algorithm, "fattree") == 0) {
			if ((smp->ftreeRouting.tierCount == 0) ||
			 	(smp->ftreeRouting.tierCount > MAX_TIER)) {
				IXmlParserPrintError(state, "Sm routing algorithm fattree requires TierCount setting between 1 and %d", MAX_TIER);
				freeXmlMemory(smp, sizeof(SMXmlConfig_t), "SMXmlConfig_t SmXmlParserEnd()");
				return;
			}
			if (!smp->ftreeRouting.fis_on_same_tier &&
				(strlen(smp->ftreeRouting.coreSwitches.member) == 0)) {
				IXmlParserPrintError(state, "Sm routing algorithm fattree requires CoreSwitches device group or HFIs on same tier");
				freeXmlMemory(smp, sizeof(SMXmlConfig_t), "SMXmlConfig_t SmXmlParserEnd()");
				return;
			} else if (smp->ftreeRouting.fis_on_same_tier &&
				(strlen(smp->ftreeRouting.coreSwitches.member) != 0)) {
				IXmlParserPrintError(state, "Sm routing algorithm fattree has CoreSwitches device group defined, this has no effect when FIsOnSameTier is indicated");
				freeXmlMemory(smp, sizeof(SMXmlConfig_t), "SMXmlConfig_t SmXmlParserEnd()");
				return;
			}
		}


	}
	if (common) {
		smCopyConfig(&configp->fm_instance_common->sm_config, smp);
	} else if (configp->fm_instance[instance]) {
		// save the Sm config for this instance
		smCopyConfig(&configp->fm_instance[instance]->sm_config, smp);
	}
	smFreeConfig(smp);
}

// "FeTrapNumberParserEnd" end tag
static void FeTrapNumberParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint16_t *trap_nums = (uint16_t *)parent;
	uint16 trap_num;

	// Check if 'All' is supplied
	if (strcasecmp(content, "All") == 0) {
		trap_num = TRAP_ALL; // TRAP_ALL Will Register for All Traps
	// Else check if valid uint16 number
	} else if (!IXmlParseUint16(state, content, len, &trap_num)) {
		IXmlParserPrintError(state, "Invalid value");
		return;
	}

	if (feTrapSubInstance >= FE_MAX_TRAP_SUBS && trap_num != TRAP_ALL) {
		IXmlParserPrintWarning(state, "Trap Subscriptions Limit Excceded, only %u Subscriptions. Trap %u will be ignored",
			FE_MAX_TRAP_SUBS, trap_num);
		return;
	}

	// Validate Trap Number
	switch (trap_num) {
	case MAD_SMT_PORT_UP:
	case MAD_SMT_PORT_DOWN:
	case MAD_SMT_MCAST_GRP_CREATED:
	case MAD_SMT_MCAST_GRP_DELETED:
	case MAD_SMT_UNPATH:
	case MAD_SMT_REPATH:
	case MAD_SMT_PORT_CHANGE:
	case MAD_SMT_LINK_INTEGRITY:
	case MAD_SMT_BUF_OVERRUN:
	case MAD_SMT_FLOW_CONTROL:
	case MAD_SMT_CAPABILITYMASK_CHANGE:
	case MAD_SMT_SYSTEMIMAGEGUID_CHANGE:
	case MAD_SMT_BAD_MKEY:
	case MAD_SMT_BAD_PKEY:
	case MAD_SMT_BAD_QKEY:
	case MAD_SMT_BAD_PKEY_ONPORT:
	case STL_SMA_TRAP_LINK_WIDTH:
	case STL_TRAP_COST_MATRIX_CHANGE:
		// Check if 1st value is Not All Trap value; else skip
		if (trap_nums[0] != TRAP_ALL) {
			// Add Trap number to List and Inc Trap Count
			trap_nums[feTrapSubInstance] = trap_num;
			feTrapSubInstance++;
		}
		break;
	case TRAP_ALL:
		// If All Traps value, Wipe Traps and only have All Traps value in first slot
		feTrapSubInstance = 1;
		trap_nums[0] = TRAP_ALL;
		break;
	default:
		IXmlParserPrintError(state, "Trap %u is not valid/supported", trap_num);
	}
	return;
}
static IXML_FIELD FeTrapSubsFields[] = {
	{ tag:"TrapNumber", format:'k', end_func:FeTrapNumberParserEnd },
	{ NULL }
};
// "FeTrapSubs" start tag
static void* FeTrapSubsXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	FEXmlConfig_t *fep = (FEXmlConfig_t *)parent;
	uint16_t *trap_nums = (uint16_t *)getXmlMemory(FE_MAX_TRAP_SUBS * sizeof(uint16_t), "trap_nums FeTrapSubsXmlParserStart()");

	if (!trap_nums) {
		PRINT_MEMORY_ERROR;
		return NULL;
	}

	memset(trap_nums, 0, FE_MAX_TRAP_SUBS * sizeof(uint16_t));

	fep->trap_count = feTrapSubInstance = 0;

	return trap_nums;
}
// "FeTrapSubs" end tag
static void FeTrapSubsXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	FEXmlConfig_t *fep = (FEXmlConfig_t *)parent;
	uint16_t *trap_nums = (uint16_t *)object;

	if (valid) {
		int i = 0;
		for (; i < feTrapSubInstance; i++) {
			fep->trap_nums[i] = trap_nums[i];
		}
		for (; i < FE_MAX_TRAP_SUBS; i++) {
			fep->trap_nums[i] = 0;
		}
		fep->trap_count = feTrapSubInstance;
	}

	freeXmlMemory(trap_nums, FE_MAX_TRAP_SUBS * sizeof(uint16_t), "trap_nums FeTrapSubsXmlParserEnd()");

	return;
}
// fields within "Fe" tag
static IXML_FIELD FeFields[] = {
	{ tag:"Start", format:'u', IXML_FIELD_INFO(FEXmlConfig_t, start) },
	{ tag:"TcpPort", format:'u', IXML_FIELD_INFO(FEXmlConfig_t, listen) },
	// Unsupported in 4.4 - PR 109767
    // { tag:"Login", format:'u', IXML_FIELD_INFO(FEXmlConfig_t, login) },
	{ tag:"SubnetSize", format:'u', IXML_FIELD_INFO(FEXmlConfig_t, subnet_size) },
	{ tag:"StartupRetries", format:'u', IXML_FIELD_INFO(FEXmlConfig_t, startup_retries) },
	{ tag:"StartupStableWait", format:'u', IXML_FIELD_INFO(FEXmlConfig_t, startup_stable_wait) },
	{ tag:"CoreDumpDir", format:'s', IXML_FIELD_INFO(FEXmlConfig_t, CoreDumpDir) },
	{ tag:"CoreDumpLimit", format:'s', IXML_FIELD_INFO(FEXmlConfig_t, CoreDumpLimit) },
	{ tag:"Debug", format:'u', IXML_FIELD_INFO(FEXmlConfig_t, debug) },
	{ tag:"RmppDebug", format:'u', IXML_FIELD_INFO(FEXmlConfig_t, debug_rmpp) },
	{ tag:"LogLevel", format:'u', IXML_FIELD_INFO(FEXmlConfig_t, log_level) },
	{ tag:"LogMode", format:'u', IXML_FIELD_INFO(FEXmlConfig_t, syslog_mode) },
	{ tag:"SyslogFacility", format:'s', IXML_FIELD_INFO(FEXmlConfig_t, syslog_facility) },
	{ tag:"TrapSubscriptions", format:'k', subfields:FeTrapSubsFields, start_func:FeTrapSubsXmlParserStart, end_func:FeTrapSubsXmlParserEnd },
	{ tag:"SslSecurityEnabled", format:'u', IXML_FIELD_INFO(FEXmlConfig_t, SslSecurityEnabled) },
	{ tag:"SslSecurityEnable", format:'u', IXML_FIELD_INFO(FEXmlConfig_t, SslSecurityEnabled) },
#ifndef __VXWORKS__
	{ tag:"SslSecurityDir", format:'s', IXML_FIELD_INFO(FEXmlConfig_t, SslSecurityDir) },
#endif
	{ tag:"SslSecurityFmCertificate", format:'s', IXML_FIELD_INFO(FEXmlConfig_t, SslSecurityFmCertificate) },
	{ tag:"SslSecurityFmPrivateKey", format:'s', IXML_FIELD_INFO(FEXmlConfig_t, SslSecurityFmPrivateKey) },
	{ tag:"SslSecurityFmCertChainDepth", format:'u', IXML_FIELD_INFO(FEXmlConfig_t, SslSecurityFmCertChainDepth) },
	{ tag:"SslSecurityFmDHParameters", format:'s', IXML_FIELD_INFO(FEXmlConfig_t, SslSecurityFmDHParameters) },
	{ tag:"SslSecurityFmCaCRLEnabled", format:'u', IXML_FIELD_INFO(FEXmlConfig_t, SslSecurityFmCaCRLEnabled) },
	{ tag:"SslSecurityFmCaCRLEnable", format:'u', IXML_FIELD_INFO(FEXmlConfig_t, SslSecurityFmCaCRLEnabled) },
	{ tag:"SslSecurityFmCaCRL", format:'s', IXML_FIELD_INFO(FEXmlConfig_t, SslSecurityFmCaCRL) },
	{ tag:"CS_LogMask", format:'u', IXML_FIELD_INFO(FEXmlConfig_t, log_masks[VIEO_CS_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"MAI_LogMask", format:'u', IXML_FIELD_INFO(FEXmlConfig_t, log_masks[VIEO_MAI_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"CAL_LogMask", format:'u', IXML_FIELD_INFO(FEXmlConfig_t, log_masks[VIEO_CAL_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"DVR_LogMask", format:'u', IXML_FIELD_INFO(FEXmlConfig_t, log_masks[VIEO_DRIVER_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"IF3_LogMask", format:'u', IXML_FIELD_INFO(FEXmlConfig_t, log_masks[VIEO_IF3_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"SM_LogMask", format:'u', IXML_FIELD_INFO(FEXmlConfig_t, log_masks[VIEO_SM_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"SA_LogMask", format:'u', IXML_FIELD_INFO(FEXmlConfig_t, log_masks[VIEO_SA_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"PM_LogMask", format:'u', IXML_FIELD_INFO(FEXmlConfig_t, log_masks[VIEO_PM_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"PA_LogMask", format:'u', IXML_FIELD_INFO(FEXmlConfig_t, log_masks[VIEO_PA_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"BM_LogMask", format:'u', IXML_FIELD_INFO(FEXmlConfig_t, log_masks[VIEO_BM_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"FE_LogMask", format:'u', IXML_FIELD_INFO(FEXmlConfig_t, log_masks[VIEO_FE_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"APP_LogMask", format:'u', IXML_FIELD_INFO(FEXmlConfig_t, log_masks[VIEO_APP_MOD_ID]), end_func:ParamU32XmlParserEnd },


	{ tag:"LogFile", format:'s', IXML_FIELD_INFO(FEXmlConfig_t, log_file) },
	{ tag:"Window", format:'u', IXML_FIELD_INFO(FEXmlConfig_t, window) },
	{ tag:"ManagerCheckRate", format:'u', IXML_FIELD_INFO(FEXmlConfig_t, manager_check_rate) },
	{ NULL }
};

// "Fe" start tag
static void* FeXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	FEXmlConfig_t *fep = getXmlMemory(sizeof(FEXmlConfig_t), "FEXmlConfig_t FeXmlParserStart()");

	if (xml_parse_debug)
		fprintf(stdout, "FeXmlParserStart instance %u common %u\n", (unsigned int)instance, (unsigned int)common);

	if (!fep) {
		PRINT_MEMORY_ERROR;
		return NULL;
	}

	// inherit the values of config items already set
	if (!common && configp->fm_instance[instance])
		*fep = configp->fm_instance[instance]->fe_config;
	else
		*fep = configp->fm_instance_common->fe_config;

	return fep;			// will be passed to FeXmlParserStartEnd as object
}

// "Fe" end tag
static void FeXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	FEXmlConfig_t *fep = (FEXmlConfig_t*)IXmlParserGetField(field, object);
#ifndef __VXWORKS__
	char facility[256];
#endif

	if (xml_parse_debug)
		fprintf(stdout, "FeXmlParserEnd instance %u common %u\n", (unsigned int)instance, (unsigned int)common);

	if (!valid) {
		fprintf(stderr, "Error processing XML Fe tag\n");
	} else {
		if (strlen(fep->CoreDumpLimit) && vs_getCoreDumpLimit(fep->CoreDumpLimit, NULL) < 0) {
			IXmlParserPrintError(state, "Invalid Fe CoreDumpLimit: '%s'", fep->CoreDumpLimit);
			freeXmlMemory(fep, sizeof(FEXmlConfig_t), "FEXmlConfig_t FeXmlParserEnd()");
			return;
		}
#ifndef __VXWORKS__
		if (strlen(fep->syslog_facility) && getFacility(fep->syslog_facility, /* test */ 1) < 0) {
			IXmlParserPrintError(state, "Fe SyslogFacility must be set to one of the following - %s", showFacilities(facility, sizeof(facility)));
			freeXmlMemory(fep, sizeof(FEXmlConfig_t), "FEXmlConfig_t FeXmlParserEnd()");
			return;
		}
#endif
		// If Not Supplied Enable Current Default of Traps 64 & 65
		if (fep->trap_count == UNDEFINED_XML32) {
			memset(&fep->trap_nums[0], 0, FE_MAX_TRAP_SUBS * sizeof(uint16_t));
			fep->trap_count = 2;
			fep->trap_nums[0] = MAD_SMT_PORT_UP;
			fep->trap_nums[1] = MAD_SMT_PORT_DOWN;
		}
	}

	if (common) {
		configp->fm_instance_common->fe_config = *fep;
	} else if (configp->fm_instance[instance]) {
		// save the Fe config for this instance
		configp->fm_instance[instance]->fe_config = *fep;
	}

	freeXmlMemory(fep, sizeof(FEXmlConfig_t), "FEXmlConfig_t FeXmlParserEnd()");
}
static void PmSweepIntervalParserEnd(IXmlParserState_t *state,
	const IXML_FIELD *field, void *object, void *parent, XML_Char *content,
	unsigned len, boolean valid)
{
	uint16 SweepInterval;

	if (!IXmlParseUint16(state, content, len, &SweepInterval)) {
		IXmlParserPrintError(state, "Invalid value");
		return;
	}

	if (SweepInterval > PM_MAX_SWEEPRATE || SweepInterval < PM_MIN_SWEEPRATE) {
		IXmlParserPrintError(state, "Pm SweepInterval (%u) must be in the range of %u-%u seconds",
			SweepInterval, PM_MIN_SWEEPRATE, PM_MAX_SWEEPRATE);
		return;
	}

	*(uint16 *)IXmlParserGetField(field, object) = SweepInterval;
}

// fields within "Pm/Thresholds" tag
static IXML_FIELD PmThresholdsFields[] = {
	{ tag:"Integrity", format:'u', IXML_FIELD_INFO(PmThresholdsXmlConfig_t, Integrity) },
	{ tag:"Congestion", format:'u', IXML_FIELD_INFO(PmThresholdsXmlConfig_t, Congestion) },
	{ tag:"SmaCongestion", format:'u', IXML_FIELD_INFO(PmThresholdsXmlConfig_t, SmaCongestion) },
	{ tag:"Bubble", format:'u', IXML_FIELD_INFO(PmThresholdsXmlConfig_t, Bubble) },
	{ tag:"Security", format:'u', IXML_FIELD_INFO(PmThresholdsXmlConfig_t, Security) },
	{ tag:"Routing", format:'u', IXML_FIELD_INFO(PmThresholdsXmlConfig_t, Routing) },
	{ NULL }
};

// "Pm/Thresholds" start tag
static void* PmThresholdsXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	return &((PMXmlConfig_t *)parent)->thresholds;
}

// fields within "Pm/ThresholdsExceededMsgLimit" tag
static IXML_FIELD PmThresholdsExceededMsgLimitFields[] = {
	{ tag:"Integrity", format:'u', IXML_FIELD_INFO(PmThresholdsExceededMsgLimitXmlConfig_t, Integrity) },
	{ tag:"Congestion", format:'u', IXML_FIELD_INFO(PmThresholdsExceededMsgLimitXmlConfig_t, Congestion) },
	{ tag:"SmaCongestion", format:'u', IXML_FIELD_INFO(PmThresholdsExceededMsgLimitXmlConfig_t, SmaCongestion) },
	{ tag:"Bubble", format:'u', IXML_FIELD_INFO(PmThresholdsExceededMsgLimitXmlConfig_t, Bubble) },
	{ tag:"Security", format:'u', IXML_FIELD_INFO(PmThresholdsExceededMsgLimitXmlConfig_t, Security) },
	{ tag:"Routing", format:'u', IXML_FIELD_INFO(PmThresholdsExceededMsgLimitXmlConfig_t, Routing) },
	{ NULL }
};

// "Pm/ThresholdsExceededMsgLimit" start tag
static void* PmThresholdsExceededMsgLimitXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	return &((PMXmlConfig_t *)parent)->thresholdsExceededMsgLimit;
}

// fields within "Pm/IntegrityWeights" tag
static IXML_FIELD PmIntegrityWeightsFields[] = {
	{ tag:"LocalLinkIntegrityErrors", format:'u', IXML_FIELD_INFO(PmIntegrityWeightsXmlConfig_t, LocalLinkIntegrityErrors) },
	{ tag:"RcvErrors", format:'u', IXML_FIELD_INFO(PmIntegrityWeightsXmlConfig_t, PortRcvErrors) },
	{ tag:"ExcessiveBufferOverruns", format:'u', IXML_FIELD_INFO(PmIntegrityWeightsXmlConfig_t, ExcessiveBufferOverruns) },
	{ tag:"LinkErrorRecovery", format:'u', IXML_FIELD_INFO(PmIntegrityWeightsXmlConfig_t, LinkErrorRecovery) },
	{ tag:"LinkDowned", format:'u', IXML_FIELD_INFO(PmIntegrityWeightsXmlConfig_t, LinkDowned) },
	{ tag:"UncorrectableErrors", format:'u', IXML_FIELD_INFO(PmIntegrityWeightsXmlConfig_t, UncorrectableErrors) },
	{ tag:"FMConfigErrors", format:'u', IXML_FIELD_INFO(PmIntegrityWeightsXmlConfig_t, FMConfigErrors) },
	{ tag:"LinkQualityIndicator", format:'u', IXML_FIELD_INFO(PmIntegrityWeightsXmlConfig_t, LinkQualityIndicator) },
	{ tag:"LinkWidthDowngrade", format:'u', IXML_FIELD_INFO(PmIntegrityWeightsXmlConfig_t, LinkWidthDowngrade) },
	{ NULL }
};

// "Pm/IntegrityWeights" start tag
static void* PmIntegrityWeightsXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	return &((PMXmlConfig_t *)parent)->integrityWeights;
}

// fields within "Pm/CongestionWeights" tag
static IXML_FIELD PmCongestionWeightsFields[] = {
	{ tag:"XmitWaitPct", format:'u', IXML_FIELD_INFO(PmCongestionWeightsXmlConfig_t, PortXmitWait) },
	{ tag:"CongDiscards", format:'u', IXML_FIELD_INFO(PmCongestionWeightsXmlConfig_t, SwPortCongestion) },
	{ tag:"RcvFECNPct", format:'u', IXML_FIELD_INFO(PmCongestionWeightsXmlConfig_t, PortRcvFECN) },
	{ tag:"RcvBECNPct", format:'u', IXML_FIELD_INFO(PmCongestionWeightsXmlConfig_t, PortRcvBECN) },
	{ tag:"XmitTimeCongPct", format:'u', IXML_FIELD_INFO(PmCongestionWeightsXmlConfig_t, PortXmitTimeCong) },
	{ tag:"MarkFECNPct", format:'u', IXML_FIELD_INFO(PmCongestionWeightsXmlConfig_t, PortMarkFECN) },
	{ NULL }
};

// "Pm/CongestionWeights" start tag
static void* PmCongestionWeightsXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	return &((PMXmlConfig_t *)parent)->congestionWeights;
}

// fields within the "Pm/Resolution" tag
static IXML_FIELD PmResolutionFields[] = {
	{ tag:"LocalLinkIntegrity", format:'u', IXML_FIELD_INFO(PmResolutionXmlConfig_t, LocalLinkIntegrity) },
	{ tag:"LinkErrorRecovery", format:'u', IXML_FIELD_INFO(PmResolutionXmlConfig_t, LinkErrorRecovery) },
	{ NULL }
};

static void* PmResolutionXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	return &((PMXmlConfig_t *)parent)->resolution;
}

// fields withing "Pm/ShortTermHistory" tag
static IXML_FIELD PmShortTermHistoryFields[] = {
	{ tag:"Enable", format:'u', IXML_FIELD_INFO(PmShortTermHistoryXmlConfig_t, enable) },
	{ tag:"StorageLocation", format:'s', IXML_FIELD_INFO(PmShortTermHistoryXmlConfig_t, StorageLocation) },
	{ tag:"TotalHistory", format:'u', IXML_FIELD_INFO(PmShortTermHistoryXmlConfig_t, totalHistory) },
	{ tag:"ImagesPerComposite", format:'u', IXML_FIELD_INFO(PmShortTermHistoryXmlConfig_t, imagesPerComposite) },
	{ tag:"MaxDiskSpace", format:'u', IXML_FIELD_INFO(PmShortTermHistoryXmlConfig_t, maxDiskSpace) },
	{ tag:"CompressionDivisions", format:'u', IXML_FIELD_INFO(PmShortTermHistoryXmlConfig_t, compressionDivisions) },
	{ NULL }
};

// "Pm/ShortTermHistory" start tag
static void* PmShortTermHistoryXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	return &((PMXmlConfig_t *)parent)->shortTermHistory;
}

// "Pm/ShortTermHistory" end tag
static void PmShortTermHistoryXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	PmShortTermHistoryXmlConfig_t *p = (PmShortTermHistoryXmlConfig_t*)IXmlParserGetField(field, object);

	if (!valid) {
		fprintf(stderr, "Error processing XML Short Term History tag\n");
	} else {
        if (embedded_call && p && p->enable) {
            // Parsing an ESM configuration file, Short Term History feature
            // not supported
            p->enable = 0;
            // print message to standard output
            IXmlParserPrintWarning(state, "Short Term History has been disabled, not supported for ESM");
#ifdef __VXWORKS__
            // log the message
            IB_LOG_WARN0("Short Term History has been disabled, not supported for ESM");
#endif
        }
	}
}

// fields within "Pm" tag
static IXML_FIELD PmFields[] = {
	{ tag:"Start", format:'u', IXML_FIELD_INFO(PMXmlConfig_t, start) },
	{ tag:"SubnetSize", format:'u', IXML_FIELD_INFO(PMXmlConfig_t, subnet_size) },
	{ tag:"SslSecurityEnabled", format:'u', IXML_FIELD_INFO(PMXmlConfig_t, SslSecurityEnabled) },
	{ tag:"SslSecurityEnable", format:'u', IXML_FIELD_INFO(PMXmlConfig_t, SslSecurityEnabled) },
#ifndef __VXWORKS__
	{ tag:"SslSecurityDir", format:'s', IXML_FIELD_INFO(PMXmlConfig_t, SslSecurityDir) },
#endif
	{ tag:"SslSecurityFmCertificate", format:'s', IXML_FIELD_INFO(PMXmlConfig_t, SslSecurityFmCertificate) },
	{ tag:"SslSecurityFmPrivateKey", format:'s', IXML_FIELD_INFO(PMXmlConfig_t, SslSecurityFmPrivateKey) },
	{ tag:"SslSecurityFmCaCertificate", format:'s', IXML_FIELD_INFO(PMXmlConfig_t, SslSecurityFmCaCertificate) },
	{ tag:"SslSecurityFmCertChainDepth", format:'u', IXML_FIELD_INFO(PMXmlConfig_t, SslSecurityFmCertChainDepth) },
	{ tag:"SslSecurityFmDHParameters", format:'s', IXML_FIELD_INFO(PMXmlConfig_t, SslSecurityFmDHParameters) },
	{ tag:"SslSecurityFmCaCRLEnabled", format:'u', IXML_FIELD_INFO(PMXmlConfig_t, SslSecurityFmCaCRLEnabled) },
	{ tag:"SslSecurityFmCaCRLEnable", format:'u', IXML_FIELD_INFO(PMXmlConfig_t, SslSecurityFmCaCRLEnabled) },
	{ tag:"SslSecurityFmCaCRL", format:'s', IXML_FIELD_INFO(PMXmlConfig_t, SslSecurityFmCaCRL) },
	{ tag:"ServiceLease", format:'u', IXML_FIELD_INFO(PMXmlConfig_t, timer) },
	{ tag:"SweepInterval", format:'u', IXML_FIELD_INFO(PMXmlConfig_t, sweep_interval), end_func:PmSweepIntervalParserEnd },
	{ tag:"ErrorClear", format:'u', IXML_FIELD_INFO(PMXmlConfig_t, ErrorClear) },
	{ tag:"ClearDataXfer", format:'u', IXML_FIELD_INFO(PMXmlConfig_t, ClearDataXfer) },
	{ tag:"Clear64bit", format:'u', IXML_FIELD_INFO(PMXmlConfig_t, Clear64bit) },
	{ tag:"Clear32bit", format:'u', IXML_FIELD_INFO(PMXmlConfig_t, Clear32bit) },
	{ tag:"Clear8bit", format:'u', IXML_FIELD_INFO(PMXmlConfig_t, Clear8bit) },
	{ tag:"ProcessVLCounters", format:'u', IXML_FIELD_INFO(PMXmlConfig_t, process_vl_counters) },
	{ tag:"ProcessHFICounters", format:'u', IXML_FIELD_INFO(PMXmlConfig_t, process_hfi_counters) },
	{ tag:"MaxAttempts", format:'u', IXML_FIELD_INFO(PMXmlConfig_t, MaxRetries) },
	{ tag:"RespTimeout", format:'u', IXML_FIELD_INFO(PMXmlConfig_t, RcvWaitInterval) },
	{ tag:"MinRespTimeout", format:'u', IXML_FIELD_INFO(PMXmlConfig_t, MinRcvWaitInterval) },
	{ tag:"SweepErrorsLogThreshold", format:'u', IXML_FIELD_INFO(PMXmlConfig_t, SweepErrorsLogThreshold) },
	{ tag:"MaxParallelNodes", format:'u', IXML_FIELD_INFO(PMXmlConfig_t, MaxParallelNodes) },
	{ tag:"PmaBatchSize", format:'u', IXML_FIELD_INFO(PMXmlConfig_t, PmaBatchSize) },
	{ tag:"FreezeFrameLease", format:'u', IXML_FIELD_INFO(PMXmlConfig_t, freeze_frame_lease) },
	{ tag:"TotalImages", format:'u', IXML_FIELD_INFO(PMXmlConfig_t, total_images) },
	{ tag:"FreezeFrameImages", format:'u', IXML_FIELD_INFO(PMXmlConfig_t, freeze_frame_images) },
	{ tag:"MaxClients", format:'u', IXML_FIELD_INFO(PMXmlConfig_t, max_clients) },
	{ tag:"ImageUpdateInterval", format:'u', IXML_FIELD_INFO(PMXmlConfig_t, image_update_interval) },
	{ tag:"Thresholds", format:'k', subfields:PmThresholdsFields, start_func:PmThresholdsXmlParserStart },
	{ tag:"ThresholdsExceededMsgLimit", format:'k', subfields:PmThresholdsExceededMsgLimitFields, start_func:PmThresholdsExceededMsgLimitXmlParserStart },
	{ tag:"IntegrityWeights", format:'k', subfields:PmIntegrityWeightsFields, start_func:PmIntegrityWeightsXmlParserStart },
	{ tag:"CongestionWeights", format:'k', subfields:PmCongestionWeightsFields, start_func:PmCongestionWeightsXmlParserStart },
	{ tag:"Resolution", format:'k', subfields:PmResolutionFields, start_func:PmResolutionXmlParserStart },
	{ tag:"Debug", format:'u', IXML_FIELD_INFO(PMXmlConfig_t, debug) },
	{ tag:"RmppDebug", format:'u', IXML_FIELD_INFO(PMXmlConfig_t, debug_rmpp) },
	{ tag:"Priority", format:'u', IXML_FIELD_INFO(PMXmlConfig_t, priority) },
	{ tag:"ElevatedPriority", format:'u', IXML_FIELD_INFO(PMXmlConfig_t, elevated_priority) },
	{ tag:"LogLevel", format:'u', IXML_FIELD_INFO(PMXmlConfig_t, log_level) },
	{ tag:"LogMode", format:'u', IXML_FIELD_INFO(PMXmlConfig_t, syslog_mode) },
	{ tag:"SyslogFacility", format:'s', IXML_FIELD_INFO(PMXmlConfig_t, syslog_facility) },
	{ tag:"CS_LogMask", format:'u', IXML_FIELD_INFO(PMXmlConfig_t, log_masks[VIEO_CS_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"MAI_LogMask", format:'u', IXML_FIELD_INFO(PMXmlConfig_t, log_masks[VIEO_MAI_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"CAL_LogMask", format:'u', IXML_FIELD_INFO(PMXmlConfig_t, log_masks[VIEO_CAL_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"DVR_LogMask", format:'u', IXML_FIELD_INFO(PMXmlConfig_t, log_masks[VIEO_DRIVER_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"IF3_LogMask", format:'u', IXML_FIELD_INFO(PMXmlConfig_t, log_masks[VIEO_IF3_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"SM_LogMask", format:'u', IXML_FIELD_INFO(PMXmlConfig_t, log_masks[VIEO_SM_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"SA_LogMask", format:'u', IXML_FIELD_INFO(PMXmlConfig_t, log_masks[VIEO_SA_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"PM_LogMask", format:'u', IXML_FIELD_INFO(PMXmlConfig_t, log_masks[VIEO_PM_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"PA_LogMask", format:'u', IXML_FIELD_INFO(PMXmlConfig_t, log_masks[VIEO_PA_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"BM_LogMask", format:'u', IXML_FIELD_INFO(PMXmlConfig_t, log_masks[VIEO_BM_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"FE_LogMask", format:'u', IXML_FIELD_INFO(PMXmlConfig_t, log_masks[VIEO_FE_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"APP_LogMask", format:'u', IXML_FIELD_INFO(PMXmlConfig_t, log_masks[VIEO_APP_MOD_ID]), end_func:ParamU32XmlParserEnd },


	{ tag:"LogFile", format:'s', IXML_FIELD_INFO(PMXmlConfig_t, log_file) },
	{ tag:"ShortTermHistory", format:'k', subfields:PmShortTermHistoryFields, start_func:PmShortTermHistoryXmlParserStart, end_func:PmShortTermHistoryXmlParserEnd },
	{ NULL }
};

// "Pm" start tag
static void* PmXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	PMXmlConfig_t *pmp = getXmlMemory(sizeof(PMXmlConfig_t), "PMXmlConfig_t PmXmlParserStart()");

	if (xml_parse_debug)
		fprintf(stdout, "PmXmlParserStart instance %u common %u\n", (unsigned int)instance, (unsigned int)common);

	if (!pmp) {
		PRINT_MEMORY_ERROR;
		return NULL;
	}

	// inherit the values of config items already set
	if (!common && configp->fm_instance[instance])
		*pmp = configp->fm_instance[instance]->pm_config;
	else
		*pmp = configp->fm_instance_common->pm_config;

	return pmp;			// will be passed to PmXmlParserStartEnd as object
}

// "Pm" end tag
static void PmXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	PMXmlConfig_t *pmp = (PMXmlConfig_t*)IXmlParserGetField(field, object);
#ifndef __VXWORKS__
	char facility[256];
#endif

	if (xml_parse_debug)
		fprintf(stdout, "PmXmlParserEnd instance %u common %u\n", (unsigned int)instance, (unsigned int)common);

	if (!valid) {
		fprintf(stderr, "Error processing XML Pm tag\n");
	} else {
		if (pmp->priority != UNDEFINED_XML32 && pmp->priority > MAX_PRIORITY) {
			IXmlParserPrintError(state, "Pm Priority must be in the range of 0-15");
			freeXmlMemory(pmp, sizeof(PMXmlConfig_t), "PMXmlConfig_t PmXmlParserEnd()");
			return;
		}
		if (pmp->elevated_priority != UNDEFINED_XML32 && pmp->elevated_priority > MAX_PRIORITY) {
			IXmlParserPrintError(state, "Pm ElevatedPriority must be in the range of 0-15");
			freeXmlMemory(pmp, sizeof(PMXmlConfig_t), "PMXmlConfig_t PmXmlParserEnd()");
			return;
		}
#ifndef __VXWORKS__
		if (strlen(pmp->syslog_facility) && getFacility(pmp->syslog_facility, /* test */ 1) < 0) {
			IXmlParserPrintError(state, "Pm SyslogFacility must be set to one of the following - %s", showFacilities(facility, sizeof(facility)));
			freeXmlMemory(pmp, sizeof(PMXmlConfig_t), "PMXmlConfig_t PmXmlParserEnd()");
			return;
		}
#endif
	}

	if (common) {
		configp->fm_instance_common->pm_config = *pmp;
	} else if (configp->fm_instance[instance]) {
		// save the Pm config for this instance
		configp->fm_instance[instance]->pm_config = *pmp;
	}

	freeXmlMemory(pmp, sizeof(PMXmlConfig_t), "PMXmlConfig_t PmXmlParserEnd()");
}



#define SID_PARSE_SUCCESS 	0
#define SID_PARSE_FAIL 		1
#define SID_PARSE_DUP		2

static int ServiceIdEnd(IXmlParserState_t *state, cl_qmap_t *dst,
	XML_Char *content, unsigned len, boolean valid, boolean asString)
{
	uint64_t sid;

	if (!valid) {
		fprintf(stderr, "Error processing XML ServiceId tag\n");
		return SID_PARSE_FAIL;
	} else if (!content) {
		IXmlParserPrintError(state, "ServiceId cannot be null");
		return SID_PARSE_FAIL;
	} else if (strlen(content) > MAX_VFABRIC_NAME - 1) {
		IXmlParserPrintError(state, "ServiceId is too long");
		return SID_PARSE_FAIL;
	}

	if (FSUCCESS != StringToUint64(&sid, content, NULL, 16, TRUE)) {
		IXmlParserPrintError(state, "ServiceId %s is formatted incorrectly", content);
		return SID_PARSE_FAIL;
	}

	if (dst) {
		if (asString) {
			char *sidStr = dupName(content);
			if (!sidStr) {
				PRINT_MEMORY_ERROR;
				return SID_PARSE_FAIL;
			}

			if (!addMap(dst, XML_QMAP_U64_CAST sidStr)) {
				freeName(sidStr);
				return SID_PARSE_DUP;
			}
		} else if (!addMap(dst, sid)) {
			return SID_PARSE_DUP;
		}
	}

	return SID_PARSE_SUCCESS;
}

// "ServiceIdRange" end tag
static int ServiceIdRangeEnd(IXmlParserState_t *state, cl_qmap_t *dst,
	XML_Char *content, unsigned len, boolean valid)
{
	if (!valid) {
		fprintf(stderr, "Error processing XML ServiceIdRange tag\n");
	} else if (!content || strlen(content) > MAX_VFABRIC_NAME - 1) {
		IXmlParserPrintError(state, "ServiceIdRange is too long");
		return SID_PARSE_FAIL;
	}

	// test conversion
	VFAppSid_t sidMapping;
	if (verifyAndConvertSidCompoundString(content, 1, &sidMapping) < 0) {
		IXmlParserPrintError(state, "Bad ServiceIdRange format, \"%s\"", content);
		return SID_PARSE_FAIL;
	}

	if (dst) {
		char *range = dupName(content);
		if (!range) {
			PRINT_MEMORY_ERROR;
			return SID_PARSE_FAIL;
		}

		if (!addMap(dst, XML_QMAP_U64_CAST range)) {
			freeName(range);
			return SID_PARSE_DUP;
		}
	}

	return SID_PARSE_SUCCESS;
}

// "ServiceIdMasked" end tag
static int ServiceIdMaskedEnd(IXmlParserState_t *state, cl_qmap_t *dst,
	XML_Char *content, unsigned len, boolean valid)
{
	if (!valid) {
		fprintf(stderr, "Error processing XML ServiceIdMasked tag\n");
	} else if (!content || strlen(content) > MAX_VFABRIC_NAME - 1) {
		IXmlParserPrintError(state, "ServiceIdMasked is too long");
		return SID_PARSE_FAIL;
	}

	// test conversion
	VFAppSid_t sidMapping;
	if (verifyAndConvertSidCompoundString(content, 0, &sidMapping) < 0) {
		IXmlParserPrintError(state, "Bad ServiceIdMasked format, \"%s\"", content);
		return SID_PARSE_FAIL;
	}

	if (dst) {
		char *masked;
		masked = dupName(content);
		if (!masked) {
			PRINT_MEMORY_ERROR;
			return SID_PARSE_FAIL;
		}

		if (!addMap(dst, XML_QMAP_U64_CAST masked)) {
			freeName(masked);
			return SID_PARSE_DUP;
		}
	}

	return SID_PARSE_SUCCESS;
}

// "ServiceId" end tag
static void VfAppServiceIdEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	if (!app) {
		IXmlParserPrintError(state, "Parser internal error");
		return;
	}

	if (xml_parse_debug)
		fprintf(stdout, "VfAppServiceIdEnd instance %u appInstance %u serviceIdInstance %u common %u\n",
			(unsigned int)instance, (unsigned int)appInstance, (unsigned int)serviceIdInstance, (unsigned int)common);

	// check for max
	if (serviceIdInstance >= MAX_VFABRIC_APP_SIDS) {
		PRINT_MEMORY_ERROR;
		return;
	}

	if (!ServiceIdEnd(state, &app->serviceIdMap, content, len, valid, 0)) {
		app->serviceIdMapSize++;
		serviceIdInstance++;
	}
}

// "ServiceIdRange" end tag
static void VfAppServiceIdRangeEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	if (!app) {
		IXmlParserPrintError(state, "Parser internal error");
		return;
	}

	if (xml_parse_debug)
		fprintf(stdout, "VfAppServiceIdRangeEnd instance %u appInstance %u serviceIdRangeInstance %u common %u\n",
			(unsigned int)instance, (unsigned int)appInstance, (unsigned int)serviceIdRangeInstance, (unsigned int)common);

	// check for max
	if (serviceIdRangeInstance >= MAX_VFABRIC_APP_SIDS) {
		PRINT_MEMORY_ERROR;
		return;
	}

	if (!ServiceIdRangeEnd(state, &app->serviceIdRangeMap, content, len, valid)) {
		app->serviceIdRangeMapSize++;
		serviceIdRangeInstance++;
	}
}

// "ServiceIdMasked" end tag
static void VfAppServiceIdMaskedEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	if (!app) {
		IXmlParserPrintError(state, "Parser internal error");
		return;
	}

	if (xml_parse_debug)
		fprintf(stdout, "VfAppServiceIdMaskedEnd instance %u appInstance %u "
			"serviceIdMaskedInstance %u common %u\n", (unsigned int)instance,
			(unsigned int)appInstance, (unsigned int)serviceIdMaskedInstance,
			(unsigned int)common);

	// check for max
	if (serviceIdMaskedInstance >= MAX_VFABRIC_APP_SIDS) {
		PRINT_MEMORY_ERROR;
		return;
	}

	if (!ServiceIdMaskedEnd(state, &app->serviceIdMaskedMap, content, len, valid)) {
		app->serviceIdMaskedMapSize++;
		serviceIdMaskedInstance++;
	}
}

// "MGID" end tag
static void VfAppMGidEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	if (xml_parse_debug)
		fprintf(stdout, "VfAppMGidEnd instance %u appInstance %u mgidInstance %u common %u\n",
			(unsigned int)instance, (unsigned int)appInstance, (unsigned int)mgidInstance, (unsigned int)common);

	// check for max
	if (mgidInstance >= MAX_VFABRIC_APP_MGIDS) {
		PRINT_MEMORY_ERROR;
		return;
	}

	if (!valid) {
		fprintf(stderr, "Error processing XML MGID tag\n");
	} else if (!content || !app || strlen(content) > MAX_VFABRIC_NAME - 1) {
		IXmlParserPrintError(state, "MGID is too long - ignoring");
		return;
	}

	// save away the MGID to the parent structure
	snprintf(app->mgid[mgidInstance].mgid, sizeof(app->mgid[mgidInstance].mgid), "%s", content);

	// test conversion
	VFAppMgid_t mgidMapping;
	if (verifyAndConvertMGidString(content, &mgidMapping) < 0) {
		IXmlParserPrintError(state, "Bad MGID format %s", content);
		return;
	}

	// index to next MGID instance
	mgidInstance++;
}

// "MGIDRange" end tag
static void VfAppMGidRangeEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	if (xml_parse_debug)
		fprintf(stdout, "VfAppMGidRangeEnd instance %u appInstance %u mgidRangeInstance %u common %u\n",
			(unsigned int)instance, (unsigned int)appInstance, (unsigned int)mgidRangeInstance, (unsigned int)common);

	// check for max
	if (mgidRangeInstance >= MAX_VFABRIC_APP_MGIDS) {
		PRINT_MEMORY_ERROR;
		return;
	}

	if (!valid) {
		fprintf(stderr, "Error processing XML MGIDRange tag\n");
	} else if (!content || !app || strlen(content) > MAX_VFABRIC_APP_ELEMENT - 1) {
		IXmlParserPrintError(state, "MGIDRange is too long - ignoring");
		return;
	}

	// save away the MGIDRange to the parent structure
	snprintf(app->mgid_range[mgidRangeInstance].range, sizeof(app->mgid_range[mgidRangeInstance].range), "%s", content);

	// test conversion
	VFAppMgid_t mgidMapping;
	if (verifyAndConvertMGidCompoundString(content, 1, &mgidMapping) < 0) {
		IXmlParserPrintError(state, "Bad MGIDRange format, \"%s\"", content);
		return;
	}

	// index to next MGIDRange instance
	mgidRangeInstance++;
}

// "MGIDMasked" end tag
static void VfAppMGidMaskedEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	if (xml_parse_debug)
		fprintf(stdout, "VfAppMGidMaskedEnd instance %u appInstance %u mgidMaskedInstance %u common %u\n",
			(unsigned int)instance, (unsigned int)appInstance, (unsigned int)mgidMaskedInstance, (unsigned int)common);

	// check for max
	if (mgidMaskedInstance >= MAX_VFABRIC_APP_MGIDS) {
		PRINT_MEMORY_ERROR;
		return;
	}

	if (!valid) {
		fprintf(stderr, "Error processing XML MGIDMasked tag\n");
	} else if (!content || !app || strlen(content) > MAX_VFABRIC_APP_ELEMENT - 1) {
		IXmlParserPrintError(state, "MGIDMasked is too long - ignoring");
		return;
	}

	// save away the MGIDMasked to the parent structure
	snprintf(app->mgid_masked[mgidMaskedInstance].masked, sizeof(app->mgid_masked[mgidMaskedInstance].masked), "%s", content);

	// test conversion
	VFAppMgid_t mgidMapping;
	if (verifyAndConvertMGidCompoundString(content, 0,  &mgidMapping) < 0) {
		IXmlParserPrintError(state, "Bad MGIDMasked format %s", content);
		return;
	}

	// index to next MGIDMasked instance
	mgidMaskedInstance++;
}

// "IncludeApplication" end tag
static void VfAppIncludedApplicationEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	if (xml_parse_debug)
		fprintf(stdout, "VfAppIncludedApplicationEnd instance %u appInstance %u includedAppInstance %u common %u\n",
			(unsigned int)instance, (unsigned int)appInstance, (unsigned int)includedAppInstance, (unsigned int)common);

	// check for max
	if (includedAppInstance >= MAX_INCLUDED_APPS) {
		PRINT_MEMORY_ERROR;
		return;
	}

	if (!valid) {
		fprintf(stderr, "Error processing XML IncludeApplication tag\n");
	} else if (!content || !app || strlen(content) > MAX_VFABRIC_NAME) {
		IXmlParserPrintError(state, "IncludeApplication is too long - ignoring");
		return;
	}

	// save away the IncludedApplication to the parent structure
	snprintf(app->included_app[includedAppInstance].node, sizeof(app->included_app[includedAppInstance].node), "%s", content);

	// index to next IncludedApplication instance
	includedAppInstance++;
}

// "Select" end tag
static void VfAppSelectEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	if (xml_parse_debug)
		fprintf(stdout, "VfAppSelectEnd instance %u appInstance %u common %u\n",
			(unsigned int)instance, (unsigned int)appInstance, (unsigned int)common);

	if (!valid) {
		fprintf(stderr, "Error processing XML Application Select tag\n");
	} else if (!content || !app || strlen(content) > MAX_VFABRIC_NAME - 1) {
		IXmlParserPrintError(state, "Application Select setting is too long - ignoring");
		return;
	}

	if (strcasecmp(content, "SA") == 0)
		app->select_sa |= 1;
	else if (strcasecmp(content, "UnmatchedServiceID") == 0)
		app->select_unmatched_sid |= 1;
	else if (strcasecmp(content, "UnmatchedMGID") == 0)
		app->select_unmatched_mgid |= 1;
	else if (strcasecmp(content, "PM") == 0)
		app->select_pm |= 1;


	else {
		IXmlParserPrintError(state, "Illegal Application Select setting of (%s)", content);
		return;
	}
}

// fields within "Application" tag
static IXML_FIELD VfAppFields[] = {
	{ tag:"Name", format:'s', IXML_FIELD_INFO(AppConfig_t, name) },
	{ tag:"ServiceID", format:'k', end_func:VfAppServiceIdEnd },
	{ tag:"ServiceIDRange", format:'k', end_func:VfAppServiceIdRangeEnd },
	{ tag:"ServiceIDMasked", format:'k', end_func:VfAppServiceIdMaskedEnd },
	{ tag:"MGID", format:'k', end_func:VfAppMGidEnd },
	{ tag:"MGIDRange", format:'k', end_func:VfAppMGidRangeEnd },
	{ tag:"MGIDMasked", format:'k', end_func:VfAppMGidMaskedEnd },
	{ tag:"IncludeApplication", format:'k', end_func:VfAppIncludedApplicationEnd },
	{ tag:"Select", format:'k', end_func:VfAppSelectEnd},
	{ NULL }
};

// "Application" start tag
static void* VfAppXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	// check for max
	if (appInstance >= MAX_VFABRIC_APPS) {
		PRINT_MEMORY_ERROR;
		return NULL;
	}

	app = getApplicationObject();

	if (xml_parse_debug)
		fprintf(stdout, "VfAppXmlParserStart instance %u appInstance %u common %u\n", (unsigned int)instance, (unsigned int)appInstance, (unsigned int)common);

	if (!app) {
		PRINT_MEMORY_ERROR;
		return NULL;
	}

	// clear instances
	serviceIdInstance = 0;
	serviceIdRangeInstance = 0;
	serviceIdMaskedInstance = 0;
	mgidInstance = 0;
	mgidRangeInstance = 0;
	mgidMaskedInstance = 0;
	includedAppInstance = 0;

	return app;	// will be passed to VfAppXmlParserEnd as object
}

// "Application" end tag
static void VfAppXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	app = (AppConfig_t*)IXmlParserGetField(field, object);
	uint32_t i;

	if (xml_parse_debug)
		fprintf(stdout, "VfAppXmlParserEnd instance %u appInstance %u common %u\n", (unsigned int)instance, (unsigned int)appInstance, (unsigned int)common);

	if (!valid) {
		fprintf(stderr, "Error processing XML Application tag\n");
		freeApplicationObject(app);
		return;
	}

	FMXmlInstance_t *instancep;
	if (common) instancep = configp->fm_instance_common;
	else instancep = configp->fm_instance[instance];

	if (!instancep) {
		// We don't care about this instance
		freeApplicationObject(app);
		return;
	}

	// Verify that the application has a name
	if (strlen(app->name) == 0) {
		IXmlParserPrintError(state, "Virtual Fabric Application must have a name");
		freeApplicationObject(app);
		return;
	}

	// keep track of the number of elements
	app->number_of_mgids = mgidInstance;
	app->number_of_mgid_ranges = mgidRangeInstance;
	app->number_of_mgid_range_maskeds = mgidMaskedInstance;
	app->number_of_included_apps = includedAppInstance;

	// Validate the includedApps exist
	for (i = 0; i < app->number_of_included_apps; i++) {
		if (cl_qmap_get(&instancep->app_config.appMap, XML_QMAP_U64_CAST app->included_app[i].node)
		== cl_qmap_end(&instancep->app_config.appMap)) {
			// Not Found
			IXmlParserPrintError(state, "Included application (%s) is not defined", app->included_app[i].node);
			freeApplicationObject(app);
			return;
		}
	}

	/* Finally, add it to the common map */
	if (!addMap(&instancep->app_config.appMap, XML_QMAP_U64_CAST app)) {
		// Couldn't add, must be a duplicate name
		IXmlParserPrintError(state, "Duplicate Virtual Fabric Application (%s) encountered", app->name);
		freeApplicationObject(app);
		return;
	}
	instancep->app_config.appMapSize++;

	// index to next App instance
	appInstance++;
}

// fields within "Applications" tag
static IXML_FIELD VfsAppFields[] = {
	{ tag:"Application", format:'k', subfields:VfAppFields, start_func:VfAppXmlParserStart, end_func:VfAppXmlParserEnd },
	{ NULL }
};

// "Applications" start tag
static void* VfsAppXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	if (xml_parse_debug)
		fprintf(stdout, "VfsAppXmlParserStart instance %u common %u\n", (unsigned int)instance, (unsigned int)common);

	if (common) {
		// reset App index
		appInstance = 0;
	} else if (configp->fm_instance[instance]) {
		// since apps can be inherited from common see how many we have now so we
		// can continue to grow the list
		appInstance = configp->fm_instance[instance]->app_config.appMapSize;
	} else {
		appInstance = 0;
	}

	return NULL;
}

// "Applications" end tag
static void VfsAppXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	if (xml_parse_debug)
		fprintf(stdout, "VfsAppXmlParserEnd instance %u common %u\n", (unsigned int)instance, (unsigned int)common);

	if (!valid) {
		fprintf(stderr, "Error processing XML Applications tag\n");
	} else {
		// as needed process or validate self consistency of config
	}
}

// "SystemImageGUID" end tag
static void VfGroupSystemImageGuidEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint64_t guid;

	if (xml_parse_debug)
		fprintf(stdout, "VfGroupSystemImageGuidEnd instance %u groupInstance %u systemImageInstance %u common %u\n",
			(unsigned int)instance, (unsigned int)groupInstance, (unsigned int)systemImageInstance, (unsigned int)common);

	if (!valid) {
		fprintf(stderr, "Error processing XML SystemImageGUID tag\n");
			return;
	} else if (!content || !dgp || strlen(content) > MAX_VFABRIC_NAME - 1) {
		IXmlParserPrintError(state, "SystemImageGUID is too long - ignoring");
				return;
			}

	// save away the SystemImageGUID to the parent structure if it is a valid string
	if (FSUCCESS != StringToUint64(&guid, content, NULL, 16, TRUE)) {
		IXmlParserPrintError(state, "SystemImageGUID %s is formatted incorrectly", content);
				return;
			}

	addMap(&dgp->system_image_guid, guid);

	// index to next SystemImageGUID instance
	systemImageInstance++;
}

// "NodeGUID" end tag
static void VfGroupNodeGuidEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint64_t guid;

	if (xml_parse_debug)
		fprintf(stdout, "VfGroupNodeGuidEnd instance %u groupInstance %u nodeGuidInstance %u common %u\n",
			(unsigned int)instance, (unsigned int)groupInstance, (unsigned int)nodeGuidInstance, (unsigned int)common);

	if (!valid) {
		fprintf(stderr, "Error processing XML NodeGUID tag\n");
		return;
	} else if (!content || !dgp || strlen(content) > MAX_VFABRIC_NAME - 1) {
		IXmlParserPrintError(state, "NodeGUID is too long - ignoring");
		return;
	}

	// save away the NodeGUID to the parent structure if it is a valid string
	if (FSUCCESS != StringToUint64(&guid, content, NULL, 16, TRUE)) {
		IXmlParserPrintError(state, "NodeGUID %s is formatted incorrectly", content);
		return;
	}

	addMap(&dgp->node_guid, guid);

	// index to next NodeGUID instance
	nodeGuidInstance++;
}

// "PortGUID" end tag
static void VfGroupPortGuidEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint64_t guid;

	if (xml_parse_debug)
		fprintf(stdout, "VfGroupPortGuidEnd instance %u groupInstance %u portGuidInstance %u common %u\n",
						(unsigned int)instance, (unsigned int)groupInstance, (unsigned int)portGuidInstance, (unsigned int)common);

	if (!valid) {
		fprintf(stderr, "Error processing XML PortGUID tag\n");
		return;
	} else if (!content || !dgp || strlen(content) > MAX_VFABRIC_NAME - 1) {
		IXmlParserPrintError(state, "PortGUID is too long - ignoring");
		return;
	}

	// save away the NodeGUID to the parent structure if it is a valid string
	if (FSUCCESS != StringToUint64(&guid, content, NULL, 16, TRUE)) {
		IXmlParserPrintError(state, "PortGUID %s is formatted incorrectly", content);
		return;
	}

	addMap(&dgp->port_guid, guid);

	portGuidInstance++;
}

// "NodeDesc" end tag
static void VfGroupNodeDescEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	XmlNode_t *temp_node_description;

	if (xml_parse_debug)
		fprintf(stdout, "VfGroupNodeDescEnd instance %u groupInstance %u nodeDescInstance %u common %u\n",
			(unsigned int)instance, (unsigned int)groupInstance, (unsigned int)nodeDescInstance, (unsigned int)common);

	if (!valid) {
		fprintf(stderr, "Error processing XML NodeDesc tag\n");
	} else if (!content || !dgp || strlen(content) > MAX_VFABRIC_NAME) {
		IXmlParserPrintError(state, "NodeDesc is too long - ignoring");
		return;
		}

	// save away the NodeDesc to the parent structure
	temp_node_description = getXmlMemory(sizeof(XmlNode_t), "XmlNode_t VfGroupNodeDescEnd()");
	if (!temp_node_description) {
		PRINT_MEMORY_ERROR;
		return;
	}
	snprintf(temp_node_description->node, sizeof(temp_node_description->node), "%s", content);
	temp_node_description->next = NULL;

	if (last_node_description == NULL)
		dgp->node_description = temp_node_description;
	else
		last_node_description->next = temp_node_description;

	last_node_description = temp_node_description;

	//allocate memory used for saving off converted regular expression of this node description
	RegExp_t *temp_reg_expr;

	// save away the RegEx to the parent structure
	temp_reg_expr = getXmlMemory(sizeof(RegExp_t), "RegExp_t VfGroupNodeDescEnd()");
	if (!temp_reg_expr) {
		PRINT_MEMORY_ERROR;
		return;
	}

	temp_reg_expr->next = NULL;

	if (last_reg_expr == NULL)
		dgp->reg_expr = temp_reg_expr;
	else
		last_reg_expr->next = temp_reg_expr;

	last_reg_expr = temp_reg_expr;


	// index to next NodeDesc instance
	nodeDescInstance++;
}

// "IncludedGroup" end tag
static void VfGroupIncludedGroupEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	XmlIncGroup_t *temp_included_group;

	if (xml_parse_debug)
		fprintf(stdout, "VfGroupIncludedGroupEnd instance %u groupInstance %u includedGroupInstance %u common %u\n",
			(unsigned int)instance, (unsigned int)groupInstance, (unsigned int)includedGroupInstance, (unsigned int)common);

	// check for max
	if (includedGroupInstance >= MAX_INCLUDED_GROUPS) {
		PRINT_MEMORY_ERROR;
		return;
	}

	if (!valid) {
		fprintf(stderr, "Error processing XML IncludeGroup tag\n");
	} else if (!content || !dgp || strlen(content) > MAX_VFABRIC_NAME) {
		IXmlParserPrintError(state, "IncludeGroup is too long - ignoring");
		return;
	}

	// save away the IncludedGroup to the parent structure
	temp_included_group = getXmlMemory(sizeof(XmlIncGroup_t), "XmlIncGroup_t VfGroupIncludedGroupEnd()");
	if (!temp_included_group) {
		PRINT_MEMORY_ERROR;
		return;
	}
	snprintf(temp_included_group->group, sizeof(temp_included_group->group), "%s", content);
	temp_included_group->next = NULL;

	if (last_included_group == NULL)
		dgp->included_group = temp_included_group;
	else
		last_included_group->next = temp_included_group;

	last_included_group = temp_included_group;

	// index to next IncludedGroup instance
	includedGroupInstance++;
}

// "Select" end tag
static void VfGroupSelectEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	if (xml_parse_debug)
		fprintf(stdout, "VfGroupSelectEnd instance %u groupInstance %u common %u\n",
			(unsigned int)instance, (unsigned int)groupInstance, (unsigned int)common);

	if (!valid) {
		fprintf(stderr, "Error processing XML DeviceGroup Select tag\n");
	} else if (!content || !dgp || strlen(content) > MAX_VFABRIC_NAME - 1) {
		IXmlParserPrintError(state, "DeviceGroup Select setting is too long - ignoring");
		return;
	}

	if (strcasecmp(content, "All") == 0)
		dgp->select_all |= 1;
	else if (strcasecmp(content, "Self") == 0)
		dgp->select_self |= 1;
	else if (strcasecmp(content, "AllSMs") == 0) {
		fprintf(stderr, "Warning: AllSMs select tag is deprecated; selecting AllMgmtAllowed instead\n");
		dgp->select_all_mgmt_allowed |= 1;
	} else if (strcasecmp(content, "HFIDirectConnect") == 0)
		dgp->select_hfi_direct_connect |= 1;
	else if (strcasecmp(content, "SWE0") == 0)
		dgp->select_swe0 |= 1;
	else if (strcasecmp(content, "AllMgmtAllowed") == 0)
		dgp->select_all_mgmt_allowed |= 1;
	else if (strcasecmp(content, "TFIs") == 0)
		dgp->select_all_tfis |= 1;


	else {
		IXmlParserPrintError(state, "Illegal DeviceGroup Select setting of (%s)", content);
		return;
	}
}

// "NodeType" end tag
static void VfGroupNodeTypeEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	if (xml_parse_debug)
		fprintf(stdout, "VfGroupNodeTypeEnd instance %u groupInstance %u common %u\n",
			(unsigned int)instance, (unsigned int)groupInstance, (unsigned int)common);

	if (!valid) {
		fprintf(stderr, "Error processing XML DeviceGroup NodeType tag\n");
	} else if (!content || !dgp || strlen(content) > MAX_VFABRIC_NAME - 1) {
		IXmlParserPrintError(state, "DeviceGroup NodeType setting is too long - ignoring");
		return;
        }

	if (strcasecmp(content, "FI") == 0)
		dgp->node_type_fi |= 1;
	else if (strcasecmp(content, "SW") == 0)
		dgp->node_type_sw |= 1;
	else {
		IXmlParserPrintError(state, "Illegal DeviceGroup NodeType setting of (%s)", content);
		return;
	}
}

// fields within "Group" tag
static IXML_FIELD VfGroupFields[] = {
	{ tag:"Name", format:'s', IXML_FIELD_INFO(DGConfig_t, name) },
	{ tag:"SystemImageGUID", format:'h', end_func:VfGroupSystemImageGuidEnd },
	{ tag:"NodeGUID", format:'k', end_func:VfGroupNodeGuidEnd },
	{ tag:"PortGUID", format:'k', end_func:VfGroupPortGuidEnd },
	{ tag:"NodeDesc", format:'k', end_func:VfGroupNodeDescEnd },
	{ tag:"IncludeGroup", format:'k', end_func:VfGroupIncludedGroupEnd },
	{ tag:"NodeType", format:'k', end_func:VfGroupNodeTypeEnd },
	{ tag:"Select", format:'k', end_func:VfGroupSelectEnd },
	{ NULL }
};

// "DeviceGroup" start tag
static void* VfGroupXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	// check for max
	if (groupInstance >= MAX_VFABRIC_GROUPS) {
		PRINT_MEMORY_ERROR;
		return NULL;
	}

	dgp = getGroupObject();

	if (xml_parse_debug)
		fprintf(stdout, "VfGroupXmlParserStart instance %u groupInstance %u common %u\n", (unsigned int)instance, (unsigned int)groupInstance, (unsigned int)common);

	if (!dgp) {
		PRINT_MEMORY_ERROR;
		return NULL;
	}

	if (common) {
		// inherit the values of the common Group
		if (configp->fm_instance_common->dg_config.dg[groupInstance] == NULL) {
			configp->fm_instance_common->dg_config.dg[groupInstance] = getGroupObject();
			if (configp->fm_instance_common->dg_config.dg[groupInstance] == NULL) {
				freeGroupObject(dgp, /* full */ 1);
				PRINT_MEMORY_ERROR;
				return NULL;
			}
		}
		// copy Group
		if (cloneGroup(configp->fm_instance_common->dg_config.dg[groupInstance], dgp, 1) < 0) {
			freeGroupObject(dgp, /* full */ 1);
			PRINT_MEMORY_ERROR;
			return NULL;
		}
	} else if (configp->fm_instance[instance]) {
		// inherit the values of Group for this instance
		if (configp->fm_instance[instance]->dg_config.dg[groupInstance] == NULL) {
			configp->fm_instance[instance]->dg_config.dg[groupInstance] = getGroupObject();
			if (configp->fm_instance[instance]->dg_config.dg[groupInstance] == NULL) {
				freeGroupObject(dgp, /* full */ 1);
				PRINT_MEMORY_ERROR;
				return NULL;
			}
		}
		// copy Group
		if (cloneGroup(configp->fm_instance[instance]->dg_config.dg[groupInstance], dgp, 1) < 0) {
			freeGroupObject(dgp, /* full */ 1);
			PRINT_MEMORY_ERROR;
			return NULL;
		}
	}

	// clear instances
	systemImageInstance = 0;
	nodeGuidInstance = 0;
	portGuidInstance = 0;
	nodeDescInstance = 0;
	includedGroupInstance = 0;

	// clear tail pointers
	last_node_description = NULL;
	last_reg_expr = NULL;
	last_included_group = NULL;

	return dgp;	// will be passed to VfGroupXmlParserEnd as object
}

// "DeviceGroup" end tag
static void VfGroupXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	dgp = (DGConfig_t*)IXmlParserGetField(field, object);
	uint32_t i;

	if (xml_parse_debug)
		fprintf(stdout, "VfGroupXmlParserEnd instance %u groupInstance %u common %u\n", (unsigned int)instance, (unsigned int)groupInstance, (unsigned int)common);

	if (!valid) {
		fprintf(stderr, "Error processing XML DeviceGroup tag\n");
		freeGroupObject(dgp, /* full */ 1);
		return;
	}

	FMXmlInstance_t *instancep;
	if (common) instancep = configp->fm_instance_common;
	else instancep = configp->fm_instance[instance];

	if (!instancep) {
		// We don't care about this instance
		freeGroupObject(dgp, /* full */ 1);
		return;
	}

	// Verify that the group has a name
	if (strlen(dgp->name) == 0) {
		IXmlParserPrintError(state, "Virtual Fabric DeviceGroup must have a name");
		freeGroupObject(dgp, /* full */ 1);
		return;
	}
	// If it has a name check for duplicates
	for (i = 0; i < groupInstance; i++) {
		if (strcmp(dgp->name, instancep->dg_config.dg[i]->name) == 0) {
			IXmlParserPrintError(state, "Duplicate Virtual Fabric DeviceGroup (%s) encountered", dgp->name);
			freeGroupObject(dgp, /* full */ 1);
			return;
		}
	}
	// Does this group include any undefined groups?
	XmlIncGroup_t *incgrp = dgp->included_group;
	while(incgrp) {
		for (i = 0; i < groupInstance; i++) {
			if (strcmp(incgrp->group, instancep->dg_config.dg[i]->name) == 0) break;
		}
		if (i == groupInstance) {
			IXmlParserPrintError(state, "Included group (%s) is not defined", incgrp->group);
			freeGroupObject(dgp, /* full */ 1);
			return;
		}
		incgrp = incgrp->next;
	}

	// keep track of the number of elements
	dgp->number_of_system_image_guids = systemImageInstance;
	dgp->number_of_node_guids = nodeGuidInstance;
	dgp->number_of_port_guids = portGuidInstance;
	dgp->number_of_node_descriptions = nodeDescInstance;
	dgp->number_of_included_groups = includedGroupInstance;

	if (common) {
		// save the common settings for this instance
		if (instancep->dg_config.dg[groupInstance] == NULL) {
			instancep->dg_config.dg[groupInstance] = getGroupObject();
			if (instancep->dg_config.dg[groupInstance] == NULL) {
				freeGroupObject(dgp, /* full */ 1);
				PRINT_MEMORY_ERROR;
				return;
			}
		}
		// copy Group
		if (cloneGroup(dgp, instancep->dg_config.dg[groupInstance], 1) < 0) {
			freeGroupObject(dgp, /* full */ 1);
			PRINT_MEMORY_ERROR;
			return;
		}
	} else {
		// save the group config for this instance
		if (instancep->dg_config.dg[groupInstance] == NULL) {
			instancep->dg_config.dg[groupInstance] = getGroupObject();
			if (instancep->dg_config.dg[groupInstance] == NULL) {
				freeGroupObject(dgp, /* full */ 1);
				PRINT_MEMORY_ERROR;
				return;
			}
		}
		// copy Group
		if (cloneGroup(dgp, instancep->dg_config.dg[groupInstance], 1) < 0) {
			freeGroupObject(dgp, /* full */ 1);
			PRINT_MEMORY_ERROR;
			return;
		}
	}

	// index to next Group instance
	groupInstance++;

	freeGroupObject(dgp, /* full */ 1);
}

// fields within "DeviceGroups" tag
static IXML_FIELD VfsGroupsFields[] = {
	{ tag:"DeviceGroup", format:'k', subfields:VfGroupFields, start_func:VfGroupXmlParserStart, end_func:VfGroupXmlParserEnd },
	{ NULL }
};

// "DeviceGroups" start tag
static void* VfsGroupsXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	if (xml_parse_debug)
		fprintf(stdout, "VfsGroupsXmlParserStart instance %u common %u\n", (unsigned int)instance, (unsigned int)common);

	if (common) {
		// reset Group index
		groupInstance = 0;
	} else if (configp->fm_instance[instance]) {
		// since groups can be inherited from common see how many we have now so we
		// can continue to grow the list
		groupInstance = configp->fm_instance[instance]->dg_config.number_of_dgs;
	} else {
		groupInstance = 0;
	}

	return NULL;
}

// "DeviceGroups" end tag
static void VfsGroupsXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	if (xml_parse_debug)
		fprintf(stdout, "VfsGroupsXmlParserEnd instance %u common %u\n", (unsigned int)instance, (unsigned int)common);

	if (!valid) {
		fprintf(stderr, "Error processing XML DeviceGroups tag\n");
	} else {
		// as needed process or validate self consistency of config
	}

	// save the number of accumulated groups
	if (common) {
		configp->fm_instance_common->dg_config.number_of_dgs = groupInstance;
	} else if (configp->fm_instance[instance]) {
		configp->fm_instance[instance]->dg_config.number_of_dgs = groupInstance;
	}
}

// "Member" end tag
static void VfMemberEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	if (xml_parse_debug)
		fprintf(stdout, "VfMemberEnd instance %u vfInstance %u fullMemInstance %u common %u\n",
			(unsigned int)instance, (unsigned int)vfInstance, (unsigned int)fullMemInstance, (unsigned int)common);

	// check for max
	if (fullMemInstance >= MAX_VFABRIC_MEMBERS_PER_VF) {
		PRINT_MEMORY_ERROR;
		return;
	}

	if (!valid) {
		fprintf(stderr, "Error processing XML Member tag\n");
	} else if (!content || !vfp || strlen(content) > MAX_VFABRIC_NAME) {
		IXmlParserPrintError(state, "Member is too long - ignoring");
		return;
	}

	// save away the Member to the parent structure
	snprintf(vfp->full_member[fullMemInstance].member, sizeof(vfp->full_member[fullMemInstance].member), "%s", content);

	// index to next Member instance
	fullMemInstance++;
}

// "LimitedMember" end tag
static void VfLimitedMemberEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	if (xml_parse_debug)
		fprintf(stdout, "VfLimitedMemberEnd instance %u vfInstance %u limitedMemInstance %u common %u\n",
			(unsigned int)instance, (unsigned int)vfInstance, (unsigned int)limitedMemInstance, (unsigned int)common);

	// check for max
	if (limitedMemInstance >= MAX_VFABRIC_MEMBERS_PER_VF) {
		PRINT_MEMORY_ERROR;
		return;
	}

	if (!valid) {
		fprintf(stderr, "Error processing XML LimitedMember tag\n");
	} else if (!content || !vfp || strlen(content) > MAX_VFABRIC_NAME) {
		IXmlParserPrintError(state, "LimitedMember is too long - ignoring");
		return;
	}

	// save away the Member to the parent structure
	snprintf(vfp->limited_member[limitedMemInstance].member, sizeof(vfp->limited_member[limitedMemInstance].member), "%s", content);

	// index to next LimitedMember instance
	limitedMemInstance++;
}

// "Application" end tag
static void VfApplicationsEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	if (xml_parse_debug)
		fprintf(stdout, "VfApplicationsEnd instance %u vfInstance %u appInstance %u common %u\n",
			(unsigned int)instance, (unsigned int)vfInstance, (unsigned int)appInstance, (unsigned int)common);

	// check for max
	if (appInstance >= MAX_VFABRIC_APPS_PER_VF) {
		PRINT_MEMORY_ERROR;
		return;
	}

	if (!valid) {
		fprintf(stderr, "Error processing XML VirtualFabric Application tag\n");
	} else if (!content || !vfp || strlen(content) > MAX_VFABRIC_NAME) {
		IXmlParserPrintError(state, "Application is too long - ignoring");
		return;
	}

	// save away the Application to the parent structure
	snprintf(vfp->application[appInstance].application, sizeof(vfp->application[appInstance].application), "%s", content);

	// index to next Application instance
	appInstance++;
	}

// fields within "VirtualFabric" tag
static IXML_FIELD VfFields[] = {
	{ tag:"Name", format:'s', IXML_FIELD_INFO(VFConfig_t, name) },
	{ tag:"Enable", format:'u', IXML_FIELD_INFO(VFConfig_t, enable) },
	{ tag:"Standby", format:'u', IXML_FIELD_INFO(VFConfig_t, standby) },
	{ tag:"PKey", format:'h', IXML_FIELD_INFO(VFConfig_t, pkey),  end_func:PKeyParserEnd},
	{ tag:"Security", format:'u', IXML_FIELD_INFO(VFConfig_t, security) },
	{ tag:"BaseSL", format:'u', IXML_FIELD_INFO(VFConfig_t, base_sl) },
	{ tag:"MulticastSL", format:'u', IXML_FIELD_INFO(VFConfig_t, mcast_sl) },
	{ tag:"FlowControlDisable", format:'u', IXML_FIELD_INFO(VFConfig_t, flowControlDisable) },
	{ tag:"QOS", format:'u', IXML_FIELD_INFO(VFConfig_t, qos_enable) },
	{ tag:"Bandwidth", format:'k', IXML_FIELD_INFO(VFConfig_t, percent_bandwidth), end_func:PercentU8XmlParserEnd },
	{ tag:"HighPriority", format:'u', IXML_FIELD_INFO(VFConfig_t, priority) },
	{ tag:"PktLifeTimeMult", format:'u', IXML_FIELD_INFO(VFConfig_t, pkt_lifetime_mult) },
	{ tag:"Member", format:'k', end_func:VfMemberEnd },
	{ tag:"LimitedMember", format:'k', end_func:VfLimitedMemberEnd },
	{ tag:"Application", format:'k', end_func:VfApplicationsEnd },
	{ tag:"MaxMTU", format:'k', IXML_FIELD_INFO(VFConfig_t, max_mtu_int), end_func:MtuU8XmlParserEnd },
	{ tag:"MaxRate", format:'k', IXML_FIELD_INFO(VFConfig_t, max_rate_int), end_func:RateU8XmlParserEnd },
	{ tag:"PreemptRank", format:'u', IXML_FIELD_INFO(VFConfig_t, preempt_rank) },
	{ tag:"HoqLife_Int", format:'k', IXML_FIELD_INFO(VFConfig_t, hoqlife_vf), end_func:IXmlParserEndHoqTimeout_Int },
	{ tag:"HoqLife", format:'k', IXML_FIELD_INFO(VFConfig_t, hoqlife_vf), end_func:IXmlParserEndHoqTimeout_Str },
	{ NULL }
};

// "VirtualFabric" start tag
static void* VfXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	// check for max
	if (vfInstance >= MAX_CONFIGURED_VFABRICS) {
		PRINT_MEMORY_ERROR;
		return NULL;
	}

	vfp = getVfObject();

	if (xml_parse_debug)
		fprintf(stdout, "VfXmlParserStart instance %u vfInstance %u common %u\n", (unsigned int)instance, (unsigned int)vfInstance, (unsigned int)common);

	if (!vfp) {
		PRINT_MEMORY_ERROR;
		return NULL;
	}

	// clear member instances
	fullMemInstance = 0;
	limitedMemInstance = 0;
	appInstance = 0;

	return vfp;		// will be passed to VfXmlParserEnd as object
}

// "VirtualFabric" end tag
static void VfXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	vfp = (VFConfig_t*)IXmlParserGetField(field, object);
	uint32_t i;

	if (xml_parse_debug)
		fprintf(stdout, "VfXmlParserEnd instance %u vfInstance %u common %u\n", (unsigned int)instance, (unsigned int)vfInstance, (unsigned int)common);

	if (!valid) {
		fprintf(stderr, "Error processing XML VirtualFabric tag\n");
		freeXmlMemory(vfp, sizeof(VFConfig_t), "VFConfig_t VfXmlParserEnd");
		return;
	}
	// If it is not enabled, just skip it
	if (!vfp->enable) {
		freeXmlMemory(vfp, sizeof(VFConfig_t), "VFConfig_t VfXmlParserEnd");
		return;
	}
	// Verify that the VF has a name
	if (strlen(vfp->name) == 0) {
		IXmlParserPrintError(state, "Virtual Fabric must have a name");
		freeXmlMemory(vfp, sizeof(VFConfig_t), "VFConfig_t VfXmlParserEnd");
		return;
	}
	if (common) {
		// If it has a name check for duplicates
		for (i = 0; i < MAX_CONFIGURED_VFABRICS; i++) {
			if (configp->fm_instance_common->vf_config.vf[i] == NULL)
				break;
			if (strcmp(vfp->name, configp->fm_instance_common->vf_config.vf[i]->name) == 0) {
				IXmlParserPrintError(state, "Duplicate VirtualFabric (%s) encountered", vfp->name);
				freeXmlMemory(vfp, sizeof(VFConfig_t), "VFConfig_t VfXmlParserEnd");
				return;
			}
		}
	} else if (configp->fm_instance[instance]) {
		// If it has a name check for duplicates
		for (i = 0; i < MAX_CONFIGURED_VFABRICS; i++) {
			if (configp->fm_instance[instance]->vf_config.vf[i] == NULL)
				break;
			if (strcmp(vfp->name, configp->fm_instance[instance]->vf_config.vf[i]->name) == 0) {
				IXmlParserPrintError(state, "Duplicate VirtualFabric (%s) encountered", vfp->name);
				freeXmlMemory(vfp, sizeof(VFConfig_t), "VFConfig_t VfXmlParserEnd");
				return;
			}
		}
	}

	// keep track of the number of elements
	vfp->number_of_full_members = fullMemInstance;
	vfp->number_of_limited_members = limitedMemInstance;
	vfp->number_of_applications = appInstance;

	// VF Standby
	if (vfp->standby > 1) {
		IXmlParserPrintError(state, "Standby must be 0=disable 1=enable");
		freeXmlMemory(vfp, sizeof(VFConfig_t), "VFConfig_t VfXmlParserEnd");
		return;
	}

	//security
	if (vfp->security == UNDEFINED_XML32)
		vfp->security = 0;


	// QOS Enable
	if (vfp->qos_enable == UNDEFINED_XML8)
		vfp->qos_enable = 0;
	else if (vfp->qos_enable > 1) {
		IXmlParserPrintError(state, "QOS must be 0=disable 1=enable");
		freeXmlMemory(vfp, sizeof(VFConfig_t), "VFConfig_t VfXmlParserEnd");
		return;
	}
	//init security if needed		
	if (vfp->security == UNDEFINED_XML32)
		vfp->security = 0;

	// validate BaseSL
	if (vfp->base_sl != UNDEFINED_XML8 && vfp->base_sl > (STL_CONFIGURABLE_SLS - 1)) {
		IXmlParserPrintError(state, "BaseSL must be 0-%u", (STL_CONFIGURABLE_SLS - 1));
		freeXmlMemory(vfp, sizeof(VFConfig_t), "VFConfig_t VfXmlParserEnd");
		return;
	}

	// validate RespSL
	if (vfp->resp_sl != UNDEFINED_XML8 && vfp->resp_sl > (STL_CONFIGURABLE_SLS - 1)) {
		IXmlParserPrintError(state, "RespSL must be 0-%u", (STL_CONFIGURABLE_SLS - 1));
		freeXmlMemory(vfp, sizeof(VFConfig_t), "VFConfig_t VfXmlParserEnd");
		return;
	}

	// validate MulticastSL
	if (vfp->mcast_sl != UNDEFINED_XML8 && vfp->mcast_sl > (STL_CONFIGURABLE_SLS - 1)) {
		IXmlParserPrintError(state, "MulticastSL must be 0-%u", (STL_CONFIGURABLE_SLS - 1));
		freeXmlMemory(vfp, sizeof(VFConfig_t), "VFConfig_t VfXmlParserEnd");
		return;
	}

	// validate FlowControlDisable
	if (vfp->flowControlDisable == UNDEFINED_XML8) {
		vfp->flowControlDisable = 0;
	} else if (vfp->flowControlDisable > 1) {
		IXmlParserPrintError(state, "FlowControlDisable must be 0=no 1=yes");
		freeXmlMemory(vfp, sizeof(VFConfig_t), "VFConfig_t VfXmlParserEnd");
		return;
	}

	// validate Priority
	if (vfp->priority == UNDEFINED_XML8) {
		vfp->priority = 0;
	} else if (vfp->priority > 1) {
		IXmlParserPrintError(state, "HighPriority must be 0=no 1=yes");
		freeXmlMemory(vfp, sizeof(VFConfig_t), "VFConfig_t VfXmlParserEnd");
		return;
	}

	// validate PktLifeTimeMult
	if (vfp->pkt_lifetime_mult != UNDEFINED_XML8) {
		if (vfp->pkt_lifetime_mult == 0) {
			IXmlParserPrintError(state, "PktLifeTimeMult cannot be 0");
			freeXmlMemory(vfp, sizeof(VFConfig_t), "VFConfig_t VfXmlParserEnd");
			return;
		} else {
			// Convert to power of 2
			vfp->pkt_lifetime_mult = CeilLog2(vfp->pkt_lifetime_mult);
		}
	}

	// validate Preemption
	if (vfp->preempt_rank != UNDEFINED_XML8 && vfp->preempt_rank > 127) {
		IXmlParserPrintError(state, "Preemption Rank must in the range of 0 .. 127");
		freeXmlMemory(vfp, sizeof(VFConfig_t), "VFConfig_t VfXmlParserEnd");
		return;
	}

	if (common) {
		// save the common settings for this instance
		if (configp->fm_instance_common->vf_config.vf[vfInstance] == NULL) {
			configp->fm_instance_common->vf_config.vf[vfInstance] = getVfObject();
			if (configp->fm_instance_common->vf_config.vf[vfInstance] == NULL) {
				PRINT_MEMORY_ERROR;
				freeXmlMemory(vfp, sizeof(VFConfig_t), "VFConfig_t VfXmlParserEnd");
				return;
			}
			*configp->fm_instance_common->vf_config.vf[vfInstance] = *vfp;
		} // else common and only doing one instance
	} else if (configp->fm_instance[instance]) {
		// save the VirtualFabric config for this instance
		configp->fm_instance[instance]->vf_config.vf[vfInstance] = getVfObject();
		if (configp->fm_instance[instance]->vf_config.vf[vfInstance] == NULL) {
			PRINT_MEMORY_ERROR;
			freeXmlMemory(vfp, sizeof(VFConfig_t), "VFConfig_t VfXmlParserEnd");
			return;
		}
		*configp->fm_instance[instance]->vf_config.vf[vfInstance] = *vfp;
	}

	// index to next VF instance
	vfInstance++;

	freeXmlMemory(vfp, sizeof(VFConfig_t), "VFConfig_t VfXmlParserEnd");
}

// fields within "VirtualFabrics" tag
static IXML_FIELD VfsFields[] = {
	{ tag:"VirtualFabric", format:'k', subfields:VfFields, start_func:VfXmlParserStart, end_func:VfXmlParserEnd },
	{ NULL }
};

// "VirtualFabrics" start tag
static void* VfsXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	VFXmlConfig_t *vf = NULL;

	if (xml_parse_debug)
		fprintf(stdout, "VfsXmlParserStart instance %u common %u\n", (unsigned int)instance, (unsigned int)common);

	if (common) {
		// reset VF index
		vfInstance = 0;
		vf = &configp->fm_instance_common->vf_config;
	} else if (configp->fm_instance[instance]) {
		// since virtual fabrics can be inherited from common see how many we have now so we
		// can continue to grow the list
		vfInstance = configp->fm_instance[instance]->vf_config.number_of_vfs;
		vf = &configp->fm_instance[instance]->vf_config;
	} else {
		vfInstance = 0;
	}

	return vf;
}

// "VirtualFabrics" end tag
static void VfsXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	if (xml_parse_debug)
		fprintf(stdout, "VfsXmlParserEnd instance %u common %u\n", (unsigned int)instance, (unsigned int)common);

	if (!valid) {
		fprintf(stderr, "Error processing XML VirtualFabrics tag\n");
	} else {
		// as needed process or validate self consistency of config
	}

	// save the number of accumulated virtual fabrics
	if (common) {
		configp->fm_instance_common->vf_config.number_of_vfs = vfInstance;
	} else if (configp->fm_instance[instance]) {
		configp->fm_instance[instance]->vf_config.number_of_vfs = vfInstance;
	}
}


// "PmPortGroup/Monitor" end tag
static void PmPgMonitorEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	int i = 0;
	boolean isMonitorValid = 0;
	if (xml_parse_debug)
		fprintf(stdout, "PmPgMonitorEnd instance %u PmPgInstance %u PmPgMonitorInstance %u common %u\n",
			(unsigned int)instance, (unsigned int)vfInstance, (unsigned int)PmPgMonitorInstance, (unsigned int)common);

	// check for max
	if (PmPgMonitorInstance >= STL_PM_MAX_DG_PER_PMPG) {
		PRINT_MEMORY_ERROR;
		return;
	}

	if (!valid) {
		fprintf(stderr, "Error processing XML PM PortGroup Monitor tag\n");
	} else if (!content || !pgp || strlen(content) > STL_PM_GROUPNAMELEN) {
		IXmlParserPrintError(state, "Monitor is too long - ignoring");
		return;
	}

	// save away the Application to the parent structure
	snprintf(pgp->Monitors[PmPgMonitorInstance].monitor, sizeof(pgp->Monitors[PmPgMonitorInstance].monitor), "%s", content);
	for (i=0; i < MAX_VFABRIC_GROUPS ; i++) {
		if (configp->fm_instance_common->dg_config.dg[i]->name) {
			if (strcmp(pgp->Monitors[PmPgMonitorInstance].monitor, configp->fm_instance_common->dg_config.dg[i]->name) == 0) {
				isMonitorValid = 1;
				pgp->Monitors[PmPgMonitorInstance].dg_Index = i;
				break;
			}
			else
				isMonitorValid = 0;
			}
        }
	if(isMonitorValid == 1)
	// index to next Application instance
		PmPgMonitorInstance++;
	else {
		IXmlParserPrintError(state, "Monitor %s is not a valid device group\n ", pgp->Monitors[PmPgMonitorInstance].monitor);
		return;
	}

}

// fields within "PmPortGroup" tag
static IXML_FIELD PmPgFields[] = {
	{ tag:"Enabled", format:'u', IXML_FIELD_INFO(PmPortGroupXmlConfig_t, Enabled) },
	{ tag:"Enable", format:'u', IXML_FIELD_INFO(PmPortGroupXmlConfig_t, Enabled) },
	{ tag:"Name", format:'s', IXML_FIELD_INFO(PmPortGroupXmlConfig_t, Name) },
	{ tag:"Monitor", format:'k', end_func:PmPgMonitorEnd },
	{ NULL }
};

// "PmPortGroup" start tag
static void* PmPgXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	pgp = getPmPgObject();

	if (xml_parse_debug)
		fprintf(stdout, "PmPgXmlParserStart instance %u PmPgInstance %u common %u\n", (unsigned int)instance, (unsigned int)PmPgInstance, (unsigned int)common);

	if (!pgp) {
		PRINT_MEMORY_ERROR;
		return NULL;
	}

	// clear instances
	PmPgMonitorInstance = 0;

	return pgp;	// will be passed to PmPgXmlParserEnd as object
}

// "PmPortGroup" end tag
static void PmPgXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	pgp = (PmPortGroupXmlConfig_t*)IXmlParserGetField(field, object);
	uint32_t i;
	char *reservedGroups[] = {"All", "HFIs", "SWs"};

	if (xml_parse_debug)
		fprintf(stdout, "PmPgXmlParserEnd instance %u PmPgInstance %u common %u\n", (unsigned int)instance, (unsigned int)PmPgInstance, (unsigned int)common);

	if (!valid) {
		fprintf(stderr, "Error processing XML PmPortGroup tag\n");
	} else {
		// Verify that the port group has a name
		if (strlen(pgp->Name) == 0) {
			IXmlParserPrintError(state, "PmPortGroup must have a name");
			freeXmlMemory(pgp, sizeof(PmPortGroupXmlConfig_t), "PmPortGroupXmlConfig_t PmPgXmlParserEnd()");
			return;
		}
		if (pgp->Enabled == 0) {
			//TODO: ACG: Display Message?
			// duplicates allowed if not enabled
			freeXmlMemory(pgp, sizeof(PmPortGroupXmlConfig_t), "PmPortGroupXmlConfig_t PmPgXmlParserEnd()");
			return;
		}
		// check for max
		if (PmPgInstance >= STL_PM_MAX_CUSTOM_PORT_GROUPS) {
			IXmlParserPrintError(state, "Max Custom PmPortGroups (%u) reached", STL_PM_MAX_CUSTOM_PORT_GROUPS);
			freeXmlMemory(pgp, sizeof(PmPortGroupXmlConfig_t), "PmPortGroupXmlConfig_t PmPgXmlParserEnd()");
			return;
		}
		// If it has a name check for duplicates
		for (i = 0; i < PmPgInstance; i++) {
			if (common) {
				if (strcmp(pgp->Name, configp->fm_instance_common->pm_config.pm_portgroups[i].Name) == 0) {
					IXmlParserPrintError(state, "Duplicate PmPortGroup (%s) encountered", pgp->Name);
					freeXmlMemory(pgp, sizeof(PmPortGroupXmlConfig_t), "PmPortGroupXmlConfig_t PmPgXmlParserEnd()");
					return;
				}
			} else if (configp->fm_instance[instance]) {
				if (strcmp(pgp->Name, configp->fm_instance[instance]->pm_config.pm_portgroups[i].Name) == 0) {
					IXmlParserPrintError(state, "Duplicate PmPortGroup (%s) encountered", pgp->Name);
					freeXmlMemory(pgp, sizeof(PmPortGroupXmlConfig_t), "PmPortGroupXmlConfig_t PmPgXmlParserEnd()");
					return;
				}
			}
		}
		// Check the name is not a reserved group
		for (i = 0; i < 3; i++) {
			if (strcmp(pgp->Name, reservedGroups[i]) == 0) {
				IXmlParserPrintError(state, "PmPortGroup (%s) cannot have name of a default port group (All, HFIs, SWs)", pgp->Name);
				freeXmlMemory(pgp, sizeof(PmPortGroupXmlConfig_t), "PmPortGroupXmlConfig_t PmPgXmlParserEnd()");
				return;
			}
		}
	}

	// keep track of the number of elements


	if (common) {
		configp->fm_instance_common->pm_config.pm_portgroups[PmPgInstance] = *pgp;
	} else if (configp->fm_instance[instance]) {
		// inherit the values of PmPortGroup for this instance
		configp->fm_instance[instance]->pm_config.pm_portgroups[PmPgInstance]= *pgp;
	}

	// index to next PmPortGroup instance
	PmPgInstance++;

	freeXmlMemory(pgp, sizeof(PmPortGroupXmlConfig_t), "PmPortGroupXmlConfig_t PmPgXmlParserEnd()");
}

// fields within "PmPortGroups" tag
static IXML_FIELD PmPgsFields[] = {
	{ tag:"PmPortGroup", format:'k', subfields:PmPgFields, start_func:PmPgXmlParserStart, end_func:PmPgXmlParserEnd },
	{ NULL }
};

// "PmPortGroups" start tag
static void* PmPgsXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	if (xml_parse_debug)
		fprintf(stdout, "PmPgsXmlParserStart instance %u common %u\n", (unsigned int)instance, (unsigned int)common);

	if (common) {
		// reset PmPg index
		PmPgInstance = 0;
	} else if (configp->fm_instance[instance]) {
		// since PortGroups can be inherited from common see how many we have now so we
		// can continue to grow the list
		PmPgInstance = configp->fm_instance[instance]->pm_config.number_of_pm_groups;
	} else {
		PmPgInstance = 0;
	}

	return NULL;
}

// "PmPortGroups" end tag
static void PmPgsXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	if (xml_parse_debug)
		fprintf(stdout, "PmPgsXmlParserEnd instance %u common %u\n", (unsigned int)instance, (unsigned int)common);

	if (!valid) {
		fprintf(stderr, "Error processing XML PmPortGroups tag\n");
	} else {
		// as needed process or validate self consistency of config
	}

	// save the number of accumulated pm port groups
	if (common) {
		configp->fm_instance_common->pm_config.number_of_pm_groups = PmPgInstance;
	} else if (configp->fm_instance[instance]) {
		configp->fm_instance[instance]->pm_config.number_of_pm_groups = PmPgInstance;
	}
}

// fields within "Shared" Fm tags
static IXML_FIELD FmSharedFields[] = {
	{ tag:"Start", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, start) },
	{ tag:"Name", format:'s', IXML_FIELD_INFO(FMXmlConfig_t, fm_name) },
	{ tag:"SubnetSize", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, subnet_size) },
	{ tag:"StartupRetries", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, startup_retries) },
	{ tag:"StartupStableWait", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, startup_stable_wait) },
	{ tag:"CoreDumpDir", format:'s', IXML_FIELD_INFO(FMXmlConfig_t, CoreDumpDir) },
	{ tag:"CoreDumpLimit", format:'s', IXML_FIELD_INFO(FMXmlConfig_t, CoreDumpLimit) },
	{ tag:"Debug", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, debug) },
	{ tag:"RmppDebug", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, debug_rmpp) },
	{ tag:"Priority", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, priority) },
	{ tag:"ElevatedPriority", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, elevated_priority) },
	{ tag:"LogLevel", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, log_level) },
	{ tag:"LogFile", format:'s', IXML_FIELD_INFO(FMXmlConfig_t, log_file) },
	{ tag:"LogMode", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, syslog_mode) },
	{ tag:"CS_LogMask", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, log_masks[VIEO_CS_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"MAI_LogMask", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, log_masks[VIEO_MAI_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"CAL_LogMask", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, log_masks[VIEO_CAL_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"DVR_LogMask", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, log_masks[VIEO_DRIVER_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"IF3_LogMask", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, log_masks[VIEO_IF3_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"SM_LogMask", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, log_masks[VIEO_SM_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"SA_LogMask", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, log_masks[VIEO_SA_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"PM_LogMask", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, log_masks[VIEO_PM_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"PA_LogMask", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, log_masks[VIEO_PA_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"BM_LogMask", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, log_masks[VIEO_BM_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"FE_LogMask", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, log_masks[VIEO_FE_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"APP_LogMask", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, log_masks[VIEO_APP_MOD_ID]), end_func:ParamU32XmlParserEnd },


	{ tag:"SyslogFacility", format:'s', IXML_FIELD_INFO(FMXmlConfig_t, syslog_facility) },
	{ tag:"ConfigConsistencyCheckLevel", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, config_consistency_check_level) },
	{ tag:"SslSecurityEnabled", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, SslSecurityEnabled) },
	{ tag:"SslSecurityEnable", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, SslSecurityEnabled) },
#ifndef __VXWORKS__
	{ tag:"SslSecurityDir", format:'s', IXML_FIELD_INFO(FMXmlConfig_t, SslSecurityDir) },
#endif
	{ tag:"SslSecurityFmCertificate", format:'s', IXML_FIELD_INFO(FMXmlConfig_t, SslSecurityFmCertificate) },
	{ tag:"SslSecurityFmPrivateKey", format:'s', IXML_FIELD_INFO(FMXmlConfig_t, SslSecurityFmPrivateKey) },
	{ tag:"SslSecurityFmCaCertificate", format:'s', IXML_FIELD_INFO(FMXmlConfig_t, SslSecurityFmCaCertificate) },
	{ tag:"SslSecurityFmCertChainDepth", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, SslSecurityFmCertChainDepth) },
	{ tag:"SslSecurityFmDHParameters", format:'s', IXML_FIELD_INFO(FMXmlConfig_t, SslSecurityFmDHParameters) },
	{ tag:"SslSecurityFmCaCRLEnabled", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, SslSecurityFmCaCRLEnabled) },
	{ tag:"SslSecurityFmCaCRLEnable", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, SslSecurityFmCaCRLEnabled) },
	{ tag:"SslSecurityFmCaCRL", format:'s', IXML_FIELD_INFO(FMXmlConfig_t, SslSecurityFmCaCRL) },
	{ tag:"Hfi", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, hca), end_func:HfiXmlParserEnd },
	{ tag:"Port", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, port) },
	{ tag:"SubnetPrefix", format:'h', IXML_FIELD_INFO(FMXmlConfig_t, subnet_prefix) },
	{ tag:"PortGUID", format:'h', IXML_FIELD_INFO(FMXmlConfig_t, port_guid) },
	{ NULL }
};

// Fm Shared start tag
static void* FmSharedXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	FMXmlConfig_t *fmp = getXmlMemory(sizeof(FMXmlConfig_t), "FMXmlConfig_t FmSharedXmlParserStart()");

	if (xml_parse_debug)
		fprintf(stdout, "FmSharedXmlParserStart instance %u common %u\n", (unsigned int)instance, (unsigned int)common);

	if (!fmp) {
		PRINT_MEMORY_ERROR;
		return NULL;
	}

	// inherit the values of the common Fm
	*fmp = configp->fm_instance_common->fm_config;

	// set fields that can be overridden per instance of FM to see if we've actually read any real values
	// default for Fm.Name
	sprintf(fmp->fm_name, "fm%u", (unsigned)instance);

	fmp->debug = UNDEFINED_XML32;
	fmp->debug_rmpp = UNDEFINED_XML32;
	fmp->subnet_size = UNDEFINED_XML32;
	fmp->priority = UNDEFINED_XML32;
	fmp->elevated_priority = UNDEFINED_XML32;
	fmp->log_level = UNDEFINED_XML32;
	fmp->syslog_mode = UNDEFINED_XML32;
	memset(fmp->log_masks, 0, sizeof(fmp->log_masks));
	fmp->config_consistency_check_level = UNDEFINED_XML32;
	memset(fmp->log_file, 0, sizeof(fmp->log_file));
	memset(fmp->CoreDumpLimit, 0, sizeof(fmp->CoreDumpLimit));
	memset(fmp->CoreDumpDir, 0, sizeof(fmp->CoreDumpDir));
	memset(fmp->syslog_facility, 0, sizeof(fmp->syslog_facility));
	fmp->SslSecurityEnabled = UNDEFINED_XML32;
	fmp->SslSecurityFmCertChainDepth = UNDEFINED_XML32;
	memset(fmp->SslSecurityDir, 0, sizeof(fmp->SslSecurityDir));
	memset(fmp->SslSecurityFmCertificate, 0, sizeof(fmp->SslSecurityFmCertificate));
	memset(fmp->SslSecurityFmPrivateKey, 0, sizeof(fmp->SslSecurityFmPrivateKey));
	memset(fmp->SslSecurityFmCaCertificate, 0, sizeof(fmp->SslSecurityFmCaCertificate));
	memset(fmp->SslSecurityFmDHParameters, 0, sizeof(fmp->SslSecurityFmDHParameters));
	fmp->SslSecurityFmCaCRLEnabled = UNDEFINED_XML32;
	memset(fmp->SslSecurityFmCaCRL, 0, sizeof(fmp->SslSecurityFmCaCRL));

	return fmp;			// will be passed to SharedFmXmlParserEnd as object
}

// FM Shared end tag
static void FmSharedXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	FMXmlConfig_t *fmp = (FMXmlConfig_t*)IXmlParserGetField(field, object);
	SMXmlConfig_t *smp;
	PMXmlConfig_t *pmp;
	FEXmlConfig_t *fep;


#ifndef __VXWORKS__
	char facility[256];
#endif
	uint32_t modid;

	if (xml_parse_debug)
		fprintf(stdout, "FmSharedXmlParserEnd instance %u common %u\n", (unsigned int)instance, (unsigned int)common);

	if (!valid) {
		fprintf(stderr, "Error processing XML Fm Shared tag\n");
	} else {
		if (fmp->priority != UNDEFINED_XML32 && fmp->priority > MAX_PRIORITY) {
			IXmlParserPrintError(state, "Fm Shared Priority must be in the range of 0-15");
			freeXmlMemory(fmp, sizeof(FMXmlConfig_t), "FMXmlConfig_t FmSharedXmlParserEnd()");
			return;
		}
		if (fmp->elevated_priority != UNDEFINED_XML32 && fmp->elevated_priority > MAX_PRIORITY) {
			IXmlParserPrintError(state, "Fm Shared ElevatedPriority must be in the range of 0-15");
			freeXmlMemory(fmp, sizeof(FMXmlConfig_t), "FMXmlConfig_t FmSharedXmlParserEnd()");
			return;
		}
		if (strlen(fmp->CoreDumpLimit) && vs_getCoreDumpLimit(fmp->CoreDumpLimit, NULL) < 0) {
			IXmlParserPrintError(state, "Invalid Fm CoreDumpLimit: '%s'", fmp->CoreDumpLimit);
			freeXmlMemory(fmp, sizeof(FMXmlConfig_t), "FMXmlConfig_t FmSharedXmlParserEnd()");
			return;
		}
#ifndef __VXWORKS__
		if (strlen(fmp->syslog_facility) && getFacility(fmp->syslog_facility, /* test */ 1) < 0) {
			IXmlParserPrintError(state, "Fm SyslogFacility must be set to one of the following - %s", showFacilities(facility, sizeof(facility)));
			freeXmlMemory(fmp, sizeof(FMXmlConfig_t), "FMXmlConfig_t FmSharedXmlParserEnd()");
			return;
		}
#endif
		if (fmp->config_consistency_check_level != UNDEFINED_XML32 ) {
			if (fmp->config_consistency_check_level == NO_CHECK_CCC_LEVEL){
				XmlParsePrintWarning("Fm ConfigConsistencyCheckLevel = 0 is deprecated. Switched to 1");
				fmp->config_consistency_check_level = CHECK_NO_ACTION_CCC_LEVEL;
			} else if(fmp->config_consistency_check_level > CHECK_ACTION_CCC_LEVEL)  {
			        IXmlParserPrintError(state, "Fm ConfigConsistencyCheckLevel must be set to  1 or 2");
			        freeXmlMemory(fmp, sizeof(FMXmlConfig_t), "FMXmlConfig_t FmSharedXmlParserEnd()");
			        return;
			}
		}
	}

	// save Fm config data for this instance
	if (configp->fm_instance[instance]) {

		configp->fm_instance[instance]->fm_config = *fmp;

		if (xml_parse_debug)
			fprintf(stdout, "FM start = %u instance %u\n", (unsigned int)configp->fm_instance[instance]->fm_config.start, (unsigned int)instance);

		// get pointers to each applications shared config data
		smp = &configp->fm_instance[instance]->sm_config;
		pmp = &configp->fm_instance[instance]->pm_config;
		fep = &configp->fm_instance[instance]->fe_config;


		// inherit shared Fm settings into specific settings for this instance
		if (fmp->subnet_size != UNDEFINED_XML32) {
			smp->subnet_size = fmp->subnet_size;
			pmp->subnet_size = fmp->subnet_size;
			fep->subnet_size = fmp->subnet_size;
		}

		if (fmp->debug != UNDEFINED_XML32) {
			smp->debug = fmp->debug;
			pmp->debug = fmp->debug;
			fep->debug = fmp->debug;
		}

		if (fmp->debug_rmpp != UNDEFINED_XML32) {
			smp->debug_rmpp = fmp->debug_rmpp;
			pmp->debug_rmpp = fmp->debug_rmpp;
			fep->debug_rmpp = fmp->debug_rmpp;
		}

		if (fmp->priority != UNDEFINED_XML32) {
			smp->priority = fmp->priority;
			pmp->priority = fmp->priority;
		}

		if (fmp->elevated_priority != UNDEFINED_XML32) {
			smp->elevated_priority = fmp->elevated_priority;
			pmp->elevated_priority = fmp->elevated_priority;
		}

		if (fmp->hca != UNDEFINED_XML32) {
			smp->hca = fmp->hca;
			pmp->hca = fmp->hca;
			fep->hca = fmp->hca;

		}

		if (fmp->port != UNDEFINED_XML32) {
			smp->port = fmp->port;
			pmp->port = fmp->port;
			fep->port = fmp->port;

		}

		if (fmp->port_guid != UNDEFINED_XML64) {
			smp->port_guid = fmp->port_guid;
			pmp->port_guid = fmp->port_guid;
			fep->port_guid = fmp->port_guid;

		}

		if (fmp->subnet_prefix != UNDEFINED_XML64) {
			smp->subnet_prefix = fmp->subnet_prefix;
		}

		if (strlen(fmp->fm_name)) {
			snprintf(smp->name, MAX_VFABRIC_NAME, "%s_sm", fmp->fm_name);
			snprintf(pmp->name, MAX_VFABRIC_NAME, "%s_pm", fmp->fm_name);
			snprintf(fep->name, MAX_VFABRIC_NAME, "%s_fe", fmp->fm_name);

		}

		if (fmp->log_level != UNDEFINED_XML32) {
			smp->log_level = fmp->log_level;
			pmp->log_level = fmp->log_level;
			fep->log_level = fmp->log_level;
		}
		for (modid=0; modid <= VIEO_LAST_MOD_ID; ++modid) {
			// test for UNDEFINED_XML8 just in case
			if (fmp->log_masks[modid].valid && fmp->log_masks[modid].valid != UNDEFINED_XML8) {
				smp->log_masks[modid] = fmp->log_masks[modid];
				pmp->log_masks[modid] = fmp->log_masks[modid];
				fep->log_masks[modid] = fmp->log_masks[modid];
			}
		}

		if (strlen(fmp->CoreDumpLimit)) {
			cs_strlcpy(smp->CoreDumpLimit, fmp->CoreDumpLimit, STRING_SIZE);
		}

		if (strlen(fmp->CoreDumpDir)) {
			cs_strlcpy(smp->CoreDumpDir, fmp->CoreDumpDir, FILENAME_SIZE);
		}

		if (strlen(fmp->log_file)) {
			cs_strlcpy(smp->log_file, fmp->log_file, LOGFILE_SIZE);
			cs_strlcpy(pmp->log_file, fmp->log_file, LOGFILE_SIZE);
			cs_strlcpy(fep->log_file, fmp->log_file, LOGFILE_SIZE);
		}

		if (strlen(fmp->syslog_facility)) {
			cs_strlcpy(smp->syslog_facility, fmp->syslog_facility, STRING_SIZE);
			cs_strlcpy(pmp->syslog_facility, fmp->syslog_facility, STRING_SIZE);
			cs_strlcpy(fep->syslog_facility, fmp->syslog_facility, STRING_SIZE);
		}

		if (fmp->syslog_mode != UNDEFINED_XML32) {
			smp->syslog_mode = fmp->syslog_mode;
			pmp->syslog_mode = fmp->syslog_mode;
			fep->syslog_mode = fmp->syslog_mode;
		}

		if (fmp->config_consistency_check_level != UNDEFINED_XML32) {
			smp->config_consistency_check_level = fmp->config_consistency_check_level;
			pmp->config_consistency_check_level = fmp->config_consistency_check_level;
		}

		if (fmp->SslSecurityEnabled != UNDEFINED_XML32) {
			smp->SslSecurityEnabled = fmp->SslSecurityEnabled;
			pmp->SslSecurityEnabled = fmp->SslSecurityEnabled;
			fep->SslSecurityEnabled = fmp->SslSecurityEnabled;
		}
		if (strlen(fmp->SslSecurityDir)) {
			cs_strlcpy(smp->SslSecurityDir, fmp->SslSecurityDir, sizeof(smp->SslSecurityDir));
			cs_strlcpy(pmp->SslSecurityDir, fmp->SslSecurityDir, sizeof(pmp->SslSecurityDir));
			cs_strlcpy(fep->SslSecurityDir, fmp->SslSecurityDir, sizeof(fep->SslSecurityDir));
		}

		if (strlen(fmp->SslSecurityFmCertificate)) {
			cs_strlcpy(smp->SslSecurityFmCertificate, fmp->SslSecurityFmCertificate, sizeof(smp->SslSecurityFmCertificate));
			cs_strlcpy(pmp->SslSecurityFmCertificate, fmp->SslSecurityFmCertificate, sizeof(pmp->SslSecurityFmCertificate));
			cs_strlcpy(fep->SslSecurityFmCertificate, fmp->SslSecurityFmCertificate, sizeof(fep->SslSecurityFmCertificate));
		}

		if (strlen(fmp->SslSecurityFmPrivateKey)) {
			cs_strlcpy(smp->SslSecurityFmPrivateKey, fmp->SslSecurityFmPrivateKey, sizeof(smp->SslSecurityFmPrivateKey));
			cs_strlcpy(pmp->SslSecurityFmPrivateKey, fmp->SslSecurityFmPrivateKey, sizeof(pmp->SslSecurityFmPrivateKey));
			cs_strlcpy(fep->SslSecurityFmPrivateKey, fmp->SslSecurityFmPrivateKey, sizeof(fep->SslSecurityFmPrivateKey));
		}

		if (strlen(fmp->SslSecurityFmCaCertificate)) {
			cs_strlcpy(smp->SslSecurityFmCaCertificate, fmp->SslSecurityFmCaCertificate, sizeof(smp->SslSecurityFmCaCertificate));
			cs_strlcpy(pmp->SslSecurityFmCaCertificate, fmp->SslSecurityFmCaCertificate, sizeof(pmp->SslSecurityFmCaCertificate));
			cs_strlcpy(fep->SslSecurityFmCaCertificate, fmp->SslSecurityFmCaCertificate, sizeof(fep->SslSecurityFmCaCertificate));
		}

		if (fmp->SslSecurityFmCertChainDepth != UNDEFINED_XML32) {
			smp->SslSecurityFmCertChainDepth = fmp->SslSecurityFmCertChainDepth;
			pmp->SslSecurityFmCertChainDepth = fmp->SslSecurityFmCertChainDepth;
			fep->SslSecurityFmCertChainDepth = fmp->SslSecurityFmCertChainDepth;
		}

		if (strlen(fmp->SslSecurityFmDHParameters)) {
			cs_strlcpy(smp->SslSecurityFmDHParameters, fmp->SslSecurityFmDHParameters, sizeof(smp->SslSecurityFmDHParameters));
			cs_strlcpy(pmp->SslSecurityFmDHParameters, fmp->SslSecurityFmDHParameters, sizeof(pmp->SslSecurityFmDHParameters));
			cs_strlcpy(fep->SslSecurityFmDHParameters, fmp->SslSecurityFmDHParameters, sizeof(fep->SslSecurityFmDHParameters));
		}

		if (fmp->SslSecurityFmCaCRLEnabled != UNDEFINED_XML32) {
			smp->SslSecurityFmCaCRLEnabled = fmp->SslSecurityFmCaCRLEnabled;
			pmp->SslSecurityFmCaCRLEnabled = fmp->SslSecurityFmCaCRLEnabled;
			fep->SslSecurityFmCaCRLEnabled = fmp->SslSecurityFmCaCRLEnabled;
		}

		if (strlen(fmp->SslSecurityFmCaCRL)) {
			cs_strlcpy(smp->SslSecurityFmCaCRL, fmp->SslSecurityFmCaCRL, sizeof(smp->SslSecurityFmCaCRL));
			cs_strlcpy(pmp->SslSecurityFmCaCRL, fmp->SslSecurityFmCaCRL, sizeof(pmp->SslSecurityFmCaCRL));
			cs_strlcpy(fep->SslSecurityFmCaCRL, fmp->SslSecurityFmCaCRL, sizeof(fep->SslSecurityFmCaCRL));
		}
	}

	freeXmlMemory(fmp, sizeof(FMXmlConfig_t), "FMXmlConfig_t FmSharedXmlParserEnd()");
}

// fields within "Fm" tags
static IXML_FIELD FmFields[] = {
	{ tag:"Shared", format:'K', subfields:FmSharedFields, start_func:FmSharedXmlParserStart, end_func:FmSharedXmlParserEnd },
	{ tag:"Sm", format:'K', subfields:SmFields, start_func:SmXmlParserStart, end_func:SmXmlParserEnd },
	{ tag:"Fe", format:'K', subfields:FeFields, start_func:FeXmlParserStart, end_func:FeXmlParserEnd },
	{ tag:"Pm", format:'K', subfields:PmFields, start_func:PmXmlParserStart, end_func:PmXmlParserEnd },


	{ tag:"Applications", format:'k', subfields:VfsAppFields, start_func:VfsAppXmlParserStart, end_func:VfsAppXmlParserEnd },
	{ tag:"DeviceGroups", format:'k', subfields:VfsGroupsFields, start_func:VfsGroupsXmlParserStart, end_func:VfsGroupsXmlParserEnd },
	{ tag:"VirtualFabrics", format:'k', subfields:VfsFields, start_func:VfsXmlParserStart, end_func:VfsXmlParserEnd },


	{ tag:"PmPortGroups", format:'k', subfields:PmPgsFields, start_func:PmPgsXmlParserStart, end_func:PmPgsXmlParserEnd },
	{ NULL }
};

// Fm start tag
static void* FmXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	if (xml_parse_debug)
		fprintf(stdout, "FmXmlParserStart instance %u\n", (unsigned int)instance);

	// Only allocate if doing a full_parse
	if (full_parse) {
		// Allocate new instance
		configp->fm_instance[instance] = getXmlMemory(sizeof(FMXmlInstance_t), "FMXmlInstance_t FmXmlParserStart()");
		if (!configp->fm_instance[instance]) {
			PRINT_MEMORY_ERROR;
			return NULL;
		}

		// Init new instance with clone from Common
		if (!cloneFmInstance(configp->fm_instance[instance], configp->fm_instance_common)) {
			freeXmlMemory(configp->fm_instance[instance], sizeof(FMXmlInstance_t), "FMXmlInstance_t FmXmlParserStart()" );
			configp->fm_instance[instance] = NULL;
			return NULL;
		}
	}
	common = 0;			// all parsing is for FM now
	return configp->fm_instance[instance];
}

static FSTATUS assign_pkey(uint32 *pkeys, int *numpk, uint32 *candidate)
{
	uint32 pk=0;

	while ( pk < *numpk && pk < MAX_ENABLED_VFABRICS){
		if ((*candidate) == pkeys[pk]) {
			(*candidate)++;
			pk=0;
		}
		else pk++;
	}
	if (pk >= MAX_ENABLED_VFABRICS)
		return FERROR;

	pkeys[pk]=(*candidate);
	(*numpk)++;
	return FSUCCESS;
}
static FSTATUS CheckAllVFsSecurity (VFXmlConfig_t *vf_c)
{
//find if there are any VF with security enabled
	boolean VF_security = FALSE;
	int i;

	for(i=0; i< vf_c->number_of_vfs && i < MAX_CONFIGURED_VFABRICS
			&& !VF_security; i++) {
		if (!vf_c->vf[i]->enable)
			continue;
		if (vf_c->vf[i]->security)
			VF_security = TRUE;
	}
	vf_c->securityEnabled = VF_security;
	return FSUCCESS;
}
// FM end tag
static void FmXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	char error[1000];
	VirtualFabrics_t *vf_config = NULL;


	// instances are limited
	if (instance >= MAX_INSTANCES) {
		IXmlParserPrintError(state, "Maximum number of Fm instances exceeded");
		return;
	}

	if (!valid) {
		fprintf(stderr, "Error processing XML Fm tag\n");
	} else if (full_parse || instance == fm_instance) {
		// verify the Virtual Fabrics Configuration if this is an instance we care about
		FMXmlInstance_t *instancep = configp->fm_instance[instance];

		// fill in configuration parameters that have not been read from the configuration and also
		// calculate the checksums for each component

		if (!fmInitConfig(&instancep->fm_config, instance)) {
			IXmlParserPrintError(state, "Virtual Fabrics XML parse error.");
			return;
		}


		// If startup_retries or startup_stable_wait is not set for a component
		// use the value set for the Fm instance. If the value is also not
		// set for the Fm instance, it will be set to the default value
		// when the component InitConfig routine is called below.
		if (instancep->sm_config.startup_retries == UNDEFINED_XML32)
			instancep->sm_config.startup_retries = instancep->fm_config.startup_retries;
		if (instancep->fe_config.startup_retries == UNDEFINED_XML32)
			instancep->fe_config.startup_retries = instancep->fm_config.startup_retries;


		if (instancep->sm_config.startup_stable_wait == UNDEFINED_XML32)
			instancep->sm_config.startup_stable_wait = instancep->fm_config.startup_stable_wait;
		if (instancep->fe_config.startup_stable_wait == UNDEFINED_XML32)
			instancep->fe_config.startup_stable_wait = instancep->fm_config.startup_stable_wait;



		// If an instance is disabled, disable all of the components in the instance
		if (!instancep->fm_config.start) {
			instancep->sm_config.start = 0;
			instancep->pm_config.start = 0;
			instancep->fe_config.start = 0;

		}


		if (!smInitConfig(&instancep->sm_config, &instancep->sm_dpl_config,
			&instancep->sm_mc_config, &instancep->sm_mls_config,
			&instancep->dg_config, instance)
		|| !pmInitConfig(&instancep->pm_config, instance)
		|| !feInitConfig(&instancep->fe_config, instance)


			) {
			IXmlParserPrintError(state, "Virtual Fabrics XML parse error.");
			return;
		}

		//check if MLIDShare overlap
		if (CheckMLIDShareOverlapping(instance, configp) != FSUCCESS) {
				IXmlParserPrintError(state,  "MLIDShare XML parse error - MLIDShare is allowing same MGID to match two different MLIDShare groups");
				return;
		}

		//assign pkeys to VFs
		uint32	vfs_pkeys[MAX_ENABLED_VFABRICS];
		int	i, default_Pkey=0;
		uint32 candidate_Pkey=1; // 1 is for Default fabric
		int num_pkey=0;

		if (instancep->vf_config.number_of_vfs > MAX_ENABLED_VFABRICS) {
			IXmlParserPrintError(state, "No more than 32 Virtual Fabrics can be enabled in the configuration.");
			return;
		}

		CheckAllVFsSecurity(&instancep->vf_config);
		//64 MAX_CONFIGURED_VFABRICS between enabled and disabled
		//32 max enabled VFs in total: 31 VFs at max + Admin: 32
		//save all already known pkeys
		for(i=0; i< instancep->vf_config.number_of_vfs && i < MAX_CONFIGURED_VFABRICS; i++) {
			if (!instancep->vf_config.vf[i]->enable)
				continue;
			if ((instancep->vf_config.vf[i]->pkey != UNDEFINED_XML32) &&
					(instancep->vf_config.vf[i]->pkey <= 0x7fff)){// 0x7FFF is STL_DEFAULT_PKEY
				if (num_pkey >= MAX_ENABLED_VFABRICS){
					IXmlParserPrintError(state, "Unable to allocate Pkey to vf. Pkey table full.");
					return;
				}
				vfs_pkeys[num_pkey++]= instancep->vf_config.vf[i]->pkey;
			}
		} // for end
		// before doing anything else assigned missing pkeys here... except disabled VFs
		for(i=0; i< instancep->vf_config.number_of_vfs && i < MAX_CONFIGURED_VFABRICS; i++) {
			if (!instancep->vf_config.vf[i]->enable)
				continue;
			if ((instancep->vf_config.vf[i]->pkey == UNDEFINED_XML32) ||
					(instancep->vf_config.vf[i]->pkey > 0x7fff)) {
				if (instancep->vf_config.vf[i]->security){
					// assign unique pkey
					if (num_pkey >= MAX_ENABLED_VFABRICS){
						IXmlParserPrintError(state, "Unable to allocate Pkey to VF. Pkey table full.");
						return;
					}
					if (assign_pkey(vfs_pkeys, &num_pkey, &candidate_Pkey)!= FSUCCESS) {
						IXmlParserPrintError(state, "Unable to allocate Pkey to VF. Pkey table full.");
						return;
					}
					instancep->vf_config.vf[i]->pkey=candidate_Pkey;
				} // end if current VF has security set
				else { // find a pkey and use it as default pkey for all VF that do not have security set
					if (default_Pkey ==0) {
						if (num_pkey >= MAX_ENABLED_VFABRICS){
							IXmlParserPrintError(state, "Unable to allocate Pkey to VF. Pkey table full.");
							return;
						}
						if (assign_pkey(vfs_pkeys, &num_pkey, &candidate_Pkey)!= FSUCCESS) {
							IXmlParserPrintError(state, "Unable to allocate Pkey to VF. Pkey table full.");
							return;
						}
						default_Pkey=candidate_Pkey;
					}
					instancep->vf_config.vf[i]->pkey=default_Pkey;
				} // end else
			}// end if pkeys are undefined
		} // finished assigning pkeys to all VF in config file.

		memset(error, 0, sizeof(error));
		if (instancep->sm_config.start && validateDefaultVirtualFabric(instance, configp, &instancep->vf_config, error)) {
			vf_config = renderVirtualFabricsConfig(instance, configp, &instancep->sm_config, error);
			if (strlen(error)) {
				IXmlParserPrintError(state, "Virtual Fabrics XML parse error - %s", error);
			}
		}
		else if (strlen(error)) {
			IXmlParserPrintError(state, "Virtual Fabrics XML parse error - %s", error);
		}


		// free memory from test
   		if (vf_config != NULL) {
			releaseVirtualFabricsConfig(vf_config);
      			vf_config = NULL;
		}
	}
	// index to next instance
	instance++;
	return;
}

// fields within Common "Shared"
static IXML_FIELD CommonSharedFields[] = {
	{ tag:"Start", format:'k', start_func:NULL, end_func:InvalidOptionParserEnd },
	{ tag:"Debug", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, debug) },
	{ tag:"RmppDebug", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, debug_rmpp) },
	{ tag:"SubnetSize", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, subnet_size) },
	{ tag:"StartupRetries", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, startup_retries) },
	{ tag:"StartupStableWait", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, startup_stable_wait) },
	{ tag:"CoreDumpDir", format:'s', IXML_FIELD_INFO(FMXmlConfig_t, CoreDumpDir) },
	{ tag:"CoreDumpLimit", format:'s', IXML_FIELD_INFO(FMXmlConfig_t, CoreDumpLimit) },
	{ tag:"SslSecurityEnabled", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, SslSecurityEnabled) },
	{ tag:"SslSecurityEnable", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, SslSecurityEnabled) },
#ifndef __VXWORKS__
	{ tag:"SslSecurityDir", format:'s', IXML_FIELD_INFO(FMXmlConfig_t, SslSecurityDir) },
#endif
	{ tag:"SslSecurityFmCertificate", format:'s', IXML_FIELD_INFO(FMXmlConfig_t, SslSecurityFmCertificate) },
	{ tag:"SslSecurityFmPrivateKey", format:'s', IXML_FIELD_INFO(FMXmlConfig_t, SslSecurityFmPrivateKey) },
	{ tag:"SslSecurityFmCaCertificate", format:'s', IXML_FIELD_INFO(FMXmlConfig_t, SslSecurityFmCaCertificate) },
	{ tag:"SslSecurityFmCertChainDepth", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, SslSecurityFmCertChainDepth) },
	{ tag:"SslSecurityFmDHParameters", format:'s', IXML_FIELD_INFO(FMXmlConfig_t, SslSecurityFmDHParameters) },
	{ tag:"SslSecurityFmCaCRLEnabled", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, SslSecurityFmCaCRLEnabled) },
	{ tag:"SslSecurityFmCaCRLEnable", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, SslSecurityFmCaCRLEnabled) },
	{ tag:"SslSecurityFmCaCRL", format:'s', IXML_FIELD_INFO(FMXmlConfig_t, SslSecurityFmCaCRL) },
	{ tag:"Priority", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, priority) },
	{ tag:"ElevatedPriority", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, elevated_priority) },
	{ tag:"LogLevel", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, log_level) },
	{ tag:"LogFile", format:'s', IXML_FIELD_INFO(FMXmlConfig_t, log_file) },
	{ tag:"LogMode", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, syslog_mode) },
	{ tag:"CS_LogMask", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, log_masks[VIEO_CS_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"MAI_LogMask", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, log_masks[VIEO_MAI_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"CAL_LogMask", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, log_masks[VIEO_CAL_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"DVR_LogMask", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, log_masks[VIEO_DRIVER_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"IF3_LogMask", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, log_masks[VIEO_IF3_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"SM_LogMask", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, log_masks[VIEO_SM_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"SA_LogMask", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, log_masks[VIEO_SA_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"PM_LogMask", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, log_masks[VIEO_PM_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"PA_LogMask", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, log_masks[VIEO_PA_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"BM_LogMask", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, log_masks[VIEO_BM_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"FE_LogMask", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, log_masks[VIEO_FE_MOD_ID]), end_func:ParamU32XmlParserEnd },
	{ tag:"APP_LogMask", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, log_masks[VIEO_APP_MOD_ID]), end_func:ParamU32XmlParserEnd },


	{ tag:"SyslogFacility", format:'s', IXML_FIELD_INFO(FMXmlConfig_t, syslog_facility) },
	{ tag:"ConfigConsistencyCheckLevel", format:'u', IXML_FIELD_INFO(FMXmlConfig_t, config_consistency_check_level) },
	{ NULL }
};

// Common Shared start tag
static void* CommonSharedXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	FMXmlConfig_t *fmp = getXmlMemory(sizeof(FMXmlConfig_t), "FMXmlConfig_t CommonSharedXmlParserStart()");

	if (xml_parse_debug)
		fprintf(stdout, "CommonSharedXmlParserStart instance %u common %u\n", (unsigned int)instance, (unsigned int)common);

	if (!fmp) {
		PRINT_MEMORY_ERROR;
		return NULL;
	}

	// inherit the values of the common Fm
	*fmp = configp->fm_instance_common->fm_config;

	return fmp;			// will be passed to CommonXmlParserEnd as object
}

// Common Shared end tag
static void CommonSharedXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	FMXmlConfig_t *fmp = (FMXmlConfig_t*)IXmlParserGetField(field, object);
	SMXmlConfig_t *smp;
	PMXmlConfig_t *pmp;
	FEXmlConfig_t *fep;
#ifndef __VXWORKS__
	char facility[256];
#endif
	uint32_t modid;

	if (xml_parse_debug)
		fprintf(stdout, "CommonSharedXmlParserEnd instance %u common %u\n", (unsigned int)instance, (unsigned int)common);

	if (!valid) {
		fprintf(stderr, "Error processing XML Common Shared tag\n");
	} else {
		if (fmp->priority != UNDEFINED_XML32 && fmp->priority > MAX_PRIORITY) {
			IXmlParserPrintError(state, "Common Shared Priority must be in the range of 0-15");
			freeXmlMemory(fmp, sizeof(FMXmlConfig_t), "FMXmlConfig_t CommonSharedXmlParserEnd()");
			return;
		}
		if (fmp->elevated_priority != UNDEFINED_XML32 && fmp->elevated_priority > MAX_PRIORITY) {
			IXmlParserPrintError(state, "Common Shared ElevatedPriority must be in the range of 0-15");
			freeXmlMemory(fmp, sizeof(FMXmlConfig_t), "FMXmlConfig_t CommonSharedXmlParserEnd()");
			return;
		}
		if (strlen(fmp->CoreDumpLimit) && vs_getCoreDumpLimit(fmp->CoreDumpLimit, NULL) < 0) {
			IXmlParserPrintError(state, "Invalid Fm CoreDumpLimit: '%s'", fmp->CoreDumpLimit);
			freeXmlMemory(fmp, sizeof(FMXmlConfig_t), "FMXmlConfig_t CommonSharedXmlParserEnd()");
			return;
		}
#ifndef __VXWORKS__
		if (strlen(fmp->syslog_facility) && getFacility(fmp->syslog_facility, /* test */ 1) < 0) {
			IXmlParserPrintError(state, "Common Shared SyslogFacility must be set to one of the following - %s", showFacilities(facility, sizeof(facility)));
			freeXmlMemory(fmp, sizeof(FMXmlConfig_t), "FMXmlConfig_t CommonSharedXmlParserEnd()");
			return;
		}
#endif
		if (fmp->config_consistency_check_level != UNDEFINED_XML32) {
			if (fmp->config_consistency_check_level == NO_CHECK_CCC_LEVEL) {
				XmlParsePrintWarning("SM ConfigConsistencyCheckLevel = 0 is deprecated. Switched to 1");
				fmp->config_consistency_check_level = CHECK_NO_ACTION_CCC_LEVEL;
			} else if (fmp->config_consistency_check_level > CHECK_ACTION_CCC_LEVEL) {
			        IXmlParserPrintError(state, "Sm ConfigConsistencyCheckLevel must be set to 1 or 2");
			        freeXmlMemory(fmp, sizeof(FMXmlConfig_t), "FMXmlConfig_t CommonSharedXmlParserEnd()");
			        return;
			}

		}
	}

	// save in common config
	configp->fm_instance_common->fm_config = *fmp;

	// inherit common settings into specific Sm settings
	smp = &configp->fm_instance_common->sm_config;
	smp->subnet_size = fmp->subnet_size;
	smp->debug = fmp->debug;
	smp->debug_rmpp = fmp->debug_rmpp;
	smp->priority = fmp->priority;
	smp->elevated_priority = fmp->elevated_priority;
	smp->log_level = fmp->log_level;
	cs_strlcpy(smp->CoreDumpLimit, fmp->CoreDumpLimit, STRING_SIZE);
	cs_strlcpy(smp->CoreDumpDir, fmp->CoreDumpDir, FILENAME_SIZE);
	cs_strlcpy(smp->log_file, fmp->log_file, LOGFILE_SIZE);
	smp->syslog_mode = fmp->syslog_mode;
	for (modid=0; modid <= VIEO_LAST_MOD_ID; ++modid)
		smp->log_masks[modid] = fmp->log_masks[modid];
	cs_strlcpy(smp->syslog_facility, fmp->syslog_facility, STRING_SIZE);
	smp->config_consistency_check_level = fmp->config_consistency_check_level;
	smp->SslSecurityEnabled = fmp->SslSecurityEnabled;
	cs_strlcpy(smp->SslSecurityDir, fmp->SslSecurityDir, sizeof(smp->SslSecurityDir));
	cs_strlcpy(smp->SslSecurityFmCertificate, fmp->SslSecurityFmCertificate, sizeof(smp->SslSecurityFmCertificate));
	cs_strlcpy(smp->SslSecurityFmPrivateKey, fmp->SslSecurityFmPrivateKey, sizeof(smp->SslSecurityFmPrivateKey));
	cs_strlcpy(smp->SslSecurityFmCaCertificate, fmp->SslSecurityFmCaCertificate, sizeof(smp->SslSecurityFmCaCertificate));
	smp->SslSecurityFmCertChainDepth = fmp->SslSecurityFmCertChainDepth;
	cs_strlcpy(smp->SslSecurityFmDHParameters, fmp->SslSecurityFmDHParameters, sizeof(smp->SslSecurityFmDHParameters));
	smp->SslSecurityFmCaCRLEnabled = fmp->SslSecurityFmCaCRLEnabled;
	cs_strlcpy(smp->SslSecurityFmCaCRL, fmp->SslSecurityFmCaCRL, sizeof(smp->SslSecurityFmCaCRL));

	// inherit common settings into specific Pm settings
	pmp = &configp->fm_instance_common->pm_config;
	pmp->subnet_size = fmp->subnet_size;
	pmp->debug = fmp->debug;
	pmp->debug_rmpp = fmp->debug_rmpp;
	pmp->priority = fmp->priority;
	pmp->elevated_priority = fmp->elevated_priority;
	pmp->log_level = fmp->log_level;
	cs_strlcpy(pmp->log_file, fmp->log_file, LOGFILE_SIZE);
	pmp->syslog_mode = fmp->syslog_mode;
	for (modid=0; modid <= VIEO_LAST_MOD_ID; ++modid)
		pmp->log_masks[modid] = fmp->log_masks[modid];
	cs_strlcpy(pmp->syslog_facility, fmp->syslog_facility, STRING_SIZE);
	pmp->SslSecurityEnabled = fmp->SslSecurityEnabled;
	cs_strlcpy(pmp->SslSecurityDir, fmp->SslSecurityDir, sizeof(pmp->SslSecurityDir));
	cs_strlcpy(pmp->SslSecurityFmCertificate, fmp->SslSecurityFmCertificate, sizeof(pmp->SslSecurityFmCertificate));
	cs_strlcpy(pmp->SslSecurityFmPrivateKey, fmp->SslSecurityFmPrivateKey, sizeof(pmp->SslSecurityFmPrivateKey));
	cs_strlcpy(pmp->SslSecurityFmCaCertificate, fmp->SslSecurityFmCaCertificate, sizeof(pmp->SslSecurityFmCaCertificate));
	pmp->SslSecurityFmCertChainDepth = fmp->SslSecurityFmCertChainDepth;
	cs_strlcpy(pmp->SslSecurityFmDHParameters, fmp->SslSecurityFmDHParameters, sizeof(pmp->SslSecurityFmDHParameters));
	pmp->SslSecurityFmCaCRLEnabled = fmp->SslSecurityFmCaCRLEnabled;
	cs_strlcpy(pmp->SslSecurityFmCaCRL, fmp->SslSecurityFmCaCRL, sizeof(pmp->SslSecurityFmCaCRL));

	// inherit common settings into specific Fe settings
	fep = &configp->fm_instance_common->fe_config;
	fep->subnet_size = fmp->subnet_size;
	fep->debug = fmp->debug;
	fep->debug_rmpp = fmp->debug_rmpp;
	fep->log_level = fmp->log_level;
	cs_strlcpy(fep->CoreDumpLimit, fmp->CoreDumpLimit, STRING_SIZE);
	cs_strlcpy(fep->CoreDumpDir, fmp->CoreDumpDir, FILENAME_SIZE);
	cs_strlcpy(fep->log_file, fmp->log_file, LOGFILE_SIZE);
	fep->syslog_mode = fmp->syslog_mode;
	for (modid=0; modid <= VIEO_LAST_MOD_ID; ++modid)
		fep->log_masks[modid] = fmp->log_masks[modid];
	cs_strlcpy(fep->syslog_facility, fmp->syslog_facility, STRING_SIZE);
	fep->SslSecurityEnabled = fmp->SslSecurityEnabled;
	cs_strlcpy(fep->SslSecurityDir, fmp->SslSecurityDir, sizeof(fep->SslSecurityDir));
	cs_strlcpy(fep->SslSecurityFmCertificate, fmp->SslSecurityFmCertificate, sizeof(fep->SslSecurityFmCertificate));
	cs_strlcpy(fep->SslSecurityFmPrivateKey, fmp->SslSecurityFmPrivateKey, sizeof(fep->SslSecurityFmPrivateKey));
	cs_strlcpy(fep->SslSecurityFmCaCertificate, fmp->SslSecurityFmCaCertificate, sizeof(fep->SslSecurityFmCaCertificate));
	fep->SslSecurityFmCertChainDepth = fmp->SslSecurityFmCertChainDepth;
	cs_strlcpy(fep->SslSecurityFmDHParameters, fmp->SslSecurityFmDHParameters, sizeof(fep->SslSecurityFmDHParameters));
	fep->SslSecurityFmCaCRLEnabled = fmp->SslSecurityFmCaCRLEnabled;
	cs_strlcpy(fep->SslSecurityFmCaCRL, fmp->SslSecurityFmCaCRL, sizeof(fep->SslSecurityFmCaCRL));

	freeXmlMemory(fmp, sizeof(FMXmlConfig_t), "FMXmlConfig_t CommonSharedXmlParserEnd()");
}

// fields within "Common"
static IXML_FIELD CommonFields[] = {
	{ tag:"Shared", format:'K', subfields:CommonSharedFields, start_func:CommonSharedXmlParserStart, end_func:CommonSharedXmlParserEnd },
	{ tag:"Sm", format:'K', subfields:SmFields, start_func:SmXmlParserStart, end_func:SmXmlParserEnd },
	{ tag:"Fe", format:'K', subfields:FeFields, start_func:FeXmlParserStart, end_func:FeXmlParserEnd },
	{ tag:"Pm", format:'K', subfields:PmFields, start_func:PmXmlParserStart, end_func:PmXmlParserEnd },


	{ tag:"Applications", format:'k', subfields:VfsAppFields, start_func:VfsAppXmlParserStart, end_func:VfsAppXmlParserEnd },
	{ tag:"DeviceGroups", format:'k', subfields:VfsGroupsFields, start_func:VfsGroupsXmlParserStart, end_func:VfsGroupsXmlParserEnd },
	{ tag:"VirtualFabrics", format:'k', subfields:VfsFields, start_func:VfsXmlParserStart, end_func:VfsXmlParserEnd },
	{ tag:"PmPortGroups", format:'k', subfields:PmPgsFields, start_func:PmPgsXmlParserStart, end_func:PmPgsXmlParserEnd },
	{ NULL }
};

// Common Shared start tag
static void* CommonXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	common = 1;			// all parsing is for Common now
	return configp->fm_instance_common;
}

// Common Shared end tag
static void CommonXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	if (!valid) {
		fprintf(stderr, "Error processing XML Common tag\n");
	}
}

static IXML_FIELD XmlDebugFields[] = {
	{ tag:"All", format:'u', IXML_FIELD_INFO(XmlDebug_t, xml_all_debug) },
	{ tag:"Vf", format:'u', IXML_FIELD_INFO(XmlDebug_t, xml_vf_debug) },
	{ tag:"Sm", format:'u', IXML_FIELD_INFO(XmlDebug_t, xml_sm_debug) },
	{ tag:"Fe", format:'u', IXML_FIELD_INFO(XmlDebug_t, xml_fe_debug) },
	{ tag:"Pm", format:'u', IXML_FIELD_INFO(XmlDebug_t, xml_pm_debug) },
	{ tag:"Parse", format:'u', IXML_FIELD_INFO(XmlDebug_t, xml_parse_debug) },
	{ NULL }
};

// XML Debug start tag
static void *XmlDebugParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	XmlDebug_t *xdp = getXmlMemory(sizeof(XmlDebug_t), "XmlDebug_t XmlDebugParserStart()");

	if (!xdp) {
		PRINT_MEMORY_ERROR;
		return NULL;
	}
	memset(xdp, 0, sizeof(XmlDebug_t));
	return xdp;
}

// XML Debug end tag
static void XmlDebugParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	XmlDebug_t *xdp = (XmlDebug_t*)IXmlParserGetField(field, object);

#ifdef XML_DEBUG
	xdp->xml_all_debug = 1;
#endif

	if (xdp->xml_all_debug) {
		xdp->xml_vf_debug = 1;
		xdp->xml_sm_debug = 1;
		xdp->xml_fe_debug = 1;
		xdp->xml_pm_debug = 1;
		xdp->xml_parse_debug = 1;
	}
	configp->xmlDebug = *xdp;
	xml_vf_debug = xdp->xml_vf_debug;
	xml_sm_debug = xdp->xml_sm_debug;
	xml_fe_debug = xdp->xml_fe_debug;
	xml_pm_debug = xdp->xml_pm_debug;
	xml_parse_debug = xdp->xml_parse_debug;

	freeXmlMemory(xdp, sizeof(XmlDebug_t), "XmlDebug_t XmlDebugParserEnd()");
}

static IXML_FIELD ConfigFields[] = {
	{ tag:"XmlDebug", format:'k', subfields:XmlDebugFields, start_func:XmlDebugParserStart, end_func:XmlDebugParserEnd },
	{ tag:"Common", format:'K', subfields:CommonFields, start_func:CommonXmlParserStart, end_func:CommonXmlParserEnd },
	{ tag:"Fm", format:'K', subfields:FmFields, start_func:FmXmlParserStart, end_func:FmXmlParserEnd },
	{ NULL }
};

// Config start tag
static void *ConfigXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	return NULL;		// no need for parent tag
}

// Config end tag
static void ConfigXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	if (!valid) {
		fprintf(stderr, "Error processing XML Config tag\n");
	} else {
		// as needed process or validate self consistency of config
	}

}

// top level tag
static IXML_FIELD TopLevelFields[] = {
	{ tag:"Config", format:'K', subfields:ConfigFields, start_func:ConfigXmlParserStart, end_func:ConfigXmlParserEnd },
	{ NULL }
};

// print debug info
void printXmlDebug(FMXmlCompositeConfig_t *config, uint32_t fm)
{
	unsigned int i;
	cl_map_item_t *cl_map_item;

	// render config data into internal application structure
	VirtualFabrics_t *test = renderVirtualFabricsConfig(fm, config, &config->fm_instance[fm]->sm_config, NULL);

	fprintf(stdout, "Printing out the test VF configuration for FM %u from printXmlDebug()\n", (unsigned int)fm);

	// if it failed get out of here
	if (!test) {
		fprintf(stdout, "Test Virtual Fabric could not be created\n");
		return;
	}

	for (i = 0; i < test->number_of_vfs_all; i++) {
		if (test->v_fabric_all[i].standby) continue;
		fprintf(stdout, "VF %u name %s\n", i, test->v_fabric_all[i].name);
		fprintf(stdout, "VF %u index %u\n", i, (unsigned int)test->v_fabric_all[i].index);
		fprintf(stdout, "VF %u pkey 0x%x\n", i, (unsigned int)test->v_fabric_all[i].pkey);
		fprintf(stdout, "VF %u security %u\n", i, test->v_fabric_all[i].security);
		fprintf(stdout, "VF %u qos_enable %u\n", i, test->v_fabric_all[i].qos_enable);
		fprintf(stdout, "VF %u flowControlDisable %u\n", i, test->v_fabric_all[i].flowControlDisable);
		fprintf(stdout, "VF %u base_sl %u\n", i, test->v_fabric_all[i].base_sl);
		fprintf(stdout, "VF %u requires_resp_sl %u\n", i, test->v_fabric_all[i].requires_resp_sl);
		fprintf(stdout, "VF %u resp_sl %u\n", i, test->v_fabric_all[i].resp_sl);
		fprintf(stdout, "VF %u mcast_sl %u\n", i, test->v_fabric_all[i].mcast_sl);
		fprintf(stdout, "VF %u percent_bandwidth %u\n", i, test->v_fabric_all[i].percent_bandwidth);
		fprintf(stdout, "VF %u priority %u\n", i, test->v_fabric_all[i].priority);
		fprintf(stdout, "VF %u pkt_lifetime_mult %u\n", i, test->v_fabric_all[i].pkt_lifetime_mult);
		fprintf(stdout, "VF %u max_mtu_int %u\n", i, test->v_fabric_all[i].max_mtu_int);
		fprintf(stdout, "VF %u max_rate_int %u\n", i, test->v_fabric_all[i].max_rate_int);
		fprintf(stdout, "VF %u preempt_rank %u\n", i, test->v_fabric_all[i].preempt_rank);
		fprintf(stdout, "VF %u hoqlife_vf %u\n", i, (unsigned int)test->v_fabric_all[i].hoqlife_vf);

		fprintf(stdout, "VF %u app select_sa %u\n", i, test->v_fabric_all[i].apps.select_sa);
		fprintf(stdout, "VF %u app select_unmatched_sid %u\n", i, test->v_fabric_all[i].apps.select_unmatched_sid);
		fprintf(stdout, "VF %u app select_unmatched_mgid %u\n", i, test->v_fabric_all[i].apps.select_unmatched_mgid);
		fprintf(stdout, "VF %u app select_pm %u\n", i, test->v_fabric_all[i].apps.select_pm);


		fprintf(stdout, "VF %u app sidMapSize %u\n", i, (unsigned int)test->v_fabric_all[i].apps.sidMapSize);
		VFAppSid_t *sid;
		for_all_qmap_ptr(&test->v_fabric_all[i].apps.sidMap, cl_map_item, sid) {
			fprintf(stdout, "VF %u app service_id 0x%16.16llx\n", i,
				(long long unsigned int)sid->service_id);
			fprintf(stdout, "VF %u app service_id_last 0x%16.16llx\n", i,
				(long long unsigned int)sid->service_id_last);
			fprintf(stdout, "VF %u app service_id_mask 0x%16.16llx\n", i,
				(long long unsigned int)sid->service_id_mask);
		}

		fprintf(stdout, "VF %u app mgidMapSize %u\n", i, (unsigned int)test->v_fabric_all[i].apps.mgidMapSize);
		VFAppMgid_t *mgid;
		for_all_qmap_ptr(&test->v_fabric_all[i].apps.mgidMap, cl_map_item, mgid) {
			mgid = XML_QMAP_VOID_CAST cl_qmap_key(cl_map_item);
			fprintf(stdout, "VF %u app mgid 0x%16.16llx:0x%16.16llx\n", i,
				(long long unsigned int)mgid->mgid[0], (long long unsigned int)mgid->mgid[1]);
			fprintf(stdout, "VF %u app mgid_last 0x%16.16llx:0x%16.16llx\n", i,
				(long long unsigned int)mgid->mgid_last[0], (long long unsigned int)mgid->mgid_last[1]);
			fprintf(stdout, "VF %u app mgid_mask 0x%16.16llx:0x%16.16llx\n", i,
				(long long unsigned int)mgid->mgid_mask[0], (long long unsigned int)mgid->mgid_mask[1]);
		}


		fprintf(stdout, "VF %u full_member select_all %u\n", i, test->v_fabric_all[i].full_members.select_all);
		fprintf(stdout, "VF %u full_member select_self %u\n", i, test->v_fabric_all[i].full_members.select_self);
		fprintf(stdout, "VF %u full_member select_hfi_direct_connect %u\n", i, test->v_fabric_all[i].full_members.select_hfi_direct_connect);
		fprintf(stdout, "VF %u full_member select_swe0 %u\n", i, test->v_fabric_all[i].full_members.select_swe0);
		fprintf(stdout, "VF %u full_member select_all_mgmt_allowed %u\n", i, test->v_fabric_all[i].full_members.select_all_mgmt_allowed);
		fprintf(stdout, "VF %u full_member select_all_tfis %u\n", i, test->v_fabric_all[i].full_members.select_all_tfis);
		fprintf(stdout, "VF %u full_member node_type_fi %u\n", i, test->v_fabric_all[i].full_members.node_type_fi);
		fprintf(stdout, "VF %u full_member node_type_sw %u\n", i, test->v_fabric_all[i].full_members.node_type_sw);

		fprintf(stdout, "VF %u full_member sysGuidMapSize %u\n", i,
			(unsigned int)test->v_fabric_all[i].full_members.sysGuidMapSize);
		{
			long long unsigned int guid;
			for_all_qmap_u64(&test->v_fabric_all[i].full_members.sysGuidMap, cl_map_item, guid) {
				fprintf(stdout, "VF %u full_member sys_guid 0x%16.16llx\n", i, guid);
			}
		}

		fprintf(stdout, "VF %u full_member nodeGuidMapSize %u\n", i,
			(unsigned int)test->v_fabric_all[i].full_members.nodeGuidMapSize);
		{
			long long unsigned int guid;
			for_all_qmap_u64(&test->v_fabric_all[i].full_members.nodeGuidMap, cl_map_item, guid) {
				fprintf(stdout, "VF %u full_member node_guid 0x%16.16llx\n", i, guid);
			}
		}

		fprintf(stdout, "VF %u full_member portGuidMapSize %u\n", i,
			(unsigned int)test->v_fabric_all[i].full_members.portGuidMapSize);
		{
			long long unsigned int guid;
			for_all_qmap_u64(&test->v_fabric_all[i].full_members.portGuidMap, cl_map_item, guid) {
				fprintf(stdout, "VF %u full_member port_guid 0x%16.16llx\n", i, guid);
			}
		}

		fprintf(stdout, "VF %u full_member nodeDescMapSize %u\n", i,
			(unsigned int)test->v_fabric_all[i].full_members.nodeDescMapSize);
		{
			char *name;
			for_all_qmap_ptr(&test->v_fabric_all[i].full_members.nodeDescMap, cl_map_item, name) {
				fprintf(stdout, "VF %u full_member node_descr %s\n", i, name);
			}
		}

		fprintf(stdout, "VF %u limited_member select_all %u\n", i, test->v_fabric_all[i].limited_members.select_all);
		fprintf(stdout, "VF %u limited_member select_self %u\n", i, test->v_fabric_all[i].limited_members.select_self);
		fprintf(stdout, "VF %u limited_member select_hfi_direct_connect %u\n", i, test->v_fabric_all[i].limited_members.select_hfi_direct_connect);
		fprintf(stdout, "VF %u limited_member select_all_mgmt_allowed %u\n", i, test->v_fabric_all[i].limited_members.select_all_mgmt_allowed);
		fprintf(stdout, "VF %u limited_member select_all_tfis %u\n", i, test->v_fabric_all[i].limited_members.select_all_tfis);
		fprintf(stdout, "VF %u limited member select_swe0 %u\n", i, test->v_fabric_all[i].limited_members.select_swe0);
		fprintf(stdout, "VF %u limited_member node_type_fi %u\n", i, test->v_fabric_all[i].limited_members.node_type_fi);
		fprintf(stdout, "VF %u limited_member node_type_sw %u\n", i, test->v_fabric_all[i].limited_members.node_type_sw);

		fprintf(stdout, "VF %u limited_member sysGuidMapSize %u\n", i,
			(unsigned int)test->v_fabric_all[i].limited_members.sysGuidMapSize);
		{
			long long unsigned int guid;
			for_all_qmap_u64(&test->v_fabric_all[i].limited_members.sysGuidMap, cl_map_item, guid) {
				fprintf(stdout, "VF %u limited_member sys_guid 0x%16.16llx\n", i, guid);
			}
		}

		fprintf(stdout, "VF %u limited_member nodeGuidMapSize %u\n", i,
			(unsigned int)test->v_fabric_all[i].limited_members.nodeGuidMapSize);
		{
			long long unsigned int guid;
			for_all_qmap_u64(&test->v_fabric_all[i].limited_members.nodeGuidMap, cl_map_item, guid) {
				fprintf(stdout, "VF %u limited_member node_guid 0x%16.16llx\n", i, guid);
			}
		}

		fprintf(stdout, "VF %u limited_member portGuidMapSize %u\n", i,
			(unsigned int)test->v_fabric_all[i].limited_members.portGuidMapSize);
		{
			long long unsigned int guid;
			for_all_qmap_u64(&test->v_fabric_all[i].limited_members.portGuidMap, cl_map_item, guid) {
				fprintf(stdout, "VF %u limited_member port_guid 0x%16.16llx\n", i, guid);
			}
		}

		fprintf(stdout, "VF %u limited_member nodeDescMapSize %u\n", i,
			(unsigned int)test->v_fabric_all[i].limited_members.nodeDescMapSize);
		{
			char *name;
			for_all_qmap_ptr(&test->v_fabric_all[i].limited_members.nodeDescMap, cl_map_item, name) {
				fprintf(stdout, "VF %u limited_member node_descr %s\n", i, name);
			}
		}

		fprintf(stdout, "VF %u number of mulitcast_groups %u\n", i,
			(unsigned int)test->v_fabric_all[i].number_of_default_groups);
		VFDg_t *default_group = test->v_fabric_all[i].default_group;
		uint32_t index = 0;
		while (default_group) {
			fprintf(stdout, "VF %u mulitcast_group %u def_mc_create %u\n", i, (unsigned int)index, default_group->def_mc_create);
			fprintf(stdout, "VF %u mulitcast_group %u def_mc_pkey 0x%x\n", i, (unsigned int)index, (unsigned int)default_group->def_mc_pkey);
			fprintf(stdout, "VF %u mulitcast_group %u def_mc_mtu_int %u\n", i, (unsigned int)index, default_group->def_mc_mtu_int);
			fprintf(stdout, "VF %u mulitcast_group %u def_mc_rate_int %u\n", i, (unsigned int)index, default_group->def_mc_rate_int);
			fprintf(stdout, "VF %u mulitcast_group %u def_mc_sl %u\n", i, (unsigned int)index, (unsigned int)default_group->def_mc_sl);
			fprintf(stdout, "VF %u mulitcast_group %u def_mc_qkey %u\n", i, (unsigned int)index, (unsigned int)default_group->def_mc_qkey);
			fprintf(stdout, "VF %u mulitcast_group %u def_mc_fl %u\n", i, (unsigned int)index, (unsigned int)default_group->def_mc_fl);
			fprintf(stdout, "VF %u mulitcast_group %u def_mc_tc %u\n", i, (unsigned int)index, (unsigned int)default_group->def_mc_tc);

			fprintf(stdout, "VF %u mulitcast_group %u mgidMapSize %u\n", i, (unsigned int)index, (unsigned int)default_group->mgidMapSize);
			VFAppMgid_t *mgid;
			for_all_qmap_ptr(&default_group->mgidMap, cl_map_item, mgid) {
				fprintf(stdout, "VF %u mulitcast_group %u mgid 0x%16.16llx:0x%16.16llx\n", i, (unsigned int)index,
					(long long unsigned int)mgid->mgid[0], (long long unsigned int)mgid->mgid[1]);
				fprintf(stdout, "VF %u mulitcast_group %u mgid_last 0x%16.16llx:0x%16.16llx\n", i, (unsigned int)index,
					(long long unsigned int)mgid->mgid_last[0], (long long unsigned int)mgid->mgid_last[1]);
				fprintf(stdout, "VF %u mulitcast_group %u mgid_mask 0x%16.16llx:0x%16.16llx\n", i, (unsigned int)index,
					(long long unsigned int)mgid->mgid_mask[0], (long long unsigned int)mgid->mgid_mask[1]);
			}
			default_group = default_group->next_default_group;
			index++;
		}
		fprintf(stdout, "VF %u Checksum %u\n", i, (unsigned int)test->v_fabric_all[i].consistency_checksum);
	}

	fprintf(stdout, "VF database Checksum %u\n", (unsigned int)test->consistency_checksum);

	// free memory from test
    if (test != NULL) {
		releaseVirtualFabricsConfig(test);
        test = NULL;
    }
}

#ifdef __VXWORKS__
void* getParserMemory(size_t size)
{
	void* memory;
	memory = getXmlMemory((uint32_t)size, "getParserMemory()");

	if (memory == NULL)
		return NULL;

	memset(memory, 0, size);

	return memory;
}

void* reallocParserMemory(void* ptr, size_t size)
{
	freeXmlMemory(ptr, 0, "freeParserMemory()");
	return getXmlMemory((uint32_t)size, "getParserMemory()");
}

void freeParserMemory(void* ptr)
{
	return freeXmlMemory(ptr, 0, "freeParserMemory()");
}
#endif

// main parser
FSTATUS XmlParseFmConfig(const char *input_file, IXmlParserFlags_t flags)
{
#ifdef XML_TEST
	return IXmlParseInputFile(input_file, flags, TopLevelFields, NULL, NULL, NULL, NULL, NULL, NULL);
#else
#ifndef __VXWORKS__
	return IXmlParseInputFile(input_file, flags, TopLevelFields, NULL, NULL, XmlParsePrintError, XmlParsePrintWarning, NULL, NULL);
#else
	XML_Memory_Handling_Suite memsuite;
	memsuite.malloc_fcn = &getParserMemory;
	memsuite.realloc_fcn = &reallocParserMemory;
	memsuite.free_fcn = &freeParserMemory;
	return IXmlParseInputFile(input_file, flags, TopLevelFields, NULL, NULL, XmlParsePrintError, XmlParsePrintWarning, NULL, NULL, &memsuite);
#endif
#endif
}

// initialize memory get function for parsing XML
void initXmlPoolGetCallback(void *function)
{
	get_memory = function;
}

// initialize memory free function
void initXmlPoolFreeCallback(void *function)
{
	free_memory = function;
}

#ifdef __VXWORKS__
// generate default XML configuration file
int generateDefaultXmlConfig(uint8_t cli)
{

	scpInProcess = 1;

	/* check for XML reset from CLI and previous default XML file name */
	if (cli &&  FileExists(IFS_FM_CFG_NAME_OLD)) {
		/* remove old default XML filename */
		FileRemove(IFS_FM_CFG_NAME_OLD);
		//sysPrintf("[%s] removed %s \n",__FUNCTION__,IFS_FM_CFG_NAME_OLD);
	}

	if (!FileExists(IFS_FM_CFG_NAME)) {

		/* check for previous default XML file name */
		if (FileExists(IFS_FM_CFG_NAME_OLD)) {
			/* make a copy of existing XML config file, use new name */
			//sysPrintf("[%s] making copy of %s \n",__FUNCTION__,IFS_FM_CFG_NAME_OLD);
			if (copyFile( IFS_FM_CFG_NAME_OLD, IFS_FM_CFG_NAME, 0,NULL))
				XmlParsePrintError("Error duplicating old XML config file");
		}
		else {
			/* generate the XML config from the embedded default file in xml_data array */
			sysPrintf("[%s] generate default %s\n",__FUNCTION__,IFS_FM_CFG_NAME);
			FileRemove(IFS_FM_CFG_NAME_UNCOMPRESSED);
			if (FileWriteBin( IFS_FM_CFG_NAME_UNCOMPRESSED, xml_data, sizeof(xml_data)) != sizeof(xml_data)) {
				scpInProcess = 0;
				XmlParsePrintError("Error generating XML config file");
				return -1;
			}
			if (copyFile(IFS_FM_CFG_NAME_UNCOMPRESSED, IFS_FM_CFG_NAME, -1,NULL)) {
				scpInProcess = 0;
				XmlParsePrintError("Error copying generated XML config file");
				remove(IFS_FM_CFG_NAME_UNCOMPRESSED);
				return -1;
			}
		}

		// mirror the XML file to remote CPU - if this is not a multi-CPU system then
		// the call will simply return without doing any mirroring
#if defined(__VXWORKS__)
		if (cli)
			cmuRed_cfgIfMasterSyncSlave(NULL);
#endif
	}

	scpInProcess = 0;
	return 0;
}

#endif

// call entry point for parsing the opafm.xml configuration file - will
// return data in FMXmlCompositeConfig_t struct
FMXmlCompositeConfig_t* parseFmConfig(char *filename, uint32_t flags, uint32_t fm, uint32_t full, uint32_t embedded)
{
	uint32_t i;

	if (!filename || strlen(filename) == 0)
       filename = IFS_FM_CFG_NAME;

	if (parsingInProcess) {
		fprintf(stdout, "Unable to parse XML file while another parse in in progress\n");
		return NULL;
	}

	parsingInProcess = 1;
	embedded_call = embedded;

#ifdef __VXWORKS__
	FILE  *aFile;
	i = 100;
	while (i) {
		aFile = fopen( IFS_FM_CFG_NAME_NEW, "r");
		if (aFile != NULL) {
			if (i == 100)
				fprintf(stdout, "Temporary file %s encountered - waiting to parse %s\n", IFS_FM_CFG_NAME_NEW, IFS_FM_CFG_NAME);
			fclose(aFile);
			vs_thread_sleep(1000000);
			i--;
		} else {
			break;
		}
	}

	// generate a default if need be
	if (generateDefaultXmlConfig( /* not a cli command */ 0)) {
		fprintf(stdout, "Unable to parse XML file since a default file cannot be generated\n");
		return NULL;
	}
#endif

	// clear the memory counter
	memory = 0;

	// calculate memory limit
	memory_limit = xml_compute_pool_size(full);

	if (xml_memory_debug) {
		fprintf(stdout, "Memory level %u before parseFmConfig()\n", (unsigned int)memory);
	}

	// get memory for composite struct
	configp = getXmlMemory(sizeof(FMXmlCompositeConfig_t), "FMXmlCompositeConfig_t parseFmConfig()");
	if (!configp) {
		fprintf(stdout, "Memory limit has been exceeded parsing the XML configuration");
		parsingInProcess = 0;
		return NULL;
	}

	// get memory for common Fm instance if we are doing a full parse
	configp->fm_instance_common = getXmlMemory(sizeof(FMXmlInstance_t), "FMXmlInstance_t parseFmConfig()");
	if (!configp->fm_instance_common) {
		fprintf(stdout, "Memory limit has been exceeded parsing the XML configuration");
		goto failure;
	}

	// set instance info - if called with full then data for all FM instances will be
	// returned in data structure pointed to by the FMXmlCompositeConfig_t*
	full_parse = full;
	if (!full_parse) {
		fm_instance = fm;
		end_instance = fm + 1;
	} else {
		fm_instance = 0;
		end_instance = MAX_INSTANCES;
	}

	for (i = 0; i < MAX_INSTANCES; i++)
		configp->fm_instance[i] = NULL;

	// if we are not doing a full parse then the common instance can become the same as the single instance
	// we are parsing
	if (!full)
		configp->fm_instance[fm] = configp->fm_instance_common;

	// init config
	xmlInitConfig();

	// parse opafm.xml file
	if (xml_parse_debug)
		fprintf(stdout, "\nParsing %s...\n", filename);

	if (FSUCCESS != XmlParseFmConfig(filename, (IXmlParserFlags_t)flags)) {
		if (xml_parse_debug)
			fprintf(stdout, "\nParsing %s failed\n", filename);
		goto failure;
	}
	configp->num_instances = instance;

	// print out some debug info
	if (xml_parse_debug || xml_vf_debug)
		printXmlDebug(configp, fm);

	if (xml_memory_debug)
		fprintf(stdout, "Memory level %u after parseFmConfig()\n", (unsigned int)memory);

	parsingInProcess = 0;
	return configp;
failure:
	releaseXmlConfig(configp, /* full */ 1);
	configp = NULL;
	parsingInProcess = 0;
	return NULL;
}

#ifdef __VXWORKS__
// call from CLI to verify XML config file
// return codes
// 0 Ok
// 1 Parse error - do a logShow to see error
// 2 SM is busy doing a sweep
// 3 File does not exist
// 4 Cannot get memory to parse
// 5 Bad pointer to filename
uint8_t
verifyFmConfig(char* filename, uint32_t flags)
{
	FMXmlCompositeConfig_t* verify;
	FILE* cFile;

	if (filename == NULL)
		return 5;

	if (parsingInProcess) {
		if (xml_parse_debug)
			fprintf(stdout, "Cannot verify configuration file %s when the parser is busy parsing another file\n", filename);
		return 2;
	}

	// verify existance of filename if backup
	cFile = fopen(filename, "r");
	if (cFile == NULL) {
		fprintf(stdout, "XML configuration file %s not found in verifyFmConfig()\n", filename);
		return 3;
	}
	fclose(cFile);

	// if we are in the Discovery state then do not allow verification and rejest
	if (get_memory && sm_state == SM_STATE_DISCOVERING) {
		if (xml_parse_debug)
			fprintf(stdout, "Cannot verify configuration file %s when Subnet Manager is changing state\n", filename);
		return 2;
	}

	// for now the only file to verify is the one in /rfa1
	verify = parseFmConfig(filename, (IXmlParserFlags_t)flags, 0, /* full parse */ 0, /* embedded */ 1);

	// verification failed if a valid FMXmlCompositeConfig_t pointer was not
	if (verify == NULL)
		return 1;

	// release memory and destroy memory pool
	releaseXmlConfig(verify, /* full */ 1);

	// verification complete with good status
	return 0;
}

int copyPEMFile(char *src, char *pem_filename) {
	char fn[64];

	/* copy new file replacing old one */
	snprintf(fn,  sizeof(fn),"%s/%s.pem", DIR_BASE_NAME, pem_filename);
	if (vcopy(src, fn, FALSE) != OK) {
		sysPrintf("%s %d vcopy %s to %s failed\n", __FUNCTION__, __LINE__, src, fn);
		updateLastScpRetCode(SCP_FF_ERR_XML_UPDATE);
		rm(src);
		return 0;
	}

    // mirror the XML file to remote CPU - if this is not a multi-CPU system then
    // the call will simply return without doing any mirroring
#if defined(__VXWORKS__)
    cmuRed_cfgIfMasterSyncSlavePEM(pem_filename);
#endif

	rm(src);
	updateLastScpRetCode(SCP_FF_ERR_OK);
	SCP_LOG( "Copy file %s to %s successful.", src, fn);
	return 1;
}

/***********************************

This function will:
	1 - copy (src) [/firmware/opafm.xml] file to compressed file
			[/firmware/temp.xml.z]
	2 - check max compressed file size limit
		2a - If file size too big, reove both files
	3 - verify XML config file. If invalid remove both files
	4 - if ok, copy file to flash [/<flashdir>/opafm.xml.z].
	5 - remove src file

*************************/
int copyCompressXMLConfigFile(char *src, char *dst_unused_param) {
	long fileSize=0;
	uint8_t ret;

	if (parsingInProcess) {
		XmlParsePrintError("scp: unable to compress XML config file while parser is in the process of parsing a file already");
		updateLastScpRetCode(SCP_FF_ERR_XML_UPDATE);
		sysPrintf ("scp:Unable to compress into %s while parser is in the process of parsing a file already\n",IFS_FM_CFG_NAME_TEMP);
		return 0;
	}

	scpInProcess = 1;

    if (copyFile(src, IFS_FM_CFG_NAME_TEMP,-1,&fileSize)) {
		XmlParsePrintError("scp: unable to compress XML config file");
		updateLastScpRetCode(SCP_FF_ERR_XML_UPDATE);
        sysPrintf ("scp:Unable to compress into %s\n", IFS_FM_CFG_NAME_TEMP);
		rm(src);
		scpInProcess = 0;
        return 0;
    }
    if (fileSize>MAX_XML_COMPRESSED_FILE_SIZE) {
		XmlParsePrintError("scp: compressed XML config file exceeds max size");
		updateLastScpRetCode(SCP_FF_ERR_XML_UPDATE);
        sysPrintf ("scp:Compressed file size exceeds limit of %ld\n",MAX_XML_COMPRESSED_FILE_SIZE);
        rm(src);
		rm(IFS_FM_CFG_NAME_TEMP);
		scpInProcess = 0;
        return 0;
    }

    ret=verifyFmConfig( IFS_FM_CFG_NAME_TEMP, 0);
    if (!ret) {
        SCP_LOG( "XML parsed ok!");
    } else {
		XmlParsePrintError("scp: Invalid XML config file!");
		updateLastScpRetCode(SCP_FF_ERR_XML_UPDATE);
        sysPrintf("scp: Invalid XML config file!\n");
        rm(src);
		rm(IFS_FM_CFG_NAME_TEMP);
		scpInProcess = 0;
		return 0;
    }

	SCP_LOG( "Removing old config file %s.",IFS_FM_CFG_NAME);
	rm(IFS_FM_CFG_NAME);

	SCP_LOG( "copying over %s to %s.",IFS_FM_CFG_NAME_TEMP, IFS_FM_CFG_NAME);
    if (copyFile( IFS_FM_CFG_NAME_TEMP, IFS_FM_CFG_NAME, 0,NULL)) {
		XmlParsePrintError("scp: unable to copy compressed XML config file to destination");
		updateLastScpRetCode(SCP_FF_ERR_XML_UPDATE);
        sysPrintf("scp:Unable to copy %s to %s\n", IFS_FM_CFG_NAME_TEMP, IFS_FM_CFG_NAME);
		rm(src);
		rm(IFS_FM_CFG_NAME_TEMP);
		scpInProcess = 0;
        return 0;
    }

	ret=verifyFmConfig( IFS_FM_CFG_NAME, 0);
	if (!ret) {
		SCP_LOG( "XML parsed ok on second pass!");
	} else {
		XmlParsePrintError("scp: Invalid XML new config file on second pass!");
		updateLastScpRetCode(SCP_FF_ERR_XML_UPDATE);
        sysPrintf("scp: Invalid XML new config file in second pass!\n");
		rm(src);
		rm(IFS_FM_CFG_NAME_TEMP);
        rm(IFS_FM_CFG_NAME);
		scpInProcess = 0;
		return 0;
    }

	rm(src);
	rm(IFS_FM_CFG_NAME_TEMP);

    // mirror the XML file to remote CPU - if this is not a multi-CPU system then
    // the call will simply return without doing any mirroring
#if defined(__VXWORKS__)
    cmuRed_cfgIfMasterSyncSlave(NULL);
#endif

	updateLastScpRetCode(SCP_FF_ERR_OK);

	scpInProcess = 0;

	return 1; // no errors encountered
}

// are we in the process of SCP'ing an XML config ?
uint32_t scpXmlInProcess(void) {
	return scpInProcess;
}

#endif

#ifdef XML_TEST

int main()
{
	FMXmlCompositeConfig_t config;

	fprintf(stdout, "Address of config = %x\n", (uint)&g_config);

	parseFmConfig(&config, IFS_FM_CFG_NAME, IXML_PARSER_FLAG_NONE, 0, /* full */ 1, /* embedded */ 0 );
	printXmlDebug(&config, 0);

	exit(0);
}
#endif

static boolean verifyrange (uint64_t mgidRangeInitH, uint64_t mgidRangeInitL, uint64_t mgidRangeLastH, uint64_t mgidRangeLastL,
		uint64_t mcgidH, uint64_t mcgidL)
{
	//check upper limit
	if (mgidRangeLastH < mcgidH)
		return FALSE;
	else if (mgidRangeLastH == mcgidH) {
			if (mcgidL > mgidRangeLastL)
				return FALSE;
		}
	//check lower limit
	if (mgidRangeInitH < mcgidH)
		return TRUE;
	else {
		if (mgidRangeInitH == mcgidH) {
			if (mcgidL < mgidRangeInitL)
				return FALSE;
			else return TRUE;
		}
		else return FALSE;
	}
}

static boolean checkMC_VFProperties (SMMcastDefGrp_t *mcp, VF_t *v_fp)
{	// check the VirtualFabric name
	if (strlen(mcp->virtual_fabric) > 0 && strncmp(mcp->virtual_fabric, v_fp->name, sizeof(mcp->virtual_fabric)) != 0) {
		return FALSE;
	}

	//check pkey
	if (mcp->def_mc_pkey != UNDEFINED_XML32 && v_fp->pkey != UNDEFINED_XML32 &&
				(mcp->def_mc_pkey & 0x7fff) != (v_fp->pkey & 0x7fff)) {
		return FALSE;
	}

	//check MTU
	if (GetBytesFromMtu(mcp->def_mc_mtu_int) > GetBytesFromMtu(v_fp->max_mtu_int))
		return FALSE;

	// check rate
	if (IbStaticRateToMbps(mcp->def_mc_rate_int) > IbStaticRateToMbps(v_fp->max_rate_int)){
		return FALSE;
	}
	// Check the SL's are the same
	if (mcp->def_mc_sl != UNDEFINED_XML8) {
		if (v_fp->qos_enable != 0) {
			if (v_fp->mcast_sl != UNDEFINED_XML8) { 
				if (mcp->def_mc_sl != v_fp->mcast_sl)
					return FALSE;
			}
			else if (mcp->def_mc_sl != v_fp->base_sl)
				return FALSE;
		}
		else 
			return FALSE;
	}
	return TRUE;
}

static boolean searchMgidsThruAppsinVF (VF_t *vf_p, uint64_t mcgidh, uint64_t mcgidl)
{
	int mgidsV = 0;
	VFAppMgid_t *mgidp;
	cl_map_item_t *cl_map_item;

	// searching through mgids in applications of this VF
	for_all_qmap_ptr(&vf_p->apps.mgidMap, cl_map_item, mgidp) {
		//if there is a mask
		if ((mgidp->mgid_mask[0] != 0xFFFFFFFFFFFFFFFFULL) || (mgidp->mgid_mask[1] != 0xFFFFFFFFFFFFFFFFULL)) {
			if (((mgidp->mgid[0] == (mcgidh & mgidp->mgid_mask[0]))) &&
					((mgidp->mgid[1] == (mcgidl & mgidp->mgid_mask[1]))))
					mgidsV++;
		}//if there is a range
		else if ((mgidp->mgid_last[0] != 0) || (mgidp->mgid_last[1] != 0)) {
				if (verifyrange(mgidp->mgid[0], mgidp->mgid[1], mgidp->mgid_last[0], mgidp->mgid_last [1],mcgidh, mcgidl))
					mgidsV++;
		}//if there is a single value
		else if ((mgidp->mgid[0] == mcgidh) && (mgidp->mgid[1] == mcgidl))
					mgidsV++;
	}

	if (mgidsV > 0)
		return TRUE;
	else
		return FALSE;
}

boolean PreCreateMCGroup (SMMcastDefGrp_t *mdgp, VF_t *matched_vf)
{	// create default group
	VFDg_t *dgip, *dgi;
	// go ahead and build the default group info
	VFAppMgid_t *mgidp;
	dgip = getDMCG(); /// type VFDg_t
	if (!dgip) {
		return FALSE;
	}

	dgip->def_mc_create = mdgp->def_mc_create;

// UNDEFINED_32XML were filled with default values before
	uint32_t pkey = 0x8000 | matched_vf->pkey;
	dgip->def_mc_pkey = pkey;
	dgip->def_mc_mtu_int = mdgp->def_mc_mtu_int;
	dgip->def_mc_rate_int = mdgp->def_mc_rate_int;
	if (matched_vf->mcast_sl != UNDEFINED_XML8)
		dgip->def_mc_sl = matched_vf->mcast_sl;
	else dgip->def_mc_sl = matched_vf->base_sl;
	dgip->def_mc_sl = mdgp->def_mc_sl;

	dgip->def_mc_qkey = mdgp->def_mc_qkey;
	dgip->def_mc_fl = mdgp->def_mc_fl;
	dgip->def_mc_tc = mdgp->def_mc_tc;

	dgip->prejoin_allowed = mdgp->prejoin_allowed;


	// add all of the individual MGID's to the list
	uint32_t entry;
	for (entry = 0; (entry < mdgp->number_of_mgids && mdgp->number_of_mgids <= MAX_VFABRIC_DG_MGIDS); entry++) {
		mgidp = getAppMgid();
		if (!mgidp) {
			freeDMCG(dgip);
			return FALSE; //
		}

		// since this is an individual MGID ID then set it up appropriately
		verifyAndConvertMGidString(mdgp->mgid[entry].mgid, mgidp);

		// if first one on list then place at head
		if (addMap(&dgip->mgidMap, XML_QMAP_U64_CAST mgidp)) {
			dgip->mgidMapSize++;
		} else {
			freeAppMgid(mgidp);
		}
	}

	/* If not first Default Group Append to List */
	if (matched_vf->default_group) {
		/* Get Tail Default Group */
		for (dgi = matched_vf->default_group; dgi->next_default_group; dgi = dgi->next_default_group);
		/* Append to Tail of Default Group List */
		dgi->next_default_group = dgip;
	} else {
		matched_vf->default_group = dgip;
	}

	matched_vf->number_of_default_groups++;
	return TRUE;
}

// MatchExplicitMGIDtoVF()
//
// Called by renderVirtualFabricsConfig() prior to its cloneVF() loop:
//   (caller specifies update_active_vfabrics=FALSE)
//   (if matching VF found, this function updates the corresponding
//    v_fabric_all[] default group list)
// Called by renderVirtualEthernetsConfig after renderVirtualFabricsConfig():
//   (caller specifies update_active_vfabrics=TRUE)
//   (if matching VF found, updates the corresponding v_fabric_all[] AND
//    v_fabric_all[] default group lists).
FSTATUS MatchExplicitMGIDtoVF (SMMcastDefGrp_t *mdgp, VirtualFabrics_t *v_fabricp, boolean update_active_vfabrics, int enforceVFPathRecs)
// now we need to associate a Multicast Group with a VirtualFabric
{	uint32_t number_of_vfs = 0;
	uint32_t mg,i,j;
	int vf = -1;
	int matched_vf[MAX_ENABLED_VFABRICS] = {0};
	uint64 mcgid[2];
	boolean matchedAllMgids = TRUE;

// all fabrics that get here are enabled (include Active and Standby)
	for (i=0; i < v_fabricp->number_of_vfs_all && i < MAX_ENABLED_VFABRICS; i++) {
		if (!checkMC_VFProperties(mdgp, &v_fabricp->v_fabric_all[i]))
			continue;
		matchedAllMgids = TRUE;
		for (mg = 0; (mg < mdgp->number_of_mgids && mdgp->number_of_mgids <= MAX_VFABRIC_DG_MGIDS); mg++) {
			StringToGid(&mcgid[0], &mcgid[1], mdgp->mgid[mg].mgid, NULL, TRUE);
			//verify MGID matching by Application otherwise check unmatched
			// search for the applications that are in the VF
			// there is none >> fail config
			if (searchMgidsThruAppsinVF(&v_fabricp->v_fabric_all[i], mcgid[0], mcgid[1]))
				continue;
			else if (v_fabricp->v_fabric_all[i].apps.select_unmatched_mgid){
				//check if no other VF contains the application gid
				boolean matchedOtherVF=FALSE;
				for (j=0; j < v_fabricp->number_of_vfs_all && j < MAX_ENABLED_VFABRICS; j++) {
					if (i==j) continue;
					if (searchMgidsThruAppsinVF(&v_fabricp->v_fabric_all[j], mcgid[0], mcgid[1])) {
						matchedOtherVF=TRUE;
						break;
					}
				}
				if (matchedOtherVF) {
					matchedAllMgids=FALSE;
					break;
				}
			}
			else {
				matchedAllMgids=FALSE;
				break;
			}
		}
		if (matchedAllMgids) {
			matched_vf[i] = 1;
			number_of_vfs++;
			if (vf == -1) vf = i; // Shortcut for later, save first matching VF.
		}
	}// end of searching for a single VF to match a MC group for explicit groups

	if (enforceVFPathRecs && number_of_vfs > 1) { // if more than one fail config
		return FINVALID_STATE;
	}

	if (number_of_vfs == 0) {// if not matched then fail
		return FNOT_FOUND;
	}

	/* This snippet below will copy the McGroups to all VFs that match.
	   However, note that the if check above that checks if the number of matching VFs
	   is more than 1. This check above makes sure that current behavior of one VF
	   per McGroup is maintained unless enforceVFPathRecs is false, by which multiple
	   VFs can then match an McGroup. */
	for (; vf < v_fabricp->number_of_vfs_all && vf < MAX_ENABLED_VFABRICS && number_of_vfs > 0; ++vf) {
		if (matched_vf[vf] != 0) {
			// If this is the first time creating this group
			if (!PreCreateMCGroup(mdgp, &v_fabricp->v_fabric_all[vf])) {
				return FUNAVAILABLE;
			}
			--number_of_vfs;
		}
	}
	return FSUCCESS;
}// end of MatchExplicitGIDtoVF


FSTATUS MatchImplicitMGIDtoVF(SMMcastDefGrp_t *mdgp, VirtualFabrics_t *v_fabricp)
{
#define MAX_NUM_IMPLICIT_MCG 8

	typedef struct implicitMgid_t {
	uint64_t Prefix;
	uint64_t Postfix;
	} implicitMgid;

	int i,j,k;
	int number_of_vfs = 0;
	uint64_t Prefix[MAX_NUM_IMPLICIT_MCG];
	VF_t *matched_vf=NULL;
	boolean matchedAllMgids = TRUE;
	implicitMgid mgids[8];
	uint64_t broadcast = 0x00000000ffffffff;
	uint64_t allNodes = 0x0000000000000001;
	uint64_t allRouters = 0x0000000000000002;
	uint64_t other = 0x0000000000000016;
	uint64_t mcastDns = 0x00000000000000fb;
	uint64_t IPv4 = 0xff12401b00000000ull;
	uint64_t IPv6 = 0xff12601b00000000ull;

	memset(mgids, 0, sizeof(mgids));

	for (i =0; i<4; i++) Prefix[i] = IPv4;
	for (i =4; i<8; i++) Prefix[i] = IPv6;

	mgids[0].Postfix = broadcast;
	mgids[1].Postfix = allNodes;
	mgids[2].Postfix = other;
	mgids[3].Postfix = mcastDns;
	mgids[4].Postfix = allNodes;
	mgids[5].Postfix = allRouters;
	mgids[6].Postfix = mcastDns;
	mgids[7].Postfix = other;

	for (i=0; i < v_fabricp->number_of_vfs_all && i < MAX_ENABLED_VFABRICS; i++) {
		if (!checkMC_VFProperties (mdgp, &v_fabricp->v_fabric_all[i]))
			continue;
		matchedAllMgids = TRUE;
		for (j=0;j<MAX_NUM_IMPLICIT_MCG;j++){
			uint32_t pkey = 0x8000 | v_fabricp->v_fabric_all[i].pkey;
			mgids[j].Prefix = Prefix[j] | (pkey << 16 );
			sprintf(mdgp->mgid[j].mgid,"0x%016"PRIx64":0x%016"PRIx64,mgids[j].Prefix,mgids[j].Postfix);
			if (searchMgidsThruAppsinVF(&v_fabricp->v_fabric_all[i], mgids[j].Prefix,mgids[j].Postfix)) {
				continue;
			}
			else if (v_fabricp->v_fabric_all[i].apps.select_unmatched_mgid){
				//check if no other VF contains the application gid
				boolean matchedOtherVF=FALSE;
				for (k=0; k < v_fabricp->number_of_vfs_all && k < MAX_ENABLED_VFABRICS; k++) {
					if (k==i) continue;
					if (searchMgidsThruAppsinVF(&v_fabricp->v_fabric_all[k], mgids[j].Prefix,mgids[j].Postfix)) {
						matchedOtherVF=TRUE;
						break;
					}
				}
				if (matchedOtherVF) {
					matchedAllMgids=FALSE;
					break;
				}
			}
			else {
				matchedAllMgids=FALSE;
				break;
			}
		}
		if (matchedAllMgids) {
			matched_vf = &v_fabricp->v_fabric_all[i];
			mdgp->number_of_mgids=MAX_NUM_IMPLICIT_MCG;
			number_of_vfs++;
			if (!PreCreateMCGroup(mdgp, matched_vf))
				return FUNAVAILABLE;
		}
	}// create a group for all matched VF's

	if (number_of_vfs == 0) // not matched
		return FNOT_FOUND;

	mdgp->number_of_mgids=0;
	return FSUCCESS;
}


//check if MLIDShare overlap
static FSTATUS CheckMLIDShareOverlapping(uint32_t fm, FMXmlCompositeConfig_t *config)
{
	SmMcastMlidShared_t *dmsp_ref, *dmsp_check;
	uint32_t i,mlidSharedInstance;
	uint32_t numshared;
	uint64_t val0H, val0L, val1H, val1L;
	IB_GID mask_ref, mask_check;
	IB_GID value_ref, value_check;
	FSTATUS status;

	if (!common && config->fm_instance[fm])
		numshared = config->fm_instance[fm]->sm_mls_config.number_of_shared;
	else
		numshared = config->fm_instance_common->sm_mls_config.number_of_shared;



	for (mlidSharedInstance=0; mlidSharedInstance < (numshared-1) && mlidSharedInstance < MAX_SUPPORTED_MCAST_GRP_CLASSES_XML; mlidSharedInstance++) {
	// inherit the values of config items already set
		if (!common && config->fm_instance[fm])
			dmsp_ref = &config->fm_instance[fm]->sm_mls_config.mcastMlid[mlidSharedInstance];
		else
			dmsp_ref = &config->fm_instance_common->sm_mls_config.mcastMlid[mlidSharedInstance];

		if (dmsp_ref->enable) {
			//for the dmsp_ref
			/* grab the mask */
			if ((status = cs_parse_gid(dmsp_ref->mcastGrpMGidLimitMaskConvert.value, mask_ref.Raw)) != VSTATUS_OK) {
				IB_LOG_ERROR_FMT(__func__, "Bad value for MGIDMask in MLIDShare %d: %s", mlidSharedInstance,
						dmsp_ref->mcastGrpMGidLimitMaskConvert.value);
				continue;
			}
			BSWAP_IB_GID(&mask_ref);

			if ((mask_ref.AsReg64s.H == 0) && (mask_ref.AsReg64s.L == 0))
				continue;

			/* grab the value */
			if ((status = cs_parse_gid(dmsp_ref->mcastGrpMGidLimitValueConvert.value, value_ref.Raw)) != VSTATUS_OK) {
				IB_LOG_ERROR_FMT(__func__, "Bad value for MGIDValue in MLIDShare %d: %s", mlidSharedInstance,
						dmsp_ref->mcastGrpMGidLimitValueConvert.value);
				continue;
			}
			BSWAP_IB_GID(&value_ref);

			for (i = mlidSharedInstance+1; i < numshared && i < MAX_SUPPORTED_MCAST_GRP_CLASSES_XML ; i++) {
				if (!common && config->fm_instance[fm])
					dmsp_check = &config->fm_instance[fm]->sm_mls_config.mcastMlid[i];
				else
					dmsp_check = &config->fm_instance_common->sm_mls_config.mcastMlid[i];

				if (dmsp_check->enable) {
					//for the dmsp_check
					/* grab the mask */
					if ((status = cs_parse_gid(dmsp_check->mcastGrpMGidLimitMaskConvert.value, mask_check.Raw)) != VSTATUS_OK) {
						IB_LOG_ERROR_FMT(__func__, "Bad value for MGIDMask in MLIDShare %d: %s", mlidSharedInstance,
								dmsp_check->mcastGrpMGidLimitMaskConvert.value);
						continue;
					}
					BSWAP_IB_GID(&mask_check);

					/* grab the value */
					if ((status = cs_parse_gid(dmsp_check->mcastGrpMGidLimitValueConvert.value, value_check.Raw)) != VSTATUS_OK) {
						IB_LOG_ERROR_FMT(__func__, "Bad value for MGIDValue in MLIDShare %d: %s", mlidSharedInstance,
								dmsp_check->mcastGrpMGidLimitValueConvert.value);
						continue;
					}
					BSWAP_IB_GID(&value_check);

					if ((mask_check.AsReg64s.H == 0) && (mask_check.AsReg64s.L == 0))
						continue;

					val0H = value_ref.AsReg64s.H & mask_check.AsReg64s.H;
					val0L = value_ref.AsReg64s.L & mask_check.AsReg64s.L;
					val1H = value_check.AsReg64s.H & mask_ref.AsReg64s.H;
					val1L = value_check.AsReg64s.L & mask_ref.AsReg64s.L;

					if (((val0H ^ val1H) == 0) && ((val0L ^ val1L) == 0)) {
						return FERROR;
					}
				} //end if enable check
			}// for end check
		} //end if enable ref
	} //for end ref
	status = FSUCCESS;
	return status;
}

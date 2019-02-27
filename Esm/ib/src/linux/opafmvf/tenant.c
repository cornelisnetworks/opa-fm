/*
 * Copyright (c) 2018, Intel Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright notice,
 *       this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Intel Corporation nor the names of its contributors
 *       may be used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "tenant.h"

#include "utils.h"
#include <ixml.h>
#include <libgen.h>
#include <sys/stat.h>


const char* default_dg_dir = "/etc/opa-fm/dgs";
const char* default_vf_dir = "/etc/opa-fm/vfs";

const char* dg_config_prefix = "opafm_dg_";
const char* vf_config_prefix = "opafm_vf_";

const char* default_vf_application = "Tenant_Apps";
const char* default_vf_qos_group = "Tenant_QOS";


/******************************************************************************
 * XML functions
 *****************************************************************************/

static void
xml_print_error(const char* message)
{
	log_error("%s", message);
}

static void
xml_print_warning(const char* message)
{
	log_error("%s", message);
}

static void*
vf_xml_parser_start(IXmlParserState_t* state, void* parent, const char** attr)
{
	return parent;
}

static void
vf_xml_parser_end(IXmlParserState_t* state, const IXML_FIELD* field, void* object, void* parent,
	XML_Char* content, unsigned len, boolean valid)
{
	// TODO: improve config correctness
	// TenantVFConfig* vf_config = (TenantVFConfig*) object;
}

static void*
dg_xml_parser_start(IXmlParserState_t* state, void* parent, const char** attr)
{
	return parent;
}

static void
dg_xml_parser_end(IXmlParserState_t* state, const IXML_FIELD* field, void* object, void* parent,
	XML_Char* content, unsigned len, boolean valid)
{
	// TODO: improve config correctness
	// TenantDGConfig* dg_config = (TenantDGConfig*) object;
}

static void
dg_port_guid_xml_parser_end(IXmlParserState_t* state, const IXML_FIELD* field, void* object, void* parent,
	XML_Char* content, unsigned len, boolean valid)
{
	TenantDGConfig* dg_config = (TenantDGConfig*) object;

	if (!valid) {
		IXmlParserPrintError(state, "Error processing XML PortGUID tag");
		return;
	}

	if (!content || !dg_config || strlen(content) > MAX_NAME_LENGTH - 1) {
		IXmlParserPrintError(state, "Bad %s content \"%s\"", field->tag, content);
		return;
	}

	uint64_t guid;
	if (FSUCCESS != StringToUint64(&guid, content, NULL, 16, TRUE)) {
		IXmlParserPrintError(state, "PortGUID %s is formatted incorrectly", content);
		return;
	}

	cl_map_item_t* item = (cl_map_item_t*)calloc(1, sizeof(cl_map_item_t));
	if (!item)
		die("Out of memory");

	if (cl_qmap_insert(&dg_config->port_guid_map, guid, item) != item) {
		IXmlParserPrintWarning(state, "PortGUID %s is duplicated", content);
		free(item);
	}
}

static void
dg_port_guid_xml_parser_format(IXmlOutputState_t* state, const char* tag, void* data)
{
	cl_qmap_t* port_guid_map = &((TenantDGConfig*) data)->port_guid_map;

	cl_map_item_t* port_guid = cl_qmap_head(port_guid_map);
	while (port_guid != cl_qmap_end(port_guid_map)) {
		IXmlOutputHexPad64(state, "PortGUID", port_guid->key);
		port_guid = cl_qmap_next(port_guid);
	}
}

static bool
xml_tag_save_to_file(const char* tag, void* data, const IXML_FIELD* fields, const char* path, bool force)
{
	IXmlOutputState_t state;
	char* stream_buffer;
	size_t stream_size;

	/* create stream buffer with the XML content */
	FILE* stream = open_memstream(&stream_buffer, &stream_size);
	if (!stream) {
		log_error("Cannot create memory stream (%s)", strerror(errno));
		return false;
	}

	IXmlInit(&state, stream, 4, IXML_OUTPUT_FLAG_NONE, NULL);
	IXmlOutputStruct(&state, tag, data, NULL, fields);
	IXmlOutputDestroy(&state);

	fclose(stream);

	/* save stream buffer to the file */
	bool result = false;

	/* divide path into dir name and file name */
	char* path_tmp1 = strdup(path);
	char* path_tmp2 = strdup(path);
	if (!path_tmp1 || !path_tmp2)
		die("Out of memory");

	char* dir_name = dirname(path_tmp1);
	char* file_name = basename(path_tmp2);

	/* before saving make sure a target directory exists */
	if ((access(dir_name, F_OK) == -1) && (mkdir(dir_name, S_IRWXU) == -1)) {
		log_error("Cannot create parent directory %s (%s)", dir_name, strerror(errno));
		goto error;
	}

	/* now save stream_buffer to the underlying hardware in atomic way */
	if (!file_save_atomic(dir_name, file_name, stream_buffer, stream_size, force)) {
		log_error("Saving %s failed (%s)", path, strerror(errno));
		goto error;
	}

	result = true;

error:
	free(path_tmp2);
	free(path_tmp1);
	free(stream_buffer);

	return result;
}

/**
 * Fields within VirtualFabric tag.
 */
static IXML_FIELD vf_fields[] = {
	{ tag:"Name", format:'s', IXML_FIELD_INFO(TenantVFConfig, name) },
	{ tag:"Enable", format:'u', IXML_FIELD_INFO(TenantVFConfig, enable) },
	{ tag:"PKey", format:'h', IXML_FIELD_INFO(TenantVFConfig, pkey) },
	{ tag:"Security", format:'u', IXML_FIELD_INFO(TenantVFConfig, security) },
	{ tag:"Member", format:'s', IXML_FIELD_INFO(TenantVFConfig, member) },
	{ tag:"Application", format:'s', IXML_FIELD_INFO(TenantVFConfig, application) },
	{ tag:"QOSGroup", format:'s', IXML_FIELD_INFO(TenantVFConfig, qos_group) },
	{ NULL }
};

/**
 * Top level fields within VirtualFabric config file.
 */
static IXML_FIELD top_level_vf_fields[] = {
	{ tag:"VirtualFabric", format:'k', subfields:vf_fields, start_func:vf_xml_parser_start,
		end_func:vf_xml_parser_end },
	{ NULL }
};

/**
 * Fields within DeviceGroup tag.
 */
static IXML_FIELD dg_fields[] = {
	{ tag:"Name", format:'s', IXML_FIELD_INFO(TenantDGConfig, name) },
	{ tag:"PortGUID", format:'k', format_func:dg_port_guid_xml_parser_format, end_func:dg_port_guid_xml_parser_end },
	{ NULL }
};

/**
 * Top level fields within DeviceGroup config file.
 */
static IXML_FIELD dg_top_level_fields[] = {
	{ tag:"DeviceGroup", format:'k', subfields:dg_fields, start_func:dg_xml_parser_start,
		end_func:dg_xml_parser_end },
	{ NULL }
};


/******************************************************************************
 * Tenant functions
 *****************************************************************************/

Tenant*
tenant_new(const char* name)
{
	if (!name || strlen(name) >= MAX_NAME_LENGTH)
		return NULL;

	/* create a new tenant instance */
	Tenant* tenant = (Tenant*)calloc(1, sizeof(Tenant));
	if (!tenant)
		die("Out of memory");

	StringCopy(tenant->name, name, sizeof(tenant->name));
	tenant->dg_config_path = StringConcat(default_dg_dir, "/", dg_config_prefix, name, ".xml", NULL);
	tenant->vf_config_path = StringConcat(default_vf_dir, "/", vf_config_prefix, name, ".xml", NULL);
	if (!tenant->dg_config_path || !tenant->vf_config_path)
		die("Out of memory");

	cl_qmap_init(&tenant->dg_config.port_guid_map, NULL);

	return tenant;
}

void
tenant_free(Tenant* tenant)
{
	tenant_remove_all_port_guids(tenant);
	free(tenant->vf_config_path);
	free(tenant->dg_config_path);
	free(tenant);
}

bool
tenant_set_pkey(Tenant* tenant, uint16_t pkey)
{
	if (pkey >= 0x7fff)
		return false;
	tenant->vf_config.pkey = pkey;
	return true;
}

void
tenant_set_default(Tenant* tenant)
{
	/* fill DG config with values */
	StringCopy(tenant->dg_config.name, tenant->name, sizeof(tenant->dg_config.name));

	/* fill VF config with values */
	StringCopy(tenant->vf_config.name, tenant->name, sizeof(tenant->vf_config.name));
	tenant->vf_config.enable = 1;
	tenant->vf_config.pkey = 0xffff;
	tenant->vf_config.security = 1;
	StringCopy(tenant->vf_config.member, tenant->dg_config.name, sizeof(tenant->vf_config.member));
	StringCopy(tenant->vf_config.application, default_vf_application, sizeof(tenant->vf_config.application));
	StringCopy(tenant->vf_config.qos_group, default_vf_qos_group, sizeof(tenant->vf_config.qos_group));
}

bool
tenant_exist(Tenant* tenant)
{
	if ((access(tenant->dg_config_path, F_OK) == -1) && (access(tenant->vf_config_path, F_OK) == -1)) {
		return false;
	}

	return true;
}

bool
tenant_load(Tenant* tenant)
{
	if (FSUCCESS != IXmlParseInputFile(tenant->dg_config_path, IXML_PARSER_FLAG_STRICT, dg_top_level_fields,
			&tenant->dg_config, NULL, xml_print_error, xml_print_warning, NULL, NULL)) {
		return false;
	}

	if (FSUCCESS != IXmlParseInputFile(tenant->vf_config_path, IXML_PARSER_FLAG_STRICT, top_level_vf_fields,
			&tenant->vf_config, NULL, xml_print_error, xml_print_warning, NULL, NULL)) {
		return false;
	}

	return true;
}

bool
tenant_save(Tenant* tenant, bool force)
{
	if (!xml_tag_save_to_file("DeviceGroup", &tenant->dg_config, dg_fields, tenant->dg_config_path, force)) {
		return false;
	}

	if (!xml_tag_save_to_file("VirtualFabric", &tenant->vf_config, vf_fields, tenant->vf_config_path, force)) {
		return false;
	}

	return true;
}

bool
tenant_delete(Tenant* tenant)
{
	unlink(tenant->vf_config_path);
	unlink(tenant->dg_config_path);

	if (access(tenant->vf_config_path, F_OK) == 0) {
		log_error("Cannot remove file %s (%s)", tenant->vf_config_path, strerror(errno));
		return false;
	}

	if (access(tenant->dg_config_path, F_OK) == 0) {
		log_error("Cannot remove file %s (%s)", tenant->dg_config_path, strerror(errno));
		return false;
	}

	return true;
}

bool
tenant_add_port_guid(Tenant* tenant, uint64_t guid)
{
	cl_qmap_t* map = &tenant->dg_config.port_guid_map;
	cl_map_item_t* item = (cl_map_item_t*)calloc(1, sizeof(cl_map_item_t));
	if (!item)
		die("Out of memory");

	if (cl_qmap_insert(map, guid, item) != item) {
		free(item);
		return false;
	}
	return true;
}

bool
tenant_remove_port_guid(Tenant* tenant, uint64_t guid)
{
	cl_qmap_t* map = &tenant->dg_config.port_guid_map;
	cl_map_item_t* item = cl_qmap_get(map, guid);

	if (item == cl_qmap_end(map))
		return false;

	cl_qmap_remove_item(map, item);
	free(item);

	return true;
}

void
tenant_remove_all_port_guids(Tenant* tenant)
{
	cl_qmap_t* map = &tenant->dg_config.port_guid_map;
	cl_map_item_t* item = cl_qmap_head(map);

	while (item != cl_qmap_end(map)) {
		cl_qmap_remove_item(map, item);
		free(item);
		item = cl_qmap_head(map);
	}
}

bool
tenant_is_valid(Tenant* tenant)
{
	if (strcmp(tenant->dg_config.name, tenant->name))
		return false;
	if (strcmp(tenant->vf_config.name, tenant->name))
		return false;
	if (strcmp(tenant->vf_config.member, tenant->name))
		return false;
	return true;
}

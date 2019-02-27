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

#ifndef _TENANT_H_
#define _TENANT_H_

#include <iba/public/iquickmap.h>
#include <stdbool.h>

#define MAX_NAME_LENGTH 64

/**
 * Tenant device group config.
 */
typedef struct _TenantDGConfig {
	char			name[MAX_NAME_LENGTH];			/**< Name of device group. */
	cl_qmap_t		port_guid_map;					/**< List of all PortGUIDs added to device group. */
} TenantDGConfig;

/**
 * Tenant virtual fabric config.
 */
typedef struct _TenantVFConfig {
	char			name[MAX_NAME_LENGTH];			/**< Name of virtual fabric. */
	uint32_t		enable;							/**< Flag enabling virtual fabric. */
	uint16_t		pkey;							/**< PKey associated with virtual fabric. */
	uint32_t		security;						/**< Flag enabling security for virtual fabric. */
	char			member[MAX_NAME_LENGTH];		/**< Name of device group added to virtual fabric. */
	char			application[MAX_NAME_LENGTH];	/**< Name of application added to virtual fabric. */
	char			qos_group[MAX_NAME_LENGTH];		/**< Name of QOSGroup associated with virtual fabric. */
} TenantVFConfig;

/**
 * Structure representing tenant data.
 */
typedef struct _Tenant Tenant;
struct _Tenant {
	char			name[MAX_NAME_LENGTH];			/**< Name of tenant. */
	char*			dg_config_path;					/**< Path to device group config. */
	char*			vf_config_path;					/**< Path to virtual fabric config. */
	TenantDGConfig	dg_config;						/**< Device group config. */
	TenantVFConfig	vf_config;						/**< Virtual fabric config. */
	cl_map_item_t	map_item;						/**< For storing in #TenantManager::tenants */
};

/**
 * Create a new tenant using given @a name.
 * @param name		name of tenant
 * @return	pointer to newly allocated tenant or NULL
 */
Tenant* tenant_new(const char* name);

/**
 * Release all data associated with tenant.
 * @param tenant	pointer to tenant
 */
void tenant_free(Tenant* tenant);

/**
 * Set new pkey to virtual fabric used by tenant.
 * @param tenant	pointer to tenant
 * @param pkey		new pkey
 * @return	true if new pkey was set, or false otherwise
 */
bool tenant_set_pkey(Tenant* tenant, uint16_t pkey);

/**
 * Fill tenant configs with default values.
 * @param tenant	pointer to tenant
 */
void tenant_set_default(Tenant* tenant);

/**
 * Check whether tenant files exist on file system.
 * @param tenant	pointer to tenant
 * @return
 */
bool tenant_exist(Tenant* tenant);

/**
 * Load tenant from the disk from dg_dir/vf_dir directories.
 * @param tenant	pointer to tenant
 * @return	true if tenant was loaded properly, or false otherwise
 */
bool tenant_load(Tenant* tenant);

/**
 * Save tenant onto the disk in dg_dir/vf_dir directories.
 * @param tenant	pointer to tenant
 * @param force		override existing tenant files if exist
 * @return	true if tenant was saved properly, or false otherwise
 */
bool tenant_save(Tenant* tenant, bool force);

/**
 * Delete tenant from the disk from dg_dir/vf_dir directories.
 * @param tenant	pointer to tenant
 * @return	true if tenant was deleted properly, or false otherwise
 */
bool tenant_delete(Tenant* tenant);

/**
 * Add new PortGUID to device group used by tenant. Function fails
 * if PortGUID already exist or is invalid.
 * @param tenant	pointer to tenant
 * @param guid		PortGUID number
 * @return	true if PortGUID was added, or false otherwise
 */
bool tenant_add_port_guid(Tenant* tenant, uint64_t guid);

/**
 * Remove PortGUID from device group used by tenant. Function fails
 * if PortGUID does not exist in tenant or is invalid.
 * @param tenant	pointer to tenant
 * @param guid		PortGUID number
 * @return	true if PortGUID was removed, or false otherwise
 */
bool tenant_remove_port_guid(Tenant* tenant, uint64_t guid);

/**
 * Remove all PortGUIDs from device group used by tenant.
 * @param tenant	pointer to tenant
 */
void tenant_remove_all_port_guids(Tenant* tenant);

/**
 * Check whether tenant configs are filled properly.
 * @param tenant	pointer to tenant
 * @return	true if tenant is valid, or false otherwise
 */
bool tenant_is_valid(Tenant* tenant);

#endif /* _TENANT_H_ */

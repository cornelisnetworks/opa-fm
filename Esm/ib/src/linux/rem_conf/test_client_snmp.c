/* BEGIN_ICS_COPYRIGHT5 ****************************************

Copyright (c) 2015-2020, Intel Corporation

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
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/stat.h>
#include <signal.h>
#include "snmp_com_api.h"

#define ICS_CHASSIS_SYS_DESC "iso.org.dod.internet.private.enterprises.ics.chassis.icsChassisMib.icsChassisSystemGroup.icsChassisSystemDescription"
#define SYS_OR_DESC "iso.org.dod.internet.mgmt.mib-2.system.sysORTable.sysOREntry.sysORDescr"
#define ICS_LOG_MOD_NAME "iso.org.dod.internet.private.enterprises.ics.mgmt.icsLog.icsLogConfigMIB.icsLogModuleConfigTable.icsLogModuleConfigEntry.icsLogModuleName"
#define ICS_CHASSIS_SYS_DESC2 "ICS-CHASSIS-MIB:icsChassisSystemGroup.icsChassisSystemDescription"
#define ICS_CHASSIS_SYS_DESC_OID "1.3.6.1.4.1.10222.2.1.1.9.1.2"




void
usage(char *cmd)
{
	printf("%s instance\n",cmd);
}

void
walk_table(char *obj_name, char* display_name, p_snmp_conx_hdl_t conx)
{
	snmp_com_errno_t	res;
	snmp_com_ret_code_t	ret_code;
	char				sys_desc[256];
	uint32_t			index[32];
	uint32_t			index_len;
	uint32_t			buf_len;
	int					i;
	
	 
	index[0] = 0;
	index_len = 0;

	memset(sys_desc,0,sizeof(sys_desc));
	buf_len = sizeof(sys_desc) - 1;
	
	
	while((ret_code = snmp_com_sync_getnext(conx,obj_name,
						 index,&index_len,&sys_desc[0],&buf_len)) == SNMP_RET_OK)
	{
		printf("%s: ",display_name); 
		printf("Index: "); 
		for(i=0;i<index_len;i++){
			printf("%lu");
			if(i < (index_len -1))
				printf(".");
		}
		printf(" %s\n",sys_desc); 

		memset(sys_desc,0,sizeof(sys_desc));
		buf_len = sizeof(sys_desc) - 1;
	}

	

}


int
main(int argc, char *argv[])
{
	char 				*host;
	p_snmp_sess_hdl_t	session;
	p_snmp_conx_hdl_t	conx;
	snmp_com_errno_t	res;
	snmp_com_ret_code_t	ret_code;
	char				sys_desc[256];
	uint32_t			index[32];
	uint32_t			index_len;
	uint32_t			buf_len;
	int					i;
	
	if(argc < 2){
		usage(argv[0]);
		return -1;
	}

	host = argv[1];


	if((res = snmp_com_init(&session,NULL,NULL)) != SNMP_COM_OK)
	{
		printf("Could not initialize session: %d", res);
		return -1;
	}

	printf("Initialized session\n");

	if((res = snmp_com_connect(&conx,session,host,161,2,"public",0,1000000)) != SNMP_COM_OK)
	{
		printf("Could not open connection: %d", res);
		return -1;
	}

	printf("Initialized connection\n");

	memset(sys_desc,0,sizeof(sys_desc));
	buf_len = sizeof(sys_desc) - 1;
	index[0] = 0;
	index_len = 1;

	if((ret_code = snmp_com_sync_get(conx,"iso.org.dod.internet.mgmt.mib-2.system.sysDescr",
						 index,index_len,&sys_desc[0],&buf_len)) != SNMP_RET_OK)
	{
		printf("Error retrieving sys description: %d",ret_code);
	}

	printf("SysDesc: %s\n",sys_desc); 
	
	index[0] = 0;
	index_len = 0;

	memset(sys_desc,0,sizeof(sys_desc));
	buf_len = sizeof(sys_desc) - 1;
	
	
	while((ret_code = snmp_com_sync_getnext(conx,ICS_CHASSIS_SYS_DESC2,
						 index,&index_len,&sys_desc[0],&buf_len)) == SNMP_RET_OK)
	{
		printf("Chassis SysDesc: "); 
		printf("Index: "); 
		for(i=0;i<index_len;i++){
			printf("%lu",index[i]);
			if(i < (index_len -1))
				printf(".");
		}
		printf(" %s\n",sys_desc); 

		memset(sys_desc,0,sizeof(sys_desc));
		buf_len = sizeof(sys_desc) - 1;
	}

	memset(sys_desc,0,sizeof(sys_desc));
	buf_len = sizeof(sys_desc) - 1;
	
	
	while((ret_code = snmp_com_sync_getnext(conx,ICS_CHASSIS_SYS_DESC_OID,
						 index,&index_len,&sys_desc[0],&buf_len)) == SNMP_RET_OK)
	{
		printf("Chassis SysDesc: "); 
		printf("Index: "); 
		for(i=0;i<index_len;i++){
			printf("%lu",index[i]);
			if(i < (index_len -1))
				printf(".");
		}
		printf(" %s\n",sys_desc); 

		memset(sys_desc,0,sizeof(sys_desc));
		buf_len = sizeof(sys_desc) - 1;
	}





	




	
	
	return 0;
}

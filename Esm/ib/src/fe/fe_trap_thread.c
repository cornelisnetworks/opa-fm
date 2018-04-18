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

/*
 * THIS MODULE IS OFED SPECIFIC
 * 
 * This implements a simple receive thread that blocks on OFEDs ib_sa kernel
 * module waiting for traps.
 *
 * SEMANTICS:
 * It is assumed that only one such thread will be run within the process.
 * The thread must be paused/resumed when accessing the trap list.  This
 * results in the caller holding the thread lock, preventing the list from
 * being modified.  The caller should then free the trap list before resuming.
 * The resulting call chain is:
 *  - fe_trap_thread_pause
 *  - fe_trap_thread_get_traplist
 *  - FE_TrapThread_FreeList
 *  - fe_trap_thread_resume
 *
 * TRAP 128:
 * Note that to preserve pre-OFED behavior with respect to the Fabric Viewer,
 * a topology change trap is automatically appended to the list of traps when
 * the client calls fe_trap_thread_get_traplist.  This trap structure is
 * statically allocated with the thread data and simply reused.
 */
#ifdef IB_STACK_OPENIB

#include "fe_trap_thread.h"
#include "sa/if3_sa.h"
#include "opamgt_sa_notice.h"

extern uint32_t g_fe_nodes_len;                 // len of node data in fe_in_buff
extern FE_ConnList *clist;                      // pointer to connection list
extern struct omgt_port *fe_omgt_session;
extern Pool_t fe_pool;

static FE_TrapThreadData_t fe_trap_thread_data;

static void fe_trap_thread_worker(uint32_t argc, uint8_t **argv);

uint32_t fe_trap_thread_create(void)
{
	Status_t status;
	
    IB_ENTER(__func__, 0, 0, 0, 0);
	
	memset((void *)&fe_trap_thread_data, 0, sizeof(fe_trap_thread_data));
	
	status = vs_lock_init(&fe_trap_thread_data.lock, VLOCK_FREE, VLOCK_THREAD);
	if (status != VSTATUS_OK) {
		IB_LOG_WARNRC("failed to init lock, rc:", status);
		return FAILED;
	}

	status = vs_thread_create(
		&fe_trap_thread_data.thread_id,
		(uint8_t *)"FE_Trap",
		fe_trap_thread_worker,
		0, NULL,
		FE_TRAP_THREAD_STACK_SIZE);
	if (status != VSTATUS_OK) {
		IB_LOG_WARNRC("failed to create thread, rc:", status);
		return FAILED;
	}
	
	// setup pre-allocated topology change trap, since it's included in every
	// set of traps that hit the wire
	fe_trap_thread_data.trap128.trapType = UNSOLICITED_TOPO_CHANGE;
	fe_trap_thread_data.trap128.lidAddr = 0;
	fe_trap_thread_data.trap128.next = NULL;
    // this is a trap initiated by the FE, so just set the trap number field of
    // the raw notice data to assist the FEC.
    memset(&fe_trap_thread_data.trap128.notice, 0, sizeof(fe_trap_thread_data.trap128.notice));
    fe_trap_thread_data.trap128.notice.Attributes.Generic.TrapNumber = STL_TRAP_LINK_PORT_CHANGE_STATE;
		
	IB_EXIT(__func__, SUCCESS);
	return SUCCESS;
}

uint32_t fe_trap_thread_get_trapcount(void)
{
	return fe_trap_thread_data.count + fe_trap_thread_data.trap128_received;
}

void fe_trap_thread_get_traplist(FE_Trap_t **list, uint32_t *count)
{
	IB_ENTER(__func__, list, count, 0, 0);
	
	if (fe_trap_thread_data.count)
	{
		/* PR 111226 - Always include trap128 (topology change) at the end to preserve
		 * pre-OFED behavior for Fabric Viewer
		 */ 
		fe_trap_thread_data.trap_list_end->next = &fe_trap_thread_data.trap128;
		*list = fe_trap_thread_data.trap_list;
		*count = fe_trap_thread_data.count + 1;
	}
	else if (fe_trap_thread_data.trap128_received)
	{
		*list = &fe_trap_thread_data.trap128;
		*count = 1;
	}
	else
	{
		*list = NULL;
		*count = 0;
	}
	
	IB_EXIT(__func__, 0);
}

void fe_trap_thread_free_traps(void)
{
	FE_Trap_t *curr;
	
	IB_ENTER(__func__, 0, 0, 0, 0);
	
	if (fe_trap_thread_data.count)
	{
		/* If the list is non-empty, trap 128 is always at the end of the list - either automatically appended
		 * in fe_trap_thread_get_traplist or actually arrived.  In any case its a statically allocated element
		 * which is added to the list, so no need to free it.
		 */
		fe_trap_thread_data.trap_list_end->next = NULL;
		
		// clean the list
		while ((curr = fe_trap_thread_data.trap_list) != NULL) {
			fe_trap_thread_data.trap_list = curr->next;
			vs_pool_free(&fe_pool, curr);
		}
	}
	
	fe_trap_thread_data.trap_list_end = NULL;
	fe_trap_thread_data.count = 0;
	
	IB_EXIT(__func__, 0);
}

uint32_t fe_trap_thread_pause(void)
{
	Status_t status;
	
	IB_ENTER(__func__, 0, 0, 0, 0);
	
	status = vs_lock(&fe_trap_thread_data.lock);
	if (status != VSTATUS_OK) {
		IB_LOG_WARNRC("failed to take thread lock, rc:", status);
		return FAILED;
	}
	
	IB_EXIT(__func__, SUCCESS);
	return SUCCESS;
}

void fe_trap_thread_resume(void)
{
	IB_ENTER(__func__, 0, 0, 0, 0);
	
	vs_unlock(&fe_trap_thread_data.lock);
	
	IB_EXIT(__func__, 0);
}

static uint32_t fe_if3_trap_thread_get_notice(STL_NOTICE *notice)
{
	FSTATUS status;
	size_t len;
	STL_NOTICE *stl_notice = NULL;

	IB_ENTER(__func__, notice, 0, 0, 0);

	status = omgt_sa_get_notice_report(fe_omgt_session, &stl_notice, &len, NULL, -1);

	if (status != FSUCCESS) {
		IB_LOG_WARN("failure while waiting for incoming fabric event; status:", status);
		IB_EXIT(__func__, FAILED);
		return FAILED;
	} else if (len < sizeof(STL_NOTICE)) {
		IB_LOG_WARN("invalid data length returned; length:", len);
		free(stl_notice);
		IB_EXIT(__func__, FAILED);
		return FAILED;
	}
	memcpy(fe_trap_thread_data.data, stl_notice, len);
	memcpy(notice, stl_notice, sizeof(*notice));
	free(stl_notice);

	IB_EXIT(__func__, SUCCESS);
	return SUCCESS;
}

static Status_t fe_trap_thread_append(FE_Trap_t *current, FE_TrapThreadData_t *data)
{
	Status_t status;
	FE_Trap_t *temp;
	
    IB_ENTER(__func__, current, data, 0, 0); 
	
	status = vs_pool_alloc(&fe_pool, sizeof(FE_Trap_t), (void*)&temp);
	if (status != VSTATUS_OK) {
		IB_LOG_WARNRC("Failed to allocate trap data structure; rc:", status);
		IB_EXIT(__func__, status);
		return status;
	}
	
	memcpy(temp, current, sizeof(FE_Trap_t));
	temp->next = NULL;
	
	if (data->trap_list == NULL)
		data->trap_list = data->trap_list_end = temp;
	else
		data->trap_list_end = (data->trap_list_end->next = temp);
	
	++data->count;
	
	IB_EXIT(__func__, VSTATUS_OK);
	return VSTATUS_OK;
}

static void fe_trap_thread_worker(uint32_t argc, uint8_t **argv)
{
	Status_t status;
	uint32_t rc;
	FE_TrapThreadData_t *data;
	STL_NOTICE notice;
	FE_Trap_t current;
	FE_Trap_Processing_State_t state;
	
    IB_ENTER(__func__, argc, argv, 0, 0); 
	
	data = &fe_trap_thread_data;
	
	// loop forever, processing incoming notices
	while (1) {
		// wait for an incoming notice
		rc = fe_if3_trap_thread_get_notice(&notice);
		if (rc != SUCCESS) {
			IB_LOG_WARN("Failed to get notice, rc:", rc);
			vs_thread_sleep(FE_TRAP_THREAD_SLEEP_TIME);
			continue;
		}
		
		// convert it to an FE trap structure
		memset(&current, 0, sizeof(current));
		memset(&state, 0, sizeof(state));

		fe_if3_ib_notice_to_fe_trap(&notice, &current, &state);
		
		// the state will indicate found only if it's a trap that
		// we're interested in
		if (state.found) {
            current.notice = notice;

			// ensure we can not run while paused
			status = vs_lock(&data->lock);
			if (status != VSTATUS_OK) {
				IB_LOG_WARNRC("failed to take thread lock, rc:", status);
				vs_thread_sleep(FE_TRAP_THREAD_SLEEP_TIME);
				continue;
			}
			
			// if there was an SM lid buried in the trap info, update the
			// pre-allocated topology change trap
			if (state.smlid)
				data->trap128.lidAddr = state.smlid;
			
			// if trap 128 was received, just note it, as we'll used the pre-
			// allocated trap instead.  otherwise, append the trap to the list
			if (state.trap128Received) {
				data->trap128_received = 1;
			} else {
				// let append log errors... nothing else we can do
				(void)fe_trap_thread_append(&current, data);
				// no trap128 this time, so reset the flag
				data->trap128_received = 0;
			}
			
			vs_unlock(&data->lock);
		}
	}
	
	IB_EXIT(__func__, 0);
	return;
}
#endif /* IB_STACK_OPENIB */

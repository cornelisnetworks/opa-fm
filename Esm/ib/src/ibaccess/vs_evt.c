
/* BEGIN_ICS_COPYRIGHT5 ****************************************

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

 * ** END_ICS_COPYRIGHT5   ****************************************/

/************************************************************************
* 
* FILE NAME
*      vs_evt.c
*
* DESCRIPTION
*      Implemenation of Common Services Events
*
* DATA STRUCTURES
*      None
*
* FUNCTIONS
*      vs_event_create          Create an event object.
*      vs_event_wait            Wait for an event.
*      vs_event_post            Post an event.
*      vs_event_delete          Delete an event.
*
* DEPENDENCIES
*      Nucleus Plus RTOS.
*
*
***********************************************************************/

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <pthread.h>
#include "ib_types.h"
#include "ib_status.h"
#include "vs_g.h"
#include "cs_g.h"
#include "cs_log.h"
#include <sys/time.h>

/* log as Common Services Mudule */
#undef  LOCAL_MOD_ID
#define LOCAL_MOD_ID VIEO_CS_MOD_ID

/*
 * Implementation private data structure 
 */
#define IMPLPRIV_EVENT_MAGIC ((uint32_t) 0x688294CCU)
#define OP_OR  1
#define OP_AND 2
#define MAX_THREAD_WAIT 25 /* Max number of threads that can wait on an event */
#define MAX_EVENTS 32

typedef struct
{
  pthread_mutex_t   mutex;
  pthread_cond_t    cond;
  uint32_t          count;
} MaskList_Event_t;

typedef struct {
	uint32_t	magic;
	int 		id;
} Implpriv_Event_t;

typedef struct MaskList_s {
    MaskList_Event_t     event[MAX_EVENTS];
	int id;
	struct MaskList_s *prev;
	struct MaskList_s *next;
} MaskList_t;



MaskList_t *MaskList_Head;
Lock_t maskList_sem;
Lock_t eventArray[MAX_EVENTS];
int    currentId;
int    maskListInited = 0;


Status_t deleteMask(int id);
Status_t createMask(void);
Status_t vs_evt_MaskListInit(void);
Status_t sendEvent(int id, Eventset_t mask);
Status_t sendWakeAllEvent(int id, Eventset_t mask);
Status_t getEvent(int id, Eventset_t mask, int ticks);


/*********************************************************************************/

Status_t
vs_event_create (Event_t * ec, unsigned char name[], Eventset_t state)
{
  static const char function[] = "vs_event_create";
  uint32_t          namesize;
  Implpriv_Event_t *implpriv;

  IB_ENTER (function, (unint) ec, (unint) name, (uint32_t) state,
	    (uint32_t) 0U);
  
  if (ec == 0)
    {
      IB_LOG_ERROR0 ("Event_t parameter is null");
      IB_EXIT (function, VSTATUS_ILLPARM);
      return VSTATUS_ILLPARM;
    }

  if (name == 0)
    {
      IB_LOG_ERROR0 ("name parameter is null");
      IB_EXIT (function, VSTATUS_ILLPARM);
      return VSTATUS_ILLPARM;
    }

  for (namesize = 0U; namesize < VS_NAME_MAX; namesize++)
    {
      if (name[namesize] == (unsigned char) 0x00)
	{
	  (void) memmove (ec->name, name, namesize + 1U);
	  break;
	}
    }

  if (namesize >= VS_NAME_MAX)
    {
      IB_LOG_ERROR0 ("name doesn't contain a terminator");
      IB_EXIT (function, VSTATUS_ILLPARM);
      return VSTATUS_ILLPARM;
    }

  /*
     ** this is a run-time check to ensure that the opaque section of Event_t
     ** is large enough to contain the Implementation private data structure.
     ** If this test fails, adjust the OPAQUE_EVENT_SIZE_WORDS define in
     ** cs_g.h and rebuild.
   */
  if (sizeof (Implpriv_Event_t) > sizeof (ec->opaque))
    {
      IB_LOG_ERROR ("Implpriv_Event_t too big:", sizeof (Implpriv_Event_t));
      IB_EXIT (function, VSTATUS_ILLPARM);
      return VSTATUS_ILLPARM;
    }

  /*
  ** this is a run-time check to make sure that the event array is supported by
  ** the ATI implementation.
  */
  if (sizeof (Eventset_t) > sizeof (uint32_t))
  {
    IB_LOG_ERROR0 ("Implpriv_Event_t does not support size of event array.");
    IB_EXIT (function, VSTATUS_NODEV);
    return VSTATUS_NODEV;
  }

  implpriv = (Implpriv_Event_t *) (void *) & ec->opaque;
  memset(implpriv, 0, sizeof(*implpriv));

  implpriv->magic = IMPLPRIV_EVENT_MAGIC;
  ec->event_handle = ec;
  IB_LOG_INFO("handle: ", ec->event_handle);
  
  if((implpriv->id = createMask()) == 0){
      IB_LOG_ERROR0 ("Mask Creation Failed");
      IB_EXIT (function, VSTATUS_NODEV);
      return VSTATUS_NODEV;
  }
  


  IB_EXIT (function, VSTATUS_OK);
  return VSTATUS_OK;
}

Status_t
vs_event_delete (Event_t * ec)
{
  static const char function[] = "vs_event_delete";
  Implpriv_Event_t *implpriv;

  if (ec == 0)
  {
    IB_LOG_ERROR0 ("handle parameter is null");
    IB_EXIT (function, VSTATUS_ILLPARM);
    return VSTATUS_ILLPARM;
  }

  implpriv = (Implpriv_Event_t *) (void *) & ec->opaque;
  if (implpriv->magic != IMPLPRIV_EVENT_MAGIC)
  {
    IB_LOG_ERRORX ("Invalid magic:", implpriv->magic);
    IB_EXIT (function, VSTATUS_NXIO);
    return VSTATUS_NXIO;
  }

  implpriv->magic = ~IMPLPRIV_EVENT_MAGIC;
  ec->event_handle = 0;

  deleteMask(implpriv->id);
  implpriv->id = 0;

  IB_EXIT (function, VSTATUS_OK);
  return VSTATUS_OK;
}

Status_t
vs_event_post (Evt_handle_t handle, uint32_t option,
		   Eventset_t mask)
{
  static const char function[] = "vs_event_post";
  Event_t      *ec; 
  Implpriv_Event_t *implpriv;

  IB_ENTER (function, (unint) handle, option, (uint32_t) mask, 0U);

  if (handle == 0)
  {
    IB_LOG_ERROR0 ("handle parameter is null");
    IB_EXIT (function, VSTATUS_ILLPARM);
    return VSTATUS_ILLPARM;
  }

  if (mask == (Eventset_t) 0x00U)
  {
    IB_LOG_ERROR0 ("mask parameter is zero");
    IB_EXIT (function, VSTATUS_ILLPARM);
    return VSTATUS_ILLPARM;
  }

  if ((option != VEVENT_WAKE_ONE) && (option != VEVENT_WAKE_ALL))
  {
    IB_LOG_ERROR ("Invalid option:", option);
    IB_EXIT (function, VSTATUS_ILLPARM);
    return VSTATUS_ILLPARM;
  }

  ec = (Event_t *) handle;
  implpriv = (Implpriv_Event_t *) (void *) & ec->opaque;

  if (implpriv->magic != IMPLPRIV_EVENT_MAGIC)
  {
    IB_LOG_ERRORX ("Invalid magic:", implpriv->magic);
    IB_EXIT (function, VSTATUS_NXIO);
    return VSTATUS_NXIO;
  }

  if (ec->event_handle != handle)
  {
    IB_LOG_ERRORX ("Invalid handle:", (unint) ec->event_handle);
    IB_EXIT (function, VSTATUS_NXIO);
    return VSTATUS_NXIO;
  }

  if(option == VEVENT_WAKE_ALL){
	 // printf("vs_event_post: Wake ALL %s\n", taskName(taskIdSelf()));
	 // printf("vs_event_post: Wake ALL %s mask 0x%x\n", taskName(taskIdSelf()),(unsigned int)mask);
	  IB_LOG_INFO ("VEVENT_WAKE_ALL", mask);
	  sendWakeAllEvent(implpriv->id,mask);
  }else{
	  /* Get the number of pending threads */
	  //printf("vs_event_post: Wake One %s mask 0x%x\n", taskName(taskIdSelf()),(unsigned int)mask);
	  IB_LOG_INFO ("VEVENT_WAKE_ONE", mask);
      sendEvent(implpriv->id,mask);
  }


  IB_EXIT (function, VSTATUS_OK);
  return VSTATUS_OK;
}

Status_t
vs_event_wait (Evt_handle_t handle, uint64_t timeout,
		   Eventset_t mask, Eventset_t * events)
{
  static const char function[] = "vs_event_wait";
  Event_t      		*ec;
  int	            ticks;
  Implpriv_Event_t *implpriv;
  Status_t          rc;

  IB_ENTER (function, (unint) handle, timeout,
	       (uint32_t) mask, 0);

  if (handle == 0)
  {
    IB_LOG_ERROR0 ("handle parameter is null");
    IB_EXIT (function, VSTATUS_ILLPARM);
    return VSTATUS_ILLPARM;
  }
  
  ec = (Event_t *) handle;
  implpriv = (Implpriv_Event_t *) (void *) & ec->opaque;
  if (implpriv->magic != IMPLPRIV_EVENT_MAGIC)
  {
    IB_LOG_ERRORX ("Invalid magic:", implpriv->magic);
    IB_EXIT (function, VSTATUS_NXIO);
    return VSTATUS_NXIO;
  }

  if (mask == (Eventset_t) 0x00U)
  {
    IB_LOG_ERROR0 ("mask parameter is zero");
    IB_EXIT (function, VSTATUS_ILLPARM);
    return VSTATUS_ILLPARM;
  }

  if (events == (Eventset_t *) 0)
  {
    IB_LOG_ERROR0 ("events parameter is null");
    IB_EXIT (function, VSTATUS_ILLPARM);
    return VSTATUS_ILLPARM;
  }
  if(timeout == 0)
  {
	  ticks = -1;
  }
  else
  {
	  ticks = timeout;
  }

  rc = getEvent(implpriv->id, mask, ticks);
  if(rc == VSTATUS_OK){
	  //printf("Task %s consumed: mask: 0x%x\n", taskName(taskIdSelf()), (unsigned int)mask);
  }

  
  
  IB_EXIT (function, rc);
  return rc;
}


int getMaskIndex(Eventset_t mask){
	int i=0;
	for(i=0;i<MAX_EVENTS;i++){
		if((mask & (0x1 << i)) != 0){
			return i;
		}
	}
	return -1;
}

void cleanMask(MaskList_t *temp){
	int i;
	for(i=0;i<MAX_EVENTS;i++){
		/* TODO: Clean up the mutexes? Not sure if this is necessary for the architecture. If we're cleaning
		         up we're shutting down. */
		//semDelete(temp->semId[i]);
	}
}

MaskList_t* getLocalMaskList(int id){
    MaskList_t *temp;
    
	vs_lock(&maskList_sem);

    if(MaskList_Head != 0){
    	for(temp = MaskList_Head; temp != NULL; temp = temp->next){
    	    if(id == temp->id){
				vs_unlock(&maskList_sem);	
				return temp;
    	    }
    	}
    }

	vs_unlock(&maskList_sem);	
	return 0;
}

Status_t sendEvent(int id, Eventset_t mask){
    MaskList_t *temp;
    Status_t rc = VSTATUS_BAD;
	int index;

	if((index = getMaskIndex(mask)) == -1){
		IB_LOG_ERROR0("Mask is either NULL or out of range");
		return VSTATUS_BAD;
	}

	if((temp = getLocalMaskList(id)) != 0){
		(void)pthread_mutex_lock (&temp->event[index].mutex);
		temp->event[index].count++; 
		(void)pthread_cond_broadcast (&temp->event[index].cond);
		(void)pthread_mutex_unlock (&temp->event[index].mutex);
		rc = VSTATUS_OK;
	}

    return rc;
}

Status_t sendWakeAllEvent(int id, Eventset_t mask){
    MaskList_t *temp;
    Status_t rc = VSTATUS_BAD;
	int index;

	if((index = getMaskIndex(mask)) == -1){
		IB_LOG_ERROR0("Mask is either NULL or out of range");
		return VSTATUS_BAD;
	}

	if((temp = getLocalMaskList(id)) != 0){
		(void)pthread_mutex_lock (&temp->event[index].mutex);
		/* Not sure if we should increment the count here or not */
		/* temp->event[index].count++; */
		(void)pthread_cond_broadcast (&temp->event[index].cond);
		(void)pthread_mutex_unlock (&temp->event[index].mutex);
		rc = VSTATUS_OK;
	}

    return rc;
}

Status_t getEvent(int id, Eventset_t mask, int ticks){
    MaskList_t *temp;
    Status_t rc = VSTATUS_BAD;
	int index;
	struct timespec abstime;
	struct	timeval	now;

	gettimeofday(&now, NULL);
    abstime.tv_sec = now.tv_sec + (ticks/1000000);
    abstime.tv_nsec = (now.tv_usec + (ticks % 1000000)) * 1000;
    // handle carry from nsec to sec
    if (abstime.tv_nsec > 1000000000) {
        abstime.tv_sec++;
        abstime.tv_nsec -= 1000000000;
    }

	if((index = getMaskIndex(mask)) == -1){
		IB_LOG_ERROR0("Mask is either NULL or out of range");
		return VSTATUS_BAD;
	}
	
	if((temp = getLocalMaskList(id)) != 0){
		(void)pthread_mutex_lock (&temp->event[index].mutex);
		if(temp->event[index].count){
			/* Event was posted before we waited on it */
			temp->event[index].count--; 
			rc = VSTATUS_OK;
		}else{
			if(pthread_cond_timedwait(&temp->event[index].cond, &temp->event[index].mutex,  &abstime) != 0){
				rc = VSTATUS_TIMEOUT;
			}else{
				rc = VSTATUS_OK;
				if(temp->event[index].count)
					temp->event[index].count--; 

			}
		}
		(void)pthread_mutex_unlock (&temp->event[index].mutex);
	}


    return rc;
}





Status_t deleteMask(int id){
    MaskList_t *temp;
    Status_t rc = VSTATUS_BAD;

	if(vs_lock(&maskList_sem) != VSTATUS_OK){
		return VSTATUS_BAD;
	}

    if(MaskList_Head != 0){
    	for(temp = MaskList_Head; temp != NULL; temp = temp->next){
    	    if(id == temp->id){
        		if((temp->next == NULL) && (temp->prev == NULL)){ // Only link in list.
        			cleanMask(temp);
        		    free(temp);
        		    MaskList_Head = NULL;
        		}else if(temp->next == NULL){ // Last link in list
        		    temp->prev->next = NULL;
        			cleanMask(temp);
        		    free(temp);
        		}else if(temp->prev == NULL){ // First Link in list
        		    MaskList_Head = temp->next;
					temp->next->prev = NULL;
        			cleanMask(temp);
        		    free(temp);
        		}else{ // In the middle of the list.
        		    temp->prev->next = temp->next;
        		    temp->next->prev = temp->prev;
        			cleanMask(temp);
        		    free(temp);
        		}
        		rc = VSTATUS_OK;
        		break;
    	    }
    	}
    }
    vs_unlock (&maskList_sem);
    return rc;
}

Status_t createMask(){
    MaskList_t *temp = NULL;
	int i;	

	vs_evt_MaskListInit();
    
    if(vs_lock(&maskList_sem) != VSTATUS_OK){
		return VSTATUS_BAD;
	}
	currentId++;

    if((temp = calloc(1,sizeof(MaskList_t))) == NULL){
		IB_LOG_ERROR0("Could not allocate space!");
		vs_unlock (&maskList_sem);
		return 0;
    }

    temp->next = NULL;
    temp->prev = NULL;
	temp->id = currentId;
    for(i=0;i<MAX_EVENTS;i++){
		if (pthread_mutex_init (&temp->event[i].mutex, NULL) != 0){
			IB_LOG_ERROR0("Error while initializing mutexes!");
			vs_unlock (&maskList_sem);
			free(temp);
			return 0;
		}
		(void)pthread_mutex_lock (&temp->event[i].mutex);
		/* Initialize the condition. This is used to signal waiting threads the event occured. */
		(void)pthread_cond_init (&temp->event[i].cond, NULL);
		temp->event[i].count = 0;
		(void)pthread_mutex_unlock (&temp->event[i].mutex);
	}

    if(MaskList_Head != 0){
		MaskList_Head->prev = temp;
		temp->next = MaskList_Head;
		MaskList_Head = temp;
    }else{
		MaskList_Head = temp;
    }

    vs_unlock (&maskList_sem);

    return temp->id;
}

Status_t vs_evt_MaskListInit(){
	if(maskListInited != 0){
		return VSTATUS_OK;
	}
	currentId = 0;

    if(vs_lock_init (&maskList_sem, VLOCK_FREE, VLOCK_THREAD) != VSTATUS_OK){
		return VSTATUS_BAD;
	}

	maskListInited = 1;
    return VSTATUS_OK;
}
#if 0
void dumpMaskListInfo(){
    MaskList_t *temp;
	int i;

	semTake(maskList_sem, WAIT_FOREVER);

    if(MaskList_Head != 0){
    	for(temp = MaskList_Head; temp != NULL; temp = temp->next){
			//printf("Mask %x \n",temp->id);
			for(i=0;i<MAX_EVENTS;i++){
				semShow(temp->semId[i],1);
			}
     	}
    }

    semGive(maskList_sem);

}

void vs_evt_cleanMaskList(){
	int id = 0;

	while(1){
    	semTake(maskList_sem, WAIT_FOREVER);
    	if(MaskList_Head != 0){
    		id = MaskList_Head->id;
    	}else{
    		id = -1;
    	}
    	semGive(maskList_sem);

    	if(id > 0){
    		deleteMask(id);
    	}else{
    		break;
    	}
	}
}

#endif

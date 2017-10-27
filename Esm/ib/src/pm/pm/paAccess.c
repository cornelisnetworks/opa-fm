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

#include "pm_topology.h"
#include "paAccess.h"
#include "iba/stl_pa_priv.h"
#include "fm_xml.h"
#include <stdio.h>
#include <time.h>
#ifndef __VXWORKS__
#include <strings.h>
#endif

#undef LOCAL_MOD_ID
#define LOCAL_MOD_ID VIEO_PA_MOD_ID

boolean	isFirstImg = TRUE;
boolean	isUnexpectedClearUserCounters = FALSE;
char* FSTATUS_Strings[] = {
	"FSUCCESS",
	"FERROR",
	"FINVALID_STATE",
	"FINVALID_OPERATION",
	"FINVALID_SETTING",
	"FINVALID_PARAMETER",
	"FINSUFFICIENT_RESOURCES",
	"FINSUFFICIENT_MEMORY",
	"FCOMPLETED",
	"FNOT_DONE",
	"FPENDING",
	"FTIMEOUT",
	"FCANCELED",
	"FREJECT",
	"FOVERRUN",
	"FPROTECTION",
	"FNOT_FOUND",
	"FUNAVAILABLE",
	"FBUSY",
	"FDISCONNECT",
	"FDUPLICATE",
	"FPOLL_NEEDED",
};
// given an Image Index, build an externally usable History ImageId
// this will allow future reference to this image via history[]
static uint64 BuildHistoryImageId(Pm_t *pm, uint32 historyIndex)
{
	ImageId_t id;

	id.AsReg64 = 0;
	id.s.type = IMAGEID_TYPE_HISTORY;
	id.s.sweepNum = pm->Image[pm->history[historyIndex]].sweepNum;
	id.s.index = historyIndex;
	if (pm_config.shortTermHistory.enable) 
		id.s.instanceId = pm->ShortTermHistory.currentInstanceId;
	return id.AsReg64;
}

// given an Image Index, build an externally usable FreezeFrame ImageId
// this will allow future reference to this image via freezeFrames[]
static uint64 BuildFreezeFrameImageId(Pm_t *pm, uint32 freezeIndex,
	uint8 clientId, uint32 *imageTime)
{
	ImageId_t id;

	id.AsReg64 = 0;
	id.s.type = IMAGEID_TYPE_FREEZE_FRAME;
	id.s.clientId = clientId;
	id.s.sweepNum = pm->Image[pm->freezeFrames[freezeIndex]].sweepNum;
	id.s.index = freezeIndex;
	if (pm_config.shortTermHistory.enable)
		id.s.instanceId = pm->ShortTermHistory.currentInstanceId;
	if (imageTime)
		*imageTime = (uint32)pm->Image[pm->freezeFrames[freezeIndex]].sweepStart;
	//printf("build Freeze Frame ImageId: type=%u, client=%u sweep=%u index=%u Image=0x%llx\n", id.s.type, id.s.clientId, id.s.sweepNum, id.s.index, id.AsReg64);
	return id.AsReg64;
}

// given a index into history[], validate index and figure out
// imageIndex into Image[]
// also build retImageId
static FSTATUS ComputeHistory(Pm_t *pm, uint32 historyIndex, int32 offset,
				uint32 baseSweepNum,
				uint32 *imageIndex, uint64 *retImageId, const char **msg)
{
	uint32 index;

	if (historyIndex >= pm_config.total_images
				|| pm->history[historyIndex] == PM_IMAGE_INDEX_INVALID) {
		*msg = "Invalid Image Id";
		return FINVALID_PARAMETER;
	}
	if (offset < 0) {
		if (-offset >= pm_config.total_images || -offset > pm->NumSweeps) {
			*msg = "negative offset exceeds duration of history";
			return FNOT_FOUND;
		}
	} else {
		if (offset >= pm_config.total_images || offset > pm->NumSweeps) {
			*msg = "positive offset exceeds duration of history";
			return FNOT_FOUND;
		}
	}

	// index into pm->history[]
	index = (historyIndex + pm_config.total_images+offset) % pm_config.total_images;
	//printf("offset=%d historyIndex=%u index=%u\n", offset, historyIndex, index);
	*imageIndex = pm->history[index];
	if (*imageIndex == PM_IMAGE_INDEX_INVALID) {
		*msg = "Invalid Image Id";
		return FINVALID_PARAMETER;
	}
	ASSERT(*imageIndex < pm_config.total_images);

	// validate it's still there
	if (pm->Image[*imageIndex].sweepNum != baseSweepNum+offset
		|| pm->Image[*imageIndex].state != PM_IMAGE_VALID) {
		*msg = "offset exceeds duration of history";
		return FNOT_FOUND;
	}
	*retImageId = BuildHistoryImageId(pm, index);
	return FSUCCESS;
}

/**
 *	GetIndexFromTime - Given a time input, find imageIndex for RAM image with matching sweepStart
 *	                   time
 *	
 *	Inputs:
 *		pm            - the pm
 *		type          - (in-RAM)image type: ANY, HISTORY, or FREEZE_FRAME
 *		requestTime   - time to find an image for 
 *		imageIndex    - will be set to an index into list of Images if corresponding image found
 *		returnImageId - will contain image information of image found (if any)
 *		msg           - error message if unsuccessful
 *		clientId      - client ID
 *
 *	on Error: 
 *		set message to string describing failure
 *		return error status 
 *
 */
static FSTATUS GetIndexFromTime(Pm_t *pm, uint8 type, time_t requestTime, 
		                        uint32 *imageIndex,
								STL_PA_IMAGE_ID_DATA *returnImageId, 
								const char **msg, uint8 *clientId)
{
	uint32 lastImageIndex = pm->history[pm->lastHistoryIndex];
	int i;

	if (pm->LastSweepIndex == PM_IMAGE_INDEX_INVALID
		|| lastImageIndex == PM_IMAGE_INDEX_INVALID
		|| pm->Image[lastImageIndex].state != PM_IMAGE_VALID) {
		*msg = "Sweep Engine not ready";
		return FUNAVAILABLE;
	}

	uint32 historyIndex = pm->lastHistoryIndex;
	for (i = 0; i < pm_config.total_images; ++i){
		if (FSUCCESS == ComputeHistory(pm, historyIndex, 0-i, pm->Image[lastImageIndex].sweepNum,
					                   imageIndex, &returnImageId->imageNumber, 
									   msg)){
			//we have a valid imageIndex, get RAM image and look at 
			//sweepStart time
			PmImage_t *pmimagep = &pm->Image[*imageIndex];
			(void)vs_rdlock(&pmimagep->imageLock);
			if (pmimagep->state == PM_IMAGE_VALID
				// Check request Time is between imageStart and the imageInterval
				&& requestTime >= pmimagep->sweepStart
				&& requestTime < (pmimagep->sweepStart + MAX(pmimagep->sweepDuration/1000000, pm->interval)) ) {

				//build returnImageId, imageNumber set in ComputeHistory
				returnImageId->imageTime.absoluteTime = requestTime;
				returnImageId->imageOffset = 0;
				
				(void)vs_rwunlock(&pmimagep->imageLock);
				return FSUCCESS;
			}
			(void)vs_rwunlock(&pmimagep->imageLock);
		}
	}

	*msg = "No Image found matching request time";
	return FNOT_FOUND;
}


// take an ImageId (could be Freeze or history/current) and resolve to a
// imageIndex.  Validate the ImageId, index and the Image index points to are
// all valid.  On error return error code and set message to a string
// describing the failure.
// must be called with pm->stateLock held as read or write lock
// imageid is opaque and should be a value previously returned or
// 0=IMAGEID_LIVE_DATA
// offset is valid for FreezeFrame, LiveData and History images
// offset<0 moves back in time, offset >0 moves forward.
// Positive offsets are only valid for FreezeFrame and History Images
// type:
// ANY - no check
// HISTORY - allow live or history
// FREEZE_FRAME - allow freeze frame only
static FSTATUS GetIndexFromImageId(Pm_t *pm, uint8 type, uint64 imageId, int32 offset,
				uint32 *imageIndex, uint64 *retImageId, const char **msg,
				uint8 *clientId)
{
	ImageId_t id;
	uint32 lastImageIndex = pm->history[pm->lastHistoryIndex];

	id.AsReg64 = imageId;

	if (pm->LastSweepIndex == PM_IMAGE_INDEX_INVALID
		|| lastImageIndex == PM_IMAGE_INDEX_INVALID
		|| pm->Image[lastImageIndex].state != PM_IMAGE_VALID) {
		*msg = "Sweep Engine not ready";
		return FUNAVAILABLE;
	}
	DEBUG_ASSERT(lastImageIndex == pm->LastSweepIndex);

	if ( type != IMAGEID_TYPE_ANY
		 && (id.s.type == IMAGEID_TYPE_FREEZE_FRAME)
					 != (type == IMAGEID_TYPE_FREEZE_FRAME)) {
		*msg = "Invalid Image Id Type";
		return FINVALID_PARAMETER;
	}

	// live data or recent history
	if (IMAGEID_LIVE_DATA == imageId) {
		if (offset > 0) {
			*msg = "Positive offset not allowed for live data";
			return FINVALID_PARAMETER;
		} else if (offset == 0) {
			*imageIndex = pm->LastSweepIndex;
			*retImageId = BuildHistoryImageId(pm, pm->lastHistoryIndex);
			return FSUCCESS;
		} else {
			return ComputeHistory(pm, pm->lastHistoryIndex, offset,
						  pm->Image[lastImageIndex].sweepNum,
						  imageIndex, retImageId, msg);
		}
	} else if (id.s.type == IMAGEID_TYPE_HISTORY) {
		return ComputeHistory(pm, id.s.index, offset, id.s.sweepNum, imageIndex,
					   			retImageId, msg);
	} else if (id.s.type == IMAGEID_TYPE_FREEZE_FRAME) {
		time_t now_time;
		PmImage_t *pmimagep;

		if (id.s.index >= pm_config.freeze_frame_images) {
			*msg = "Invalid Image Id";
			return FINVALID_PARAMETER;
		}
		*imageIndex = pm->freezeFrames[id.s.index];
		if (*imageIndex == PM_IMAGE_INDEX_INVALID) {
			*msg = "Invalid Image Id";
			return FINVALID_PARAMETER;
		}
		ASSERT(*imageIndex < pm_config.total_images);
		// validate it's still there
		pmimagep = &pm->Image[*imageIndex];
		if (pmimagep->sweepNum != id.s.sweepNum
			|| pmimagep->state != PM_IMAGE_VALID) {
			*msg = "Freeze Frame expired or invalid";
			return FNOT_FOUND;
		}
		if (0 == pmimagep->ffRefCount) {
			// cleanup, must have expired
			pm->freezeFrames[id.s.index] = PM_IMAGE_INDEX_INVALID;
			*msg = "Freeze Frame expired or invalid";
			return FNOT_FOUND;
		}
		if (0 == (pmimagep->ffRefCount & (1<<id.s.clientId))) {
			*msg = "Freeze Frame expired or invalid";
			return FNOT_FOUND;
		}
		if (type == IMAGEID_TYPE_FREEZE_FRAME && offset != 0) {
			*msg = "Freeze Frame offset not allowed in this query";
			return FINVALID_PARAMETER;
		}
		// stdtime is 32 bits, so should be atomic write and avoid race
		vs_stdtime_get(&now_time);
		pmimagep->lastUsed= now_time;	// update age due to access
		if (clientId)
			*clientId = id.s.clientId;
		if (offset == 0) {
			*retImageId = imageId;	// no translation, return what given
			return FSUCCESS;
		} else {
			return ComputeHistory(pm, pmimagep->historyIndex, offset,
						   		id.s.sweepNum, imageIndex, retImageId, msg);
		}
	} else {
		*msg = "Invalid Image Id";
		return FINVALID_PARAMETER;
	}
}
#ifndef __VXWORKS__

FSTATUS CheckComposite(Pm_t *pm, uint64 imageId, PmCompositeImage_t *cimg) {
	ImageId_t temp;
	temp.AsReg64 = imageId;
	temp.s.type = IMAGEID_TYPE_HISTORY;
	temp.s.clientId = 0;
	temp.s.index = 0;
	int i;
	for (i = 0; i < PM_HISTORY_MAX_IMAGES_PER_COMPOSITE; i++) {
		if (cimg->header.common.imageIDs[i] == temp.AsReg64) 
			return FSUCCESS;
	}
	return FNOT_FOUND;
}

#endif

/*
 *Comparison function when requesting history records by time
 * */
int sweepTimeCompare(const uint64 map_key, const uint64 requestTime){
	if (requestTime >= (map_key >> 32) + (map_key & 0xFFFFFFFF)) return -1;
	else if (requestTime < map_key >> 32) return 1;
	else return 0;
}

/************************************************************************************* 
*   FindImageByTime - given a specific time, find the image with corresponding sweepStart
*      (in-RAM or Short-Term History)
*  
*   Inputs:
*       pm - the PM
*       type - (in-RAM)image type: ANY, HISTORY, or FREEZE_FRAME
*       requestTime - requested sweep start time
*       imageIndex - if image is found in-RAM, this will be the image index
*       retImageId - the image ID of the image which was found
*       record - if the image was found in Short-Term History, this will be its record
*       msg - error message
*       clientId - client ID
*       cimg - if the image is frozen, or the current composite, this will point to
*   			that image
*
*   Returns:
*   	FSUCCESS if success, FNOT_FOUND if not found
*  
*************************************************************************************/
FSTATUS FindImageByTime(
	Pm_t *pm, 
	uint8 type, 
	time_t requestTime, 
	uint32 *imageIndex, 
	STL_PA_IMAGE_ID_DATA *retImageId, 
	PmHistoryRecord_t **record,
	const char **msg, 
	uint8 *clientId,
	PmCompositeImage_t **cimg
	)
{
	FSTATUS status;

	//validate requestTime
	if (!requestTime){
		*msg = "Invalid request time";
		return FINVALID_PARAMETER;
	}

	// CASE 1: Image with requested time in RAM
	status = GetIndexFromTime(pm, type, requestTime, imageIndex, retImageId, msg, clientId);
	if (FSUCCESS == status || FUNAVAILABLE == status) return status;

#ifndef __VXWORKS__
	// CASE 2: Image with requested time in STH
	if (!pm_config.shortTermHistory.enable) return status;

	PmHistoryRecord_t *found;
	cl_map_item_t *mi;
	mi = cl_qmap_get_compare(&pm->ShortTermHistory.imageTimes, requestTime, sweepTimeCompare);

	if (mi == cl_qmap_end(&pm->ShortTermHistory.imageTimes)) {
		IB_LOG_ERROR_FMT(__func__, "Unable to find image Time: %"PRIu64" in record map", (uint64)requestTime);
		*msg = "Invalid history image time";
		return FNOT_FOUND;
	}

	// find the parent record of the entries
	found = PARENT_STRUCT(mi, PmHistoryRecord_t, imageTimeEntry);
	if (!found) {
		*msg = "Error looking up image Time no parent entries found";
		return FNOT_FOUND;
	}

	*record = found;
	status = FSUCCESS;

#endif

	return status;
}

/************************************************************************************* 
*   FindImage - given and image ID and an offset, find the corresponding image
*      (in-RAM or Short-Term History)
*  
*   Inputs:
*   	pm - the PM
*       type - (in-RAM)image type: ANY, HISTORY, or FREEZE_FRAME
*       imageId - requested image ID
*       offset - requested offset from image ID
*       imageIndex - if image is found in-RAM, this will be the image index
*       retImageId - the image ID of the image which was found
*       record - if the image was found in Short-Term History, this will be its record
*       msg - error message
*       clientId - client ID
*       cimg - if the image is frozen, or the current composite, this will point to
*   			that image
*
*   Returns:
*   	FSUCCESS if success, FNOT_FOUND if not found
*  
*************************************************************************************/
FSTATUS FindImage(
	Pm_t *pm, 
	uint8 type, 
	STL_PA_IMAGE_ID_DATA imageId,
	uint32 *imageIndex, 
	uint64 *retImageId, 
	PmHistoryRecord_t **record,
	const char **msg, 
	uint8 *clientId,
	PmCompositeImage_t **cimg
	)
{
	FSTATUS status = FNOT_FOUND;
	
	STL_PA_IMAGE_ID_DATA id_data = {0};

	if (IMAGEID_ABSOLUTE_TIME == imageId.imageNumber){
		status = FindImageByTime(pm, type, (time_t) imageId.imageTime.absoluteTime, imageIndex,
				               &id_data, record, msg, clientId, cimg);
		if (FSUCCESS == status)
			*retImageId = id_data.imageNumber;
		return status;
	}

	// only check the RAM Images if the requested instance ID matches the current instance
	if(imageId.imageNumber == IMAGEID_LIVE_DATA || !pm_config.shortTermHistory.enable ||
		(pm_config.shortTermHistory.enable && ((ImageId_t)imageId.imageNumber).s.instanceId == pm->ShortTermHistory.currentInstanceId))
	{
		// image ID in RAM, offset in RAM
		status = GetIndexFromImageId(pm, type, imageId.imageNumber, imageId.imageOffset, imageIndex, retImageId, msg, clientId);
		if (status == FSUCCESS)	return status;
	}

	if (!pm_config.shortTermHistory.enable || status == FUNAVAILABLE)
		return status;
	
#ifndef __VXWORKS__
	PmHistoryRecord_t *found;
	ImageId_t histId;
	if (imageId.imageNumber) {
		histId.AsReg64 = imageId.imageNumber;
		histId.s.type = IMAGEID_TYPE_HISTORY;
		histId.s.clientId = 0;
		histId.s.index = 0;
	} else {
		histId.AsReg64 = 0;
	}
	
	int ireq = -1;	// index of the requested imageID in history
	int icurr = -1;	// index of the record currently be built in history
	int oneg = -1;	// maximum negative offset allowed
	int opos = -1;	// maximum positive offset allowed
	int tot = -1;	// total number of records in history
    int32 offset = imageId.imageOffset;
	// only check the RAM images if the requested instance ID matches the current instance
	if(imageId.imageNumber == IMAGEID_LIVE_DATA ||
		((ImageId_t)imageId.imageNumber).s.instanceId == pm->ShortTermHistory.currentInstanceId)
	{
		// image ID in RAM, offset negative into history
		status = GetIndexFromImageId(pm, type, imageId.imageNumber, 0, imageIndex, retImageId, msg, clientId);
	}
	if (status == FSUCCESS && offset < 0) {
	// offset needs to be adjusted to account for the overlap between history images and RAM images
	// new offset should be: old offset - distance required to cover the rest of the RAM images + offset from 1st record to 1st 'uncovered' record
		int32 r, t, i, c, o;
		// r: offset from live to requested image ID
		r = imageId.imageNumber != IMAGEID_LIVE_DATA ? histId.s.sweepNum - pm->NumSweeps : 0;
		// t: current total number of RAM images
		t = (int32)MIN(pm_config.total_images, pm->NumSweeps);
		// i: images per composite
		i = (int32)pm_config.shortTermHistory.imagesPerComposite;
		// c: number of images in the composite currently being built
		c = (int32)(pm->ShortTermHistory.currentComposite->header.common.imagesPerComposite == pm_config.shortTermHistory.imagesPerComposite ? 0 : pm->ShortTermHistory.currentComposite->header.common.imagesPerComposite);
		// o: offset from 1st history record to 1st history record not covered by RAM images
		o = c > t ? 1 : (c - t + 1) / i;
		offset = offset + r + t + o; // total new offset
		if (offset > 0) { // this means we went back from RAM images to the current composite
			*cimg = pm->ShortTermHistory.currentComposite;
			return status;
		}
		// now we will be working from the first history record
		ireq = pm->ShortTermHistory.currentRecordIndex==0?pm->ShortTermHistory.totalHistoryRecords - 1:pm->ShortTermHistory.currentRecordIndex - 1;
	} else if (status == FSUCCESS && offset > 0) {
		// image is in RAM, positive offset - should have been found by first check, return not found
		*msg = "positive offset exceeds duration of history";
		return FNOT_FOUND;
	} else {
		// image ID in history
		// check frozen composite
		if (!offset && pm->ShortTermHistory.cachedComposite) {
			status = CheckComposite(pm, histId.AsReg64, pm->ShortTermHistory.cachedComposite);
			if (status == FSUCCESS) {
				// frozen cached composite was requested directly by image ID
				*cimg = pm->ShortTermHistory.cachedComposite;
				return status;
			}
		}
		// check the current composite
		if (pm->ShortTermHistory.currentComposite && pm->ShortTermHistory.currentComposite->header.common.imagesPerComposite >= pm_config.total_images) {
			status = CheckComposite(pm, histId.AsReg64, pm->ShortTermHistory.currentComposite);
			if (status == FSUCCESS) {
				if (!offset) {
					// current composite was requested directly by image ID
					*cimg = pm->ShortTermHistory.currentComposite;
					return status;
				} else {
					// image ID was for current composite, so use 1st history record as starting place, adjust offset
					ireq = pm->ShortTermHistory.currentRecordIndex==0?pm->ShortTermHistory.totalHistoryRecords - 1:pm->ShortTermHistory.currentRecordIndex - 1;
					offset++;
				}
			}
		}
		// ireq has not already been set by finding image ID in RAM or current composite, check history records
		if (ireq < 0) {
			// find this image
			cl_map_item_t *mi;
			// look up the image Id in the history images map
			mi = cl_qmap_get(&pm->ShortTermHistory.historyImages, histId.AsReg64);
			if (mi == cl_qmap_end(&pm->ShortTermHistory.historyImages)) {
				IB_LOG_ERROR_FMT(__func__, "Unable to find image ID: 0x"FMT_U64" in record map", imageId.imageNumber);
				*msg = "Invalid history image ID";
				return FNOT_FOUND;
			}
			// find the parent entry for the map item
			PmHistoryImageEntry_t *entry = PARENT_STRUCT(mi, PmHistoryImageEntry_t, historyImageEntry);
			if (!entry) {
				*msg = "Error looking up image ID entry";
				return FNOT_FOUND;
			}
			if (entry->inx == INDEX_NOT_IN_USE) {
				*msg = "Error looking up image ID entry not in use";
				return FNOT_FOUND;
			}
			// find the parent record of the entries
			PmHistoryImageEntry_t *entries = entry - (entry->inx);
			found = PARENT_STRUCT(entries, PmHistoryRecord_t, historyImageEntries);
			if (!found || found->index == INDEX_NOT_IN_USE) {
				*msg = "Error looking up image ID no parent entries found";
				return FNOT_FOUND;
			}
			// if offset is 0, then this is the requested record
			if (offset == 0) {
				*record = found;
				return FSUCCESS;
			}
			// otherwise, set ireq
			ireq = found->index;
		}
	}
	icurr = pm->ShortTermHistory.currentRecordIndex;
	tot = pm->ShortTermHistory.totalHistoryRecords;

	// check: image ID in history, offset positive into RAM
	if (offset > 0) {
		// check: offset goes into RAM but doesn't exceed RAM
		// minimum offset needed to get back into RAM = distance from ireq to icurr - offset of 1st record to 'uncovered' records
		int32 maxRamOff, minRamOff; // the maximum and minimum values for offset that would place the requested image in RAM
		// calculate maxRamOff and minRamOff
		int32 t, c, i, o;
		// t: total number of RAM images right now
		t = (int32)MIN(pm_config.total_images, pm->NumSweeps);
		// i: images per composite
		i = (int32)pm_config.shortTermHistory.imagesPerComposite;
		// c: number of images in current composite
		c = (int32)((!pm->ShortTermHistory.currentComposite || pm->ShortTermHistory.currentComposite->header.common.imagesPerComposite == pm_config.shortTermHistory.imagesPerComposite) ? 0 : pm->ShortTermHistory.currentComposite->header.common.imagesPerComposite);
		// o: offset of icurr from uncovered records
		o = c > t ? -1 : (t - c - 1) / i;
		minRamOff = ((ireq < icurr)?(icurr - ireq):(tot - (ireq - icurr))) - o;
		maxRamOff = minRamOff + (int32)MIN(pm_config.total_images, pm->NumSweeps) - 2;
		if (offset <= maxRamOff && offset >= minRamOff) {
			// offset is within acceptable range
			// calculate new Offset (offset from the live RAM image)
			int32 newOffset = offset - maxRamOff;
			// call GetIndexFromImageId again, with adjusted offset and live image ID
			status = GetIndexFromImageId(pm, type, IMAGEID_LIVE_DATA, newOffset, imageIndex, retImageId, msg, clientId);
			if (status == FSUCCESS) {
				return status;
			}
			// we failed - probably because the image is being overwritten by the next sweep
			// so just look in the short-term history instead
		} else if (offset > maxRamOff) {
			*msg = "Offset exceeds duration of history";
			return FNOT_FOUND;
		}
		// otherwise check the Short-Term History images
	}	
	
	// offset into history
	
	// establish positive and negative bounds for the offset
	if (ireq >= icurr) {
		oneg = ireq - icurr;
		opos = tot - (oneg + 1);
	} else {
		oneg = tot - (icurr - ireq);
		opos = icurr - ireq - 1;
	}

	if (offset < -oneg) {
		*msg = "Negative offset exceeds duration of Short-Term History";
		return FNOT_FOUND;
	} else if (offset > opos) {
		if ((offset - opos) == 1 && pm->ShortTermHistory.currentComposite && pm->ShortTermHistory.currentComposite->header.common.imagesPerComposite > pm_config.total_images) {
			// if offset is opos+1, and the current composite spans outside of the range of RAM images, return the current composite
			*cimg = pm->ShortTermHistory.currentComposite;
			return FSUCCESS;
		}
		*msg = "Positive offset exceeds duration of Short-Term History";
		return FNOT_FOUND;
	} else {
		// find the record
		int r = (offset < 0)?(ireq + offset):((ireq + offset)%tot);
		if (r < 0) {
			// wrap
			r = tot + r;
		}
		*record = pm->ShortTermHistory.historyRecords[r];
		if (!*record || !(*record)->header.timestamp || (*record)->index == INDEX_NOT_IN_USE) {
			*msg = "Image request found empty history record";
			return FNOT_FOUND;
		}
		*imageIndex = 0;
		return FSUCCESS;
	}
#else
	return status;
#endif
}

// locate group by name
static FSTATUS LocateGroup(Pm_t *pm, const char *groupName, PmGroup_t **pmGroupP)
{
	int i;

	if (strcmp(groupName, pm->AllPorts->Name) == 0) {
		*pmGroupP = pm->AllPorts;
		return FSUCCESS;
	}

	for (i = 0; i < pm->NumGroups; i++) {
		if (strcmp(groupName, pm->Groups[i]->Name) == 0) {
			*pmGroupP = pm->Groups[i];
			return FSUCCESS;
		}
	}

	return FNOT_FOUND;
}

/*************************************************************************************
*
* paGetGroupList - return list of group names
*
*  Inputs:
*     pm - pointer to Pm_t (the PM main data type)
*     pmGroupList - pointer to caller-declared data area to return names
*
*  Return:
*     FSTATUS - FSUCCESS if OK
*
*
*************************************************************************************/

FSTATUS paGetGroupList(Pm_t *pm, PmGroupList_t *GroupList)
{
	Status_t			vStatus;
	FSTATUS				fStatus = FSUCCESS;
	int					i;

	// check input parameters
	if (!pm || !GroupList)
		return(FINVALID_PARAMETER);

	AtomicIncrementVoid(&pm->refCount);	// prevent engine from stopping
	if (! PmEngineRunning()) {	// see if is already stopped/stopping
		fStatus = FUNAVAILABLE;
		goto done;
	}
	GroupList->NumGroups = pm->NumGroups + 1; // Number of Groups plus ALL Group

	vStatus = vs_pool_alloc(&pm_pool, GroupList->NumGroups * STL_PM_GROUPNAMELEN,
						   (void*)&GroupList->GroupList);
	if (vStatus != VSTATUS_OK) {
		IB_LOG_ERRORRC("Failed to allocate name list buffer for GroupList rc:", vStatus);
		fStatus = FINSUFFICIENT_MEMORY;
		goto done;
	}
	// no lock needed, group names are constant once PM starts
	snprintf(GroupList->GroupList[0].Name, STL_PM_GROUPNAMELEN, "%s", pm->AllPorts->Name);

	for (i = 0; i < pm->NumGroups; i++)
		snprintf(GroupList->GroupList[i+1].Name, STL_PM_GROUPNAMELEN, "%s", pm->Groups[i]->Name);

done:
	AtomicDecrementVoid(&pm->refCount);
	return(fStatus);
}

/*************************************************************************************
*
* paGetGroupInfo - return group information
*
*  Inputs:
*     pm - pointer to Pm_t (the PM main data type)
*     groupName - pointer to name of group
*     pmGroupInfo - pointer to caller-declared data area to return group information
*
*  Return:
*     FSTATUS - FSUCCESS if OK, FERROR
*
*
*************************************************************************************/

FSTATUS paGetGroupInfo(Pm_t *pm, char *groupName, PmGroupInfo_t *pmGroupInfo,
	STL_PA_IMAGE_ID_DATA imageId, STL_PA_IMAGE_ID_DATA *returnImageId)
{
	STL_PA_IMAGE_ID_DATA retImageId = {0};
	FSTATUS				status = FSUCCESS;
	PmGroup_t			*pmGroupP = NULL;
	PmGroupImage_t		pmGroupImage;
	PmImage_t			*pmImageP = NULL;
	PmPortImage_t		*pmPortImageP = NULL, *pmPortImageNeighborP = NULL;
	PmPort_t			*pmPortP = NULL;
	uint32				imageIndex, imageInterval;
	const char 			*msg;
	boolean				sth = 0;
	STL_LID_32 			lid;
	boolean				isInternal = FALSE;
	boolean				isGroupAll = FALSE;
	PmHistoryRecord_t	*record = NULL;
	PmCompositeImage_t	*cimg = NULL;

	// check input parameters
	if (!pm || !groupName || !pmGroupInfo)
		return(FINVALID_PARAMETER);
	if (groupName[0] == '\0') {
		IB_LOG_WARN_FMT(__func__, "Illegal groupName parameter: Empty String\n");
		return(FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER) ;
	}

	AtomicIncrementVoid(&pm->refCount);	// prevent engine from stopping
	if (! PmEngineRunning()) {	// see if is already stopped/stopping
		status = FUNAVAILABLE;
		goto done;
	}

	// collect statistics from last sweep and populate pmGroupInfo
	(void)vs_rdlock(&pm->stateLock);
	status = FindImage(pm, IMAGEID_TYPE_ANY, imageId, &imageIndex, &retImageId.imageNumber, &record, &msg, NULL, &cimg);

	if (FSUCCESS != status) {
		IB_LOG_WARN_FMT(__func__, "Unable to get index from ImageId: %s: %s", FSTATUS_ToString(status), msg);
		goto error;
	}

	if (record || cimg) {
		sth = 1;
		if (record) {
			// try to load
			status = PmLoadComposite(pm, record, &cimg);
			if (status != FSUCCESS || !cimg) {
				IB_LOG_WARN_FMT(__func__, "Unable to load composite image: %s", FSTATUS_ToString(status));
				goto error;
			}
		}
		// set the return ID
		retImageId.imageNumber = cimg->header.common.imageIDs[0];
		imageInterval = cimg->header.common.imageSweepInterval;
		// composite is loaded, reconstitute so we can use it
		status = PmReconstitute(&pm->ShortTermHistory, cimg);
		if (record) PmFreeComposite(cimg);
		if (status != FSUCCESS) {
			IB_LOG_WARN_FMT(__func__, "Unable to reconstitute composite image: %s", FSTATUS_ToString(status));
			goto error;
		}
		pmImageP = pm->ShortTermHistory.LoadedImage.img;
		// look for the group
		if (!strcmp(pm->ShortTermHistory.LoadedImage.AllGroup->Name, groupName)) {
			pmGroupP = pm->ShortTermHistory.LoadedImage.AllGroup;
			isInternal = isGroupAll = TRUE;
		} else {
			int i;
			for (i = 0; i < PM_MAX_GROUPS; i++) {
				if (!strcmp(pm->ShortTermHistory.LoadedImage.Groups[i]->Name, groupName)) {
					pmGroupP = pm->ShortTermHistory.LoadedImage.Groups[i];
					break;
				}
			}
		}
		if (!pmGroupP) {
			IB_LOG_WARN_FMT(__func__, "Group %.*s not Found", (int)sizeof(groupName), groupName);
			status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_GROUP;
			goto error;
		}
		imageIndex = 0; // STH always uses imageIndex 0
	} else {
		status = LocateGroup(pm, groupName, &pmGroupP);
		if (status != FSUCCESS) {
			IB_LOG_WARN_FMT(__func__, "Group %.*s not Found: %s", (int)sizeof(groupName), groupName, FSTATUS_ToString(status));
			status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_GROUP;
			goto error;
		}
		if (pmGroupP == pm->AllPorts) isInternal = isGroupAll = TRUE;

		pmImageP = &pm->Image[imageIndex];
		imageInterval = MAX(pm->interval, (pmImageP->sweepDuration/1000000));
		(void)vs_rdlock(&pmImageP->imageLock);
	}

	// Grab ImageTime from Pm Image
	retImageId.imageTime.absoluteTime = (uint32)pmImageP->sweepStart;

	(void)vs_rwunlock(&pm->stateLock);
	memset(&pmGroupImage, 0, sizeof(PmGroupImage_t));
	ClearGroupStats(&pmGroupImage);

	for (lid = 1; lid <= pmImageP->maxLid; lid++ ) {
		PmNode_t *pmNodeP = pmImageP->LidMap[lid];
		if (!pmNodeP) continue;
		if (pmNodeP->nodeType == STL_NODE_SW) {
			int p;
			for (p=0; p <= pmNodeP->numPorts; p++) { // Includes port 0
				pmPortP = pmNodeP->up.swPorts[p];
				// if this is a sth image, the port may be 'empty' but not null 
				// 'Empty' ports should be excluded from the count, and can be indentified by their having a port num and guid of 0
				if (!pmPortP || (sth && !pmPortP->guid && !pmPortP->portNum)) continue;

                pmPortImageP = &pmPortP->Image[imageIndex];
                if (PmIsPortInGroup(pm, pmPortP, pmPortImageP, pmGroupP, sth, &isInternal)) {
					if (isGroupAll || isInternal) {
						if (pmPortImageP->u.s.queryStatus != PM_QUERY_STATUS_OK) {
							PA_INC_COUNTER_NO_OVERFLOW(pmGroupImage.IntUtil.pmaNoRespPorts, IB_UINT16_MAX);
						}
						pmGroupImage.NumIntPorts++;
						UpdateInGroupStats(pm, imageIndex, pmPortP, &pmGroupImage, imageInterval);
						if (pmPortImageP->neighbor == NULL && pmPortP->portNum != 0) {
							PA_INC_COUNTER_NO_OVERFLOW(pmGroupImage.IntUtil.topoIncompPorts, IB_UINT16_MAX);
						}
					} else {
						if (pmPortImageP->u.s.queryStatus != PM_QUERY_STATUS_OK) {
							PA_INC_COUNTER_NO_OVERFLOW(pmGroupImage.SendUtil.pmaNoRespPorts, IB_UINT16_MAX);
						}
						pmGroupImage.NumExtPorts++;
						if (pmPortImageP->neighbor == NULL) {
							PA_INC_COUNTER_NO_OVERFLOW(pmGroupImage.RecvUtil.topoIncompPorts, IB_UINT16_MAX);
						} else {
							pmPortImageNeighborP = &pmPortImageP->neighbor->Image[imageIndex];
							if (pmPortImageNeighborP->u.s.queryStatus != PM_QUERY_STATUS_OK) {
								PA_INC_COUNTER_NO_OVERFLOW(pmGroupImage.RecvUtil.pmaNoRespPorts, IB_UINT16_MAX);
							}
						}
						UpdateExtGroupStats(pm, imageIndex, pmPortP, &pmGroupImage, imageInterval);
					}
				}
			}
		} else {
			pmPortP = pmNodeP->up.caPortp;
			if (!pmPortP) continue;
			pmPortImageP = &pmPortP->Image[imageIndex];
			if (PmIsPortInGroup(pm, pmPortP, pmPortImageP, pmGroupP, sth, &isInternal)) {
				if (isGroupAll || isInternal) {
					if (pmPortImageP->u.s.queryStatus != PM_QUERY_STATUS_OK) {
						PA_INC_COUNTER_NO_OVERFLOW(pmGroupImage.IntUtil.pmaNoRespPorts, IB_UINT16_MAX);
					}
					pmGroupImage.NumIntPorts++;
					UpdateInGroupStats(pm, imageIndex, pmPortP, &pmGroupImage, imageInterval);
					if (pmPortImageP->neighbor == NULL && pmPortP->portNum != 0) {
						PA_INC_COUNTER_NO_OVERFLOW(pmGroupImage.IntUtil.topoIncompPorts, IB_UINT16_MAX);
					}
				} else {
					if (pmPortImageP->u.s.queryStatus != PM_QUERY_STATUS_OK) {
						PA_INC_COUNTER_NO_OVERFLOW(pmGroupImage.SendUtil.pmaNoRespPorts, IB_UINT16_MAX);
					}
					pmGroupImage.NumExtPorts++;
					if (pmPortImageP->neighbor == NULL) {
						PA_INC_COUNTER_NO_OVERFLOW(pmGroupImage.RecvUtil.topoIncompPorts, IB_UINT16_MAX);
					} else {
						pmPortImageNeighborP = &pmPortImageP->neighbor->Image[imageIndex];
						if (pmPortImageNeighborP->u.s.queryStatus != PM_QUERY_STATUS_OK) {
							PA_INC_COUNTER_NO_OVERFLOW(pmGroupImage.RecvUtil.pmaNoRespPorts, IB_UINT16_MAX);
						}
					}
					UpdateExtGroupStats(pm, imageIndex, pmPortP, &pmGroupImage, imageInterval);
				}
			}
		}
	}
	FinalizeGroupStats(&pmGroupImage);
	cs_strlcpy(pmGroupInfo->groupName, pmGroupP->Name, STL_PM_GROUPNAMELEN);
	pmGroupInfo->NumIntPorts = pmGroupImage.NumIntPorts;
	pmGroupInfo->NumExtPorts = pmGroupImage.NumExtPorts;
	memcpy(&pmGroupInfo->IntUtil, &pmGroupImage.IntUtil, sizeof(PmUtilStats_t));
	memcpy(&pmGroupInfo->SendUtil, &pmGroupImage.SendUtil, sizeof(PmUtilStats_t));
	memcpy(&pmGroupInfo->RecvUtil, &pmGroupImage.RecvUtil, sizeof(PmUtilStats_t));
	memcpy(&pmGroupInfo->IntErr, &pmGroupImage.IntErr, sizeof(PmErrStats_t));
	memcpy(&pmGroupInfo->ExtErr, &pmGroupImage.ExtErr, sizeof(PmErrStats_t));
	pmGroupInfo->MinIntRate = pmGroupImage.MinIntRate;
	pmGroupInfo->MaxIntRate = pmGroupImage.MaxIntRate;
	pmGroupInfo->MinExtRate = pmGroupImage.MinExtRate;
	pmGroupInfo->MaxExtRate = pmGroupImage.MaxExtRate;

	if (!sth) {
		(void)vs_rwunlock(&pmImageP->imageLock);
	} 

	*returnImageId = retImageId;

done:
	AtomicDecrementVoid(&pm->refCount);
	return(status);
error:
	(void)vs_rwunlock(&pm->stateLock);
	returnImageId->imageNumber = BAD_IMAGE_ID;
	goto done;
}

#define PORTLISTCHUNK	256

/*************************************************************************************
*
* paGetGroupConfig - return group configuration information
*
*  Inputs:
*     pm - pointer to Pm_t (the PM main data type)
*     groupName - pointer to name of group
*     pmGroupConfig - pointer to caller-declared data area to return group config info
*
*  Return:
*     FSTATUS - FSUCCESS if OK
*
*
*************************************************************************************/

FSTATUS paGetGroupConfig(Pm_t *pm, char *groupName, PmGroupConfig_t *pmGroupConfig,
	STL_PA_IMAGE_ID_DATA imageId, STL_PA_IMAGE_ID_DATA *returnImageId)
{
	STL_PA_IMAGE_ID_DATA retImageId = {0};
	PmGroup_t			*pmGroupP = NULL;
	STL_LID_32			lid;
	uint32				imageIndex;
	const char 			*msg;
	PmImage_t			*pmimagep = NULL;
	FSTATUS				status = FSUCCESS;
	boolean				sth = 0;
	PmHistoryRecord_t	*record = NULL;
	PmCompositeImage_t	*cimg = NULL;

	// check input parameters
	if (!pm || !groupName || !pmGroupConfig)
		return(FINVALID_PARAMETER);
	if (groupName[0] == '\0') {
		IB_LOG_WARN_FMT(__func__, "Illegal groupName parameter: Empty String\n");
		return(FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER) ;
	}

	// initialize group config port list counts
	pmGroupConfig->NumPorts = 0;
	pmGroupConfig->portListSize = 0;
	pmGroupConfig->portList = NULL;

	AtomicIncrementVoid(&pm->refCount);	// prevent engine from stopping
	if (! PmEngineRunning()) {	// see if is already stopped/stopping
		status = FUNAVAILABLE;
		goto done;
	}

	// pmGroupP points to our group
	// check all ports for membership in our group
	(void)vs_rdlock(&pm->stateLock);
	status = FindImage(pm, IMAGEID_TYPE_ANY, imageId, &imageIndex, &retImageId.imageNumber, &record, &msg, NULL, &cimg);

	if (FSUCCESS != status) {
		IB_LOG_WARN_FMT(__func__, "Unable to get index from ImageId: %s: %s", FSTATUS_ToString(status), msg);
		goto error;
	}
	
	if (record || cimg) {
		sth = 1;
		if (record) {
			// found the record, try to load it
			status = PmLoadComposite(pm, record, &cimg);
			if (status != FSUCCESS || !cimg) {
				IB_LOG_WARN_FMT(__func__, "Unable to load composite image: %s", FSTATUS_ToString(status));
				goto error;
			}
		}
		retImageId.imageNumber = cimg->header.common.imageIDs[0];
		// composite is loaded, reconstitute so we can use it
		status = PmReconstitute(&pm->ShortTermHistory, cimg);
		if (record) PmFreeComposite(cimg);
		if (status != FSUCCESS) {
			IB_LOG_WARN_FMT(__func__, "Unable to reconstitute composite image: %s", FSTATUS_ToString(status));
			goto error;
		}
		// look for the group
		if (!strcmp(pm->ShortTermHistory.LoadedImage.AllGroup->Name, groupName)) {
			pmGroupP = pm->ShortTermHistory.LoadedImage.AllGroup;
		} else {
			int i;
			for (i = 0; i < PM_MAX_GROUPS; i++) {
				if (!strcmp(pm->ShortTermHistory.LoadedImage.Groups[i]->Name, groupName)) {
					pmGroupP = pm->ShortTermHistory.LoadedImage.Groups[i];
					break;
				}
			}
		}
		if (!pmGroupP) {
			IB_LOG_WARN_FMT(__func__, "Group %.*s not Found", (int)sizeof(groupName), groupName);
			status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_GROUP;
			goto error;
		}
		imageIndex = 0; // STH always uses imageIndex 0
		pmimagep = pm->ShortTermHistory.LoadedImage.img;
	} else {
		status = LocateGroup(pm, groupName, &pmGroupP);
		if (status != FSUCCESS) {
			IB_LOG_WARN_FMT(__func__, "Group %.*s not Found: %s", (int)sizeof(groupName), groupName, FSTATUS_ToString(status));
			status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_GROUP;
			goto error;
		}
		pmimagep = &pm->Image[imageIndex];
		(void)vs_rdlock(&pmimagep->imageLock);
	}

	// Grab ImageTime from Pm Image
	retImageId.imageTime.absoluteTime = (uint32)pmimagep->sweepStart;

	(void)vs_rwunlock(&pm->stateLock);
	for (lid=1; lid<= pmimagep->maxLid; ++lid) {
		uint8 portnum;
		PmNode_t *pmnodep = pmimagep->LidMap[lid];
		if (! pmnodep)
			continue;
		if (pmnodep->nodeType == STL_NODE_SW) {
			for (portnum=0; portnum<=pmnodep->numPorts; ++portnum) {
				PmPort_t *pmportp = pmnodep->up.swPorts[portnum];
				PmPortImage_t *portImage;
				if (! pmportp)
					continue;
				portImage = &pmportp->Image[imageIndex];
				if (PmIsPortInGroup(pm, pmportp, portImage, pmGroupP, sth, NULL))
				{
					if (pmGroupConfig->portListSize == pmGroupConfig->NumPorts) {
						pmGroupConfig->portListSize += PORTLISTCHUNK;
					}
					pmGroupConfig->NumPorts++;
				}
			}
		} else {
			PmPort_t *pmportp = pmnodep->up.caPortp;
			PmPortImage_t *portImage = &pmportp->Image[imageIndex];
			if (PmIsPortInGroup(pm, pmportp, portImage, pmGroupP, sth, NULL))
			{
				if (pmGroupConfig->portListSize == pmGroupConfig->NumPorts) {
					pmGroupConfig->portListSize += PORTLISTCHUNK;
				}
				pmGroupConfig->NumPorts++;
			}
		}
	}
	// check if there are ports to sort
	if (!pmGroupConfig->NumPorts) {
		IB_LOG_INFO_FMT(__func__, "Group %.*s has no ports", (int)sizeof(groupName), groupName);
		goto norecords;
	}
	// allocate the port list
	Status_t ret = vs_pool_alloc(&pm_pool, pmGroupConfig->portListSize * sizeof(PmPortConfig_t), (void *)&pmGroupConfig->portList);
	if (ret != VSTATUS_OK) {
		if (!sth) (void)vs_rwunlock(&pmimagep->imageLock);
		status = FINSUFFICIENT_MEMORY;
		IB_LOG_ERRORRC("Failed to allocate port list buffer for pmGroupConfig rc:", ret);
		goto done;
	}
	// copy the port list
	int i = 0;
	for (lid=1; lid <= pmimagep->maxLid; ++lid) {
		uint8 portnum;
		PmNode_t *pmnodep = pmimagep->LidMap[lid];
		if (!pmnodep) 
			continue;
		if (pmnodep->nodeType == STL_NODE_SW) {
			for (portnum=0; portnum <= pmnodep->numPorts; ++portnum) {
				PmPort_t *pmportp = pmnodep->up.swPorts[portnum];
				PmPortImage_t *portImage;
				if (!pmportp) 
					continue;
				portImage = &pmportp->Image[imageIndex];
				if (PmIsPortInGroup(pm, pmportp, portImage, pmGroupP, sth, NULL)) {
					pmGroupConfig->portList[i].lid = lid;
					pmGroupConfig->portList[i].portNum = pmportp->portNum;
					pmGroupConfig->portList[i].guid = pmnodep->guid;
					memcpy(pmGroupConfig->portList[i].nodeDesc, (char *)pmnodep->nodeDesc.NodeString,
						   sizeof(pmGroupConfig->portList[i].nodeDesc));
					i++;
				}
			}
		} else {
			PmPort_t *pmportp = pmnodep->up.caPortp;
			PmPortImage_t *portImage = &pmportp->Image[imageIndex];
			if (PmIsPortInGroup(pm, pmportp, portImage, pmGroupP, sth, NULL)) {
				pmGroupConfig->portList[i].lid = lid;
				pmGroupConfig->portList[i].portNum = pmportp->portNum;
				pmGroupConfig->portList[i].guid = pmnodep->guid;
				memcpy(pmGroupConfig->portList[i].nodeDesc, (char *)pmnodep->nodeDesc.NodeString,
					   sizeof(pmGroupConfig->portList[i].nodeDesc));
				i++;
			}
		}
	}
norecords:
	cs_strlcpy(pmGroupConfig->groupName, pmGroupP->Name, STL_PM_GROUPNAMELEN);
	*returnImageId = retImageId;

	(void)vs_rwunlock(&pmimagep->imageLock);
done:
	if (sth)  {
#ifndef __VXWORKS__
		clearLoadedImage(&pm->ShortTermHistory);
#endif
	}
	AtomicDecrementVoid(&pm->refCount);
	return(status);
error:
	(void)vs_rwunlock(&pm->stateLock);
	returnImageId->imageNumber = BAD_IMAGE_ID;
	goto done;
}

/*************************************************************************************
*
* paGetPortStats - return port statistics
*			 Get Running totals for a Port.  This simulates a PMA get so
*			 that tools like opareport can work against the Running totals
*			 until we have a history feature.
*
*  Inputs:
*     pm - pointer to Pm_t (the PM main data type)
*     lid, portNum - lid and portNum to select port to get
*     portCounterP - pointer to Consolidated Port Counters data area
*     delta - 1 for delta counters, 0 for running counters
*
*  Return:
*     FSTATUS - FSUCCESS if OK
*
*
*************************************************************************************/

FSTATUS paGetPortStats(Pm_t *pm, STL_LID_32 lid, uint8 portNum, PmCompositePortCounters_t *portCountersP,
	uint32 delta, uint32 userCntrs, STL_PA_IMAGE_ID_DATA imageId, uint32 *flagsp,
	STL_PA_IMAGE_ID_DATA *returnImageId)
{
	STL_PA_IMAGE_ID_DATA retImageId = {0};
	FSTATUS				status = FSUCCESS;
	PmPort_t			*pmPortP;
	PmPortImage_t		*pmPortImageP;
	const char 			*msg;
	PmImage_t			*pmImageP;
	uint32				imageIndex = PM_IMAGE_INDEX_INVALID;
	boolean				sth = 0;
	PmHistoryRecord_t	*record = NULL;
	PmCompositeImage_t	*cimg = NULL;

	// check input parameters
	if (!pm || !portCountersP || !flagsp)
		return (FINVALID_PARAMETER);
	if (!lid) {
		IB_LOG_WARN_FMT(__func__,  "Illegal LID parameter: must not be zero");
		return (FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER);
	}
	if (userCntrs && (delta || imageId.imageOffset)) {
		IB_LOG_WARN_FMT(__func__,  "Illegal combination of parameters: Offset (%d) and delta(%d) must be zero if UserCounters(%d) flag is set",
			imageId.imageOffset, delta, userCntrs);
		return (FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER);
	}

	AtomicIncrementVoid(&pm->refCount); // prevent engine from stopping
	if (!PmEngineRunning()) {  // see if is already stopped/stopping
		status = FUNAVAILABLE;
		goto done;
	}

	(void)vs_rdlock(&pm->stateLock);
	if (userCntrs) {
		STL_PA_IMAGE_ID_DATA liveImgId = { 0 };
		status = FindImage(pm, IMAGEID_TYPE_ANY, liveImgId, &imageIndex, &retImageId.imageNumber, &record, &msg, NULL, &cimg);
	} else {
		status = FindImage(pm, IMAGEID_TYPE_ANY, imageId, &imageIndex, &retImageId.imageNumber, &record, &msg, NULL, &cimg);
	}
	if (FSUCCESS != status) {
		IB_LOG_WARN_FMT(__func__, "Unable to get index from ImageId: %s: %s", FSTATUS_ToString(status), msg);
		goto error;
	}

	if (record || cimg) {
		sth = 1;
		if (record) {
			status = PmLoadComposite(pm, record, &cimg);
			if (status != FSUCCESS || !cimg) {
				IB_LOG_WARN_FMT(__func__, "Unable to load composite image: %s", FSTATUS_ToString(status));
				goto error;
			}
		}
		retImageId.imageNumber = cimg->header.common.imageIDs[0];
		status = PmReconstitute(&pm->ShortTermHistory, cimg);
		if (record) PmFreeComposite(cimg);
		if (status != FSUCCESS) {
			IB_LOG_WARN_FMT(__func__, "Unable to reconstitute composite image: %s", FSTATUS_ToString(status));
			goto error;
		}

		pmImageP = pm->ShortTermHistory.LoadedImage.img;
		imageIndex = 0;
	} else {
		pmImageP = &pm->Image[imageIndex];
		(void)vs_rdlock(&pmImageP->imageLock);
	}

	pmPortP = pm_find_port(pmImageP, lid, portNum);
	if (!pmPortP) {
		IB_LOG_WARN_FMT(__func__, "Port not found: Lid 0x%x Port %u", lid, portNum);
		status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_PORT;
		goto unlock;
	}
	pmPortImageP = &pmPortP->Image[imageIndex];
	if (pmPortImageP->u.s.queryStatus != PM_QUERY_STATUS_OK) {
		IB_LOG_WARN_FMT(__func__, "Port Query Status Invalid: %s: Lid 0x%x Port %u",
			(pmPortImageP->u.s.queryStatus == PM_QUERY_STATUS_SKIP ? "Skipped" :
			(pmPortImageP->u.s.queryStatus == PM_QUERY_STATUS_FAIL_QUERY ? "Failed Query" : "Failed Clear")),
			lid, portNum);
		status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_IMAGE;
		goto unlock;
	}

	if (userCntrs) {
		(void)vs_rdlock(&pm->totalsLock);
		//*portCountersP = pmPortP->StlPortCountersTotal;
		memcpy(portCountersP, &pmPortP->StlPortCountersTotal, sizeof(PmCompositePortCounters_t));
		(void)vs_rwunlock(&pm->totalsLock);

		*flagsp |= STL_PA_PC_FLAG_USER_COUNTERS |
			(isUnexpectedClearUserCounters ? STL_PA_PC_FLAG_UNEXPECTED_CLEAR : 0);
		*returnImageId = (STL_PA_IMAGE_ID_DATA){0};
	} else {
		// Grab ImageTime from Pm Image
		retImageId.imageTime.absoluteTime = (uint32)pmImageP->sweepStart;

		if (delta) {
			//*portCountersP = pmPortImageP->DeltaStlPortCounters;
			memcpy(portCountersP, &pmPortImageP->DeltaStlPortCounters, sizeof(PmCompositePortCounters_t));
			*flagsp |= STL_PA_PC_FLAG_DELTA;
		} else {
			//*portCountersP = pmPortImageP->StlPortCounters;
			memcpy(portCountersP, &pmPortImageP->StlPortCounters, sizeof(PmCompositePortCounters_t));
		}
		*flagsp |= (pmPortImageP->u.s.UnexpectedClear ? STL_PA_PC_FLAG_UNEXPECTED_CLEAR : 0);
		*returnImageId = retImageId;
	}

	if (status == FSUCCESS) {
		(void)vs_rwunlock(&pm->stateLock);
	}
unlock:
	if (!sth) {
		(void)vs_rwunlock(&pmImageP->imageLock);
	}
	if (status != FSUCCESS){
		goto error;
	}
done:
	AtomicDecrementVoid(&pm->refCount);
	return(status);
error:
	(void)vs_rwunlock(&pm->stateLock);
	returnImageId->imageNumber = BAD_IMAGE_ID;
	goto done;
}

/*************************************************************************************
*
* paClearPortStats - Clear port statistics for a port
*			 Clear Running totals for a port.  This simulates a PMA clear so
*			 that tools like opareport can work against the Running totals
*			 until we have a history feature.
*
*  Inputs:
*     pm - pointer to Pm_t (the PM main data type)
*     lid, portNum - lid and portNum to select port to clear
*     select - selects counters to clear
*
*  Return:
*     FSTATUS - FSUCCESS if OK
*
*
*************************************************************************************/

FSTATUS paClearPortStats(Pm_t *pm, STL_LID_32 lid, uint8 portNum, CounterSelectMask_t select)
{
	FSTATUS				status = FSUCCESS;
	PmImage_t			*pmimagep;
	uint32				imageIndex;

	// check input parameters
	if (!pm)
		return(FINVALID_PARAMETER);
	if (!lid) {
		IB_LOG_WARN_FMT(__func__,  "Illegal LID parameter: must not be zero");
		return(FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER);
	}
	if (!select.AsReg32) {
		IB_LOG_WARN_FMT(__func__, "Illegal select parameter: Must not be zero\n");
		return(FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER);
	}

	AtomicIncrementVoid(&pm->refCount);	// prevent engine from stopping
	if (! PmEngineRunning()) {	// see if is already stopped/stopping
		status = FUNAVAILABLE;
		goto done;
	}

	(void)vs_rdlock(&pm->stateLock);
	// Live Data only, no offset
	imageIndex = pm->LastSweepIndex;
	if (pm->LastSweepIndex == PM_IMAGE_INDEX_INVALID) {
		IB_LOG_WARN_FMT(__func__, "Unable to Clear PortStats: PM has not completed a sweep.");
		(void)vs_rwunlock(&pm->stateLock);
		status = FUNAVAILABLE | STL_MAD_STATUS_STL_PA_UNAVAILABLE;
		goto done;
	}
	pmimagep = &pm->Image[imageIndex];
	(void)vs_rdlock(&pmimagep->imageLock);
	(void)vs_rwunlock(&pm->stateLock);

	(void)vs_wrlock(&pm->totalsLock);
	if (portNum == PM_ALL_PORT_SELECT) {
		PmNode_t *pmnodep = pm_find_node(pmimagep, lid);
		if (! pmnodep) {
			IB_LOG_WARN_FMT(__func__, "Switch not found: LID: 0x%x", lid);
			status = FNOT_FOUND;
		} else if (pmnodep->nodeType != STL_NODE_SW) {
			IB_LOG_WARN_FMT(__func__, "Illegal port parameter: All Port Select (0xFF) can only be used on switches: LID: 0x%x", lid);
			status = FNOT_FOUND;
		} else {
			status = PmClearNodeRunningCounters(pmnodep, select);
			if (IB_EXPECT_FALSE(status != FSUCCESS)) {
				IB_LOG_WARN_FMT(__func__, "Unable to Clear Counters on LID: 0x%x", lid);
			}
		}
	} else {
		PmPort_t	*pmportp = pm_find_port(pmimagep, lid, portNum);
		if (! pmportp) {
			IB_LOG_WARN_FMT(__func__, "Port not found LID 0x%x Port %u", lid, portNum);
			status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_PORT;
		} else {
			status = PmClearPortRunningCounters(pmportp, select);
			if (IB_EXPECT_FALSE(status != FSUCCESS)) {
				IB_LOG_WARN_FMT(__func__, "Unable to Clear Counters on LID: 0x%x  Port: %u", lid, portNum);
			}
		}
	}
	(void)vs_rwunlock(&pm->totalsLock);

	(void)vs_rwunlock(&pmimagep->imageLock);
done:
	AtomicDecrementVoid(&pm->refCount);
	return(status);
}

/*************************************************************************************
*
* paClearAllPortStats - Clear port statistics for a port
*			 Clear Running totals for all Ports.  This simulates a PMA clear so
*			 that tools like opareport can work against the Running totals
*			 until we have a history feature.
*
*  Inputs:
*     pm - pointer to Pm_t (the PM main data type)
*     lid, portNum - lid and portNum to select port to clear
*     select - selects counters to clear
*
*  Return:
*     FSTATUS - FSUCCESS if OK
*
*
*************************************************************************************/

FSTATUS paClearAllPortStats(Pm_t *pm, CounterSelectMask_t select)
{
	STL_LID_32 lid;
	FSTATUS status = FSUCCESS;
	PmImage_t			*pmimagep;
	uint32				imageIndex;

	// check input parameters
	if (!pm)
		return(FINVALID_PARAMETER);
	if (!select.AsReg32) {
		IB_LOG_WARN_FMT(__func__, "Illegal select parameter: Must not be zero\n");
		return(FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER);
	}
	AtomicIncrementVoid(&pm->refCount);	// prevent engine from stopping
	if (! PmEngineRunning()) {	// see if is already stopped/stopping
		status = FUNAVAILABLE;
		goto done;
	}

	(void)vs_rdlock(&pm->stateLock);
	// Live Data only, no offset
	imageIndex = pm->LastSweepIndex;

	if (pm->LastSweepIndex == PM_IMAGE_INDEX_INVALID) {
		IB_LOG_WARN_FMT(__func__, "Unable to Clear PortStats: PM has not completed a sweep.");
		status = FUNAVAILABLE | STL_MAD_STATUS_STL_PA_UNAVAILABLE;
		(void)vs_rwunlock(&pm->stateLock);
		goto done;
	}
	pmimagep = &pm->Image[imageIndex];
	(void)vs_rdlock(&pmimagep->imageLock);
	(void)vs_rwunlock(&pm->stateLock);
	(void)vs_wrlock(&pm->totalsLock);
	for (lid=1; lid<= pmimagep->maxLid; ++lid) {
		PmNode_t *pmnodep = pmimagep->LidMap[lid];
		if (! pmnodep)
			continue;
		if (pmnodep->nodeType == STL_NODE_SW) {
			uint8 i;
			for (i=0; i<= pmnodep->numPorts; ++i) {
				PmPort_t *pmportp = pmnodep->up.swPorts[i];
				if (! pmportp)
					continue;
				status = PmClearPortRunningCounters(pmportp, select);
				if (status != FSUCCESS)
					IB_LOG_WARN_FMT(__func__,"Failed to Clear Counters on LID: 0x%x  Port: %u", lid, i);
			}
		} else {
			status = PmClearPortRunningCounters(pmnodep->up.caPortp, select);
			if (status != FSUCCESS)
				IB_LOG_WARN_FMT(__func__,"Failed to Clear Counters on LID: 0x%x", lid);
		}
	}
	(void)vs_rwunlock(&pm->totalsLock);
	(void)vs_rwunlock(&pmimagep->imageLock);
done:
	AtomicDecrementVoid(&pm->refCount);
	return status;
}

/*************************************************************************************
*
* paFreezeFrameRenew - access FF to renew lease
*
*  Inputs:
*     pm - pointer to Pm_t (the PM main data type)
*     imageId - 64 bit opaque imageId
*
*  Return:
*     FSTATUS - FSUCCESS if OK
*
*************************************************************************************/

FSTATUS paFreezeFrameRenew(Pm_t *pm, STL_PA_IMAGE_ID_DATA *imageId)
{
	FSTATUS				status = FSUCCESS;
	uint32				imageIndex = PM_IMAGE_INDEX_INVALID;
	const char 			*msg;
	STL_PA_IMAGE_ID_DATA retImageId = {0};
	PmHistoryRecord_t	*record = NULL;
	PmCompositeImage_t	*cimg = NULL;

	// check input parameters
	if (!pm)
		return(FINVALID_PARAMETER);

	AtomicIncrementVoid(&pm->refCount);	// prevent engine from stopping
	if (! PmEngineRunning()) {	// see if is already stopped/stopping
		status = FUNAVAILABLE;
		goto done;
	}

	(void)vs_rdlock(&pm->stateLock);
	// just touching it will renew the image
	status = FindImage(pm, IMAGEID_TYPE_FREEZE_FRAME, *imageId, &imageIndex, &retImageId.imageNumber, &record, &msg, NULL, &cimg);
	if (FSUCCESS != status) {
		IB_LOG_WARN_FMT(__func__, "Unable to get index from ImageId: %s: %s", FSTATUS_ToString(status), msg);
	}
	
	if (record || (cimg && cimg !=  pm->ShortTermHistory.cachedComposite)) // not in the cache, never found
			IB_LOG_WARN_FMT(__func__, "Unable to access cached composite image: %s", msg);

	if (cimg && cimg == pm->ShortTermHistory.cachedComposite) {
		imageId->imageTime.absoluteTime = (uint32)cimg->sweepStart;
	} else if (imageIndex != PM_IMAGE_INDEX_INVALID) {
		imageId->imageTime.absoluteTime = (uint32)pm->Image[imageIndex].sweepStart;
	}
	(void)vs_rwunlock(&pm->stateLock);
done:
	AtomicDecrementVoid(&pm->refCount);
	return(status);
}

/*************************************************************************************
*
* paFreezeFrameRelease - release FreezeFrame
*
*  Inputs:
*     pm - pointer to Pm_t (the PM main data type)
*     imageId - 64 bit opaque imageId
*
*  Return:
*     FSTATUS - FSUCCESS if OK
*
*************************************************************************************/

FSTATUS paFreezeFrameRelease(Pm_t *pm, STL_PA_IMAGE_ID_DATA *imageId)
{
	FSTATUS				status = FSUCCESS;
	uint32				imageIndex;
	const char 			*msg;
	STL_PA_IMAGE_ID_DATA retImageId = {0};
	uint8				clientId;
	PmHistoryRecord_t	*record = NULL;
	PmCompositeImage_t	*cimg = NULL;

	// check input parameters
	if (!pm)
		return(FINVALID_PARAMETER);

	AtomicIncrementVoid(&pm->refCount);	// prevent engine from stopping
	if (! PmEngineRunning()) {	// see if is already stopped/stopping
		status = FUNAVAILABLE;
		goto done;
	}

	(void)vs_wrlock(&pm->stateLock);
	status = FindImage(pm, IMAGEID_TYPE_FREEZE_FRAME, *imageId, &imageIndex, &retImageId.imageNumber, &record, &msg, &clientId, &cimg);
	if (FSUCCESS != status) {
		IB_LOG_WARN_FMT(__func__, "Unable to get index from ImageId: %s: %s", FSTATUS_ToString(status), msg);
		goto unlock;
	}

	if (record || cimg) {
		if (cimg && cimg == pm->ShortTermHistory.cachedComposite) {
			imageId->imageTime.absoluteTime = (uint32)cimg->sweepStart;
			PmFreeComposite(pm->ShortTermHistory.cachedComposite);
			pm->ShortTermHistory.cachedComposite = NULL;
			status = FSUCCESS;
		} else {
			IB_LOG_WARN_FMT(__func__, "Unable to find freeze frame: %s", msg);
			status = FINVALID_PARAMETER;
		}
		goto unlock;
	} else if (!(pm->Image[imageIndex].ffRefCount & ((uint64)1<<(uint64)clientId)) ) { /*not frozen*/
		IB_LOG_ERROR_FMT(__func__, "Attempted to release freeze frame with no references");
		status = FINVALID_PARAMETER;
		goto unlock;
	}
	imageId->imageTime.absoluteTime = (uint32)pm->Image[imageIndex].sweepStart;
	pm->Image[imageIndex].ffRefCount &= ~((uint64)1<<(uint64)clientId);	// release image

unlock:
	(void)vs_rwunlock(&pm->stateLock);
done:
	AtomicDecrementVoid(&pm->refCount);
	return(status);
}

// Find a Freeze Frame Slot
// returns a valid index, on error returns pm_config.freeze_frame_images
static uint32 allocFreezeFrame(Pm_t *pm, uint32 imageIndex)
{
	uint8				freezeIndex;
	if (pm->Image[imageIndex].ffRefCount) {
		// there are other references
		// see if we can find a freezeFrame which already points to this image
		for (freezeIndex = 0; freezeIndex < pm_config.freeze_frame_images; freezeIndex ++) {
			if (pm->freezeFrames[freezeIndex] == imageIndex)
				return freezeIndex;	// points to index we want
		}
	}
	// need to find an empty Freeze Frame Slot
	for (freezeIndex = 0; freezeIndex < pm_config.freeze_frame_images; freezeIndex ++) {
		if (pm->freezeFrames[freezeIndex] == PM_IMAGE_INDEX_INVALID
			|| pm->Image[pm->freezeFrames[freezeIndex]].ffRefCount == 0) {
			// empty or stale FF image
			pm->freezeFrames[freezeIndex] = PM_IMAGE_INDEX_INVALID;
			return freezeIndex;
		}
	}
	return freezeIndex;	// >= pm_config.freeze_frame_images means failure
}

// get clientId which should be used to create a new freezeFrame for imageIndex
// a return >=64 indicates none available
static uint8 getNextClientId(Pm_t *pm, uint32 imageIndex)
{
	uint8				i;
	uint8				clientId;
	// to avoid a unfreeze/freeze against same imageIndex getting same clientId
	// we have a rolling count (eg. starting point) per Image
	for (i = 0; i < 64; i++) {
		clientId = (pm->Image[imageIndex].nextClientId+i)&63;
		if (0 == (pm->Image[imageIndex].ffRefCount & ((uint64)1 << (uint64)clientId))) {
			pm->Image[imageIndex].ffRefCount |= ((uint64)1 << (uint64)clientId);
			return clientId;
		}
	}
	return 255;	// none available
}

/*************************************************************************************
*
* paFreezeFrameCreate - create FreezeFrame
*
*  Inputs:
*     pm - pointer to Pm_t (the PM main data type)
*     imageId - 64 bit opaque imageId
*
*  Return:
*     FSTATUS - FSUCCESS if OK
*
*************************************************************************************/

FSTATUS paFreezeFrameCreate(Pm_t *pm, STL_PA_IMAGE_ID_DATA imageId, STL_PA_IMAGE_ID_DATA *retImageId)
{
	FSTATUS				status = FSUCCESS;
	uint32				imageIndex;
	const char 			*msg;
	uint8				clientId;
	uint8				freezeIndex;
	PmHistoryRecord_t	*record = NULL;
	PmCompositeImage_t	*cimg = NULL;

	// check input parameters
	if (!pm)
		return(FINVALID_PARAMETER);

	AtomicIncrementVoid(&pm->refCount);	// prevent engine from stopping
	if (! PmEngineRunning()) {	// see if is already stopped/stopping
		status = FUNAVAILABLE;
		goto done;
	}
	(void)vs_wrlock(&pm->stateLock);
	status = FindImage(pm, IMAGEID_TYPE_ANY, imageId, &imageIndex, &retImageId->imageNumber, &record, &msg, NULL, &cimg);
	if (FSUCCESS != status) {
		IB_LOG_WARN_FMT(__func__, "Unable to get index from ImageId: %s: %s", FSTATUS_ToString(status), msg);
		goto error;
	}
	if (record || cimg) {
		retImageId->imageNumber = imageId.imageNumber;
		if (record) { // cache the disk image
			status = PmFreezeComposite(pm, record);
			if (status != FSUCCESS) {
				IB_LOG_WARN_FMT(__func__, "Unable to freeze composite image: %s", FSTATUS_ToString(status));
				goto error;
			}
			retImageId->imageNumber = record->header.imageIDs[0];
		} else if (cimg && pm->ShortTermHistory.cachedComposite) { // already frozen
			retImageId->imageNumber = pm->ShortTermHistory.cachedComposite->header.common.imageIDs[0];
		} else { // trying to freeze the current composite (unlikely but possible)
			status = PmFreezeCurrent(pm);
			if (status != FSUCCESS) {
				IB_LOG_WARN_FMT(__func__, "Unable to freeze current composite image: %s", FSTATUS_ToString(status));
				goto error;
			}
		}
		retImageId->imageTime.absoluteTime = (uint32)pm->ShortTermHistory.cachedComposite->sweepStart;
		(void)vs_rwunlock(&pm->stateLock);
		goto done;
	}
	
	// Find a Freeze Frame Slot
	freezeIndex = allocFreezeFrame(pm, imageIndex);
	if (freezeIndex >= pm_config.freeze_frame_images) {
		IB_LOG_WARN0( "Out of Freeze Frame Images");
		status = FINSUFFICIENT_MEMORY | STL_MAD_STATUS_STL_PA_NO_IMAGE;
		goto error;
	}
	clientId = getNextClientId(pm, imageIndex);
	if (clientId >= 64) {
		IB_LOG_WARN0( "Too many freezes of 1 image");
		status = FINSUFFICIENT_MEMORY | STL_MAD_STATUS_STL_PA_NO_IMAGE;
		goto error;
	}
	pm->Image[imageIndex].nextClientId = (clientId+1) & 63;
	pm->freezeFrames[freezeIndex] = imageIndex;
	retImageId->imageNumber = BuildFreezeFrameImageId(pm, freezeIndex, clientId, &retImageId->imageTime.absoluteTime);
	(void)vs_rwunlock(&pm->stateLock);

done:
	AtomicDecrementVoid(&pm->refCount);
	return(status);
error:
	(void)vs_rwunlock(&pm->stateLock);
	retImageId->imageNumber = BAD_IMAGE_ID;
	goto done;
}

/*************************************************************************************
*
* paFreezeFrameMove - atomically release and create a  FreezeFrame
*
*  Inputs:
*     pm - pointer to Pm_t (the PM main data type)
*     ffImageId - imageId containing information for FF to release (number)
*     imageId - imageId containing information for image to FreezeFrame (number, offset)
*
*  Return:
*     FSTATUS - FSUCCESS if OK
*     on error the ffImageId is not released
*
*************************************************************************************/

FSTATUS paFreezeFrameMove(Pm_t *pm, STL_PA_IMAGE_ID_DATA ffImageId, STL_PA_IMAGE_ID_DATA imageId,
	STL_PA_IMAGE_ID_DATA *returnImageId)
{
	FSTATUS				status = FSUCCESS;
	uint32				ffImageIndex = 0;
	uint8				ffClientId = 0;
	uint32				imageIndex = 0;
	uint8				clientId = 0;
	const char 			*msg;
	uint8				freezeIndex = 0;
	boolean				oldIsComp = 0, newIsComp = 0;
	PmHistoryRecord_t	*record = NULL;
	PmCompositeImage_t	*cimg = NULL;

	// check input parameters
	if (!pm)
		return(FINVALID_PARAMETER);

	AtomicIncrementVoid(&pm->refCount);	// prevent engine from stopping
	if (! PmEngineRunning()) {	// see if is already stopped/stopping
		status = FUNAVAILABLE;
		goto done;
	}

	(void)vs_wrlock(&pm->stateLock);
	status = FindImage(pm, IMAGEID_TYPE_FREEZE_FRAME, ffImageId, &ffImageIndex, &returnImageId->imageNumber, &record, &msg, &ffClientId, &cimg);
	if (FSUCCESS != status) {
		IB_LOG_WARN_FMT(__func__, "Unable to get freeze frame index from ImageId: %s: %s", FSTATUS_ToString(status), msg);
		goto error;
	}
	if (cimg && cimg == pm->ShortTermHistory.cachedComposite) {
		oldIsComp = 1;
	} else if (cimg && cimg != pm->ShortTermHistory.cachedComposite) {
		IB_LOG_WARN_FMT(__func__, "Unable to find freeze frame for ImageID: %s", msg);
		status = FINVALID_PARAMETER;
		goto error;
	}
	
	record = NULL;
	cimg = NULL;
	status = FindImage(pm, IMAGEID_TYPE_ANY, imageId, &imageIndex, &returnImageId->imageNumber, &record, &msg, NULL, &cimg);
	if (FSUCCESS != status) {
		IB_LOG_WARN_FMT(__func__, "Unable to get index from ImageId: %s: %s", FSTATUS_ToString(status), msg);
		goto error;
	}
	if (record || cimg) {
		newIsComp = 1;
		if (record) { //composite isn't already cached
			status = PmFreezeComposite(pm, record);
			if (status != FSUCCESS) {
				IB_LOG_WARN_FMT(__func__, "Unable to freeze composite image: %s", FSTATUS_ToString(status));
				goto error;
			}
		} else {
			status = PmFreezeCurrent(pm);
			if (status != FSUCCESS) {
				IB_LOG_WARN_FMT(__func__, "Unable to freeze current composite image: %s", FSTATUS_ToString(status));
				goto error;
			}
		}
	}

	if (!newIsComp) {
		if (!oldIsComp && pm->Image[imageIndex].ffRefCount == ((uint64)1 << (uint64)ffClientId)) {
			// we are last/only client using this image
			// so we can simply use the same Freeze Frame Slot as old freeze
			ImageId_t id;
			id.AsReg64 = ffImageId.imageNumber;
			freezeIndex = id.s.index;
		} else {
			// Find a empty Freeze Frame Slot
			freezeIndex = allocFreezeFrame(pm, imageIndex);
		}
		clientId = getNextClientId(pm, imageIndex);
		if (clientId >= 64) {
			IB_LOG_WARN0( "Too many freezes of 1 image");
			status = FINSUFFICIENT_MEMORY | STL_MAD_STATUS_STL_PA_NO_IMAGE;
			goto error;
		}
		// freeze the selected image
		pm->Image[imageIndex].nextClientId = (clientId+1) & 63;
		pm->freezeFrames[freezeIndex] = imageIndex;

		returnImageId->imageNumber = BuildFreezeFrameImageId(pm, freezeIndex, clientId,
			&returnImageId->imageTime.absoluteTime);
	} else {
		returnImageId->imageNumber = pm->ShortTermHistory.cachedComposite->header.common.imageIDs[0];
		returnImageId->imageTime.absoluteTime = (uint32)pm->ShortTermHistory.cachedComposite->sweepStart;
	}
	if (!oldIsComp) pm->Image[ffImageIndex].ffRefCount &= ~((uint64)1 << (uint64)ffClientId); // release old image
	(void)vs_rwunlock(&pm->stateLock);
done:
	AtomicDecrementVoid(&pm->refCount);
	return(status);
error:
	(void)vs_rwunlock(&pm->stateLock);
	returnImageId->imageNumber = BAD_IMAGE_ID;
	goto done;

}

typedef uint32 (*ComputeFunc_t)(Pm_t *pm, uint32 imageIndex, PmPort_t *port, void *data);
typedef uint32 (*CompareFunc_t)(uint64 value1, uint64 value2);

uint32 compareGE(uint64 value1, uint64 value2) { return(value1 >= value2); }
uint32 compareLE(uint64 value1, uint64 value2) { return(value1 <= value2); }
uint32 compareGT(uint64 value1, uint64 value2) { return(value1 > value2); }
uint32 compareLT(uint64 value1, uint64 value2) { return(value1 < value2); }
int neighborInList(STL_LID_32 lid, uint8 portNum, uint32 imageIndex,
	PmPort_t *pmNeighborportp, PmPortImage_t *neighborPortImage,
	sortInfo_t *sortInfo)
{
	sortedValueEntry_t	*listp;
	STL_LID_32			neighborLid;
	uint8				neighborPort;

	neighborPort = pmNeighborportp->portNum;
	neighborLid = pmNeighborportp->pmnodep->Image[imageIndex].lid;

	for (listp = sortInfo->sortedValueListHead; listp != NULL; listp = listp->next) {
		if (listp->neighborPortp &&
			(lid == listp->neighborPortp->pmnodep->Image[imageIndex].lid) &&
			(portNum == listp->neighborPortp->portNum)) {
			IB_LOG_DEBUG2_FMT(__func__, "Lid 0x%x portNum %d neighborLid 0x%x neighborPort %d already in list\n",
				lid, portNum, neighborLid, neighborPort);
			return(1);
		}
	}

	return(0);
}

FSTATUS processFocusPort(Pm_t *pm, PmPort_t *pmportp, PmPortImage_t *portImage,
	uint32 imageIndex, STL_LID_32 lid, uint8 portNum, ComputeFunc_t computeFunc, 
	CompareFunc_t compareFunc, CompareFunc_t candidateFunc, void *computeData, sortInfo_t *sortInfo)
{
	uint64				computedValue = 0;
	uint64				nbrComputedValue = 0;
	uint64				sortValue;
	uint8				localStatus = STL_PA_FOCUS_STATUS_OK;
	uint8				neighborStatus = STL_PA_FOCUS_STATUS_OK;
	PmPort_t			*nbrPt = NULL;
	PmPortImage_t		*nbrPI = NULL;
	sortedValueEntry_t	*newListEntry = NULL;
	sortedValueEntry_t	*thisListEntry = NULL;
	sortedValueEntry_t	*prevListEntry = NULL;

	if (portNum == 0) {
		// SW Port zero has no neighbor
		neighborStatus = STL_PA_FOCUS_STATUS_OK;
	} else {
		nbrPt = portImage->neighbor;
		if (nbrPt == NULL) {
			// Neighbor should never be NULL unless there was a failure during 
			// the PM's Copy of the SM Topology
			neighborStatus = STL_PA_FOCUS_STATUS_TOPO_FAILURE;
		} else {
			nbrPI = &nbrPt->Image[imageIndex];
			if (neighborInList(lid, portNum, imageIndex, nbrPt, nbrPI, sortInfo) == 1)
				return (FSUCCESS);
			if (nbrPt->u.s.PmaAvoid) {
				// This means the PM was told to ignore this port during a sweep
				neighborStatus = STL_PA_FOCUS_STATUS_PMA_IGNORE;
			} else if (nbrPI->u.s.queryStatus != PM_QUERY_STATUS_OK) {
				// This means there was a failure during the PM sweep when 
				// querying this port
				neighborStatus = STL_PA_FOCUS_STATUS_PMA_FAILURE;
			}
			nbrComputedValue = computeFunc(pm, imageIndex, nbrPt, computeData);
		}
	}

	if (pmportp->u.s.PmaAvoid) {
		// This means the PM was told to ignore this port during a sweep
		localStatus = STL_PA_FOCUS_STATUS_PMA_IGNORE;
	} else if (portImage->u.s.queryStatus != PM_QUERY_STATUS_OK) {
		// This means there was a failure during the PM sweep when querying 
		// this port
		localStatus = STL_PA_FOCUS_STATUS_PMA_FAILURE;
	}
	computedValue = computeFunc(pm, imageIndex, pmportp, computeData);

	sortValue = MAX(computedValue, nbrComputedValue); 

	if (sortInfo->sortedValueListHead == NULL) {
		// list is new - add entry and make it head and tail
		sortInfo->sortedValueListHead = sortInfo->sortedValueListPool;
		sortInfo->sortedValueListTail = sortInfo->sortedValueListPool;
		sortInfo->sortedValueListHead->value = computedValue;
		sortInfo->sortedValueListHead->neighborValue = nbrComputedValue;
		sortInfo->sortedValueListHead->sortValue = sortValue;
		sortInfo->sortedValueListHead->portp = pmportp;
		sortInfo->sortedValueListHead->neighborPortp = nbrPt;
		sortInfo->sortedValueListHead->lid = lid;
		sortInfo->sortedValueListHead->portNum = portNum;
		sortInfo->sortedValueListHead->localStatus = localStatus;
		sortInfo->sortedValueListHead->neighborStatus = neighborStatus;
		sortInfo->sortedValueListHead->next = NULL;
		sortInfo->sortedValueListHead->prev = NULL;
		sortInfo->numValueEntries++;
	} else if (sortInfo->numValueEntries < sortInfo->sortedValueListSize) {
		// list is not yet full - use available pool entry and insert sorted
		newListEntry = &sortInfo->sortedValueListPool[sortInfo->numValueEntries++];
		newListEntry->value = computedValue;
		newListEntry->neighborValue = nbrComputedValue;
		newListEntry->sortValue = sortValue;
		newListEntry->portp = pmportp;
		newListEntry->neighborPortp = nbrPt;
		newListEntry->lid = lid;
		newListEntry->portNum = portNum;
		newListEntry->localStatus = localStatus;
		newListEntry->neighborStatus = neighborStatus;
		newListEntry->next = NULL;
		thisListEntry = sortInfo->sortedValueListHead;
		while ((thisListEntry != NULL) && compareFunc(sortValue, thisListEntry->sortValue)) {
			prevListEntry = thisListEntry;
			thisListEntry = thisListEntry->next;
		}
		if (prevListEntry == NULL) {
			// put new entry at head
			newListEntry->next = sortInfo->sortedValueListHead;
			sortInfo->sortedValueListHead->prev = newListEntry;
			newListEntry->prev = NULL;
			sortInfo->sortedValueListHead = newListEntry;
		} else {
			// put between prev and its next (even if its next is NULL)
			newListEntry->next = prevListEntry->next;
			newListEntry->prev = prevListEntry;
			if (prevListEntry->next)
				prevListEntry->next->prev = newListEntry;
			prevListEntry->next = newListEntry;
			if (newListEntry->next == NULL)
				sortInfo->sortedValueListTail = newListEntry;
		}
	} else {
		// see if this is a candidate for the list
		if (candidateFunc(sortValue, sortInfo->sortedValueListTail->sortValue)) {
			// if list size is 1, simple replace values
			if (sortInfo->sortedValueListSize == 1) {
				sortInfo->sortedValueListHead->value = computedValue;
				sortInfo->sortedValueListHead->neighborValue = nbrComputedValue;
				sortInfo->sortedValueListHead->sortValue = sortValue;
				sortInfo->sortedValueListHead->portp = pmportp;
				sortInfo->sortedValueListHead->neighborPortp = nbrPt;
				sortInfo->sortedValueListHead->lid = lid;
				sortInfo->sortedValueListHead->portNum = portNum;
				sortInfo->sortedValueListHead->localStatus = localStatus;
				sortInfo->sortedValueListHead->neighborStatus = neighborStatus;
			} else {
				// list is full - bump tail and insert sorted
				// first, copy into tail entry and adjust tail
				newListEntry = sortInfo->sortedValueListTail;
				newListEntry->prev->next = NULL;
				newListEntry->value = computedValue;
				newListEntry->neighborValue = nbrComputedValue;
				newListEntry->sortValue = sortValue;
				newListEntry->portp = pmportp;
				newListEntry->neighborPortp = nbrPt;
				newListEntry->lid = lid;
				newListEntry->portNum = portNum;
				newListEntry->localStatus = localStatus;
				newListEntry->neighborStatus = neighborStatus;
				sortInfo->sortedValueListTail = sortInfo->sortedValueListTail->prev;
				// now insert sorted entry
				thisListEntry = sortInfo->sortedValueListHead;
				while ((thisListEntry != NULL) && compareFunc(sortValue, thisListEntry->sortValue)) {
					prevListEntry = thisListEntry;
					thisListEntry = thisListEntry->next;
				}
				if (prevListEntry == NULL) {
					// put new entry at head
					newListEntry->next = sortInfo->sortedValueListHead;
					sortInfo->sortedValueListHead->prev = newListEntry;
					newListEntry->prev = NULL;
					sortInfo->sortedValueListHead = newListEntry;
				} else {
					// put between prev and its next (even if its next is NULL)
					newListEntry->next = prevListEntry->next;
					newListEntry->prev = prevListEntry;
					if (prevListEntry->next)
						prevListEntry->next->prev = newListEntry;
					prevListEntry->next = newListEntry;
					if (newListEntry->next == NULL)
						sortInfo->sortedValueListTail = newListEntry;
				}
			}
		}
	}

	return(FSUCCESS);
}

FSTATUS addSortedPorts(PmFocusPorts_t *pmFocusPorts, sortInfo_t *sortInfo, uint32 imageIndex)
{
	Status_t			status;
	sortedValueEntry_t	*listp;
	int					portCount = 0;

	if (sortInfo->sortedValueListHead == NULL) {
		pmFocusPorts->NumPorts = 0;
		return(FSUCCESS);
	}

	status = vs_pool_alloc(&pm_pool, pmFocusPorts->NumPorts * sizeof(PmFocusPortEntry_t),
						   (void*)&pmFocusPorts->portList);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("Failed to allocate sorted port list buffer for pmFocusPorts rc:", status);
		return(FINSUFFICIENT_MEMORY);
	}
	listp = sortInfo->sortedValueListHead;
	while ((listp != NULL) && (portCount < pmFocusPorts->NumPorts)) {
		pmFocusPorts->portList[portCount].lid = listp->lid;
		pmFocusPorts->portList[portCount].portNum = listp->portNum;
		pmFocusPorts->portList[portCount].localStatus = listp->localStatus;
		pmFocusPorts->portList[portCount].rate = PmCalculateRate(listp->portp->Image[imageIndex].u.s.activeSpeed, listp->portp->Image[imageIndex].u.s.rxActiveWidth);
		pmFocusPorts->portList[portCount].mtu = listp->portp->Image[imageIndex].u.s.mtu;
		pmFocusPorts->portList[portCount].value = listp->value;
		pmFocusPorts->portList[portCount].guid = (uint64_t)(listp->portp->pmnodep->guid);
		strncpy(pmFocusPorts->portList[portCount].nodeDesc, (char *)listp->portp->pmnodep->nodeDesc.NodeString,
			sizeof(pmFocusPorts->portList[portCount].nodeDesc)-1);
		if (listp->portNum != 0 && listp->neighborPortp != NULL) {
			pmFocusPorts->portList[portCount].neighborStatus = listp->neighborStatus;
			pmFocusPorts->portList[portCount].neighborLid = listp->neighborPortp->pmnodep->Image[imageIndex].lid;
			pmFocusPorts->portList[portCount].neighborPortNum = listp->neighborPortp->portNum;
			pmFocusPorts->portList[portCount].neighborValue = listp->neighborValue;
			pmFocusPorts->portList[portCount].neighborGuid = (uint64_t)(listp->neighborPortp->pmnodep->guid);
			strncpy(pmFocusPorts->portList[portCount].neighborNodeDesc, (char *)listp->neighborPortp->pmnodep->nodeDesc.NodeString,
				sizeof(pmFocusPorts->portList[portCount].neighborNodeDesc)-1);
		} else {
			pmFocusPorts->portList[portCount].neighborStatus = listp->neighborStatus;
			pmFocusPorts->portList[portCount].neighborLid = 0;
			pmFocusPorts->portList[portCount].neighborPortNum = 0;
			pmFocusPorts->portList[portCount].neighborValue = 0;
			pmFocusPorts->portList[portCount].neighborGuid = 0;
			pmFocusPorts->portList[portCount].neighborNodeDesc[0] = 0;
		}
		portCount++;
		listp = listp->next;
	}

	return(FSUCCESS);
}

/*************************************************************************************
*
* paGetFocusPorts - return a set of focus ports
*
*  Inputs:
*     pm - pointer to Pm_t (the PM main data type)
*     groupName - pointer to name of group
*     pmGroupConfig - pointer to caller-declared data area to return group config info
*
*  Return:
*     FSTATUS - FSUCCESS if OK
*
*
*************************************************************************************/

FSTATUS paGetFocusPorts(Pm_t *pm, char *groupName, PmFocusPorts_t *pmFocusPorts,
	STL_PA_IMAGE_ID_DATA imageId, STL_PA_IMAGE_ID_DATA *returnImageId, uint32 select,
	uint32 start, uint32 range)
{
	STL_PA_IMAGE_ID_DATA retImageId = {0};
	PmGroup_t			*pmGroupP = NULL;
	STL_LID_32			lid;
	uint32				imageIndex, imageInterval;
	const char 			*msg;
	PmImage_t			*pmimagep;
	FSTATUS				status = FSUCCESS;
	Status_t			allocStatus;
	sortInfo_t			sortInfo;
	ComputeFunc_t		computeFunc = NULL;
	CompareFunc_t		compareFunc = NULL;
	CompareFunc_t		candidateFunc = NULL;
	boolean				sth = 0;
	PmHistoryRecord_t	*record = NULL;
	PmCompositeImage_t	*cimg = NULL;
	void *computeData = NULL;

	// check input parameters
	if (!pm || !groupName || !pmFocusPorts)
		return (FINVALID_PARAMETER);
	if (groupName[0] == '\0') {
		IB_LOG_WARN_FMT(__func__, "Illegal groupName parameter: Empty String\n");
		return(FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER) ;
	}
	if (start) {
		IB_LOG_WARN_FMT(__func__, "Illegal start parameter: %d: must be zero\n", start);
		return(FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER);
	}
	if (!range) {
		IB_LOG_WARN_FMT(__func__, "Illegal range parameter: %d: must be greater than zero\n", range);
		return(FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER);
	}
	switch (select) {
	case STL_PA_SELECT_UTIL_HIGH:
		computeFunc = &computeUtilizationPct10;
		compareFunc = &compareLE;
		candidateFunc = &compareGT;
		computeData = (void *)&imageInterval;
		break;
	case STL_PA_SELECT_UTIL_PKTS_HIGH:
		computeFunc = &computeSendKPkts;
		compareFunc = &compareLE;
		candidateFunc = &compareGT;
		computeData = (void *)&imageInterval;
		break;
	case STL_PA_SELECT_UTIL_LOW:
		computeFunc = &computeUtilizationPct10;
		compareFunc = &compareGE;
		candidateFunc = &compareLT;
		computeData = (void *)&imageInterval;
		break;
	case STL_PA_SELECT_CATEGORY_INTEG:
		computeFunc = &computeIntegrity;
		compareFunc = &compareLE;
		candidateFunc = &compareGT;
		computeData = &pm->integrityWeights;
		break;
	case STL_PA_SELECT_CATEGORY_CONG:
		computeFunc = &computeCongestion;
		compareFunc = &compareLE;
		candidateFunc = &compareGT;
		computeData = &pm->congestionWeights;
		break;
	case STL_PA_SELECT_CATEGORY_SMA_CONG:
		computeFunc = &computeSmaCongestion;
		compareFunc = &compareLE;
		candidateFunc = &compareGT;
		computeData = &pm->congestionWeights;
		break;
	case STL_PA_SELECT_CATEGORY_BUBBLE:
		computeFunc = &computeBubble;
		compareFunc = &compareLE;
		candidateFunc = &compareGT;
		break;
	case STL_PA_SELECT_CATEGORY_SEC:
		computeFunc = &computeSecurity;
		compareFunc = &compareLE;
		candidateFunc = &compareGT;
		break;
	case STL_PA_SELECT_CATEGORY_ROUT:
		computeFunc = &computeRouting;
		compareFunc = &compareLE;
		candidateFunc = &compareGT;
		break;
	default:
		IB_LOG_WARN_FMT(__func__, "Illegal select parameter: 0x%x\n", select);
		return FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER;
		break;
	}

	// initialize group config port list counts
	pmFocusPorts->NumPorts = 0;
	pmFocusPorts->portList = NULL;
	memset(&sortInfo, 0, sizeof(sortInfo));
	allocStatus = vs_pool_alloc(&pm_pool, range * sizeof(sortedValueEntry_t), (void*)&sortInfo.sortedValueListPool);
	if (allocStatus != VSTATUS_OK) {
		IB_LOG_ERRORRC("Failed to allocate list buffer for pmFocusPorts rc:", allocStatus);
		return FINSUFFICIENT_MEMORY;
	}
	sortInfo.sortedValueListHead = NULL;
	sortInfo.sortedValueListTail = NULL;
	sortInfo.sortedValueListSize = range;
	sortInfo.numValueEntries = 0;

	AtomicIncrementVoid(&pm->refCount);	// prevent engine from stopping
	if (! PmEngineRunning()) {	// see if is already stopped/stopping
		status = FUNAVAILABLE;
		goto done;
	}

	// pmGroupP points to our group
	// check all ports for membership in our group
	(void)vs_rdlock(&pm->stateLock);
	status = FindImage(pm, IMAGEID_TYPE_ANY, imageId, &imageIndex, &retImageId.imageNumber, &record, &msg, NULL, &cimg);
	if (FSUCCESS != status) {
		IB_LOG_WARN_FMT(__func__, "Unable to get index from ImageId: %s: %s", FSTATUS_ToString(status), msg);
		goto error;
	}

	if (record || cimg) {
		sth = 1;
		if (record) {
			status = PmLoadComposite(pm, record, &cimg);
			if (status != FSUCCESS || !cimg) {
				IB_LOG_WARN_FMT(__func__, "Unable to load composite image: %s", FSTATUS_ToString(status));
				goto error;
			}
		}
		retImageId.imageNumber = cimg->header.common.imageIDs[0];
		imageInterval = cimg->header.common.imageSweepInterval;
		status = PmReconstitute(&pm->ShortTermHistory, cimg);
		if (record) PmFreeComposite(cimg);
		if (status != FSUCCESS) {
			IB_LOG_WARN_FMT(__func__, "Unable to reconstitute composite image: %s", FSTATUS_ToString(status));
			goto error;
		}
		// find the group
		if (!strcmp(pm->ShortTermHistory.LoadedImage.AllGroup->Name, groupName)) {
			pmGroupP = pm->ShortTermHistory.LoadedImage.AllGroup;
		} else {
			int i;
			for (i = 0; i < PM_MAX_GROUPS; i++) {
				if (!strcmp(pm->ShortTermHistory.LoadedImage.Groups[i]->Name, groupName)) {
					pmGroupP = pm->ShortTermHistory.LoadedImage.Groups[i];
					break;
				}
			}
		}
		if (!pmGroupP) {
			IB_LOG_WARN_FMT(__func__, "Group %.*s not Found", (int)sizeof(groupName), groupName);
			status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_GROUP;
			goto error;
		}
		pmimagep = pm->ShortTermHistory.LoadedImage.img;
		imageIndex = 0;
	} else {
		status = LocateGroup(pm, groupName, &pmGroupP);
		if (status != FSUCCESS) {
			IB_LOG_WARN_FMT(__func__, "Group %.*s not Found: %s", (int)sizeof(groupName), groupName, FSTATUS_ToString(status));
			status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_GROUP;
			goto error;
		}
		pmimagep = &pm->Image[imageIndex];
		imageInterval = MAX(pm->interval, (pmimagep->sweepDuration/1000000));
		(void)vs_rdlock(&pmimagep->imageLock);
	}

	// Grab ImageTime from Pm Image
	retImageId.imageTime.absoluteTime = (uint32)pmimagep->sweepStart;

	(void)vs_rwunlock(&pm->stateLock); 

	for (lid=1; lid<= pmimagep->maxLid; ++lid) {
		uint8 portnum;
		PmNode_t *pmnodep = pmimagep->LidMap[lid];
		if (! pmnodep)
			continue;
		if (pmnodep->nodeType == STL_NODE_SW) {
			for (portnum = 0; portnum <= pmnodep->numPorts; ++portnum) {
				PmPort_t *pmportp = pmnodep->up.swPorts[portnum];
				PmPortImage_t *portImage;
				if (!pmportp) continue;
				portImage = &pmportp->Image[imageIndex];
				if (PmIsPortInGroup(pm, pmportp, portImage, pmGroupP, sth, NULL)) {
					processFocusPort(pm, pmportp, portImage, imageIndex, lid, portnum, 
						computeFunc, compareFunc, candidateFunc, computeData, &sortInfo);
				}
			}
		} else {
			PmPort_t *pmportp = pmnodep->up.caPortp;
			PmPortImage_t *portImage = &pmportp->Image[imageIndex];
			if (PmIsPortInGroup(pm, pmportp, portImage, pmGroupP, sth, NULL)) {
				processFocusPort(pm, pmportp, portImage, imageIndex, lid, pmportp->portNum,
					computeFunc, compareFunc, candidateFunc, computeData, &sortInfo);
			}
		}
	}

	cs_strlcpy(pmFocusPorts->groupName, pmGroupP->Name, STL_PM_GROUPNAMELEN);
	pmFocusPorts->NumPorts = sortInfo.numValueEntries;
	if (pmFocusPorts->NumPorts)
		status = addSortedPorts(pmFocusPorts, &sortInfo, imageIndex);

	if (!sth) {
		(void)vs_rwunlock(&pmimagep->imageLock);
	}
	if (status != FSUCCESS)
		goto done;
	*returnImageId = retImageId;

done:
	if (sortInfo.sortedValueListPool != NULL)
		vs_pool_free(&pm_pool, sortInfo.sortedValueListPool);
#ifndef __VXWORKS__
	if (sth) {
		clearLoadedImage(&pm->ShortTermHistory);
	}
#endif
	AtomicDecrementVoid(&pm->refCount);
	return(status);
error:
	(void)vs_rwunlock(&pm->stateLock);
	returnImageId->imageNumber = BAD_IMAGE_ID;
	goto done;
}

FSTATUS paGetImageInfo(Pm_t *pm, STL_PA_IMAGE_ID_DATA imageId, PmImageInfo_t *imageInfo,
	STL_PA_IMAGE_ID_DATA *returnImageId)
{
	FSTATUS				status = FSUCCESS;
	STL_PA_IMAGE_ID_DATA retImageId = {0};
	uint32				imageIndex, imageInterval;
	int					i;
	const char 			*msg;
	PmImage_t			*pmimagep;
	boolean 			sth = 0;
	PmHistoryRecord_t 	*record = NULL;
	PmCompositeImage_t	*cimg = NULL;

	// check input parameters
	if (!pm || !imageInfo)
		return FINVALID_PARAMETER;

	AtomicIncrementVoid(&pm->refCount);	// prevent engine from stopping
	if (! PmEngineRunning()) {	// see if is already stopped/stopping
		status = FUNAVAILABLE;
		goto done;
	}

	(void)vs_rdlock(&pm->stateLock);
	status = FindImage(pm, IMAGEID_TYPE_ANY, imageId, &imageIndex, &retImageId.imageNumber, &record, &msg, NULL, &cimg);
	if (FSUCCESS != status) {
		IB_LOG_WARN_FMT(__func__, "Unable to get index from ImageId: %s: %s", FSTATUS_ToString(status), msg);
		goto error;
	}

	if (record || cimg) {
		sth = 1;
		if (record) {
			status = PmLoadComposite(pm, record, &cimg);
			if (status != FSUCCESS || !cimg) {
				IB_LOG_WARN_FMT(__func__, "Unable to load composite image: %s", FSTATUS_ToString(status));
				goto error;
			}
		}
		imageInterval = cimg->header.common.imageSweepInterval;
		retImageId.imageNumber = cimg->header.common.imageIDs[0];
		status = PmReconstitute(&pm->ShortTermHistory, cimg);
		if (record) PmFreeComposite(cimg);
		if (status != FSUCCESS) {
			IB_LOG_WARN_FMT(__func__, "Unable to reconstitute composite image: %s", FSTATUS_ToString(status));
			goto error;
		}
		pmimagep = pm->ShortTermHistory.LoadedImage.img;
	}  else {
		pmimagep = &pm->Image[imageIndex];
		imageInterval = MAX(pm->interval, (pmimagep->sweepDuration/1000000));
	}

	// Grab ImageTime from Pm Image
	retImageId.imageTime.absoluteTime = (uint32)pmimagep->sweepStart;

	imageInfo->sweepStart				= pmimagep->sweepStart;
	imageInfo->sweepDuration			= pmimagep->sweepDuration;
	imageInfo->numHFIPorts	   			= pmimagep->HFIPorts;
	imageInfo->numSwitchNodes			= pmimagep->SwitchNodes;
	imageInfo->numSwitchPorts			= pmimagep->SwitchPorts;
	imageInfo->numLinks					= pmimagep->NumLinks;
	imageInfo->numSMs					= pmimagep->NumSMs;
	imageInfo->numNoRespNodes			= pmimagep->NoRespNodes;
	imageInfo->numNoRespPorts			= pmimagep->NoRespPorts;
	imageInfo->numSkippedNodes			= pmimagep->SkippedNodes;
	imageInfo->numSkippedPorts			= pmimagep->SkippedPorts;
	imageInfo->numUnexpectedClearPorts	= pmimagep->UnexpectedClearPorts;
	imageInfo->imageInterval			= imageInterval;
	for (i = 0; i < 2; i++) {
		STL_LID_32 smLid = pmimagep->SMs[i].smLid;
		PmNode_t *pmnodep = pmimagep->LidMap[smLid];
		imageInfo->SMInfo[i].smLid		= smLid;
		imageInfo->SMInfo[i].priority	= pmimagep->SMs[i].priority;
		imageInfo->SMInfo[i].state		= pmimagep->SMs[i].state;
		if (pmnodep != NULL) {
			PmPort_t *pmportp = pm_node_lided_port(pmnodep);
			imageInfo->SMInfo[i].portNumber = pmportp->portNum;
			imageInfo->SMInfo[i].smPortGuid	= pmportp->guid;
			strncpy(imageInfo->SMInfo[i].smNodeDesc, (char *)pmnodep->nodeDesc.NodeString,
				sizeof(imageInfo->SMInfo[i].smNodeDesc)-1);
		} else {
			imageInfo->SMInfo[i].portNumber	= 0;
			imageInfo->SMInfo[i].smPortGuid	= 0;
			imageInfo->SMInfo[i].smNodeDesc[0] = 0;
		}
	}

	*returnImageId = retImageId;
	if (sth) {
		clearLoadedImage(&pm->ShortTermHistory);
	}

	(void)vs_rwunlock(&pm->stateLock);
done:
	AtomicDecrementVoid(&pm->refCount);
	return(status);
error:
	(void)vs_rwunlock(&pm->stateLock);
	returnImageId->imageNumber = BAD_IMAGE_ID;
	goto done;
}

// this function is used by ESM CLI
static void pm_print_port_running_totals(FILE *out, Pm_t *pm, PmPort_t *pmportp,
				uint32 imageIndex)
{
	PmCompositePortCounters_t *pPortCounters;
	PmPortImage_t *portImage = &pmportp->Image[imageIndex];

	if (! portImage->u.s.active)
		return;
	fprintf(out, "%.*s Guid "FMT_U64" LID 0x%x Port %u\n",
				(int)sizeof(pmportp->pmnodep->nodeDesc.NodeString),
				pmportp->pmnodep->nodeDesc.NodeString, pmportp->pmnodep->guid,
				pmportp->pmnodep->Image[imageIndex].lid, pmportp->portNum);
	if (portImage->neighbor) {
		PmPort_t *neighbor = portImage->neighbor;
		fprintf(out, "    Neighbor: %.*s Guid "FMT_U64" LID 0x%x Port %u\n",
				(int)sizeof(neighbor->pmnodep->nodeDesc.NodeString),
				neighbor->pmnodep->nodeDesc.NodeString, neighbor->pmnodep->guid,
				neighbor->pmnodep->Image[imageIndex].lid,
				neighbor->portNum);
	}
	fprintf(out, "    txRate: %4s rxRate: %4s  MTU: %5s%s\n",
				StlStaticRateToText(PmCalculateRate(portImage->u.s.activeSpeed, portImage->u.s.txActiveWidth)),
				StlStaticRateToText(PmCalculateRate(portImage->u.s.activeSpeed, portImage->u.s.rxActiveWidth)),
				IbMTUToText(portImage->u.s.mtu),
				portImage->u.s.UnexpectedClear?"  Unexpected Clear":"");
	if (pmportp->u.s.PmaAvoid ) {
		fprintf(out, "    PMA Counters Not Available\n");
		return;
	}

	pPortCounters = &pmportp->StlPortCountersTotal;
	fprintf(out, "    Xmit: Data:%-10" PRIu64 " MB (%-10" PRIu64
				" Flits) Pkts:%-10" PRIu64 "\n",
				pPortCounters->PortXmitData / FLITS_PER_MB,
				pPortCounters->PortXmitData, pPortCounters->PortXmitPkts );
	fprintf(out, "    Recv: Data:%-10" PRIu64 " MB (%-10" PRIu64
				" Flits) Pkts:%-10" PRIu64 "\n",
				pPortCounters->PortRcvData / FLITS_PER_MB,
				pPortCounters->PortRcvData, pPortCounters->PortRcvPkts );
	fprintf(out, "    Integrity:                        SmaCongest:\n" );
	fprintf(out, "      Link Recovery:%-10u\n",
				pPortCounters->LinkErrorRecovery);
	fprintf(out, "      Link Downed:%-10u\n", pPortCounters->LinkDowned);
	fprintf(out, "      Rcv Errors:%-10" PRIu64 "           Security:\n",
				pPortCounters->PortRcvErrors );
	fprintf(out, "      Loc Lnk Integrity:%-10" PRIu64 "      Rcv Constrain:%-10" PRIu64 "\n",
				pPortCounters->LocalLinkIntegrityErrors,
				pPortCounters->PortRcvConstraintErrors );
	fprintf(out, "      Excess Bfr Overrun*:%-10" PRIu64 "    Xmt Constrain*:%-10" PRIu64 "\n",
                pPortCounters->ExcessiveBufferOverruns,
				pPortCounters->PortXmitConstraintErrors );
	fprintf(out, "    Congestion:                       Routing:\n" );
	fprintf(out, "      Xmt Discards*:%-10" PRIu64 "     Rcv Sw Relay:%-10" PRIu64 "\n",
				pPortCounters->PortXmitDiscards,
				pPortCounters->PortRcvSwitchRelayErrors );
#if 0
	fprintf(out, "      Xmt Congest*:%-10" PRIu64 "\n",
				pPortCounters->PortXmitCongestion );
	fprintf(out, "    Rcv Rmt Phy:%-10u       Adapt Route:%-10" PRIu64 "\n",
				pPortCounters->PortRcvRemotePhysicalErrors,
				pPortCounters->PortAdaptiveRouting );
#else
	fprintf(out, "    Rcv Rmt Phy:%-10" PRIu64 "                                      \n",
				pPortCounters->PortRcvRemotePhysicalErrors);
#endif
#if 0
	fprintf(out, "    Xmt Congest:%-10" PRIu64 "       Check Rate:0x%4x\n",
				pPortCounters->PortXmitCongestion, pPortCounters->PortCheckRate );
#endif
}

// this function is used by ESM CLI
void pm_print_running_totals_to_stream(FILE * out)
{
	FSTATUS				status;
	const char 			*msg;
	PmImage_t			*pmimagep;
	uint32				imageIndex;
	uint64				retImageId;
	STL_LID_32			lid;
	extern Pm_t g_pmSweepData;
	Pm_t *pm = &g_pmSweepData;

	if (topology_passcount < 1) {
		fprintf(out, "\nSM is currently in the %s state, count = %d\n\n", IbSMStateToText(sm_state), (int)sm_smInfo.ActCount);
		return;
	}

	AtomicIncrementVoid(&pm->refCount);	// prevent engine from stopping
	if (! PmEngineRunning()) {
		fprintf(out, "\nPM is currently not running\n\n");
		goto done;
	}

	(void)vs_rdlock(&pm->stateLock);
	status = GetIndexFromImageId(pm, IMAGEID_TYPE_ANY, IMAGEID_LIVE_DATA, 0,
					   				&imageIndex, &retImageId, &msg, NULL);
	if (FSUCCESS != status) {
		fprintf(out, "Unable to access Running Totals: %s\n", msg);
		(void)vs_rwunlock(&pm->stateLock);
		goto done;
	}
	pmimagep = &pm->Image[imageIndex];
	(void)vs_rdlock(&pmimagep->imageLock);
	(void)vs_rwunlock(&pm->stateLock);

	for (lid=1; lid<= pmimagep->maxLid; ++lid) {
		uint8 portnum;
		PmNode_t *pmnodep = pmimagep->LidMap[lid];
		if (! pmnodep)
			continue;
		if (pmnodep->nodeType == STL_NODE_SW) {
			for (portnum=0; portnum<=pmnodep->numPorts; ++portnum) {
				PmPort_t *pmportp = pmnodep->up.swPorts[portnum];
				if (! pmportp)
					continue;
				(void)vs_rdlock(&pm->totalsLock);	// limit in case print slow
				pm_print_port_running_totals(out, pm, pmportp, imageIndex);
				(void)vs_rwunlock(&pm->totalsLock);
			}
		} else {
			PmPort_t *pmportp = pmnodep->up.caPortp;
			(void)vs_rdlock(&pm->totalsLock);	// limit in case print slow
			pm_print_port_running_totals(out, pm, pmportp, imageIndex);
			(void)vs_rwunlock(&pm->totalsLock);
		}
	}
	(void)vs_rwunlock(&pmimagep->imageLock);
done:
	AtomicDecrementVoid(&pm->refCount);
}

// locate vf by name
static FSTATUS LocateVF(Pm_t *pm, const char *vfName, PmVF_t **pmVFP, uint8 activeOnly, uint32 imageIndex)
{
	int i;

	for (i = 0; i < pm->numVFs; i++) {
		if (strcmp(vfName, pm->VFs[i]->Name) == 0) {
			*pmVFP = pm->VFs[i];
			if(*pmVFP && activeOnly <= (pm->VFs[i]->Image[imageIndex].isActive)) {
									// True cases: activeOnly=0 & pm->VFs[i]->Image[imageIndex].isActive = 0 or 1	Locates VF in list of active and standby VFs
				return FSUCCESS;	//             activeOnly=1 & pm->VFs[i]->Image[imageIndex].isActive = 1		Locates VF in list of only active
			}
		}
	}
	return FNOT_FOUND;
}

FSTATUS paGetVFList(Pm_t *pm, PmVFList_t *VFList, uint32 imageIndex)
{
	Status_t			vStatus;
	FSTATUS				fStatus = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_VF;
	int					i, j;

	// check input parameters
	if (!pm || !VFList)
		return FINVALID_PARAMETER;

	AtomicIncrementVoid(&pm->refCount);	// prevent engine from stopping
	if (! PmEngineRunning()) {	// see if is already stopped/stopping
		fStatus = FUNAVAILABLE;
		goto done;
	}
	VFList->NumVFs = pm->numVFsActive;
	vStatus = vs_pool_alloc(&pm_pool, VFList->NumVFs * STL_PM_VFNAMELEN,
						   (void*)&VFList->VfList);
	if (vStatus != VSTATUS_OK) {
		IB_LOG_ERRORRC("Failed to allocate name list buffer for VFList rc:", vStatus);
		fStatus = FINSUFFICIENT_MEMORY;
		goto done;
	}

	(void)vs_rdlock(&pm->stateLock);
	for (i = 0, j = 0; i < pm->numVFs; i++) {
		if (pm->VFs[i]->Image[imageIndex].isActive) {
			strncpy(VFList->VfList[j++].Name, pm->VFs[i]->Name, STL_PM_VFNAMELEN-1);
		}
	}
	if (VFList->NumVFs)
		fStatus = FSUCCESS;

	(void)vs_rwunlock(&pm->stateLock);

done:
	AtomicDecrementVoid(&pm->refCount);
	return(fStatus);
}

FSTATUS paGetVFInfo(Pm_t *pm, char *vfName, PmVFInfo_t *pmVFInfo, STL_PA_IMAGE_ID_DATA imageId,
		STL_PA_IMAGE_ID_DATA *returnImageId)
{
	STL_PA_IMAGE_ID_DATA retImageId = {0};
	FSTATUS				status = FSUCCESS;
	PmVF_t				*pmVFP = NULL;
	PmVFImage_t			pmVFImage;
	PmImage_t			*pmImageP = NULL;
	PmPortImage_t		*pmPortImageP = NULL;
	PmPort_t			*pmPortP = NULL;
	uint32				imageIndex, imageInterval;
	const char 			*msg;
	boolean				sth = FALSE;
	int 				lid;
	PmHistoryRecord_t	*record = NULL;
	PmCompositeImage_t	*cimg = NULL;

	if (!pm || !vfName || !pmVFInfo)
		return(FINVALID_PARAMETER);
	if (vfName[0] == '\0') {
		IB_LOG_WARN_FMT(__func__, "Illegal vfName parameter: Empty String\n");
		return(FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER);
	}

	AtomicIncrementVoid(&pm->refCount);	// prevent engine from stopping
	if (! PmEngineRunning()) {	// see if is already stopped/stopping
		status = FUNAVAILABLE;
		goto done;
	}

	(void)vs_rdlock(&pm->stateLock);
	status = FindImage(pm, IMAGEID_TYPE_ANY, imageId, &imageIndex, &retImageId.imageNumber, &record, &msg, NULL, &cimg);
	if (FSUCCESS != status) {
		IB_LOG_WARN_FMT(__func__, "Unable to get index from ImageId: %s: %s", FSTATUS_ToString(status), msg);
		goto error;
	}

	if (record || cimg) {
		sth = 1;
		if (record) {
			// load the composite
			status = PmLoadComposite(pm, record, &cimg);
			if (status != FSUCCESS || !cimg) {
				IB_LOG_WARN_FMT(__func__, "Unable to load composite image: %s", FSTATUS_ToString(status));
				goto error;
			}
		}
		retImageId.imageNumber = cimg->header.common.imageIDs[0];
		imageInterval = cimg->header.common.imageSweepInterval;
		// composite is loaded, reconstitute so we can use it
		status = PmReconstitute(&pm->ShortTermHistory, cimg);
		if (record) PmFreeComposite(cimg);
		if (status != FSUCCESS) {
			IB_LOG_WARN_FMT(__func__, "Unable to reconstitute composite image: %s", FSTATUS_ToString(status));
			goto error;
		}
		pmImageP = pm->ShortTermHistory.LoadedImage.img;
		int i;
		for (i = 0; i < MAX_VFABRICS; i++) {
			if (!strcmp(pm->ShortTermHistory.LoadedImage.VFs[i]->Name, vfName)) {
				pmVFP = pm->ShortTermHistory.LoadedImage.VFs[i];
				break;
			}
		}
		if (!pmVFP) {
			IB_LOG_WARN_FMT(__func__, "VF %.*s not Found", (int)sizeof(vfName), vfName);
			status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_VF;
			goto error;
		}
		imageIndex = 0; // STH always uses imageIndex 0
		if (!pmVFP->Image[imageIndex].isActive) {
			IB_LOG_WARN_FMT(__func__, "VF %.*s not active", (int)sizeof(pmVFP->Name), pmVFP->Name);
			status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_VF;
			goto error;
		}
	} else {
		status = LocateVF(pm, vfName, &pmVFP, 1, imageIndex);
		if (status != FSUCCESS){
			IB_LOG_WARN_FMT(__func__, "VF %.*s not Found: %s", (int)sizeof(vfName), vfName, FSTATUS_ToString(status));
			status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_VF;
			goto error;
		}

		pmImageP = &pm->Image[imageIndex];
		imageInterval = MAX(pm->interval, (pmImageP->sweepDuration/1000000));
		(void)vs_rdlock(&pmImageP->imageLock);
	}

	// Grab ImageTime from Pm Image
	retImageId.imageTime.absoluteTime = (uint32)pmImageP->sweepStart;

	(void)vs_rwunlock(&pm->stateLock);
	memset(&pmVFImage, 0, sizeof(PmVFImage_t));
	ClearVFStats(&pmVFImage);

	for (lid = 1; lid <= pmImageP->maxLid; lid++) {
		PmNode_t *pmNodeP = pmImageP->LidMap[lid];
		if (!pmNodeP) continue;
		if (pmNodeP->nodeType == STL_NODE_SW) {
			int p;
			for (p = 0; p <= pmNodeP->numPorts; p++) { // Includes port 0
				pmPortP = pmNodeP->up.swPorts[p];
				if (!pmPortP) continue;
				pmPortImageP = &pmPortP->Image[imageIndex];
				if (PmIsPortInVF(pm, pmPortP, pmPortImageP, pmVFP)) {
					if (pmPortImageP->u.s.queryStatus != PM_QUERY_STATUS_OK) {
						PA_INC_COUNTER_NO_OVERFLOW(pmVFImage.IntUtil.pmaNoRespPorts, IB_UINT16_MAX);
					}
					pmVFImage.NumPorts++;
					UpdateVFStats(pm, imageIndex, pmPortP, &pmVFImage, imageInterval);
					if (pmPortImageP->neighbor == NULL && pmPortP->portNum != 0) {
						PA_INC_COUNTER_NO_OVERFLOW(pmVFImage.IntUtil.topoIncompPorts, IB_UINT16_MAX);
					}
				}
			}
		} else {
			pmPortP = pmNodeP->up.caPortp;
			if (!pmPortP) continue;
			pmPortImageP = &pmPortP->Image[imageIndex];
			if (PmIsPortInVF(pm, pmPortP, pmPortImageP, pmVFP)) {
				if (pmPortImageP->u.s.queryStatus != PM_QUERY_STATUS_OK) {
					PA_INC_COUNTER_NO_OVERFLOW(pmVFImage.IntUtil.pmaNoRespPorts, IB_UINT16_MAX);
				}
				pmVFImage.NumPorts++;
				UpdateVFStats(pm, imageIndex, pmPortP, &pmVFImage, imageInterval);
				if (pmPortImageP->neighbor == NULL) {
					PA_INC_COUNTER_NO_OVERFLOW(pmVFImage.IntUtil.topoIncompPorts, IB_UINT16_MAX);
				}
			}
		}
	}
	FinalizeVFStats(&pmVFImage);
	strcpy(pmVFInfo->vfName, vfName);
	pmVFInfo->NumPorts = pmVFImage.NumPorts;
	memcpy(&pmVFInfo->IntUtil, &pmVFImage.IntUtil, sizeof(PmUtilStats_t));
	memcpy(&pmVFInfo->IntErr, &pmVFImage.IntErr, sizeof(PmErrStats_t));
	pmVFInfo->MinIntRate = pmVFImage.MinIntRate;
	pmVFInfo->MaxIntRate = pmVFImage.MaxIntRate;
	if (!sth) {
		(void)vs_rwunlock(&pmImageP->imageLock);
	}

	*returnImageId = retImageId;

done:
	AtomicDecrementVoid(&pm->refCount);
	return(status);
error:
	(void)vs_rwunlock(&pm->stateLock);
	returnImageId->imageNumber = BAD_IMAGE_ID;
	goto done;
}

FSTATUS paGetVFConfig(Pm_t *pm, char *vfName, uint64 vfSid, PmVFConfig_t *pmVFConfig,
	STL_PA_IMAGE_ID_DATA imageId, STL_PA_IMAGE_ID_DATA *returnImageId)
{
	STL_PA_IMAGE_ID_DATA retImageId = {0};
	PmVF_t				*pmVFP = NULL;
	STL_LID_32			lid;
	uint32				imageIndex;
	const char 			*msg;
	PmImage_t			*pmImageP = NULL;
	FSTATUS				status = FSUCCESS;
	boolean				sth = 0;
	PmHistoryRecord_t	*record = NULL;
	PmCompositeImage_t	*cimg = NULL;

	// check input parameters
	if (!pm || !vfName || !pmVFConfig)
		return(FINVALID_PARAMETER);
	if (vfName[0] == '\0') {
		IB_LOG_WARN_FMT(__func__, "Illegal vfName parameter: Empty String\n");
		return(FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER);
	}

	// initialize vf config port list counts
	pmVFConfig->NumPorts = 0;
	pmVFConfig->portListSize = 0;
	pmVFConfig->portList = NULL;

	AtomicIncrementVoid(&pm->refCount);	// prevent engine from stopping
	if (! PmEngineRunning()) {	// see if is already stopped/stopping
		status = FUNAVAILABLE;
		goto done;
	}

	// pmVFP points to our vf
	// check all ports for membership in our group
	(void)vs_rdlock(&pm->stateLock);
	status = FindImage(pm, IMAGEID_TYPE_ANY, imageId, &imageIndex, &retImageId.imageNumber, &record, &msg, NULL, &cimg);
	if (FSUCCESS != status) {
		IB_LOG_WARN_FMT(__func__, "Unable to get index from ImageId: %s: %s", FSTATUS_ToString(status), msg);
		goto error;
	}

	if (record || cimg) {
		sth = 1;
		int i;
		if (record) {
			status = PmLoadComposite(pm, record, &cimg);
			if (status != FSUCCESS || !cimg) {
				IB_LOG_WARN_FMT(__func__, "Unable to load composite image: %s", FSTATUS_ToString(status));
				goto error;
			}
		}
		retImageId.imageNumber = cimg->header.common.imageIDs[0];
		status = PmReconstitute(&pm->ShortTermHistory, cimg);
		if (record) PmFreeComposite(cimg);
		if (status != FSUCCESS) {
			IB_LOG_WARN_FMT(__func__, "Unable to reconstitute composite image: %s", FSTATUS_ToString(status));
			goto error;
		}
		pmImageP = pm->ShortTermHistory.LoadedImage.img;
		for (i = 0; i < MAX_VFABRICS; i++) {
			if (!strcmp(pm->ShortTermHistory.LoadedImage.VFs[i]->Name, vfName)) {
				pmVFP = pm->ShortTermHistory.LoadedImage.VFs[i];
				break;
			}
		}
		if (!pmVFP) {
			IB_LOG_WARN_FMT(__func__, "VF %.*s not Found", (int)sizeof(vfName), vfName);
			status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_VF;
			goto error;
		}
		imageIndex = 0; // STH always uses imageIndex 0
		if (!pmVFP->Image[imageIndex].isActive) {
			IB_LOG_WARN_FMT(__func__, "VF %.*s not active", (int)sizeof(pmVFP->Name), pmVFP->Name);
			status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_VF;
			goto error;
		}
	} else {
		status = LocateVF(pm, vfName, &pmVFP, 1, imageIndex);
		if (status != FSUCCESS){
			IB_LOG_WARN_FMT(__func__, "VF %.*s not Found: %s", (int)sizeof(vfName), vfName, FSTATUS_ToString(status));
			status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_VF;
			goto error;
		}

		pmImageP = &pm->Image[imageIndex];
		(void)vs_rdlock(&pmImageP->imageLock);
	}

	// Grab ImageTime from Pm Image
	retImageId.imageTime.absoluteTime = (uint32)pmImageP->sweepStart;

	(void)vs_rwunlock(&pm->stateLock);
	for (lid=1; lid<= pmImageP->maxLid; ++lid) {
		uint8 portnum;
		PmNode_t *pmnodep = pmImageP->LidMap[lid];
		if (! pmnodep)
			continue;
		if (pmnodep->nodeType == STL_NODE_SW) {
			for (portnum=0; portnum<=pmnodep->numPorts; ++portnum) {
				PmPort_t *pmportp = pmnodep->up.swPorts[portnum];
				PmPortImage_t *portImage;
				if (! pmportp)
					continue;
				portImage = &pmportp->Image[imageIndex];
				if ( PmIsPortInVF(pm, pmportp, portImage, pmVFP) ) // Port 0 on switches is in all VF
				{
					if (pmVFConfig->portListSize == pmVFConfig->NumPorts) {
						pmVFConfig->portListSize += PORTLISTCHUNK;
					}
					pmVFConfig->NumPorts++;
				}
			}
		} else {
			PmPort_t *pmportp = pmnodep->up.caPortp;
			PmPortImage_t *portImage = &pmportp->Image[imageIndex];
			if (PmIsPortInVF(pm, pmportp, portImage, pmVFP))
			{
				if (pmVFConfig->portListSize == pmVFConfig->NumPorts) {
					pmVFConfig->portListSize += PORTLISTCHUNK;
				}
				pmVFConfig->NumPorts++;
			}
		}
	}
	// check if there are ports to sort
	if (!pmVFConfig->NumPorts) {
		IB_LOG_INFO_FMT(__func__, "VF %.*s has no ports", (int)sizeof(vfName), vfName);
		goto norecords;
	}
	// allocate the port list
	Status_t ret = vs_pool_alloc(&pm_pool, pmVFConfig->portListSize * sizeof(PmPortConfig_t), (void *)&pmVFConfig->portList);
	if (ret !=  VSTATUS_OK) {
		if (!sth) (void)vs_rwunlock(&pmImageP->imageLock);
		IB_LOG_ERRORRC("Failed to allocate list buffer for pmVFConfig rc:", ret);
		status = FINSUFFICIENT_MEMORY;
		goto done;
	}
	// copy the port list
	int i = 0;
	for (lid=1; lid <= pmImageP->maxLid; ++lid) {
		uint8 portnum;
		PmNode_t *pmnodep = pmImageP->LidMap[lid];
		if (!pmnodep) 
			continue;
		if (pmnodep->nodeType == STL_NODE_SW) {
			for (portnum=0; portnum <= pmnodep->numPorts; ++portnum) {
				PmPort_t *pmportp = pmnodep->up.swPorts[portnum];
				PmPortImage_t *portImage;
				if (!pmportp) 
					continue;
				portImage = &pmportp->Image[imageIndex];
				if (PmIsPortInVF(pm, pmportp, portImage, pmVFP)) {
					pmVFConfig->portList[i].lid = lid;
					pmVFConfig->portList[i].portNum = pmportp->portNum;
					pmVFConfig->portList[i].guid = pmnodep->guid;
					memcpy(pmVFConfig->portList[i].nodeDesc, (char *)pmnodep->nodeDesc.NodeString,
						sizeof(pmVFConfig->portList[i].nodeDesc));
					i++;
				}
			}
		} else {
			PmPort_t *pmportp = pmnodep->up.caPortp;
			PmPortImage_t *portImage = &pmportp->Image[imageIndex];
			if (PmIsPortInVF(pm, pmportp, portImage, pmVFP)) {
				pmVFConfig->portList[i].lid = lid;
				pmVFConfig->portList[i].portNum = pmportp->portNum;
				pmVFConfig->portList[i].guid = pmnodep->guid;
				memcpy(pmVFConfig->portList[i].nodeDesc, (char *)pmnodep->nodeDesc.NodeString,
					sizeof(pmVFConfig->portList[i].nodeDesc));
				i++;
			}
		}
	}

norecords:
	cs_strlcpy(pmVFConfig->vfName, pmVFP->Name, STL_PM_VFNAMELEN);
	*returnImageId = retImageId;

	if (!sth) (void)vs_rwunlock(&pmImageP->imageLock);
done:
#ifndef __VXWORKS__
	if (sth) clearLoadedImage(&pm->ShortTermHistory);
#endif
	AtomicDecrementVoid(&pm->refCount);
	return(status);
error:
	(void)vs_rwunlock(&pm->stateLock);
	returnImageId->imageNumber = BAD_IMAGE_ID;
	goto done;
}

FSTATUS GetVfPortCounters(PmCompositeVLCounters_t *vfPortCountersP, const PmVF_t *pmVFP, boolean useHiddenVF,
	const PmPortImage_t *pmPortImageP, PmCompositeVLCounters_t *vlPortCountersP, uint32 *flagsp)
{
	uint32 SingleVLBit, vl, idx;
	uint32 VlSelectMask = 0, VlSelectMaskShared = 0, VFVlSelectMask = 0;
	// Start at -1 if using HiddenVF
	int i = (useHiddenVF ? -1 : 0);
	FSTATUS status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_VF;

	// Iterate through All VFs in Port
	for (; i < (int)pmPortImageP->numVFs; i++) {
		uint32 vlmask = (i == -1 ? 0x8000 : pmPortImageP->vfvlmap[i].vlmask);

		// Iterate through All VLs within each VF
		for (vl = 0; vl < STL_MAX_VLS && (vlmask >> vl); vl++) {
			SingleVLBit = 1 << vl;

			// Skip unassigned VLs (0) in VF
			if ((vlmask & SingleVLBit) == 0) continue;

			// If using Hidden VF only enter when i == -1 (one time for one vl[15])
			// OR is VF pointer matches argument
			if ((i == -1) || (pmPortImageP->vfvlmap[i].pVF == pmVFP)) {

				// Keep track of VLs within this VF
				VFVlSelectMask |= SingleVLBit;
				idx = vl_to_idx(vl);
				vfPortCountersP->PortVLXmitData     += vlPortCountersP[idx].PortVLXmitData;
				vfPortCountersP->PortVLRcvData      += vlPortCountersP[idx].PortVLRcvData;
				vfPortCountersP->PortVLXmitPkts     += vlPortCountersP[idx].PortVLXmitPkts;
				vfPortCountersP->PortVLRcvPkts      += vlPortCountersP[idx].PortVLRcvPkts;
				vfPortCountersP->PortVLXmitWait     += vlPortCountersP[idx].PortVLXmitWait;
				vfPortCountersP->SwPortVLCongestion += vlPortCountersP[idx].SwPortVLCongestion;
				vfPortCountersP->PortVLRcvFECN      += vlPortCountersP[idx].PortVLRcvFECN;
				vfPortCountersP->PortVLRcvBECN      += vlPortCountersP[idx].PortVLRcvBECN;
				vfPortCountersP->PortVLXmitTimeCong += vlPortCountersP[idx].PortVLXmitTimeCong;
				vfPortCountersP->PortVLXmitWastedBW += vlPortCountersP[idx].PortVLXmitWastedBW;
				vfPortCountersP->PortVLXmitWaitData += vlPortCountersP[idx].PortVLXmitWaitData;
				vfPortCountersP->PortVLRcvBubble    += vlPortCountersP[idx].PortVLRcvBubble;
				vfPortCountersP->PortVLMarkFECN     += vlPortCountersP[idx].PortVLMarkFECN;
				vfPortCountersP->PortVLXmitDiscards += vlPortCountersP[idx].PortVLXmitDiscards;

				status = FSUCCESS;
			}

			// If VL bit is already set in Port's VLMask
			if (VlSelectMask & SingleVLBit) {
				// Add bit to indicate this VL is shared between 2 or more VFs
				VlSelectMaskShared |= SingleVLBit;
			} else {
				// If bit is not already set, set it
				VlSelectMask |= SingleVLBit;
			}
		}
	}
	// If one of the VLs in the shared mask is one of the VLs in the VF indicate
	//  this data is shared with at least one other VF
	if (VlSelectMaskShared & VFVlSelectMask) {
		*flagsp |= STL_PA_PC_FLAG_SHARED_VL;
	}

	IB_LOG_DEBUG3_FMT(__func__,
		"vfName: %s, useHiddenVF: %s; Port's VLMask: 0x%08x; Port's Shared VLMask: 0x%08x; VF's VLMask: 0x%08x; Status: 0x%x",
		(pmVFP ? pmVFP->Name : (useHiddenVF ? HIDDEN_VL15_VF : "(NULL)")), useHiddenVF ? "True" : "False",
		VlSelectMask, VlSelectMaskShared, VFVlSelectMask, status);

	return status;
}
FSTATUS paGetVFPortStats(Pm_t *pm, STL_LID_32 lid, uint8 portNum, char *vfName,
	PmCompositeVLCounters_t *vfPortCountersP, uint32 delta, uint32 userCntrs,
	STL_PA_IMAGE_ID_DATA imageId, uint32 *flagsp, STL_PA_IMAGE_ID_DATA *returnImageId)
{
	STL_PA_IMAGE_ID_DATA retImageId = {0};
	FSTATUS				status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_VF;
	PmPort_t			*pmPortP;
	PmPortImage_t		*pmPortImageP;
	const char 			*msg;
	PmImage_t			*pmImageP;
	uint32				imageIndex;
	boolean				sth = 0;
	PmVF_t				*pmVFP = NULL;
	int					i;
	boolean				useHiddenVF = !strcmp(HIDDEN_VL15_VF, vfName);
	PmHistoryRecord_t	*record = NULL;
	PmCompositeImage_t	*cimg = NULL;

	if (!pm || !vfPortCountersP) {
		return(FINVALID_PARAMETER);
	}
	if (userCntrs && (delta || imageId.imageOffset)) {
		IB_LOG_WARN_FMT(__func__,  "Illegal combination of parameters: Offset (%d) and delta(%d) must be zero if UserCounters(%d) flag is set",
						imageId.imageOffset, delta, userCntrs);
		return(FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER);
	}
	if (vfName[0] == '\0') {
		IB_LOG_WARN_FMT(__func__, "Illegal vfName parameter: Empty String\n");
		return(FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER);
	}
	if (!lid) {
		IB_LOG_WARN_FMT(__func__,  "Illegal LID parameter: must not be zero");
		return(FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER);
	}

	AtomicIncrementVoid(&pm->refCount);	// prevent engine from stopping
	if (! PmEngineRunning()) {	// see if is already stopped/stopping
		status = FUNAVAILABLE;
		goto done;
	}

	(void)vs_rdlock(&pm->stateLock);
	if (userCntrs) {
		STL_PA_IMAGE_ID_DATA liveImgId = { 0 };
		status = FindImage(pm, IMAGEID_TYPE_ANY, liveImgId, &imageIndex, &retImageId.imageNumber, &record, &msg, NULL, &cimg);
	} else {
		status = FindImage(pm, IMAGEID_TYPE_ANY, imageId, &imageIndex, &retImageId.imageNumber, &record, &msg, NULL, &cimg);
	}
	if (FSUCCESS != status) {
		IB_LOG_WARN_FMT(__func__, "Unable to get index from ImageId: %s: %s", FSTATUS_ToString(status), msg);
		goto error;
	}

	if (record || cimg) {
		sth = 1;
		if (record) {
			status = PmLoadComposite(pm, record, &cimg);
			if (status != FSUCCESS || !cimg) {
				IB_LOG_WARN_FMT(__func__, "Unable to load composite image: %s", FSTATUS_ToString(status));
				goto error;
			}
		}
		retImageId.imageNumber = cimg->header.common.imageIDs[0];
		status = PmReconstitute(&pm->ShortTermHistory, cimg);
		if (record) PmFreeComposite(cimg);
		if (status != FSUCCESS) {
			IB_LOG_WARN_FMT(__func__, "Unable to reconstitute composite image: %s", FSTATUS_ToString(status));
			goto error;
		}

		if (!useHiddenVF) {
			for (i = 0; i < MAX_VFABRICS; i++) {
				if (!strcmp(pm->ShortTermHistory.LoadedImage.VFs[i]->Name, vfName)) {
					pmVFP = pm->ShortTermHistory.LoadedImage.VFs[i];
					break;
				}
			}
			if (!pmVFP) {
				IB_LOG_WARN_FMT(__func__, "Unable to Get VF Port Stats %s", "No such VF");
				status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_VF;
				goto error;
			}
			imageIndex = 0; // STH always uses imageIndex 0
			if (!pmVFP->Image[imageIndex].isActive) {
				IB_LOG_WARN_FMT(__func__, "VF %.*s not active", (int)sizeof(pmVFP->Name), pmVFP->Name);
				status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_VF;
				goto error;
			}
		}
		pmImageP = pm->ShortTermHistory.LoadedImage.img;
		imageIndex = 0;
	} else {
		if (!useHiddenVF) {
			status = LocateVF(pm, vfName, &pmVFP, 1, imageIndex);
			if (status != FSUCCESS) {
				IB_LOG_WARN_FMT(__func__, "VF %.*s not Found: %s", (int)sizeof(vfName), vfName, FSTATUS_ToString(status));
				status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_VF;
				goto error;
			}
		}

		pmImageP = &pm->Image[imageIndex];
		(void)vs_rdlock(&pmImageP->imageLock);
	}

	pmPortP = pm_find_port(pmImageP, lid, portNum);
	if (! pmPortP) {
		IB_LOG_WARN_FMT(__func__, "Port not found: Lid 0x%x Port %u", lid, portNum);
		status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_PORT;
		goto unlock;
	} 
	pmPortImageP = &pmPortP->Image[imageIndex];
	if (pmPortImageP->u.s.queryStatus != PM_QUERY_STATUS_OK) {
		IB_LOG_WARN_FMT(__func__, "Port Query Status Invalid: %s: Lid 0x%x Port %u",
			(pmPortImageP->u.s.queryStatus == PM_QUERY_STATUS_SKIP ? "Skipped" :
			(pmPortImageP->u.s.queryStatus == PM_QUERY_STATUS_FAIL_QUERY ? "Failed Query" : "Failed Clear")),
			lid, portNum);
		status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_IMAGE;
		goto unlock;
	}

	if (userCntrs) {
		vs_rdlock(&pm->totalsLock);
		// Get Delta VF PortCounters From PA User Counters
		status = GetVfPortCounters(vfPortCountersP, pmVFP, useHiddenVF, pmPortImageP,
			pmPortP->StlVLPortCountersTotal, flagsp);
		vs_rwunlock(&pm->totalsLock);

		*flagsp |= STL_PA_PC_FLAG_USER_COUNTERS |
			(isUnexpectedClearUserCounters ? STL_PA_PC_FLAG_UNEXPECTED_CLEAR : 0);
		*returnImageId = (STL_PA_IMAGE_ID_DATA){0};
	} else {
		// Grab ImageTime from Pm Image
		retImageId.imageTime.absoluteTime = (uint32)pmImageP->sweepStart;

		if (delta) {
			// Get Delta VF PortCounters From Image's PmImage Counters
			status = GetVfPortCounters(vfPortCountersP, pmVFP, useHiddenVF, pmPortImageP,
				pmPortImageP->DeltaStlVLPortCounters, flagsp);
			*flagsp |= STL_PA_PC_FLAG_DELTA;
		} else {
			// Get VF PortCounters From Image's PmImage Counters
			status = GetVfPortCounters(vfPortCountersP, pmVFP, useHiddenVF, pmPortImageP,
				pmPortImageP->StlVLPortCounters, flagsp);
		}
		*flagsp |= (pmPortImageP->u.s.UnexpectedClear ? STL_PA_PC_FLAG_UNEXPECTED_CLEAR : 0);
		*returnImageId = retImageId;
	}

	if (status == FSUCCESS) {
		(void)vs_rwunlock(&pm->stateLock);
	}
unlock:
	if (!sth){
		(void)vs_rwunlock(&pmImageP->imageLock);
	}
	if (status != FSUCCESS) {
		goto error;
	}
done:
	AtomicDecrementVoid(&pm->refCount);
	return(status);
error:
	(void)vs_rwunlock(&pm->stateLock);
	returnImageId->imageNumber = BAD_IMAGE_ID;
	goto done;
}

FSTATUS paClearVFPortStats(Pm_t *pm, STL_LID_32 lid, uint8 portNum, STLVlCounterSelectMask select, char *vfName)
{
	FSTATUS				status = FSUCCESS;
	PmImage_t			*pmimagep;
	uint32				imageIndex;

	if (!vfName) {
		return(FINVALID_PARAMETER);
	}
	if(!lid) {
		IB_LOG_WARN_FMT(__func__, "Illegal LID parameter: Must not be zero\n");
		return(FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER);
	}
	if (!select.AsReg32) {
		IB_LOG_WARN_FMT(__func__, "Illegal select parameter: Must not be zero\n");
		return(FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER);
	}
	if (vfName[0] == '\0') {
		IB_LOG_WARN_FMT(__func__, "Illegal vfName parameter: Empty String\n");
		return(FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER);
	}

	AtomicIncrementVoid(&pm->refCount);	// prevent engine from stopping
	if (! PmEngineRunning()) {	// see if is already stopped/stopping
		status = FUNAVAILABLE;
		goto done;
	}
	(void)vs_rdlock(&pm->stateLock);
	// Live Data only, no offset
	imageIndex = pm->LastSweepIndex;

	if (pm->LastSweepIndex == PM_IMAGE_INDEX_INVALID) {
		IB_LOG_WARN_FMT(__func__, "Unable to Clear PortStats: PM has not completed a sweep.");
		status = FUNAVAILABLE | STL_MAD_STATUS_STL_PA_UNAVAILABLE;
		goto error;
	}
	pmimagep = &pm->Image[imageIndex];
	(void)vs_rdlock(&pmimagep->imageLock);
	(void)vs_rwunlock(&pm->stateLock);

	(void)vs_wrlock(&pm->totalsLock);
	if (portNum == PM_ALL_PORT_SELECT) {
		PmNode_t *pmnodep = pm_find_node(pmimagep, lid);
		if (! pmnodep || pmnodep->nodeType != STL_NODE_SW) {
			IB_LOG_WARN_FMT(__func__, "Switch not found: LID: 0x%x", lid);
			status = FNOT_FOUND;
		} else {
			status = PmClearNodeRunningVFCounters(pmnodep, select, vfName);
		}
	} else {
		PmPort_t	*pmportp = pm_find_port(pmimagep, lid, portNum);
		if (! pmportp) {
			IB_LOG_WARN_FMT(__func__, "Port not found: Lid 0x%x Port %u", lid, portNum);
			status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_PORT;
		} else {
			status = PmClearPortRunningVFCounters(pmportp, select, vfName);
		}
	}
	(void)vs_rwunlock(&pm->totalsLock);

	(void)vs_rwunlock(&pmimagep->imageLock);
done:
	AtomicDecrementVoid(&pm->refCount);
	return(status);
error:
	(void)vs_rwunlock(&pm->stateLock);
	goto done;
}

FSTATUS addVFSortedPorts(PmVFFocusPorts_t *pmVFFocusPorts, sortInfo_t *sortInfo, uint32 imageIndex)
{
	Status_t			status;
	sortedValueEntry_t	*listp;
	int					portCount = 0;

	if (sortInfo->sortedValueListHead == NULL) {
		pmVFFocusPorts->NumPorts = 0;
		return(FSUCCESS);
	}

	status = vs_pool_alloc(&pm_pool, pmVFFocusPorts->NumPorts * sizeof(PmFocusPortEntry_t), (void*)&pmVFFocusPorts->portList);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("Failed to allocate sorted port list buffer for pmVFFocusPorts rc:", status);
		return(FINSUFFICIENT_MEMORY);
	}
	listp = sortInfo->sortedValueListHead;
	while ((listp != NULL) && (portCount < pmVFFocusPorts->NumPorts)) {
		pmVFFocusPorts->portList[portCount].lid = listp->lid;
		pmVFFocusPorts->portList[portCount].portNum = listp->portNum;
		pmVFFocusPorts->portList[portCount].localStatus = listp->localStatus;
		pmVFFocusPorts->portList[portCount].rate = PmCalculateRate(listp->portp->Image[imageIndex].u.s.activeSpeed, listp->portp->Image[imageIndex].u.s.rxActiveWidth);
		pmVFFocusPorts->portList[portCount].mtu = listp->portp->Image[imageIndex].u.s.mtu;
		pmVFFocusPorts->portList[portCount].value = listp->value;
		pmVFFocusPorts->portList[portCount].guid = (uint64_t)(listp->portp->pmnodep->guid);
		strncpy(pmVFFocusPorts->portList[portCount].nodeDesc, (char *)listp->portp->pmnodep->nodeDesc.NodeString,
			sizeof(pmVFFocusPorts->portList[portCount].nodeDesc)-1);
		if (listp->portNum != 0 && listp->neighborPortp != NULL) {
			pmVFFocusPorts->portList[portCount].neighborStatus = listp->neighborStatus;
			pmVFFocusPorts->portList[portCount].neighborLid = listp->neighborPortp->pmnodep->Image[imageIndex].lid;
			pmVFFocusPorts->portList[portCount].neighborPortNum = listp->neighborPortp->portNum;
			pmVFFocusPorts->portList[portCount].neighborValue = listp->neighborValue;
			pmVFFocusPorts->portList[portCount].neighborGuid = (uint64_t)(listp->neighborPortp->pmnodep->guid);
			strncpy(pmVFFocusPorts->portList[portCount].neighborNodeDesc, (char *)listp->neighborPortp->pmnodep->nodeDesc.NodeString,
				sizeof(pmVFFocusPorts->portList[portCount].neighborNodeDesc)-1);
		} else {
			pmVFFocusPorts->portList[portCount].neighborStatus = listp->neighborStatus;
			pmVFFocusPorts->portList[portCount].neighborLid = 0;
			pmVFFocusPorts->portList[portCount].neighborPortNum = 0;
			pmVFFocusPorts->portList[portCount].neighborValue = 0;
			pmVFFocusPorts->portList[portCount].neighborGuid = 0;
			pmVFFocusPorts->portList[portCount].neighborNodeDesc[0] = 0;
		}
		portCount++;
		listp = listp->next;
	}

	return(FSUCCESS);
}

FSTATUS paGetVFFocusPorts(Pm_t *pm, char *vfName, PmVFFocusPorts_t *pmVFFocusPorts,
	STL_PA_IMAGE_ID_DATA imageId, STL_PA_IMAGE_ID_DATA *returnImageId, uint32 select,
	uint32 start, uint32 range)
{
	STL_PA_IMAGE_ID_DATA retImageId = {0};
	PmVF_t				*pmVFP = NULL;
	STL_LID_32			lid;
	uint32				imageIndex, imageInterval;
	const char 			*msg;
	PmImage_t			*pmimagep;
	FSTATUS				status = FSUCCESS;
	Status_t			allocStatus;
	sortInfo_t			sortInfo;
	ComputeFunc_t		computeFunc = NULL;
	CompareFunc_t		compareFunc = NULL;
	CompareFunc_t		candidateFunc = NULL;
	boolean 			sth = 0;
	PmHistoryRecord_t	*record = NULL;
	PmCompositeImage_t	*cimg = NULL;
	void *computeData = NULL;

	// check input parameters
	if (!pm || !vfName || !pmVFFocusPorts)
		return(FINVALID_PARAMETER);
	if (vfName[0] == '\0') {
		IB_LOG_WARN_FMT(__func__, "Illegal vfName parameter: Empty String\n");
		return(FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER);
	}
	if (start) {
		IB_LOG_WARN_FMT(__func__, "Illegal start parameter: %d: must be zero\n", start);
		return(FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER);
	}
	if (!range) {
		IB_LOG_WARN_FMT(__func__, "Illegal range parameter: %d: must be greater than zero\n", range);
		return(FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER);
	}
	switch (select) {
	case STL_PA_SELECT_UTIL_HIGH:
		computeFunc = &computeUtilizationPct10;
		compareFunc = &compareLE;
		candidateFunc = &compareGT;
		computeData = (void *)&imageInterval;
		break;
	case STL_PA_SELECT_UTIL_PKTS_HIGH:
		computeFunc = &computeSendKPkts;
		compareFunc = &compareLE;
		candidateFunc = &compareGT;
		computeData = (void *)&imageInterval;
		break;
	case STL_PA_SELECT_UTIL_LOW:
		computeFunc = &computeUtilizationPct10;
		compareFunc = &compareGE;
		candidateFunc = &compareLT;
		computeData = (void *)&imageInterval;
		break;
	case STL_PA_SELECT_CATEGORY_INTEG:
		computeFunc = &computeIntegrity;
		compareFunc = &compareLE;
		candidateFunc = &compareGT;
		computeData = &pm->integrityWeights;
		break;
	case STL_PA_SELECT_CATEGORY_CONG:
		computeFunc = &computeCongestion;
		compareFunc = &compareLE;
		candidateFunc = &compareGT;
		computeData = &pm->congestionWeights;
		break;
	case STL_PA_SELECT_CATEGORY_SMA_CONG:
		computeFunc = &computeSmaCongestion;
		compareFunc = &compareLE;
		candidateFunc = &compareGT;
		computeData = &pm->congestionWeights;
		break;
	case STL_PA_SELECT_CATEGORY_BUBBLE:
		computeFunc = &computeBubble;
		compareFunc = &compareLE;
		candidateFunc = &compareGT;
		break;
	case STL_PA_SELECT_CATEGORY_SEC:
		computeFunc = &computeSecurity;
		compareFunc = &compareLE;
		candidateFunc = &compareGT;
		break;
	case STL_PA_SELECT_CATEGORY_ROUT:
		computeFunc = &computeRouting;
		compareFunc = &compareLE;
		candidateFunc = &compareGT;
		break;
	default:
		IB_LOG_WARN_FMT(__func__, "Illegal select parameter: 0x%x\n", select);
		return(FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER);
		break;
	}

	// initialize group config port list counts
	pmVFFocusPorts->NumPorts = 0;
	pmVFFocusPorts->portList = NULL;
	memset(&sortInfo, 0, sizeof(sortInfo));
	allocStatus = vs_pool_alloc(&pm_pool, range * sizeof(sortedValueEntry_t), (void*)&sortInfo.sortedValueListPool);
	if (allocStatus != VSTATUS_OK) {
		IB_LOG_ERRORRC("Failed to allocate sort list buffer for pmVFFocusPorts rc:", allocStatus);
		return FINSUFFICIENT_MEMORY;
	}
	sortInfo.sortedValueListHead = NULL;
	sortInfo.sortedValueListTail = NULL;
	sortInfo.sortedValueListSize = range;
	sortInfo.numValueEntries = 0;

	AtomicIncrementVoid(&pm->refCount);	// prevent engine from stopping
	if (! PmEngineRunning()) {	// see if is already stopped/stopping
		status = FUNAVAILABLE;
		goto done;
	}

	(void)vs_rdlock(&pm->stateLock);
	status = FindImage(pm, IMAGEID_TYPE_ANY, imageId, &imageIndex, &retImageId.imageNumber, &record, &msg, NULL, &cimg);
	if (FSUCCESS != status) {
		IB_LOG_WARN_FMT(__func__, "Unable to get index from ImageId: %s: %s", FSTATUS_ToString(status), msg);
		goto error;
	}
	if (record || cimg) {
		int i;
		sth = 1;
		if (record) {
			status = PmLoadComposite(pm, record, &cimg);
			if (status != FSUCCESS || !cimg) {
				IB_LOG_WARN_FMT(__func__, "Unable to load composite image: %s", FSTATUS_ToString(status));
				goto error;
			}
		}
		retImageId.imageNumber = cimg->header.common.imageIDs[0];
		imageInterval = cimg->header.common.imageSweepInterval;
		status = PmReconstitute(&pm->ShortTermHistory, cimg);
		if (record) PmFreeComposite(cimg);
		if (status != FSUCCESS) {
			IB_LOG_WARN_FMT(__func__, "Unable to reconstitute composite image: %s", FSTATUS_ToString(status));
			goto error;
		}
		pmimagep = pm->ShortTermHistory.LoadedImage.img;
		for (i = 0; i < MAX_VFABRICS; i++) {
			if (!strcmp(pm->ShortTermHistory.LoadedImage.VFs[i]->Name, vfName)) {
				pmVFP = pm->ShortTermHistory.LoadedImage.VFs[i];
				break;
			}
		}
		if (!pmVFP) {
			IB_LOG_WARN_FMT(__func__, "VF %.*s not Found", (int)sizeof(vfName), vfName);
			status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_VF;
			goto error;
		}
		imageIndex = 0; // STH always uses imageIndex 0
		if (!pmVFP->Image[imageIndex].isActive) {
			IB_LOG_WARN_FMT(__func__, "VF %.*s not active", (int)sizeof(pmVFP->Name), pmVFP->Name);
			status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_VF;
			goto error;
		}
	} else {
		status = LocateVF(pm, vfName, &pmVFP, 1, imageIndex);
		if (status != FSUCCESS) {
			IB_LOG_WARN_FMT(__func__, "VF %.*s not Found: %s", (int)sizeof(vfName), vfName, FSTATUS_ToString(status));
			status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_VF;
			goto error;
		}
		pmimagep = &pm->Image[imageIndex];
		imageInterval = MAX(pm->interval, (pmimagep->sweepDuration/1000000));
		(void)vs_rdlock(&pmimagep->imageLock);
	}

	// Grab ImageTime from Pm Image
	retImageId.imageTime.absoluteTime = (uint32)pmimagep->sweepStart;

	(void)vs_rwunlock(&pm->stateLock);
	for (lid = 1; lid <= pmimagep->maxLid; ++lid) {
		uint8 portnum;
		PmNode_t *pmnodep = pmimagep->LidMap[lid];
		if (!pmnodep)
			continue;
		if (pmnodep->nodeType == STL_NODE_SW) {
			for (portnum = 0; portnum <= pmnodep->numPorts; ++portnum) {
				PmPort_t *pmportp = pmnodep->up.swPorts[portnum];
				PmPortImage_t *portImage;
				if (!pmportp) continue;
				portImage = &pmportp->Image[imageIndex];
				if (PmIsPortInVF(pm, pmportp, portImage, pmVFP)) {
					processFocusPort(pm, pmportp, portImage, imageIndex, lid, portnum, 
						computeFunc, compareFunc, candidateFunc, computeData, &sortInfo);
				}
			}
		} else {
			PmPort_t *pmportp = pmnodep->up.caPortp;
			PmPortImage_t *portImage = &pmportp->Image[imageIndex];
			if (PmIsPortInVF(pm, pmportp, portImage, pmVFP)) {
				processFocusPort(pm, pmportp, portImage, imageIndex, lid, pmportp->portNum,
					computeFunc, compareFunc, candidateFunc, computeData, &sortInfo);
			}
		}
	}

	cs_strlcpy(pmVFFocusPorts->vfName, pmVFP->Name, STL_PM_VFNAMELEN);
	pmVFFocusPorts->NumPorts = sortInfo.numValueEntries;
	if (pmVFFocusPorts->NumPorts)
		status = addVFSortedPorts(pmVFFocusPorts, &sortInfo, imageIndex);
	if (!sth) {
		(void)vs_rwunlock(&pmimagep->imageLock);
	}
	if (status != FSUCCESS)
		goto done;
	*returnImageId = retImageId;

done:
	if (sortInfo.sortedValueListPool != NULL)
		vs_pool_free(&pm_pool, sortInfo.sortedValueListPool);
	if (sth) {
#ifndef __VXWORKS__
		clearLoadedImage(&pm->ShortTermHistory);
#endif
	}

	AtomicDecrementVoid(&pm->refCount);
	return(status);
error:
	(void)vs_rwunlock(&pm->stateLock);
	returnImageId->imageNumber = BAD_IMAGE_ID;
	goto done;
}

// Append details about frozen images into memory pointed to by buffer 
static void appendFreezeFrameDetails(uint8_t *buffer, uint32_t *index)
{
	extern Pm_t 	g_pmSweepData;
	Pm_t			*pm=&g_pmSweepData;
	uint8			freezeIndex;
	uint8			numFreezeImages=0;
	uint64_t		ffImageId=0;

	for (freezeIndex = 0; freezeIndex < pm_config.freeze_frame_images; freezeIndex ++) {
		if (pm->freezeFrames[freezeIndex] != PM_IMAGE_INDEX_INVALID) {
			numFreezeImages++;
		}
	}
	if (pm_config.shortTermHistory.enable && pm->ShortTermHistory.cachedComposite) {
		numFreezeImages++;
	}
	buffer[(*index)++]=numFreezeImages;
	if (numFreezeImages == 0) {
		return;
	}
	for (freezeIndex = 0; freezeIndex < pm_config.freeze_frame_images; freezeIndex ++) {
		if (pm->freezeFrames[freezeIndex] != PM_IMAGE_INDEX_INVALID) {
			ffImageId = BuildFreezeFrameImageId(pm, freezeIndex, 0 /*client id */, NULL);
			memcpy(&buffer[*index], &ffImageId, sizeof(uint64_t));
			*index += sizeof(uint64_t);
			IB_LOG_VERBOSELX("Appending freeze frame id", ffImageId);
		}
	}
	if (pm_config.shortTermHistory.enable && pm->ShortTermHistory.cachedComposite) {
		ffImageId = pm->ShortTermHistory.cachedComposite->header.common.imageIDs[0];
		memcpy(&buffer[*index], &ffImageId, sizeof(uint64_t));
		*index += sizeof(uint64_t);
		IB_LOG_VERBOSELX("Appending Hist freeze frame id", ffImageId);
	}
	return;
}

// return Master PM Sweep History image file data copied into memory pointed to by buffer 
int getPMHistFileData(char *histPath, char *filename, uint32_t histindex, uint8_t *buffer, uint32_t bufflen, uint32_t *filelen)
{
    const size_t MAX_PATH = PM_HISTORY_FILENAME_LEN + 1 + 256;
    char path[MAX_PATH];
    uint32_t      index=0, nextByte = 0;
    FILE          *file;

    if (!buffer || !filelen) 
        return -1;

    if (filename[0] == '\0') {
        IB_LOG_VERBOSE_FMT(__func__, "Missing hist filename.");
        return -1;
    }

    snprintf(path, MAX_PATH, "%s/%s", histPath, filename);
    file = fopen(path, "r" );
    if (!file) {
        IB_LOG_ERROR("Error opening PA history image file! rc:",0x0020);
        return -1;
    }
    while((nextByte = fgetc(file)) != EOF) {
        buffer[index++] = nextByte;
        if (index >= bufflen) {
            *filelen = 0;
            IB_LOG_ERROR("PM Composite image file overrunning buffer! rc:",0x0020);
            fclose(file);
            return -1;
        }
    }
    *filelen = index;
    fclose(file);

    return 0;

}	// End of getPMHistFileData()

// return latest Master PM Sweep Image Data copied into memory pointed to by buffer 
int getPMSweepImageData(char *filename, uint32_t histindex, uint8_t isCompressed, uint8_t *buffer, uint32_t bufflen, uint32_t *filelen)
{
	extern Pm_t g_pmSweepData;
	Pm_t	*pm=&g_pmSweepData;
	uint32_t	index=0;

	if (!buffer || !filelen) 
		return -1;

	if (! PmEngineRunning()) {	// see if is already stopped/stopping
		return -1;
	}

	if (filename[0] != '\0') {
		return getPMHistFileData(pm->ShortTermHistory.filepath, filename, histindex, buffer, bufflen, filelen);
	}
	else { /* file name not specified */
		if (pm->history[histindex] == PM_IMAGE_INDEX_INVALID) {
			return -1;
		}
		if (pm->Image[pm->history[histindex]].state != PM_IMAGE_VALID) {
			IB_LOG_VERBOSE_FMT(__func__,"Invalid history index :0x%x", histindex);
			return -1;
		}
		IB_LOG_VERBOSE_FMT(__func__, "Going to send latest hist imageIndex=0x%x size=%"PRISZT, histindex, computeCompositeSize());
		snprintf(filename, SMDBSYNCFILE_NAME_LEN, "%s/latest_sweep", pm->ShortTermHistory.filepath);
		writeImageToBuffer(pm, histindex, isCompressed, buffer, &index);
		appendFreezeFrameDetails(buffer, &index);
		*filelen = index;
	}
	return 0;
}	// End of getPMSweepImageData()

// copy latest PM Sweep Image Data received from Master PM to Standby PM.
int putPMSweepImageData(char *filename, uint8_t *buffer, uint32_t filelen)
{
	extern Pm_t g_pmSweepData;
	Pm_t		*pm = &g_pmSweepData;

	if (!buffer || !filename) {
		IB_LOG_VERBOSE_FMT(__func__, "Null buffer/file");
		return -1;
	}

	if (!pm_config.shortTermHistory.enable || !PmEngineRunning()) {	// see if is already stopped/stopping
		return -1;
	}

	return injectHistoryFile(pm, filename, buffer, filelen);

}	// End of putPMSweepImageData()

// Temporary declarations
FSTATUS compoundNewImage(Pm_t *pm);
boolean PmCompareHFIPort(PmPort_t *pmportp, char *groupName);
boolean PmCompareTCAPort(PmPort_t *pmportp, char *groupName);
boolean PmCompareSWPort(PmPort_t *pmportp, char *groupName);

extern void release_pmnode(Pm_t *pm, PmNode_t *pmnodep);
extern void free_pmportList(Pm_t *pm, PmNode_t *pmnodep);
extern void free_pmport(Pm_t *pm, PmPort_t *pmportp);
extern PmNode_t *get_pmnodep(Pm_t *pm, Guid_t guid, uint16_t lid);
extern PmPort_t *new_pmport(Pm_t *pm);
extern uint32 connect_neighbor(Pm_t *pm, PmPort_t *pmportp);

// Free Node List
void freeNodeList(Pm_t *pm, PmImage_t *pmimagep) {
	uint32_t	i;
	PmNode_t	*pmnodep;

	if (pmimagep->LidMap) {
		for (i = 0; i <= pmimagep->maxLid; i++) {
			pmnodep = pmimagep->LidMap[i];
			if (pmnodep) {
				release_pmnode(pm, pmnodep);
			}
		}
		vs_pool_free(&pm_pool, pmimagep->LidMap);
		pmimagep->LidMap = NULL;
	}
	return;

}	// End of freeNodeList()

// Find Group by name
static FSTATUS FindPmGroup(Pm_t *pm, PmGroup_t **pmgrouppp, PmGroup_t *sthgroupp) {
    FSTATUS		ret = FNOT_FOUND;
	uint32_t	i;

	if (!pmgrouppp) {
		ret = FINVALID_PARAMETER;
		goto exit;
	}

	*pmgrouppp = NULL;
	if (sthgroupp) {
		if (!pm->AllPorts) {
			ret = FINVALID_PARAMETER;	// Can't reference NULL data
			goto exit;
		}
		if (!strcmp(sthgroupp->Name, pm->AllPorts->Name)) {
			*pmgrouppp = pm->AllPorts;
			ret = FSUCCESS;
			goto exit;
		}
		for (i = 0; i < pm->NumGroups; i++) {
			if (!pm->Groups[i]) {
				ret = FINVALID_PARAMETER;	// Can't reference NULL data
				goto exit;
			}
			if (!strcmp(sthgroupp->Name, pm->Groups[i]->Name)) {
				*pmgrouppp = pm->Groups[i];
				ret = FSUCCESS;
				goto exit;
			}
		}
	}

exit:
	return ret;

}	// End of FindPmGroup()

// Find VF by name
static FSTATUS FindPmVF(Pm_t *pm, PmVF_t **pmvfpp, PmVF_t *sthvfp) {
    FSTATUS		ret = FNOT_FOUND;
	uint32_t	i;

	if (!pmvfpp) {
		ret = FINVALID_PARAMETER;
		goto exit;
	}

	*pmvfpp = NULL;
	if (sthvfp) {
		for (i = 0; i < pm->numVFs; i++) {
			if (!pm->VFs[i]) {
				ret = FINVALID_PARAMETER;	// Can't reference NULL data
				goto exit;
			}
			if (!strcmp(sthvfp->Name, pm->VFs[i]->Name)) {
				*pmvfpp = pm->VFs[i];
				ret = FSUCCESS;
				goto exit;
			}
		}
	}
exit:
	return ret;

}	// End of FindPmVF()

// Copy Short Term History Port to PmImage Port
FSTATUS CopyPortToPmImage(Pm_t *pm, PmNode_t *pmnodep, PmPort_t **pmportpp, PmPort_t *sthportp) {
    FSTATUS			ret = FSUCCESS;
	uint32_t		i, j;
	PmPort_t		*pmportp = NULL;
	PmPortImage_t	*pmportimgp;
	PmPortImage_t	*sthportimgp;

	if (!pmnodep || !pmportpp) {
		return FINVALID_PARAMETER;
	}

	if (!sthportp || (!sthportp->guid && !sthportp->portNum)) {
		// No port to copy
		goto exit;
	}

	pmportp = *pmportpp;

	// Allocate port
	if (!pmportp) {
		pmportp = new_pmport(pm);
		if (!pmportp) goto exit;

		pmportp->guid = sthportp->guid;
		pmportp->portNum = sthportp->portNum;
		pmportp->capmask = sthportp->capmask;
		pmportp->pmnodep = pmnodep;
	}
	pmportp->u.AsReg8 = sthportp->u.AsReg8;
	pmportp->neighbor_lid = sthportp->neighbor_lid;
	pmportp->neighbor_portNum = sthportp->neighbor_portNum;
	pmportp->groupWarnings = sthportp->groupWarnings;
	pmportp->StlPortCountersTotal = sthportp->StlPortCountersTotal;
	for (i = 0; i < MAX_PM_VLS; i++)
		pmportp->StlVLPortCountersTotal[i] = sthportp->StlVLPortCountersTotal[i];

	// Don't Copy port counters. Reconstituted image doesn't contain counters.
	
	// Copy port image
	pmportimgp = &pmportp->Image[pm->SweepIndex];
	sthportimgp = &sthportp->Image[0];
	*pmportimgp = *sthportimgp;

	// Connect neighbors later
	pmportimgp->neighbor = NULL;

	// Copy port image port groups
	memset(&pmportimgp->Groups, 0, sizeof(PmGroup_t *) * PM_MAX_GROUPS_PER_PORT);
#if PM_COMPRESS_GROUPS
	pmportimgp->numGroups = 0;
	for (j = 0, i=0; i < sthportimgp->numGroups; i++)
#else
	for (j = 0, i=0; i<PM_MAX_GROUPS_PER_PORT; i++)
#endif
	{
		if (!sthportimgp->Groups[i]) continue;
		ret = FindPmGroup(pm, &pmportimgp->Groups[j], sthportimgp->Groups[i]);
		if (ret == FSUCCESS) {
			j++;
		} else if (ret == FNOT_FOUND) {
			IB_LOG_ERROR_FMT(__func__, "Port group not found: %s", sthportimgp->Groups[i]->Name);
			ret = FSUCCESS;
		} else {
			IB_LOG_ERROR_FMT(__func__, "Error in Port Image Group:%d", ret);
			goto exit_dealloc;
		}
	}
#if PM_COMPRESS_GROUPS
	pmportimgp->numGroups = j;
#endif

	// Copy port image VF groups
	memset(&pmportimgp->vfvlmap, 0, sizeof(vfmap_t) * MAX_VFABRICS);
	for (j = 0, i = 0; i < sthportimgp->numVFs; i++) {
		if (!sthportimgp->vfvlmap[i].pVF) continue;
		ret = FindPmVF(pm, &pmportimgp->vfvlmap[j].pVF, sthportimgp->vfvlmap[i].pVF);
		if (ret == FSUCCESS) {
			pmportimgp->vfvlmap[j].vlmask = sthportimgp->vfvlmap[i].vlmask;
			j++;
		} else if (ret == FNOT_FOUND) {
			IB_LOG_ERROR_FMT(__func__, "Virtual Fabric not found: %s", sthportimgp->vfvlmap[i].pVF->Name);
			ret = FSUCCESS;
		} else {
			IB_LOG_ERROR_FMT(__func__, "Error in Port Image VF:%d", ret);
			goto exit_dealloc;
		}
	}

	goto exit;

exit_dealloc:
	free_pmport(pm, pmportp);
	pmportp = NULL;

exit:
	// Update *pmportpp
	*pmportpp = pmportp;
	return ret;

}	// End of CopyPortToPmImage()

// Copy Short Term History Node to PmImage Node
FSTATUS CopyNodeToPmImage(Pm_t *pm, uint16_t lid, PmNode_t **pmnodepp, PmNode_t *sthnodep) {
    FSTATUS		ret = FSUCCESS;
	Status_t	rc;
	uint32_t	i;
	PmNode_t	*pmnodep;
	Guid_t		guid;

	if (!pmnodepp) {
		ret = FINVALID_PARAMETER;
		goto exit;
	}

	if (!sthnodep) {
		// No node to copy
		goto exit;
	}

	if (sthnodep->nodeType == STL_NODE_SW)
		guid = sthnodep->up.swPorts[0]->guid;
	else
		guid = sthnodep->up.caPortp->guid;

	pmnodep = get_pmnodep(pm, guid, lid);
	if (pmnodep) {
		ASSERT(pmnodep->nodeType == sthnodep->nodeType);
		ASSERT(pmnodep->numPorts == sthnodep->numPorts);
	} else {
		// Allocate node
		rc = vs_pool_alloc(&pm_pool, pm->PmNodeSize, (void *)&pmnodep);
		*pmnodepp = pmnodep;
		if (rc != VSTATUS_OK || !pmnodep) {
			IB_LOG_ERRORRC( "Failed to allocate PM Node rc:", rc);
			ret = FINSUFFICIENT_MEMORY;
			goto exit;
		}
		MemoryClear(pmnodep, pm->PmNodeSize);
		AtomicWrite(&pmnodep->refCount, 1);
		pmnodep->nodeType = sthnodep->nodeType;
		pmnodep->numPorts = sthnodep->numPorts;
		pmnodep->guid = guid;
		cl_map_item_t *mi;
    	mi = cl_qmap_insert(&pm->AllNodes, guid, &pmnodep->AllNodesEntry);
   		if (mi != &pmnodep->AllNodesEntry) {
        		IB_LOG_ERRORLX("duplicate Node for portGuid", guid);
        		goto exit_dealloc;
    	}
	}
	pmnodep->nodeDesc = sthnodep->nodeDesc;
	pmnodep->changed_count = pm->SweepIndex;
	pmnodep->deviceRevision = sthnodep->deviceRevision;
	pmnodep->u = sthnodep->u;
	pmnodep->dlid = sthnodep->dlid;
	pmnodep->pkey = sthnodep->pkey;
	pmnodep->qpn = sthnodep->qpn;
	pmnodep->qkey = sthnodep->qkey;
	pmnodep->Image[pm->SweepIndex] = sthnodep->Image[0];

	if (sthnodep->nodeType == STL_NODE_SW) {
		if (!pmnodep->up.swPorts) {
			rc = vs_pool_alloc(&pm_pool, sizeof(PmPort_t *) * (pmnodep->numPorts + 1), (void *)&pmnodep->up.swPorts);
			if (rc != VSTATUS_OK || !pmnodep->up.swPorts) {
				IB_LOG_ERRORRC( "Failed to allocate Node Port List rc:", rc);
				ret = FINSUFFICIENT_MEMORY;
				goto exit_dealloc;
			}
			MemoryClear(pmnodep->up.swPorts, sizeof(PmPort_t *) * (pmnodep->numPorts + 1));
		}
		for (i = 0; i <= pmnodep->numPorts; i++) {
			ret = CopyPortToPmImage(pm, pmnodep, &pmnodep->up.swPorts[i], sthnodep ? sthnodep->up.swPorts[i] : NULL);
			if (ret != FSUCCESS) {
				IB_LOG_ERROR_FMT(__func__, "Error in Port Copy:%d", ret);
				goto exit_dealloc;
			}
		}
	} else {
		ret = CopyPortToPmImage(pm, pmnodep, &pmnodep->up.caPortp, sthnodep ? sthnodep->up.caPortp : NULL);
		if (ret != FSUCCESS) {
			IB_LOG_ERROR_FMT(__func__, "Error in Port Copy:%d", ret);
			goto exit_dealloc;
		}
	}

	// Update *pmnodepp
	*pmnodepp = pmnodep;

	goto exit;

exit_dealloc:
	if (pmnodep)
		release_pmnode(pm, pmnodep);
	*pmnodepp = NULL;

exit:
	return ret;

}	// End of CopyNodeToPmImage()

void set_neighbor(Pm_t *pm, PmPort_t *pmportp)
{
	PmPort_t *neighbor;
	PmPortImage_t *portImage = &pmportp->Image[pm->SweepIndex];

	if (portImage->u.s.Initialized && pmportp->neighbor_lid) {
		neighbor = pm_find_port(&pm->Image[pm->SweepIndex], pmportp->neighbor_lid, pmportp->neighbor_portNum);
		if (neighbor) {
			portImage->neighbor = neighbor;
		}
	}
}

// Copy Short Term History Image to PmImage
FSTATUS CopyToPmImage(Pm_t *pm, PmImage_t *pmimagep, PmImage_t *sthimagep) {
    FSTATUS		ret = FSUCCESS;
	Status_t	rc;
	uint32_t	i, j;
    Lock_t		orgImageLock;	// Lock image data (except state and imageId).

	if (!pmimagep || !sthimagep) {
		ret = FINVALID_PARAMETER;
		goto exit;
	}

	// retain PmImage lock 
    orgImageLock = pmimagep->imageLock;

	*pmimagep = *sthimagep;
	pmimagep->LidMap = NULL;
	rc = vs_pool_alloc(&pm_pool, sizeof(PmNode_t *) * (pmimagep->maxLid + 1), (void *)&pmimagep->LidMap);
	if (rc != VSTATUS_OK || !pmimagep->LidMap) {
		IB_LOG_ERRORRC( "Failed to allocate PM Lid Map rc:", rc);
		ret = FINSUFFICIENT_MEMORY;
		goto exit_dealloc;
	}
	MemoryClear(pmimagep->LidMap, sizeof(PmNode_t *) * (pmimagep->maxLid + 1));
	for (i = 1; i <= pmimagep->maxLid; i++) {
		ret = CopyNodeToPmImage(pm, i, &pmimagep->LidMap[i], sthimagep->LidMap[i]);
		if (ret != FSUCCESS) {
			IB_LOG_ERROR_FMT(__func__, "Error in Node Copy:%d", ret);
			goto exit_dealloc;
		}
	}
	for (i = 1; i <= pmimagep->maxLid; i++) {
		if (pmimagep->LidMap[i]) {
			if (pmimagep->LidMap[i]->nodeType == STL_NODE_SW) {
				for (j = 0; j <= pmimagep->LidMap[i]->numPorts; j++) {
					if (pmimagep->LidMap[i]->up.swPorts[j]) {
						set_neighbor(pm, pmimagep->LidMap[i]->up.swPorts[j]);
					}
				}
			} else {
				set_neighbor(pm, pmimagep->LidMap[i]->up.caPortp);
			}
		}
	}

	goto exit_unlock;

exit_dealloc:
	freeNodeList(pm, pmimagep);

exit_unlock:
	// restore PmImage lock 
    pmimagep->imageLock = orgImageLock;

exit:
	return ret;

}	// End of CopyToPmImage()

// Integrate ShortTermHistory.LoadedImage into PM RAM-resident image storage
FSTATUS PmReintegrate(Pm_t *pm, PmShortTermHistory_t *sth) {
    FSTATUS		ret = FSUCCESS;
	PmImage_t	*pmimagep;
	PmImage_t	*sthimagep;
	PmGroup_t	*pmgroupp;
	PmVF_t		*pmvfp;
	uint32_t	i;

	pmimagep = &pm->Image[pm->SweepIndex];
	sthimagep = sth->LoadedImage.img;

	// More image processing (from PmSweepAllPortCounters)
	pmimagep->NoRespNodes = pmimagep->NoRespPorts = 0;
	pmimagep->SkippedNodes = pmimagep->SkippedPorts = 0;
	pmimagep->UnexpectedClearPorts = 0;
	pmimagep->DowngradedPorts = 0;
//	(void)PmClearAllNodes(pm);

	freeNodeList(pm, pmimagep);	// Free old Node List (LidMap) if present

	// Copy *LoadedImage.AllGroup
	pm->AllPorts->Image[pm->SweepIndex] = sth->LoadedImage.AllGroup->Image[0];

	// Copy *LoadedImage.Groups[]
	for (i = 0; i < PM_MAX_GROUPS; i++) {
		if (sth->LoadedImage.Groups[i]) {
			ret = FindPmGroup(pm, &pmgroupp, sth->LoadedImage.Groups[i]);
			if (ret == FSUCCESS) {
				pmgroupp->Image[pm->SweepIndex] = sth->LoadedImage.Groups[i]->Image[0];
			}
		}
	}

	// Copy *LoadedImage.VFs[]
	for (i = 0; i < MAX_VFABRICS; i++) {
		if (sth->LoadedImage.VFs[i]) {
			ret = FindPmVF(pm, &pmvfp, sth->LoadedImage.VFs[i]);
			if (ret == FSUCCESS) {
				pmvfp->Image[pm->SweepIndex] = sth->LoadedImage.VFs[i]->Image[0];
			}
		}
	}

	// Copy *LoadedImage.img
	ret = CopyToPmImage(pm, pmimagep, sthimagep);
	if (ret != FSUCCESS) {
		IB_LOG_ERROR_FMT(__func__, "Error in Image Copy:%d", ret);
		goto exit;
	}

exit:
	return ret;

}	// End of PmReintegrate()

// copy latest PM RAM-Resident Sweep Image Data received from Master PM to Standby PM.
FSTATUS putPMSweepImageDataR(uint8_t *p_img_in, uint32_t len_img_in) {
    FSTATUS		ret = FSUCCESS;
	uint64		size;
	uint32		sweep_num;
	time_t		now_time;
	extern		Pm_t g_pmSweepData;
	Pm_t		*pm = &g_pmSweepData;
	PmShortTermHistory_t *sth = &pm->ShortTermHistory;
	PmImage_t	*pmimagep;
#ifndef __VXWORKS__
	Status_t	status;
#endif
	PmCompositeImage_t	*cimg_in = (PmCompositeImage_t *)p_img_in;
	PmCompositeImage_t	*cimg_out = NULL;
	unsigned char *p_decompress = NULL;
	unsigned char *bf_decompress = NULL;
	static time_t		firstImageSweepStart;
	static uint32		processedSweepNum=0;
	uint32				history_version;
	double				isTdelta;
	boolean				skipCompounding = FALSE;
	uint8		tempInstanceId;

    if (!p_img_in || !len_img_in)
        return FINVALID_PARAMETER;

	if (!PmEngineRunning()) {	// see if is already stopped/stopping
		return -1;
	}

	// check the version
	history_version = cimg_in->header.common.historyVersion;
	BSWAP_PM_HISTORY_VERSION(&history_version);
	if (history_version != PM_HISTORY_VERSION) {
		IB_LOG_INFO_FMT(__func__, "Received image buffer version (v%u) does not match current version: v%u",
			history_version, PM_HISTORY_VERSION);
        return FINVALID_PARAMETER;
	}

	BSWAP_PM_FILE_HEADER(&cimg_in->header);
#ifndef __VXWORKS__
	// Decompress image if compressed
	if (cimg_in->header.common.isCompressed) { 
		status = vs_pool_alloc(&pm_pool, cimg_in->header.flatSize, (void *)&bf_decompress);
		if (status != VSTATUS_OK || !bf_decompress) {
			IB_LOG_ERRORRC("Unable to allocate flat buffer rc:", status);
			ret = FINSUFFICIENT_MEMORY;
			goto exit_free;
		}
		MemoryClear(bf_decompress, cimg_in->header.flatSize);
		// copy the header
		memcpy(bf_decompress, p_img_in, sizeof(PmFileHeader_t));
		// decompress the rest
		ret = decompressAndReassemble(p_img_in + sizeof(PmFileHeader_t),
									  len_img_in - sizeof(PmFileHeader_t),
									  cimg_in->header.numDivisions, 
									  cimg_in->header.divisionSizes, 
									  bf_decompress + sizeof(PmFileHeader_t), 
									  cimg_in->header.flatSize - sizeof(PmFileHeader_t));
		if (ret != FSUCCESS) {
			IB_LOG_ERROR0("Unable to decompress image buffer");
			goto exit_free;
		}
		p_decompress = bf_decompress;
	} else {
#endif
		p_decompress = p_img_in;
#ifndef __VXWORKS__
	}
#endif
	BSWAP_PM_COMPOSITE_IMAGE_FLAT((PmCompositeImage_t *)p_decompress, 0, history_version);

	// Rebuild composite
	//status = vs_pool_alloc(&pm_pool, sizeof(PmCompositeImage_t), (void *)&cimg_out);
	cimg_out = calloc(1,sizeof(PmCompositeImage_t));
	if (!cimg_out) {
		IB_LOG_ERROR0("Unable to allocate image buffer");
		ret = FINSUFFICIENT_MEMORY;
		goto exit_free;
	}

	MemoryClear(cimg_out, sizeof(PmCompositeImage_t));
	memcpy(cimg_out, p_decompress, sizeof(PmFileHeader_t));
	// Get sweepNum from image ID
	sweep_num = ((ImageId_t)cimg_out->header.common.imageIDs[0]).s.sweepNum;
	// Get the instance ID from the image ID
	tempInstanceId = ((ImageId_t)cimg_out->header.common.imageIDs[0]).s.instanceId;
	ret = rebuildComposite(cimg_out, p_decompress + sizeof(PmFileHeader_t), history_version);
	if (ret != FSUCCESS) {
		IB_LOG_ERRORRC("Error rebuilding PM Composite Image rc:", ret);
		goto exit_free;
	}

	(void)vs_wrlock(&pm->stateLock);
	pmimagep = &pm->Image[pm->SweepIndex];
	(void)vs_wrlock(&pmimagep->imageLock);

	// Reconstitute composite
	ret = PmReconstitute(sth, cimg_out);
	if (ret != FSUCCESS) {
		IB_LOG_ERRORRC("Error reconstituting PM Composite Image rc:", ret);
		goto exit_unlock;
	}

	if (isFirstImg) {
		firstImageSweepStart = sth->LoadedImage.img->sweepStart;
		IB_LOG_INFO_FMT(__func__, "First New Sweep Image Received...");
		processedSweepNum = sweep_num;
	}
	else {
		/* compare received image sweepStart time with that of first image. */
		isTdelta = difftime(firstImageSweepStart, sth->LoadedImage.img->sweepStart);
		if (isTdelta > 0) { /* firstImageSweepStart time is greater; i.e. received an older RAM image.*/
			skipCompounding = TRUE;
			IB_LOG_INFO_FMT( __func__, "TDelta = %f, Older Sweep Image Received. Skip processing.", isTdelta);
		}
		else {
			if ((sweep_num > processedSweepNum) || /* wrap around */(sweep_num == (processedSweepNum+1))) {	
				IB_LOG_INFO_FMT( __func__, "TDelta = %f, New Sweep Image Received. Processing..", isTdelta);
				processedSweepNum = sweep_num;
			}
			else {
				skipCompounding = TRUE;
				IB_LOG_INFO_FMT( __func__, "Same/older Sweep Image Received. Skip processing..");
			}
		}
	}

	// Reintegrate image into g_pmSweepData as a sweep image
	if (!skipCompounding) {
		ret = PmReintegrate(pm, sth);
		if (ret != FSUCCESS) {
			IB_LOG_ERRORRC("Error reintegrating PM Composite Image rc:", ret);
			goto exit_unlock;
		}
	}

	vs_stdtime_get(&now_time);

	if (!skipCompounding) {
		// Complete sweep image processing
		pm->LastSweepIndex = pm->SweepIndex;	// last valid sweep
		pm->lastHistoryIndex = (pm->lastHistoryIndex + 1) % pm_config.total_images;
		pm->history[pm->lastHistoryIndex] = pm->LastSweepIndex;
		pm->ShortTermHistory.currentInstanceId = tempInstanceId;
		pmimagep->historyIndex = pm->lastHistoryIndex;
		pmimagep->sweepNum = sweep_num;
		pmimagep->state = PM_IMAGE_VALID;

#if CPU_LE
		// Process STH files only on LE CPUs
#ifndef __VXWORKS__
		if (pm_config.shortTermHistory.enable && !isFirstImg) {
			// compound the image that was just created into the current composite image
			IB_LOG_INFO_FMT(__func__, "compoundNewImage LastSweepIndex=%d, lastHistoryIndex=%d",pm->LastSweepIndex,pm->lastHistoryIndex);
			ret = compoundNewImage(pm);
			if (ret != FSUCCESS) {
				IB_LOG_WARNRC("Error while trying to compound new sweep image rc:", ret);
			}
		} else if (isFirstImg) {
			isFirstImg = FALSE;
		}

#endif
#endif	// End of #if CPU_LE

		// find next free Image to use, skip FreezeFrame images
		do {
			pm->SweepIndex = (pm->SweepIndex + 1) % pm_config.total_images;
			if (! pm->Image[pm->SweepIndex].ffRefCount)
				break;
			// check lease
			if (now_time > pm->Image[pm->SweepIndex].lastUsed &&
					now_time - pm->Image[pm->SweepIndex].lastUsed > pm_config.freeze_frame_lease) {
				pm->Image[pm->SweepIndex].ffRefCount = 0;
				// paAccess will clean up FreezeFrame on next access or FF Create
			}
			// skip past images which are frozen
		} while (pm->Image[pm->SweepIndex].ffRefCount);

		// Mark current image valid; mark next image in-progress
		pmimagep->state = PM_IMAGE_VALID;
		pm->Image[pm->SweepIndex].state = PM_IMAGE_INPROGRESS;
		// pm->NumSweeps follows image ID sweepNum; +1 anticipates becoming master before next image
		pm->NumSweeps = sweep_num + 1;

	}	// End of if (!skipCompounding)

	// Log sweep image statistics
	vs_pool_size(&pm_pool, &size);
	IB_LOG_INFO_FMT( __func__, "Image Received From Master: Sweep: %u Memory Used: %"PRIu64" MB (%"PRIu64" KB)",
		sweep_num, size / (1024*1024), size/1024 );
	// Additional debug output can be enabled
//	IB_LOG_VERBOSE_FMT( __func__, "... SweepIndex:%u LastSweepIndex:%u LastHistoryIndex:%u IsCompound:%u ImgSweN:%u swe_n:%u",
//		pm->SweepIndex, pm->LastSweepIndex, pm->lastHistoryIndex, !skipCompounding, pmimagep->sweepNum, sweep_num );
//	IB_LOG_VERBOSE_FMT( __func__, "... H[0]:%u H[1]:%u H[2]:%u H[3]:%u H[4]:%u H[5]:%u H[6]:%u H[7]:%u H[8]:%u H[9]:%u",
//		pm->history[0], pm->history[1], pm->history[2], pm->history[3], pm->history[4],
//		pm->history[5], pm->history[6], pm->history[7], pm->history[8], pm->history[9] );

exit_unlock:
	(void)vs_rwunlock(&pmimagep->imageLock);
	(void)vs_rwunlock(&pm->stateLock);
//    IB_LOG_DEBUG1_FMT(__func__, " EXIT-UNLOCK: ret:0x%X", ret);
// Fall-through to exit_free

exit_free:
	if (bf_decompress)
		vs_pool_free(&pm_pool, bf_decompress);
	if (cimg_out)
		PmFreeComposite(cimg_out);

	return ret;

}	// End of putPMSweepImageDataR()


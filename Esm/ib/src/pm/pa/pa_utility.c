/* BEGIN_ICS_COPYRIGHT3 ****************************************

Copyright (c) 2018, Intel Corporation

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
#include "pa_access.h"

static const char *FSTATUS_Strings[] = {
	"SUCCESS",
	"ERROR",
	"INVALID_STATE",
	"INVALID_OPERATION",
	"INVALID_SETTING",
	"INVALID_PARAMETER",
	"INSUFFICIENT_RESOURCES",
	"INSUFFICIENT_MEMORY",
	"COMPLETED",
	"NOT_DONE",
	"PENDING",
	"TIMEOUT",
	"CANCELED",
	"REJECT",
	"OVERRUN",
	"PROTECTION",
	"NOT_FOUND",
	"UNAVAILABLE",
	"BUSY",
	"DISCONNECT",
	"DUPLICATE",
	"POLL_NEEDED",
};
const char *
FSTATUS_ToString(FSTATUS status)
{
	return ((status & 0xFF) < FSTATUS_COUNT ? FSTATUS_Strings[status] : "");
}
// given an Image Index, build an externally usable FreezeFrame ImageId
// this will allow future reference to this image via freezeFrames[]
uint64
BuildFreezeFrameImageId(Pm_t *pm, uint32 freezeIndex, uint8 clientId, uint32 *imageTime)
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

// locate group by name
FSTATUS
LocateGroup(PmImage_t *pmimagep, const char *groupName, int *groupIndex)
{
	int i;
	*groupIndex = -1;
	if (strncmp(groupName, PA_ALL_GROUP_NAME, STL_PM_GROUPNAMELEN) == 0) {
		return FSUCCESS;
	}

	for (i = 0; i < pmimagep->NumGroups; i++) {
		if (strncmp(groupName, pmimagep->Groups[i].Name, STL_PM_GROUPNAMELEN) == 0) {
			*groupIndex = i;
			return FSUCCESS;
		}
	}

	return FNOT_FOUND;
}

// locate vf by name
FSTATUS
LocateVF(PmImage_t *pmimagep, const char *vfName, int *vfIdx)
{
	int i;

	*vfIdx = -1;
	for (i = 0; i < pmimagep->NumVFs; i++) {
		if (!pmimagep->VFs[i].isActive) continue;

		if (strncmp(vfName, pmimagep->VFs[i].Name, STL_PM_VFNAMELEN) == 0) {
			*vfIdx = i;
			return FSUCCESS;
		}
	}
	return FNOT_FOUND;
}

// given an Image Index, build an externally usable History ImageId
// this will allow future reference to this image via history[]
static uint64
BuildHistoryImageId(Pm_t *pm, uint32 historyIndex)
{
	ImageId_t img_id;

	img_id.AsReg64 = 0;
	img_id.s.type = IMAGEID_TYPE_HISTORY;
	img_id.s.sweepNum = pm->Image[pm->history[historyIndex]].sweepNum;
	img_id.s.index = historyIndex;
	if (pm_config.shortTermHistory.enable)
		img_id.s.instanceId = pm->ShortTermHistory.currentInstanceId;

	return img_id.AsReg64;
}
// given a index into history[], validate index and figure out
// imageIndex into Image[]
// also build retImageId
static FSTATUS
ComputeHistory(Pm_t *pm, uint32 historyIndex, int32 offset, uint32 baseSweepNum,
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
	index = (historyIndex + pm_config.total_images + offset) % pm_config.total_images;
	//printf("offset=%d historyIndex=%u index=%u\n", offset, historyIndex, index);
	*imageIndex = pm->history[index];
	if (*imageIndex == PM_IMAGE_INDEX_INVALID) {
		*msg = "Invalid Image Id";
		return FINVALID_PARAMETER;
	}
	ASSERT(*imageIndex < pm_config.total_images);

	// validate it's still there
	if (pm->Image[*imageIndex].sweepNum != (baseSweepNum + offset)
		|| pm->Image[*imageIndex].state != PM_IMAGE_VALID)
	{
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
static FSTATUS
GetIndexFromTime(Pm_t *pm, uint8 type, time_t requestTime, uint32 *imageIndex,
	STL_PA_IMAGE_ID_DATA *returnImageId, const char **msg, uint8 *clientId)
{
	uint32 lastImageIndex = pm->history[pm->lastHistoryIndex];
	int i;

	if (pm->LastSweepIndex == PM_IMAGE_INDEX_INVALID
		|| lastImageIndex == PM_IMAGE_INDEX_INVALID
		|| pm->Image[lastImageIndex].state != PM_IMAGE_VALID)
	{
		*msg = "Sweep Engine not ready";
		return FUNAVAILABLE;
	}

	uint32 historyIndex = pm->lastHistoryIndex;

	for (i = 0; i < pm_config.total_images; ++i) {
		FSTATUS status = ComputeHistory(pm, historyIndex, 0 - i, pm->Image[lastImageIndex].sweepNum,
			imageIndex, &returnImageId->imageNumber, msg);

		if (FSUCCESS == status) {
			//we have a valid imageIndex, get RAM image and look at
			//sweepStart time
			PmImage_t *pmimagep = &pm->Image[*imageIndex];

			(void)vs_rdlock(&pmimagep->imageLock);
			if (pmimagep->state == PM_IMAGE_VALID
				// Check request Time is between imageStart and the imageInterval
				&& requestTime >= pmimagep->sweepStart
				&& requestTime < (pmimagep->sweepStart + pmimagep->imageInterval)) {

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

#ifndef __VXWORKS__
/* Comparison function when requesting history records by time */
static int
sweepTimeCompare(const uint64 map_key, const uint64 requestTime)
{
	if (requestTime >= ((map_key >> 32) + (map_key & 0xFFFFFFFF))) return -1;
	else if (requestTime < (map_key >> 32)) return 1;
	else return 0;
}
#endif
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
static FSTATUS
FindImageByTime(Pm_t *pm, uint8 type, time_t requestTime, uint32 *imageIndex,
	STL_PA_IMAGE_ID_DATA *retImageId, PmHistoryRecord_t **record, const char **msg,
	uint8 *clientId, PmCompositeImage_t **cimg)
{
	FSTATUS status;

	//validate requestTime
	if (!requestTime) {
		*msg = "Invalid request time";
		return FINVALID_PARAMETER;
	}

	// CASE 1: Image with requested time in RAM
	status = GetIndexFromTime(pm, type, requestTime, imageIndex, retImageId, msg, clientId);
#ifndef __VXWORKS__
	if (FSUCCESS == status || FUNAVAILABLE == status) return status;
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

#ifndef __VXWORKS__
static FSTATUS
CheckComposite(Pm_t *pm, uint64 imageId, PmCompositeImage_t *cimg)
{
	ImageId_t temp;

	temp.AsReg64 = imageId;
	temp.s.clientId = 0;
	temp.s.index = 0;

	int i;

	temp.s.type = ((ImageId_t *)(&cimg->header.common.imageIDs[0]))->s.type;

	for (i = 0; i < PM_HISTORY_MAX_IMAGES_PER_COMPOSITE; i++) {
		if (cimg->header.common.imageIDs[i] == temp.AsReg64)
			return FSUCCESS;
	}
	return FNOT_FOUND;
}

#endif
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
static FSTATUS
GetIndexFromImageId(Pm_t *pm, uint8 type, uint64 imageId, int32 offset, uint32 *imageIndex,
	uint64 *retImageId, const char **msg, uint8 *clientId)
{
	ImageId_t id;
	uint32 lastImageIndex = pm->history[pm->lastHistoryIndex];

	id.AsReg64 = imageId;

	if (pm->LastSweepIndex == PM_IMAGE_INDEX_INVALID
		|| lastImageIndex == PM_IMAGE_INDEX_INVALID
		|| pm->Image[lastImageIndex].state != PM_IMAGE_VALID)
	{
		*msg = "Sweep Engine not ready";
		return FUNAVAILABLE;
	}
	DEBUG_ASSERT(lastImageIndex == pm->LastSweepIndex);

	if ((type != IMAGEID_TYPE_ANY)
		&& ((id.s.type == IMAGEID_TYPE_FREEZE_FRAME)
			!= (type == IMAGEID_TYPE_FREEZE_FRAME)))
	{
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
				pm->Image[lastImageIndex].sweepNum, imageIndex, retImageId, msg);
		}
	} else if (id.s.type == IMAGEID_TYPE_HISTORY) {
		return ComputeHistory(pm, id.s.index, offset, id.s.sweepNum,
			imageIndex, retImageId, msg);
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
			|| pmimagep->state != PM_IMAGE_VALID)
		{
			*msg = "Freeze Frame expired or invalid";
			return FNOT_FOUND;
		}
		if (0 == pmimagep->ffRefCount) {
			// cleanup, must have expired
			pm->freezeFrames[id.s.index] = PM_IMAGE_INDEX_INVALID;
			*msg = "Freeze Frame expired or invalid";
			return FNOT_FOUND;
		}
		if (0 == (pmimagep->ffRefCount & (1 << id.s.clientId))) {
			*msg = "Freeze Frame expired or invalid";
			return FNOT_FOUND;
		}
		if (type == IMAGEID_TYPE_FREEZE_FRAME && offset != 0) {
			*msg = "Freeze Frame offset not allowed in this query";
			return FINVALID_PARAMETER;
		}
		// stdtime is 32 bits, so should be atomic write and avoid race
		vs_stdtime_get(&now_time);
		pmimagep->lastUsed = now_time;   // update age due to access
		if (clientId)
			*clientId = id.s.clientId;
		if (offset == 0) {
			*retImageId = imageId;  // no translation, return what given
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
FSTATUS
FindImage(Pm_t *pm, uint8 type, STL_PA_IMAGE_ID_DATA imageId, uint32 *imageIndex,
	uint64 *retImageId, PmHistoryRecord_t **record, const char **msg, uint8 *clientId,
	PmCompositeImage_t **cimg)
{
	FSTATUS status = FNOT_FOUND;
	STL_PA_IMAGE_ID_DATA id_data = {0};

	if (IMAGEID_ABSOLUTE_TIME == imageId.imageNumber) {
		status = FindImageByTime(pm, type, (time_t)imageId.imageTime.absoluteTime,
			imageIndex, &id_data, record, msg, clientId, cimg);
		if (FSUCCESS == status)
			*retImageId = id_data.imageNumber;
		return status;
	}

	// only check the RAM Images if the requested instance ID matches the current instance
	if (imageId.imageNumber == IMAGEID_LIVE_DATA || !pm_config.shortTermHistory.enable
		|| (pm_config.shortTermHistory.enable
			&& ((ImageId_t)imageId.imageNumber).s.instanceId == pm->ShortTermHistory.currentInstanceId))
	{
		// image ID in RAM, offset in RAM
		status = GetIndexFromImageId(pm, type, imageId.imageNumber, imageId.imageOffset,
			imageIndex, retImageId, msg, clientId);
		if (status == FSUCCESS)	return status;
	}

#ifndef __VXWORKS__
	if (!pm_config.shortTermHistory.enable || status == FUNAVAILABLE)
		return status;

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

	int ireq = -1;  // index of the requested imageID in history
	int icurr = -1; // index of the record currently be built in history
	int oneg = -1;  // maximum negative offset allowed
	int opos = -1;  // maximum positive offset allowed
	int tot = -1;   // total number of records in history
	int32 offset = imageId.imageOffset;

	// only check the RAM images if the requested instance ID matches the current instance
	if (imageId.imageNumber == IMAGEID_LIVE_DATA
		|| ((ImageId_t)imageId.imageNumber).s.instanceId == pm->ShortTermHistory.currentInstanceId)
	{
		// image ID in RAM, offset negative into history
		status = GetIndexFromImageId(pm, type, imageId.imageNumber, 0, imageIndex, retImageId, msg, clientId);
	}
	if (status == FSUCCESS && offset < 0) {
		// offset needs to be adjusted to account for the overlap between history images and RAM images
		// new offset should be: old offset - distance required to cover the rest of the
		// RAM images + offset from 1st record to 1st 'uncovered' record
		int32 r, t, i, c, o;

		// r: offset from live to requested image ID
		r = imageId.imageNumber != IMAGEID_LIVE_DATA ? histId.s.sweepNum - pm->NumSweeps : 0;
		// t: current total number of RAM images
		t = (int32)MIN(pm_config.total_images, pm->NumSweeps);
		// i: images per composite
		i = (int32)pm_config.shortTermHistory.imagesPerComposite;
		// c: number of images in the composite currently being built
		c = (int32)((pm->ShortTermHistory.currentComposite->header.common.imagesPerComposite
				== pm_config.shortTermHistory.imagesPerComposite)
			? 0 : pm->ShortTermHistory.currentComposite->header.common.imagesPerComposite);
		// o: offset from 1st history record to 1st history record not covered by RAM images
		o = c > t ? 1 : (c - t + 1) / i;
		offset = offset + r + t + o; // total new offset
		if (offset > 0) { // this means we went back from RAM images to the current composite
			*cimg = pm->ShortTermHistory.currentComposite;
			return status;
		}
		// now we will be working from the first history record
		ireq = pm->ShortTermHistory.currentRecordIndex == 0
			? pm->ShortTermHistory.totalHistoryRecords - 1
			: pm->ShortTermHistory.currentRecordIndex - 1;
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
		if (pm->ShortTermHistory.currentComposite
			&& (pm->ShortTermHistory.currentComposite->header.common.imagesPerComposite >= pm_config.total_images))
		{
			status = CheckComposite(pm, histId.AsReg64, pm->ShortTermHistory.currentComposite);
			if (status == FSUCCESS) {
				if (!offset) {
					// current composite was requested directly by image ID
					*cimg = pm->ShortTermHistory.currentComposite;
					return status;
				} else {
					// image ID was for current composite, so use 1st history record as starting place, adjust offset
					ireq = pm->ShortTermHistory.currentRecordIndex == 0
						? pm->ShortTermHistory.totalHistoryRecords - 1
						: pm->ShortTermHistory.currentRecordIndex - 1;
					offset++;
				}
			}
		}
		// ireq has not already been set by finding image ID in RAM or current composite, check history records
		if (ireq < 0) {
			// find this image
			cl_map_item_t *mi;

			// imageID will be of type HISTORY_DISK if not in RAM or current composite.
			histId.s.type = IMAGEID_TYPE_HISTORY_DISK;
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
		c = (int32)(
			(!pm->ShortTermHistory.currentComposite
				|| (pm->ShortTermHistory.currentComposite->header.common.imagesPerComposite
					== pm_config.shortTermHistory.imagesPerComposite))
			? 0 : pm->ShortTermHistory.currentComposite->header.common.imagesPerComposite);
		// o: offset of icurr from uncovered records
		o = c > t ? -1 : (t - c - 1) / i;
		minRamOff = ((ireq < icurr) ? (icurr - ireq) : (tot - (ireq - icurr))) - o;
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
		status = FNOT_FOUND;
	} else if (offset > opos) {
		if ((offset - opos) == 1 && pm->ShortTermHistory.currentComposite
			&& pm->ShortTermHistory.currentComposite->header.common.imagesPerComposite > pm_config.total_images)
		{
			// if offset is opos+1, and the current composite spans outside of the range of RAM images, return the current composite
			*cimg = pm->ShortTermHistory.currentComposite;
			status = FSUCCESS;
		} else {
			*msg = "Positive offset exceeds duration of Short-Term History";
			status = FNOT_FOUND;
		}
	} else {
		// find the record
		int r = (offset < 0) ? (ireq + offset) : ((ireq + offset) % tot);

		if (r < 0) {
			// wrap
			r = tot + r;
		}
		*record = pm->ShortTermHistory.historyRecords[r];
		if (!*record || !(*record)->header.timestamp || (*record)->index == INDEX_NOT_IN_USE) {
			*msg = "Image request found empty history record";
			status = FNOT_FOUND;
		} else {
			*imageIndex = 0;
			status = FSUCCESS;
		}
	}
#endif /* NOT __VXWORKS__ */
	return status;
}
/**
 * Function to return pointer to PmImage at Requested Image
 *
 * @param func         __func__
 * @param pm           Pointer to Pm_t.
 * @param req_img      Requested Image ID.
 * @param rsp_img      Optional: Resolved Response Image ID.
 * @param pm_image     Pointer to Pointer of the PmImage at Request image.
 * @param imageIndex   Optional: Index into PmPort and PmNode Image arrays
 *  				   pointed at by PmImage.
 * @param requiresLock Pointer to return whether this PmImage requires a lock
 *
 * @return FSTATUS
 */
FSTATUS
FindPmImage(const char *func, Pm_t *pm, STL_PA_IMAGE_ID_DATA req_img, STL_PA_IMAGE_ID_DATA *rsp_img,
	PmImage_t **pm_image, uint32 *imageIndex, boolean *requiresLock)
{
	FSTATUS status;
	PmImage_t *pmimagep;
	uint32 imgIndex;
	STL_PA_IMAGE_ID_DATA ret_img = {0};
	boolean isSthImg = FALSE;
	PmHistoryRecord_t *record = NULL;
	PmCompositeImage_t *cimg = NULL;
	const char *msg;

	status = FindImage(pm, IMAGEID_TYPE_ANY, req_img, &imgIndex, &ret_img.imageNumber, &record, &msg, NULL, &cimg);
	if (FSUCCESS != status) {
		IB_LOG_WARN_FMT(func, "Unable to get index from ImageId: %s: %s", FSTATUS_ToString(status), msg);
		goto error;
	}
	if (record || cimg) {
		isSthImg = TRUE;
		if (record) {
			// try to load
			status = PmLoadComposite(pm, record, &cimg);
			if (status != FSUCCESS || !cimg) {
				IB_LOG_WARN_FMT(func, "Unable to load composite image: %s", FSTATUS_ToString(status));
				goto error;
			}
		}
		// set the return ID
		ret_img.imageNumber = cimg->header.common.imageIDs[0];
		// composite is loaded, reconstitute so we can use it
		status = PmReconstitute(&pm->ShortTermHistory, cimg);
		if (record) PmFreeComposite(cimg);
		if (status != FSUCCESS) {
			IB_LOG_WARN_FMT(func, "Unable to reconstitute composite image: %s", FSTATUS_ToString(status));
			goto error;
		}
		pmimagep = pm->ShortTermHistory.LoadedImage.img;
		imgIndex = 0; // STH always uses imageIndex 0
	} else {
		pmimagep = &pm->Image[imgIndex];
	}
	// Grab ImageTime from Pm Image (should be under lock, but should be fine).
	ret_img.imageTime.absoluteTime = (uint32)pmimagep->sweepStart;

	*pm_image = pmimagep;
	 // If not STH, then it requires a lock
	*requiresLock = !isSthImg; // TBD - Multi-user would require STH to lock images as well.
	if (imageIndex) *imageIndex = imgIndex;
	if (rsp_img) *rsp_img = ret_img;
	return FSUCCESS;
error:
	*pm_image = NULL;
	if (rsp_img) rsp_img->imageNumber = BAD_IMAGE_ID;
	if (status == FSUCCESS) status = FERROR;
	return status;
}

// this function is used by ESM CLI
static void pm_print_port_running_totals(FILE *out, Pm_t *pm, PmPort_t *pmportp,
	uint32 imageIndex)
{
	PmCompositePortCounters_t *pPortCounters;
	PmPortImage_t *portImage = &pmportp->Image[imageIndex];

	if (!portImage->u.s.active)
		return;
	fprintf(out, "%.*s Guid "FMT_U64" LID 0x%x Port %u\n",
		(int)sizeof(pmportp->pmnodep->nodeDesc.NodeString),
		pmportp->pmnodep->nodeDesc.NodeString, pmportp->pmnodep->NodeGUID,
		pmportp->pmnodep->Image[imageIndex].lid, pmportp->portNum);
	if (portImage->neighbor) {
		PmPort_t *neighbor = portImage->neighbor;

		fprintf(out, "    Neighbor: %.*s Guid "FMT_U64" LID 0x%x Port %u\n",
			(int)sizeof(neighbor->pmnodep->nodeDesc.NodeString),
			neighbor->pmnodep->nodeDesc.NodeString, neighbor->pmnodep->NodeGUID,
			neighbor->pmnodep->Image[imageIndex].lid,
			neighbor->portNum);
	}
	fprintf(out, "    txRate: %4s rxRate: %4s  MTU: %5s%s\n",
		StlStaticRateToText(PmCalculateRate(portImage->u.s.activeSpeed, portImage->u.s.txActiveWidth)),
		StlStaticRateToText(PmCalculateRate(portImage->u.s.activeSpeed, portImage->u.s.rxActiveWidth)),
		IbMTUToText(portImage->u.s.mtu),
		portImage->u.s.UnexpectedClear ? "  Unexpected Clear" : "");
	if (pmportp->u.s.PmaAvoid) {
		fprintf(out, "    PMA Counters Not Available\n");
		return;
	}

	pPortCounters = &pmportp->StlPortCountersTotal;
	fprintf(out, "    Xmit: Data:%-10" PRIu64 " MB (%-10" PRIu64
		" Flits) Pkts:%-10" PRIu64 "\n",
		pPortCounters->PortXmitData / FLITS_PER_MB,
		pPortCounters->PortXmitData, pPortCounters->PortXmitPkts);
	fprintf(out, "    Recv: Data:%-10" PRIu64 " MB (%-10" PRIu64
		" Flits) Pkts:%-10" PRIu64 "\n",
		pPortCounters->PortRcvData / FLITS_PER_MB,
		pPortCounters->PortRcvData, pPortCounters->PortRcvPkts);
	fprintf(out, "    Integrity:                        SmaCongest:\n");
	fprintf(out, "      Link Recovery:%-10u\n",
		pPortCounters->LinkErrorRecovery);
	fprintf(out, "      Link Downed:%-10u\n", pPortCounters->LinkDowned);
	fprintf(out, "      Rcv Errors:%-10" PRIu64 "           Security:\n",
		pPortCounters->PortRcvErrors);
	fprintf(out, "      Loc Lnk Integrity:%-10" PRIu64 "      Rcv Constrain:%-10" PRIu64 "\n",
		pPortCounters->LocalLinkIntegrityErrors,
		pPortCounters->PortRcvConstraintErrors);
	fprintf(out, "      Excess Bfr Overrun*:%-10" PRIu64 "    Xmt Constrain*:%-10" PRIu64 "\n",
		pPortCounters->ExcessiveBufferOverruns,
		pPortCounters->PortXmitConstraintErrors);
	fprintf(out, "    Congestion:                       Routing:\n");
	fprintf(out, "      Xmt Discards*:%-10" PRIu64 "     Rcv Sw Relay:%-10" PRIu64 "\n",
		pPortCounters->PortXmitDiscards,
		pPortCounters->PortRcvSwitchRelayErrors);
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
void pm_print_running_totals_to_stream(FILE *out)
{
	FSTATUS				status;
	const char 			*msg;
	PmImage_t			*pmimagep;
	uint32				imageIndex;
	uint64				retImageId;
	STL_LID 			lid;
	extern Pm_t g_pmSweepData;
	Pm_t *pm = &g_pmSweepData;

	if (topology_passcount < 1) {
		fprintf(out, "\nSM is currently in the %s state, count = %d\n\n", IbSMStateToText(sm_state), (int)sm_smInfo.ActCount);
		return;
	}

	AtomicIncrementVoid(&pm->refCount); // prevent engine from stopping
	if (!PmEngineRunning()) {
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

	for (lid = 1; lid <= pmimagep->maxLid; ++lid) {
		uint8 portnum;
		PmNode_t *pmnodep = pmimagep->LidMap[lid];

		if (!pmnodep)
			continue;
		if (pmnodep->nodeType == STL_NODE_SW) {
			for (portnum = 0; portnum <= pmnodep->numPorts; ++portnum) {
				PmPort_t *pmportp = pmnodep->up.swPorts[portnum];

				if (!pmportp)
					continue;
				(void)vs_rdlock(&pm->totalsLock);   // limit in case print slow
				pm_print_port_running_totals(out, pm, pmportp, imageIndex);
				(void)vs_rwunlock(&pm->totalsLock);
			}
		} else {
			PmPort_t *pmportp = pmnodep->up.caPortp;

			(void)vs_rdlock(&pm->totalsLock);   // limit in case print slow
			pm_print_port_running_totals(out, pm, pmportp, imageIndex);
			(void)vs_rwunlock(&pm->totalsLock);
		}
	}
	(void)vs_rwunlock(&pmimagep->imageLock);
done:
	AtomicDecrementVoid(&pm->refCount);
}


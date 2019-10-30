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

/*
 * Abstract:
 *	Declaration of quick map, a binary tree where the caller always provides
 *	all necessary storage.
 *
 * Environment:
 *	All
 *
 * $Revision$
 */


#ifndef _IBA_PUBLIC_IQUICKMAP_H_
#define _IBA_PUBLIC_IQUICKMAP_H_


#include "iba/public/datatypes.h"

#define QMAP_DEBUG 0 /* beware setting this changes ABI so must rebuild all */

typedef struct _cl_list_item
{
	struct _cl_list_item	*p_next;
	struct _cl_list_item	*p_prev;
#if QMAP_DEBUG
	struct _cl_qlist		*p_list;
#endif

} cl_list_item_t;


typedef struct _cl_pool_item
{
	cl_list_item_t		list_item;
#if QMAP_DEBUG
	/* Pointer to the owner pool used for sanity checks. */
	struct _cl_qcpool	*p_pool;
#endif

} cl_pool_item_t;


typedef enum _cl_state
{
	CL_UNINITIALIZED = 1,
	CL_INITIALIZED,
	CL_DESTROYING,
	CL_DESTROYED

} cl_state_t;

#define CL_INLINE	static __inline
#define CL_EXPORT	

/****h* Component Library/Quick Map
* NAME
*	Quick Map
*
* DESCRIPTION
*	Quick map implements a binary tree that stores user provided cl_map_item_t
*	structures.  Each item stored in a quick map has a unique 64-bit key
*	(duplicates are not allowed).  Quick map provides the ability to
*	efficiently search for an item given a key.
*
*	Quick map does not allocate any memory, and can therefore not fail
*	any operations due to insufficient memory.  Quick map can thus be useful
*	in minimizing the error paths in code.
*
*	Quick map is not thread safe, and users must provide serialization when
*	adding and removing items from the map.
*
*	The quick map functions operate on a cl_qmap_t structure which should be
*	treated as opaque and should be manipulated only through the provided
*	functions.
*
* SEE ALSO
*	Structures:
*		cl_qmap_t, cl_map_item_t, cl_map_obj_t
*
*	Callbacks:
*		cl_pfn_qmap_apply_t
*
*	Item Manipulation:
*		cl_qmap_set_obj, cl_qmap_obj, cl_qmap_key
*
*	Initialization:
*		cl_qmap_init
*
*	Iteration:
*		cl_qmap_end, cl_qmap_head, cl_qmap_tail, cl_qmap_next, cl_qmap_prev
*
*	Manipulation:
*		cl_qmap_insert, cl_qmap_get, cl_qmap_remove_item, cl_qmap_remove,
*		cl_qmap_remove_all, cl_qmap_merge, cl_qmap_delta
*
*	Search:
*		cl_qmap_apply_func
*
*	Attributes:
*		cl_qmap_count, cl_is_qmap_empty,
*********/


/****i* Component Library: Quick Map/cl_map_color_t
* NAME
*	cl_map_color_t
*
* DESCRIPTION
*	The cl_map_color_t enumerated type is used to note the color of
*	nodes in a map.
*
* SYNOPSIS
*/
typedef enum _cl_map_color
{
	CL_MAP_RED,
	CL_MAP_BLACK

} cl_map_color_t;
/*
* VALUES
*	CL_MAP_RED
*		The node in the map is red.
*
*	CL_MAP_BLACK
*		The node in the map is black.
*
* SEE ALSO
*	Quick Map, cl_map_item_t
*********/


/****s* Component Library: Quick Map/cl_map_item_t
* NAME
*	cl_map_item_t
*
* DESCRIPTION
*	The cl_map_item_t structure is used by maps to store objects.
*
*	The cl_map_item_t structure should be treated as opaque and should
*	be manipulated only through the provided functions.
*
* SYNOPSIS
*/
typedef struct _cl_map_item
{
	/* Must be first to allow casting. */
	cl_pool_item_t			pool_item;
	struct _cl_map_item		*p_left;
	struct _cl_map_item		*p_right;
	struct _cl_map_item		*p_up;
	cl_map_color_t			color;
	uint64					key;
#if QMAP_DEBUG
	struct _cl_qmap			*p_map;
#endif

} cl_map_item_t;
/*
* FIELDS
*	pool_item
*		Used to store the item in a doubly linked list, allowing more
*		efficient map traversal.
*
*	p_left
*		Pointer to the map item that is a child to the left of the node.
*
*	p_right
*		Pointer to the map item that is a child to the right of the node.
*
*	p_up
*		Pointer to the map item that is the parent of the node.
*
*	p_nil
*		Pointer to the map's NIL item, used as a terminator for leaves.
*		The NIL sentinel is in the cl_qmap_t structure.
*
*	color
*		Indicates whether a node is red or black in the map.
*
*	key
*		Value that uniquely represents a node in a map.  This value is set by
*		calling cl_qmap_insert and can be retrieved by calling cl_qmap_key.
*
* NOTES
*	None of the fields of this structure should be manipulated by users, as
*	they are crititcal to the proper operation of the map in which they
*	are stored.
*
*	To allow storing items in either a quick list, a quick pool, or a quick
*	map, the map implementation guarantees that the map item can be safely
*	cast to a pool item used for storing an object in a quick pool, or cast to
*	a list item used for storing an object in a quick list.  This removes the
*	need to embed a map item, a list item, and a pool item in objects that need
*	to be stored in a quick list, a quick pool, and a quick map.
*
* SEE ALSO
*	Quick Map, cl_qmap_insert, cl_qmap_key, cl_pool_item_t, cl_list_item_t
*********/


/****s* Component Library: Quick Map/cl_map_obj_t
* NAME
*	cl_map_obj_t
*
* DESCRIPTION
*	The cl_map_obj_t structure is used to store objects in maps.
*
*	The cl_map_obj_t structure should be treated as opaque and should
*	be manipulated only through the provided functions.
*
* SYNOPSIS
*/
typedef struct _cl_map_obj
{
	cl_map_item_t			item;
	const void				*p_object;

} cl_map_obj_t;
/*
* FIELDS
*	item
*		Map item used by internally by the map to store an object.
*
*	p_object
*		User defined context. Users should not access this field directly.
*		Use cl_qmap_set_obj and cl_qmap_obj to set and retrieve the value
*		of this field.
*
* NOTES
*	None of the fields of this structure should be manipulated by users, as
*	they are crititcal to the proper operation of the map in which they
*	are stored.
*
*	Use cl_qmap_set_obj and cl_qmap_obj to set and retrieve the object
*	stored in a map item, respectively.
*
* SEE ALSO
*	Quick Map, cl_qmap_set_obj, cl_qmap_obj, cl_map_item_t
*********/

/****d* Component Library: Quick Map/cl_pfn_qmap_key_compare_t
* NAME
*	cl_pfn_qmap_key_compare_t
*
* DESCRIPTION
*	The cl_pfn_qmap_akey_comparet function type defines the prototype for functions
*	used to compare keys for items in a quick map.
*
* SYNOPSIS
*/
typedef int
(*cl_pfn_qmap_compare_key_t)(
	IN	const uint64			key1,
	IN	const uint64			key2 );
/*
* PARAMETERS
*	key1
*		[in] first key to compare
*
*	key2
*		[in] second key to compare
*
* RETURN VALUE
*	-1 key1 < key2
*	0 key1 == key2
*	1 key1 > key2
*
* NOTES
*	This function type is provided as function prototype reference for the
*	function provided by users as a parameter to the cl_qmap_init and
*	some other functions.
*
*	Comparision must be consistent.  Eg.
*		if key1 < key2
*		then key2 > key1
*	
*	often key1 and key2 are cast by the function to a pointer to a key object or
*		subfield within the map object.  However they could also do
*		odd-collating sequences (or ignore bits) in uint64 keys
*
* SEE ALSO
*	Quick Map, cl_qmap_init
*********/

/****d* Component Library: Quick Map/cl_pfn_qmap_item_compare_t
* NAME
*	cl_pfn_qmap_item_compare_t
*
* DESCRIPTION
*	The cl_pfn_qmap_item_compare_t function type defines the prototype for functions
*	used to compare an item to a key for items in a quick map.
*
* SYNOPSIS
*/
typedef int
(*cl_pfn_qmap_item_compare_t)(
	IN	const cl_map_item_t*	item,
	IN	const uint64			key );
/*
* PARAMETERS
*	item
*		[in] item to compare
*
*	key
*		[in] key to compare
*
* RETURN VALUE
*	-1 item < key
*	0 item == key
*	1 item > key
*
* NOTES
*	This function type is provided as function prototype reference for the
*	function provided by users as a parameter to cl_qmap_get_item_compare
*	and some other functions.
*
*	Comparision must be consistent.  Eg.
*		if item < key
*		then key > item
*	
* SEE ALSO
*	Quick Map, cl_qmap_get_item_compare
*********/

/****s* Component Library: Quick Map/cl_qmap_t
* NAME
*	cl_qmap_t
*
* DESCRIPTION
*	Quick map structure.
*
*	The cl_qmap_t structure should be treated as opaque and should
*	be manipulated only through the provided functions.
*
* SYNOPSIS
*/
typedef struct _cl_qmap
{
	cl_map_item_t	root;
	cl_map_item_t	nil_item;
	cl_state_t		state;
	size_t			count;
	cl_pfn_qmap_compare_key_t key_compare; /* optional compare function */

} cl_qmap_t;
/*
* PARAMETERS
*	root
*		Map item that serves as root of the map.  The root is set up to
*		always have itself as parent.  The left pointer is set to point to
*		the item at the root.
*
*	nil_item
*		Map item that serves as terminator for all leaves, as well as providing
*		the list item used as quick list for storing map items in a list for
*		faster traversal.
*
*	state
*		State of the map, used to verify that operations are permitted.
*
*	count
*		Number of items in the map.
*
* SEE ALSO
*	Quick Map
*********/


/****d* Component Library: Quick Map/cl_pfn_qmap_apply_t
* NAME
*	cl_pfn_qmap_apply_t
*
* DESCRIPTION
*	The cl_pfn_qmap_apply_t function type defines the prototype for functions
*	used to iterate items in a quick map.
*
* SYNOPSIS
*/
typedef void
(*cl_pfn_qmap_apply_t)(
	IN	cl_map_item_t* const	p_map_item,
	IN	void*					context );
/*
* PARAMETERS
*	p_map_item
*		[in] Pointer to a cl_map_item_t structure.
*
*	context
*		[in] Value passed to the callback function.
*
* RETURN VALUE
*	This function does not return a value.
*
* NOTES
*	This function type is provided as function prototype reference for the
*	function provided by users as a parameter to the cl_qmap_apply_func
*	function.
*
* SEE ALSO
*	Quick Map, cl_qmap_apply_func
*********/


#ifdef __cplusplus
extern "C" {
#endif


/****f* Component Library: Quick Map/cl_qmap_count
* NAME
*	cl_qmap_count
*
* DESCRIPTION
*	The cl_qmap_count function returns the number of items stored
*	in a quick map.
*
* SYNOPSIS
*/
CL_INLINE size_t
cl_qmap_count(
	IN	const cl_qmap_t* const	p_map )
{
	ASSERT( p_map );
	ASSERT( p_map->state == CL_INITIALIZED );
	return( p_map->count );
}
/*
* PARAMETERS
*	p_map
*		[in] Pointer to a cl_qmap_t structure whose item count to return.
*
* RETURN VALUE
*	Returns the number of items stored in the map.
*
* SEE ALSO
*	Quick Map, cl_is_qmap_empty
*********/


/****f* Component Library: Quick Map/cl_is_qmap_empty
* NAME
*	cl_is_qmap_empty
*
* DESCRIPTION
*	The cl_is_qmap_empty function returns whether a quick map is empty.
*
* SYNOPSIS
*/
CL_INLINE boolean
cl_is_qmap_empty(
	IN	const cl_qmap_t* const	p_map )
{
	ASSERT( p_map );
	ASSERT( p_map->state == CL_INITIALIZED );

	return( p_map->count == 0 );
}
/*
* PARAMETERS
*	p_map
*		[in] Pointer to a cl_qmap_t structure to test for emptiness.
*
* RETURN VALUES
*	TRUE if the quick map is empty.
*
*	FALSE otherwise.
*
* SEE ALSO
*	Quick Map, cl_qmap_count, cl_qmap_remove_all
*********/


/****f* Component Library: Quick Map/cl_qmap_set_obj
* NAME
*	cl_qmap_set_obj
*
* DESCRIPTION
*	The cl_qmap_set_obj function sets the object stored in a map object.
*
* SYNOPSIS
*/
CL_INLINE void
cl_qmap_set_obj(
	IN	cl_map_obj_t* const	p_map_obj,
	IN	const void* const	p_object )
{
	ASSERT( p_map_obj );
	p_map_obj->p_object = p_object;
}
/*
* PARAMETERS
*	p_map_obj
*		[in] Pointer to a map object stucture whose object pointer
*		is to be set.
*
*	p_object
*		[in] User defined context.
*
* RETURN VALUE
*	This function does not return a value.
*
* SEE ALSO
*	Quick Map, cl_qmap_obj
*********/


/****f* Component Library: Quick Map/cl_qmap_obj
* NAME
*	cl_qmap_obj
*
* DESCRIPTION
*	The cl_qmap_obj function returns the object stored in a map object.
*
* SYNOPSIS
*/
CL_INLINE void*
cl_qmap_obj(
	IN	const cl_map_obj_t* const	p_map_obj )
{
	ASSERT( p_map_obj );
	return( (void*)p_map_obj->p_object );
}
/*
* PARAMETERS
*	p_map_obj
*		[in] Pointer to a map object stucture whose object pointer to return.
*
* RETURN VALUE
*	Returns the value of the object pointer stored in the map object.
*
* SEE ALSO
*	Quick Map, cl_qmap_set_obj
*********/


/****f* Component Library: Quick Map/cl_qmap_key
* NAME
*	cl_qmap_key
*
* DESCRIPTION
*	The cl_qmap_key function retrieves the key value of a map item.
*
* SYNOPSIS
*/
CL_INLINE uint64
cl_qmap_key(
	IN	const cl_map_item_t* const	p_item )
{
	ASSERT( p_item );
	return( p_item->key );
}
/*
* PARAMETERS
*	p_item
*		[in] Pointer to a map item whose key value to return.
*
* RETURN VALUE
*	Returns the 64-bit key value for the specified map item.
*
* NOTES
*	The key value is set in a call to cl_qmap_insert.
*
* SEE ALSO
*	Quick Map, cl_qmap_insert
*********/


/****f* Component Library: Quick Map/cl_qmap_init
* NAME
*	cl_qmap_init
*
* DESCRIPTION
*	The cl_qmap_init function initialized a quick map for use.
*
* SYNOPSIS
*/
CL_EXPORT void
cl_qmap_init(
	IN	cl_qmap_t* const	p_map,
	IN  cl_pfn_qmap_compare_key_t key_compare );
/*
* PARAMETERS
*	p_map
*		[in] Pointer to a cl_qmap_t structure to initialize.
*
*	key_compare
*		[in] Optional key_compare function (NULL if uint64 keys used)
*
* RETURN VALUES
*	This function does not return a value.
*
* NOTES
*	Allows calling quick map manipulation functions.
*
* SEE ALSO
*	Quick Map, cl_qmap_insert, cl_qmap_remove
*********/


/****f* Component Library: Quick Map/cl_qmap_end
* NAME
*	cl_qmap_end
*
* DESCRIPTION
*	The cl_qmap_end function returns the end of a quick map.
*
* SYNOPSIS
*/
CL_INLINE const cl_map_item_t *
cl_qmap_end(
	IN	const cl_qmap_t* const	p_map )
{
	ASSERT( p_map );
	ASSERT( p_map->state == CL_INITIALIZED );
	/* Nil is the end of the map. */
	return( &p_map->nil_item );
}
/*
* PARAMETERS
*	p_map
*		[in] Pointer to a cl_qmap_t structure whose end to return.
*
* RETURN VALUE
*	Pointer to the end of the map.
*
* NOTES
*	cl_qmap_end is useful for determining the validity of map items returned
*	by cl_qmap_head, cl_qmap_tail, cl_qmap_next, or cl_qmap_prev.  If the map
*	item pointer returned by any of these functions compares to the end, the
*	end of the map was encoutered.
*	When using cl_qmap_head or cl_qmap_tail, this condition indicates that
*	the map is empty.
*
* SEE ALSO
*	Quick Map, cl_qmap_head, cl_qmap_tail, cl_qmap_next, cl_qmap_prev
*********/


/****f* Component Library: Quick Map/cl_qmap_head
* NAME
*	cl_qmap_head
*
* DESCRIPTION
*	The cl_qmap_head function returns the map item with the lowest key
*	value stored in a quick map.
*
* SYNOPSIS
*/
CL_INLINE cl_map_item_t*
cl_qmap_head(
	IN	const cl_qmap_t* const	p_map )
{
	ASSERT( p_map );
	ASSERT( p_map->state == CL_INITIALIZED );
	return( (cl_map_item_t*)p_map->nil_item.pool_item.list_item.p_next );
}
/*
* PARAMETERS
*	p_map
*		[in] Pointer to a cl_qmap_t structure whose item with the lowest key
*		is returned.
*
* RETURN VALUES
*	Pointer to the map item with the lowest key in the quick map.
*
*	Pointer to the map end if the quick map was empty.
*
* NOTES
*	cl_qmap_head does not remove the item from the map.
*
* SEE ALSO
*	Quick Map, cl_qmap_tail, cl_qmap_next, cl_qmap_prev, cl_qmap_end,
*	cl_qmap_item_t
*********/


/****f* Component Library: Quick Map/cl_qmap_tail
* NAME
*	cl_qmap_tail
*
* DESCRIPTION
*	The cl_qmap_tail function returns the map item with the highest key
*	value stored in a quick map.
*
* SYNOPSIS
*/
CL_INLINE cl_map_item_t*
cl_qmap_tail(
	IN	const cl_qmap_t* const	p_map )
{
	ASSERT( p_map );
	ASSERT( p_map->state == CL_INITIALIZED );
	return( (cl_map_item_t*)p_map->nil_item.pool_item.list_item.p_prev );
}
/*
* PARAMETERS
*	p_map
*		[in] Pointer to a cl_qmap_t structure whose item with the highest key
*		is returned.
*
* RETURN VALUES
*	Pointer to the map item with the highest key in the quick map.
*
*	Pointer to the map end if the quick map was empty.
*
* NOTES
*	cl_qmap_end does not remove the item from the map.
*
* SEE ALSO
*	Quick Map, cl_qmap_head, cl_qmap_next, cl_qmap_prev, cl_qmap_end,
*	cl_qmap_item_t
*********/


/****f* Component Library: Quick Map/cl_qmap_next
* NAME
*	cl_qmap_next
*
* DESCRIPTION
*	The cl_qmap_next function returns the map item with the next higher
*	key value than a specified map item.
*
* SYNOPSIS
*/
CL_INLINE cl_map_item_t*
cl_qmap_next(
	IN	const cl_map_item_t* const	p_item )
{
	ASSERT( p_item );
	return( (cl_map_item_t*)p_item->pool_item.list_item.p_next );
}
/*
* PARAMETERS
*	p_item
*		[in] Pointer to a map item whose successor to return.
*
* RETURN VALUES
*	Pointer to the map item with the next higher key value in a quick map.
*
*	Pointer to the map end if the specified item was the last item in
*	the quick map.
*
* SEE ALSO
*	Quick Map, cl_qmap_head, cl_qmap_tail, cl_qmap_prev, cl_qmap_end,
*	cl_map_item_t
*********/


/****f* Component Library: Quick Map/cl_qmap_prev
* NAME
*	cl_qmap_prev
*
* DESCRIPTION
*	The cl_qmap_prev function returns the map item with the next lower
*	key value than a precified map item.
*
* SYNOPSIS
*/
CL_INLINE cl_map_item_t*
cl_qmap_prev(
	IN	const cl_map_item_t* const	p_item )
{
	ASSERT( p_item );
	return( (cl_map_item_t*)p_item->pool_item.list_item.p_prev );
}
/*
* PARAMETERS
*	p_item
*		[in] Pointer to a map item whose predecessor to return.
*
* RETURN VALUES
*	Pointer to the map item with the next lower key value in a quick map.
*
*	Pointer to the map end if the specifid item was the first item in
*	the quick map.
*
* SEE ALSO
*	Quick Map, cl_qmap_head, cl_qmap_tail, cl_qmap_next, cl_qmap_end,
*	cl_map_item_t
*********/


/****f* Component Library: Quick Map/cl_qmap_insert
* NAME
*	cl_qmap_insert
*
* DESCRIPTION
*	The cl_qmap_insert function inserts a map item into a quick map.
*
* SYNOPSIS
*/
CL_EXPORT cl_map_item_t*
cl_qmap_insert(
	IN	cl_qmap_t* const		p_map,
	IN	const uint64			key,
	IN	cl_map_item_t* const	p_item );
/*
* PARAMETERS
*	p_map
*		[in] Pointer to a cl_qmap_t structure into which to add the item.
*
*	key
*		[in] Value to assign to the item.  If Quickmap uses a key
*		compare function, this must be pointer to key structure for
*		object added to list
*
*	p_item
*		[in] Pointer to a cl_map_item_t stucture to insert into the quick map.
*
* RETURN VALUE
*	Pointer to the item in the map with the specified key.  If insertion
*	was successful, this is the pointer to the item.  If an item with the
*	specified key already exists in the map, the pointer to that item is
*	returned.
*
* NOTES
*	Insertion operations may cause the quick map to rebalance.
*
* SEE ALSO
*	Quick Map, cl_qmap_remove, cl_map_item_t
*********/


/****f* Component Library: Quick Map/cl_qmap_get/cl_qmap_get_next
* NAME
*	cl_qmap_get
*
* DESCRIPTION
*	The cl_qmap_get function returns the map item associated with a key.
*
* SYNOPSIS
*/
CL_EXPORT cl_map_item_t*
cl_qmap_get(
	IN	const cl_qmap_t* const	p_map,
	IN	const uint64			key );

CL_EXPORT cl_map_item_t*
cl_qmap_get_next(
	IN	const cl_qmap_t* const	p_map,
	IN	const uint64			key );

/*
* PARAMETERS
*	p_map
*		[in] Pointer to a cl_qmap_t structure from which to retrieve the
*		item with the specified key.
*
*	key
*		[in] Key value used to search for the desired map item.
*		If Quickmap uses a key compare function, this must be pointer to
*		key structure to compare against keys of object in map
* RETURN VALUES
*	Pointer to the map item with the desired key value.
*
*	Pointer to the map end if there was no item with the desired key value
*	stored in the quick map.
*
* NOTES
*	cl_qmap_get does not remove the item from the quick map.
*
* SEE ALSO
*	Quick Map, cl_qmap_remove
*********/

/****f* Component Library: Quick Map/cl_qmap_get_compare
* NAME
*	cl_qmap_get_compare
*
* DESCRIPTION
*	The cl_qmap_get_compare function returns the map item associated with a key.
*
* SYNOPSIS
*/
CL_EXPORT cl_map_item_t*
cl_qmap_get_compare(
	IN	const cl_qmap_t* const	p_map,
	IN	const uint64			key,
	IN  cl_pfn_qmap_compare_key_t key_compare);
/*
* PARAMETERS
*	p_map
*		[in] Pointer to a cl_qmap_t structure from which to retrieve the
*		item with the specified key.
*
*	key
*		[in] Key value used to search for the desired map item.
*		If Quickmap uses a key compare function, this must be pointer to
*		key structure to compare against keys of object in map
*
* 	key_compare
* 		[in] function to compare keys.  Keys of objects in map will
* 		be key1 argument.  key provided as argument to cl_qmap_get_compare
* 		will be used as key2 argument.  Function must be semantically
* 		same comparision as key_compare function of map.
* RETURN VALUES
*	Pointer to the map item with the desired key value.
*
*	Pointer to the map end if there was no item with the desired key value
*	stored in the quick map.
*
* NOTES
*	cl_qmap_get_compare does not remove the item from the quick map.
*
*	This is provided to allow alternate structures for comparison
*
* SEE ALSO
*	Quick Map, cl_qmap_remove
*********/

/****f* Component Library: Quick Map/cl_qmap_get_item_compare
* NAME
*	cl_qmap_get_item_compare
*
* DESCRIPTION
*	The cl_qmap_get_item_compare function returns the map item associated with a key.
*
* SYNOPSIS
*/
CL_EXPORT cl_map_item_t*
cl_qmap_get_item_compare(
	IN	const cl_qmap_t* const	p_map,
	IN	const uint64			key,
	IN  cl_pfn_qmap_item_compare_t compare);
/*
* PARAMETERS
*	p_map
*		[in] Pointer to a cl_qmap_t structure from which to retrieve the
*		item with the specified key.
*
*	key
*		[in] Key value used to search for the desired map item.
*		If Quickmap uses a key compare function, this must be pointer to
*		key structure to compare against keys of object in map
*
* 	compare
* 		[in] function to compare items to key.  objects in map will
* 		be item argument.  key provided as argument to cl_qmap_get_item_compare
* 		will be used as key argument.  Function must be semantically
* 		same comparision as key_compare function of map.
* RETURN VALUES
*	Pointer to the map item with the desired key value.
*
*	Pointer to the map end if there was no item with the desired key value
*	stored in the quick map.
*
* NOTES
*	cl_qmap_get_item_compare does not remove the item from the quick map.
*
*	This is provided to allow alternate structures for comparison
*
* SEE ALSO
*	Quick Map, cl_qmap_remove
*********/

/****f* Component Library: Quick Map/cl_qmap_remove_item
* NAME
*	cl_qmap_remove_item
*
* DESCRIPTION
*	The cl_qmap_remove_item function removes the specified map item
*	from a quick map.
*
* SYNOPSIS
*/
CL_EXPORT void
cl_qmap_remove_item(
	IN	cl_qmap_t* const		p_map,
	IN	cl_map_item_t* const	p_item );
/*
* PARAMETERS
*	p_item
*		[in] Pointer to a map item to remove from its quick map.
*
* RETURN VALUES
*	This function does not return a value.
*
*	In a debug build, cl_qmap_remove_item asserts that the item being removed
*	is in the specified map.
*
* NOTES
*	Removes the map item pointed to by p_item from its quick map.
*
* SEE ALSO
*	Quick Map, cl_qmap_remove, cl_qmap_remove_all, cl_qmap_insert
*********/


/****f* Component Library: Quick Map/cl_qmap_remove
* NAME
*	cl_qmap_remove
*
* DESCRIPTION
*	The cl_qmap_remove function removes the map item with the specified key
*	from a quick map.
*
* SYNOPSIS
*/
CL_EXPORT cl_map_item_t*
cl_qmap_remove(
	IN	cl_qmap_t* const	p_map,
	IN	const uint64		key );
/*
* PARAMETERS
*	p_map
*		[in] Pointer to a cl_qmap_t structure from which to remove the item
*		with the specified key.
*
*	key
*		[in] Key value used to search for the map item to remove.
*		If Quickmap uses a key compare function, this must be pointer to
*		key structure to compare against keys of object in map
*
* RETURN VALUES
*	Pointer to the removed map item if it was found.
*
*	Pointer to the map end if no item with the specified key exists in the
*	quick map.
*
* SEE ALSO
*	Quick Map, cl_qmap_remove_item, cl_qmap_remove_all, cl_qmap_insert
*********/

/****f* Component Library: Quick Map/cl_qmap_remove_compare
* NAME
*	cl_qmap_remove_compare
*
* DESCRIPTION
*	The cl_qmap_remove_compare function removes the map item with the specified key
*	from a quick map.
*
* SYNOPSIS
*/
CL_EXPORT cl_map_item_t*
cl_qmap_remove_compare(
	IN	cl_qmap_t* const	p_map,
	IN	const uint64		key,
	IN  cl_pfn_qmap_compare_key_t key_compare);
/*
* PARAMETERS
*	p_map
*		[in] Pointer to a cl_qmap_t structure from which to remove the item
*		with the specified key.
*
*	key
*		[in] Key value used to search for the map item to remove.
*		If Quickmap uses a key compare function, this must be pointer to
*		key structure to compare against keys of object in map
*
* 	key_compare
* 		[in] function to compare keys.  Keys of objects in map will
* 		be key1 argument.  key provided as argument to cl_qmap_remove_compare
* 		will be used as key2 argument.  Function must be semantically
* 		same comparision as key_compare function of map.
* RETURN VALUES
*	Pointer to the removed map item if it was found.
*
*	Pointer to the map end if no item with the specified key exists in the
*	quick map.
*
*	This is provided to allow alternate structures for comparison
*
* SEE ALSO
*	Quick Map, cl_qmap_remove_item, cl_qmap_remove_all, cl_qmap_insert
*********/

/****f* Component Library: Quick Map/cl_qmap_remove_all
* NAME
*	cl_qmap_remove_all
*
* DESCRIPTION
*	The cl_qmap_remove_all function removes all items in a quick map,
*	leaving it empty.
*
* SYNOPSIS
*/
CL_INLINE void
cl_qmap_remove_all(
	IN	cl_qmap_t* const	p_map )
{
	ASSERT( p_map );
	ASSERT( p_map->state == CL_INITIALIZED );

	p_map->root.p_left = &p_map->nil_item;
	p_map->nil_item.pool_item.list_item.p_next = &p_map->nil_item.pool_item.list_item;
	p_map->nil_item.pool_item.list_item.p_prev = &p_map->nil_item.pool_item.list_item;
	p_map->count = 0;
}
/*
* PARAMETERS
*	p_map
*		[in] Pointer to a cl_qmap_t structure to empty.
*
* RETURN VALUES
*	This function does not return a value.
*
* SEE ALSO
*	Quick Map, cl_qmap_remove, cl_qmap_remove_item
*********/


/****f* Component Library: Quick Map/cl_qmap_merge
* NAME
*	cl_qmap_merge
*
* DESCRIPTION
*	The cl_qmap_merge function moves all items from one map to another,
*	excluding duplicates.
*
* SYNOPSIS
*/
CL_EXPORT void
cl_qmap_merge(
	OUT		cl_qmap_t* const	p_dest_map,
	IN OUT	cl_qmap_t* const	p_src_map );
/*
* PARAMETERS
*	p_dest_map
*		[out] Pointer to a cl_qmap_t structure to which items should be added.
*
*	p_src_map
*		[in/out] Pointer to a cl_qmap_t structure whose items to add
*		to p_dest_map.
*
* RETURN VALUES
*	This function does not return a value.
*
* NOTES
*	Items are evaluated based on their keys only.
*
*	Upon return from cl_qmap_merge, the quick map referenced by p_src_map
*	contains all duplicate items.
*
*	Only valid if p_dest_map and p_src_map have same key compare function
*	(or lack thereof).  Undefined results otherwise.
*
* SEE ALSO
*	Quick Map, cl_qmap_delta
*********/


/****f* Component Library: Quick Map/cl_qmap_delta
* NAME
*	cl_qmap_delta
*
* DESCRIPTION
*	The cl_qmap_delta function computes the differences between two maps.
*
* SYNOPSIS
*/
CL_EXPORT void
cl_qmap_delta(
	IN OUT	cl_qmap_t* const	p_map1,
	IN OUT	cl_qmap_t* const	p_map2,
	OUT		cl_qmap_t* const	p_new,
	OUT		cl_qmap_t* const	p_old );
/*
* PARAMETERS
*	p_map1
*		[in/out] Pointer to the first of two cl_qmap_t structures whose
*		differences to compute.
*
*	p_map2
*		[in/out] Pointer to the second of two cl_qmap_t structures whose
*		differences to compute.
*
*	p_new
*		[out] Pointer to an empty cl_qmap_t structure that contains the items
*		unique to p_map2 upon return from the function.
*
*	p_old
*		[out] Pointer to an empty cl_qmap_t structure that contains the items
*		unique to p_map1 upon return from the function.
*
* RETURN VALUES
*	This function does not return a value.
*
* NOTES
*	Items are evaluated based on their keys.  Items that exist in both
*	p_map1 and p_map2 remain in their respective maps.  Items that
*	exist only p_map1 are moved to p_old.  Likewise, items that exist only
*	in p_map2 are moved to p_new.  This function can be usefull in evaluating
*	changes between two maps.
*
*	Both maps pointed to by p_new and p_old must be empty on input.  This
*	requirement removes the possibility of failures.
*
*	Only valid if p_dest_map and p_src_map have same key compare function
*	(or lack thereof).  Undefined results otherwise.
*
* SEE ALSO
*	Quick Map, cl_qmap_merge
*********/


/****f* Component Library: Quick Map/cl_qmap_apply_func
* NAME
*	cl_qmap_apply_func
*
* DESCRIPTION
*	The cl_qmap_apply_func function executes a specified function
*	for every item stored in a quick map.
*
* SYNOPSIS
*/
CL_EXPORT void
cl_qmap_apply_func(
	IN	const cl_qmap_t* const	p_map,
	IN	cl_pfn_qmap_apply_t		pfn_func,
	IN	const void* const		context );
/*
* PARAMETERS
*	p_map
*		[in] Pointer to a cl_qmap_t structure.
*
*	pfn_func
*		[in] Function invoked for every item in the quick map.
*		See the cl_pfn_qmap_apply_t function type declaration for details
*		about the callback function.
*
*	context
*		[in] Value to pass to the callback functions to provide context.
*
* RETURN VALUE
*	This function does not return a value.
*
* NOTES
*	The function provided must not perform any map operations, as these
*	would corrupt the quick map.
*
* SEE ALSO
*	Quick Map, cl_pfn_qmap_apply_t
*********/


/****i* Component Library: Quick List/__cl_primitive_insert
* NAME
*       __cl_primitive_insert
*
* DESCRIPTION
*       Add a new item in front of the specified item.  This is a low level
*       function for use internally by the queuing routines.
*
* SYNOPSIS
*/
CL_INLINE void
__cl_primitive_insert(
        IN      cl_list_item_t* const   p_list_item,
        IN      cl_list_item_t* const   p_new_item )
{
        /* ASSERT that a non-null pointer is provided. */
        ASSERT( p_list_item );
        /* ASSERT that a non-null pointer is provided. */
        ASSERT( p_new_item );

        p_new_item->p_next = p_list_item;
        p_new_item->p_prev = p_list_item->p_prev;
        p_list_item->p_prev = p_new_item;
        p_new_item->p_prev->p_next = p_new_item;
}

/*
* PARAMETERS
*       p_list_item
*               [in] Pointer to cl_list_item_t to insert in front of
*
*       p_new_item
*               [in] Pointer to cl_list_item_t to add
*
* RETURN VALUE
*       This function does not return a value.
*********/


/****i* Component Library: Quick List/__cl_primitive_remove
* NAME
*       __cl_primitive_remove
*
* DESCRIPTION
*       Remove an item from a list.  This is a low level routine
*       for use internally by the queuing routines.
*
* SYNOPSIS
*/
CL_INLINE void
__cl_primitive_remove(
        IN      cl_list_item_t* const   p_list_item )
{
        /* ASSERT that a non-null pointer is provided. */
        ASSERT( p_list_item );

        /* set the back pointer */
        p_list_item->p_next->p_prev= p_list_item->p_prev;
        /* set the next pointer */
        p_list_item->p_prev->p_next= p_list_item->p_next;

        /* if we're debugging, spruce up the pointers to help find bugs */
#if QMAP_DEBUG
        if( p_list_item != p_list_item->p_next )
        {
                p_list_item->p_next = NULL;
                p_list_item->p_prev = NULL;
        }
#endif  /* QMAP_DEBUG */
}

#ifdef __cplusplus
}
#endif


#endif	/* _IBA_PUBLIC_IQUICKMAP_H_ */

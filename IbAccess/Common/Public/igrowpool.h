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


#ifndef _IBA_PUBLIC_IGROWPOOL_H_
#define _IBA_PUBLIC_IGROWPOOL_H_

#include "iba/public/datatypes.h"
#include "iba/public/iobjmgr.h"
#include "iba/public/ilist.h"

#ifdef __cplusplus
extern "C" {
#endif


/****************************************************************************
 ****************************************************************************
 ***********													   **********
 ***********			DECLARATION OF QUICK COMPOSITE POOL		   **********
 ***********													   **********
 ****************************************************************************
 ****************************************************************************/

typedef enum _POOL_STATE
{
	PoolReset,
	PoolInitError,
	PoolReady

} POOL_STATE;

/****************************************************************************
 * Quick composite pool callback 
 * function types.
 */
typedef LIST_ITEM* (*QCPOOL_CTOR_FUNC)( 
	IN void** const pCompArray, 
	IN uint32 NumComponents, 
	IN void* const Context ); 

typedef FSTATUS (*QCPOOL_INIT_FUNC)(
	IN LIST_ITEM* const pListItem, 
	IN void* const Context ); 

typedef void (*QCPOOL_DTOR_FUNC)(
	IN LIST_ITEM* const pListItem, 
	IN void* const Context ); 


/****************************************************************************
 * Quick composite pool structure.
 */
typedef struct _QCOMP_POOL
{
	/* These fields are private and should NOt be directly accessed */
	/* Number of components that make up a composite object. */
	uint32				m_NumComponents;

	/* Array of sizes for components that make up a user object. */
	uint32				*m_ComponentSizes;

	/* Array of pointers to components, used for constructor callback. */
	void				**m_pComponents;

	/* Number of objects in the pool */
	uint32				m_NumObjects;

	/* Number of objects to auto-grow */
	uint32				m_GrowSize;

	/* Pointer to object constructor */
	QCPOOL_CTOR_FUNC	m_pfnCtor;

	/* Pointer to object initializer */
	QCPOOL_INIT_FUNC	m_pfnInit;

	/* Pointer to object destructor */
	QCPOOL_DTOR_FUNC	m_pfnDtor;

	/* Context to pass to callback functions. */
	void				*m_Context;

	/* Stack of available objects */
	QUICK_LIST			m_FreeList;

	/* Object manager holds allocations */
	OBJECT_MGR			m_ObjMgr;

	POOL_STATE			m_State;

} QCOMP_POOL;



/****************************************************************************
 * Quick composite pool access methods.
 */

/*
 * Initialize the state of a quick composite pool.
 */
void 
QCompPoolInitState(
	IN QCOMP_POOL* const pPool );


/*
 * Initialize a quick composite pool.
 */
FSTATUS 
QCompPoolInit(
	IN QCOMP_POOL* const pPool,
	IN const uint32 MinCount,
	IN const uint32* const ComponentSizes,
	IN const uint32 NumComponents,
	IN const uint32 GrowSize,
	IN QCPOOL_CTOR_FUNC pfnConstructor,
	IN QCPOOL_INIT_FUNC pfnInitializer OPTIONAL,
	IN QCPOOL_DTOR_FUNC pfnDestructor OPTIONAL,
	IN void* const Context );


/*
 * Destroy a quick composite pool.
 */
void 
QCompPoolDestroy(
	IN QCOMP_POOL* const pPool );

/*
 * Returns the number of objects available in a quick composite pool.
 */
static __inline uint32 
QCompPoolCount(
	IN QCOMP_POOL* const pPool )
{
	ASSERT( pPool );
	return( QListCount( &pPool->m_FreeList ) );
}


/*
 * Get an object from a quick composite pool.
 */
LIST_ITEM* 
QCompPoolGet(
	IN QCOMP_POOL* const pPool );


/*
 * Puts an object back in a quick composite pool.
 */
static __inline void
QCompPoolPut(
	IN QCOMP_POOL* const pPool,
	IN LIST_ITEM* const pListItem )
{
	/* return this lil' doggy to the pool */
	QListInsertHead( &pPool->m_FreeList, pListItem );
	/* check for funny-business */
	ASSERT( QCompPoolCount(pPool) <= pPool->m_NumObjects );
}


/*
 * Grows a quick composite pool by the specified number of objects.
 */
FSTATUS 
QCompPoolGrow(
	IN QCOMP_POOL* const pPool, 
	IN uint32 ObjCount );


/****************************************************************************
 ****************************************************************************
 ***********													   **********
 ***********			DECLARATION OF QUICK GROW POOL			   **********
 ***********													   **********
 ****************************************************************************
 ****************************************************************************/

/****************************************************************************
 * Quick grow pool callback function types.
 */
typedef LIST_ITEM* (*QGPOOL_CTOR_FUNC)( 
	IN void* const pObject,
	IN void* const Context );

typedef FSTATUS (*QGPOOL_INIT_FUNC)( 
	IN LIST_ITEM* const pListItem,
	IN void* const Context );

typedef void (*QGPOOL_DTOR_FUNC)( 
	IN LIST_ITEM* const pListItem,
	IN void* const Context );


/****************************************************************************
 * Grow pool structure.
 */
typedef struct _QGROW_POOL
{
	/* These fields are private and should NOT be directly accessed */
	QCOMP_POOL			m_QCPool;

	QGPOOL_CTOR_FUNC	m_pfnCtor;
	QGPOOL_INIT_FUNC	m_pfnInit;
	QGPOOL_DTOR_FUNC	m_pfnDtor;
	void				*m_Context;

} QGROW_POOL;


/****************************************************************************
 * Quick grow pool access methods.
 */

/*
 * Initializes the state of a quick grow pool.
 */
void 
QGrowPoolInitState(
	IN QGROW_POOL* const pPool );


/*
 * Initializes a quick grow pool.
 */
FSTATUS 
QGrowPoolInit(
	IN QGROW_POOL* const pPool,
	IN const uint32 MinCount,
	IN const uint32 ObjectSize,
	IN const uint32 GrowSize,
	IN QGPOOL_CTOR_FUNC pfnConstructor OPTIONAL,
	IN QGPOOL_INIT_FUNC pfnInitializer OPTIONAL,
	IN QGPOOL_DTOR_FUNC pnfDestructor OPTIONAL,
	IN void* const Context );


/*
 * Destroy a quick grow pool.
 */
void 
QGrowPoolDestroy(
	IN QGROW_POOL* const pPool );


/*
 * Gets an object from a quick grow pool.
 */
static __inline LIST_ITEM* 
QGrowPoolGet(
	IN QGROW_POOL* const pPool )
{
	ASSERT( pPool );
	return( QCompPoolGet( &pPool->m_QCPool ) );
}


/*
 * Puts an object back in the pool.
 */
static __inline void
QGrowPoolPut(
	IN QGROW_POOL* const pPool, 
	LIST_ITEM* const pListItem )
{
	ASSERT( pPool );
	QCompPoolPut( &pPool->m_QCPool, pListItem );
}


/*
 * Grows a quick grow pool by the specified number of objects.
 */
static __inline FSTATUS 
QGrowPoolGrow(
	IN QGROW_POOL* const pPool, 
	IN const uint32 ObjCount )
{
	ASSERT( pPool );
	return( QCompPoolGrow( &pPool->m_QCPool, ObjCount ) );
}


/*
 * Returns the number of objects available in a quick grow pool.
 */
static __inline uint32 
QGrowPoolCount(
	IN QGROW_POOL* const pPool )
{
	ASSERT( pPool );
	return( QCompPoolCount( &pPool->m_QCPool ) );
}


/****************************************************************************
 ****************************************************************************
 ***********													   **********
 ***********			DECLARATION OF COMPOSITE POOL			   **********
 ***********													   **********
 ****************************************************************************
 ****************************************************************************/

/****************************************************************************
 * Composite pool callback function types.
 */
typedef void (*CPOOL_CTOR_FUNC)( 
	IN void** const pCompArray, 
	IN uint32 NumComponents, 
	IN void* const Context ); 

typedef FSTATUS (*CPOOL_INIT_FUNC)(
	IN void* const pObject, 
	IN void* const Context ); 

typedef void (*CPOOL_DTOR_FUNC)(
	IN void* const pObject, 
	IN void* const Context ); 


/****************************************************************************
 * Composite pool structure.
 */
typedef struct _COMP_POOL
{
	/* These fields are private and should NOT be directly accessed */
	QCOMP_POOL			m_QCPool;

	/* Callback pointers. */
	CPOOL_CTOR_FUNC		m_pfnCtor;
	CPOOL_INIT_FUNC		m_pfnInit;
	CPOOL_DTOR_FUNC		m_pfnDtor;
	/* Context to pass to callback functions. */
	void				*m_Context;

} COMP_POOL;


/****************************************************************************
 * Composite pool access methods.
 */

/*
 * Initializes the state of a composite pool.
 */
void 
CompPoolInitState(
	IN COMP_POOL* const pCPool );


/*
 * Initializes a composite pool.
 */
FSTATUS 
CompPoolInit(
	IN COMP_POOL* const pCPool,
	IN const uint32 MinCount,
	IN const uint32* const ComponentSizes,
	IN const uint32 NumComponents,
	IN const uint32 GrowSize,
	IN CPOOL_CTOR_FUNC pfnConstructor,
	IN CPOOL_INIT_FUNC pfnInitializer OPTIONAL,
	IN CPOOL_DTOR_FUNC pfnDestructor OPTIONAL,
	IN void* const Context );


/*
 * Destroys a composite pool.
 */
void 
CompPoolDestroy(
	IN COMP_POOL* const pCPool );


/*
 * Gets an object from a composite pool.
 */
static __inline void* 
CompPoolGet(
	IN COMP_POOL* const pCPool )
{
	LIST_ITEM	*pListItem;

	ASSERT( pCPool );

	pListItem = QCompPoolGet( &pCPool->m_QCPool );
	if( !pListItem )
		return( NULL );

	return( QListObj( pListItem ) );
}


/*
 * Puts an object back in a composite pool.
 */
static __inline void
CompPoolPut(
	IN COMP_POOL* const pCPool,
	IN void* const pObject )
{
	LIST_ITEM	*pListItem;

	ASSERT( pCPool );
	ASSERT( pObject );

	pListItem = (LIST_ITEM*)((uchar*)pObject - sizeof(LIST_ITEM));

	/* good sanity check */
	ASSERT( pListItem->pObject == pObject );

	QCompPoolPut( &pCPool->m_QCPool, pListItem );
}


/*
 * Grows a composite pool by the specified number of objects.
 */
static __inline FSTATUS 
CompPoolGrow(
	IN COMP_POOL* const pCPool, 
	IN uint32 ObjCount )
{
	ASSERT( pCPool );
	return( QCompPoolGrow( &pCPool->m_QCPool, ObjCount ) );
}


/*
 * Returns the number of objects available in a composite pool.
 */
static __inline uint32 
CompPoolCount(
	IN COMP_POOL* const pCPool )
{
	ASSERT( pCPool );
	return( QCompPoolCount( &pCPool->m_QCPool ) );
}


/****************************************************************************
 ****************************************************************************
 ***********													   **********
 ***********				DECLARATION OF GROW POOL			   **********
 ***********													   **********
 ****************************************************************************
 ****************************************************************************/

/****************************************************************************
 * Grow pool callback function types.
 */
typedef void (*GPOOL_CTOR_FUNC)( 
	IN void* const pObject,
	IN void* const Context );

typedef FSTATUS (*GPOOL_INIT_FUNC)( 
	IN void* const pObject,
	IN void* const Context );

typedef void (*GPOOL_DTOR_FUNC)( 
	IN void* const pObject,
	IN void* const Context );


/****************************************************************************
 * Grow pool structure.
 */
typedef struct _GROW_POOL
{
	/* These fields are private and should NOT be directly accessed */
	QCOMP_POOL			m_QCPool;

	GPOOL_CTOR_FUNC		m_pfnCtor;
	GPOOL_INIT_FUNC		m_pfnInit;
	GPOOL_DTOR_FUNC		m_pfnDtor;
	void				*m_Context;

} GROW_POOL;


/****************************************************************************
 * Grow pool access methods.
 */

/*
 * Initializes the state of a grow pool.
 */
void 
GrowPoolInitState(
	IN GROW_POOL* const pGPool );


/*
 * Initialize a grow pool.
 */
FSTATUS 
GrowPoolInit(
	IN GROW_POOL* const pGPool,
	IN const uint32 MinCount,
	IN uint32 ObjectSize,
	IN const uint32 GrowSize,
	IN GPOOL_CTOR_FUNC pfnConstructor OPTIONAL,
	IN GPOOL_INIT_FUNC pfnInitializer OPTIONAL,
	IN GPOOL_DTOR_FUNC pnfDestructor OPTIONAL,
	IN void* const Context );


/*
 * Destroy a grow pool.
 */
void 
GrowPoolDestroy(
	IN GROW_POOL* const pGPool );

	
/*
 * Get an object from a grow pool.
 */
static __inline void* 
GrowPoolGet(
	IN GROW_POOL* const pGPool )
{
	LIST_ITEM	*pListItem;

	ASSERT( pGPool );

	pListItem = QCompPoolGet( &pGPool->m_QCPool );
	if( !pListItem )
		return( NULL );

	ASSERT( QListObj( pListItem ) != NULL );
	return( QListObj( pListItem ) );
}


/*
 * Put an object back in a grow pool.
 */
static __inline void
GrowPoolPut(
	IN GROW_POOL* const pGPool, 
	void* const pObject )
{
	LIST_ITEM	*pListItem;

	ASSERT( pGPool );
	ASSERT( pObject );

	pListItem = (LIST_ITEM*)((uchar*)pObject - sizeof(LIST_ITEM));

	/* good sanity check */
	ASSERT( pListItem->pObject == pObject );

	QCompPoolPut( &pGPool->m_QCPool, pListItem );
}


/*
 * Grows the resource pool by the specified number of objects.
 */
static __inline FSTATUS 
GrowPoolGrow(
	IN GROW_POOL* const pGPool, 
	IN const uint32 ObjCount )
{
	ASSERT( pGPool );
	return( QCompPoolGrow( &pGPool->m_QCPool, ObjCount ) );
}


/*
 * Returns the number of objects available in the free pool.
 */
static __inline uint32 
GrowPoolCount(
	IN GROW_POOL* const pGPool )
{
	ASSERT( pGPool );
	return( QCompPoolCount( &pGPool->m_QCPool ) );
}


#ifdef __cplusplus
}	/* extern "C" */
#endif


#endif	/* _IBA_PUBLIC_IGROWPOOL_H_ */

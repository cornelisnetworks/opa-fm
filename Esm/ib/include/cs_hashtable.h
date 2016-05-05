/*
 * Copyright (c) 2002, Christopher Clark
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * 
 * * Neither the name of the original author; nor the names of any contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 * 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef __CS_HASHTABLE_H__
#define __CS_HASHTABLE_H__

#include "vs_g.h"

/* Example of use:
 *
 *      struct cs_hashtable  *h;
 *      struct a_key      *k;
 *      struct a_value    *v;
 *
 *      static unsigned int         hash_from_key_fn( void *k );
 *      static int                  keys_equal_fn ( void *key1, void *key2 );
 *
 *      h = cs_create_hashtable(16, hash_from_key_fn, keys_equal_fn);
 *      k = (struct a_key *)     malloc(sizeof(struct some_key));
 *      v = (struct a_value *)   malloc(sizeof(struct some_value));
 *
 *      (initialise k and v to suitable values)
 * 
 *      if (! cs_hashtable_insert(h,k,v) )
 *      {     exit(-1);               }
 *
 *      if (NULL == (found = cs_hashtable_search(h,k) ))
 *      {    printf("not found!");                  }
 *
 *      if (NULL == (found = cs_hashtable_remove(h,k) ))
 *      {    printf("Not found\n");                 }
 *
 */

/* Macros may be used to define type-safe(r) hashtable access functions, with
 * methods specialized to take known key and value types as parameters.
 * 
 * Example:
 *
 * Insert this at the start of your file:
 *
 * DEFINE_HASHTABLE_INSERT(hashInsert, struct a_key, struct a_value);
 * DEFINE_HASHTABLE_SEARCH(hashSearch, struct a_key, struct a_value);
 * DEFINE_HASHTABLE_REMOVE(hashRemove, struct a_key, struct a_value);
 *
 * This defines the functions 'hashInsert', 'hashSearch' and 'hashRemove'.
 * These operate just like cs_hashtable_insert etc., with the same parameters,
 * but their function signatures have 'struct a_key *' rather than
 * 'void *', and hence can generate compile time errors if your program is
 * supplying incorrect data as a key (and similarly for value).
 *
 * Note that the hash and key equality functions passed to cs_create_hashtable
 * still take 'void *' parameters instead of 'a_key *'. This shouldn't be
 * a difficult issue as they're only defined and passed once, and the other
 * functions will ensure that only valid keys are supplied to them.
 *
 * The cost for this checking is increased code size and runtime overhead
 * - if performance is important, it may be worth switching back to the
 * unsafe methods once your program has been debugged with the safe methods.
 * This just requires switching to some simple alternative defines - eg:
 * #define hashInsert cs_hashtable_insert
 *
 */

typedef enum {
    CS_HASH_KEY_NOT_ALLOCATED,
    CS_HASH_KEY_ALLOCATED
} CS_Hash_KeyType_t;

/*
 *  hash_entry
 */
struct hash_entry
{
    void *k, *v;
    uint64_t h;
    struct hash_entry *hashNext;
    struct hash_entry *listNext;
    struct hash_entry *listPrev;
};
typedef struct hash_entry CS_HashEntry_t;
typedef struct hash_entry * CS_HashEntryp;

struct cs_hashtable {
    const char *name;
    CS_HashEntry_t *listHead;
    CS_HashEntry_t *listTail;
    CS_HashEntry_t *freeHead;
    uint32_t tablelength;
    CS_HashEntryp *table;
    uint32_t entrycount;
    uint32_t loadlimit;
    uint32_t primeindex;
    CS_Hash_KeyType_t keyType;
    uint64_t (*hashfn) (void *k);
    int32_t (*eqfn) (void *k1, void *k2);
};
typedef struct cs_hashtable CS_HashTable_t;
typedef struct cs_hashtable * CS_HashTablep;

/*
 * This struct is only concrete here to allow the inlining of two of the
 * accessor functions. 
 */
struct cs_hashtable_itr
{
    CS_HashTablep h;
    CS_HashEntryp e;
};
typedef struct cs_hashtable_itr CS_HashTableItr_t;
typedef struct cs_hashtable_itr * CS_HashTableItrp;


/*
 * indexFor 
 * */
static inline uint32_t
indexFor(uint32_t tablelength, uint32_t hashvalue) {
    return (hashvalue % tablelength);
};


/*
 *  Free the key
 */
#define freekey(X) free(X)

/*****************************************************************************
 * cs_create_hashtable
   
 * @name                    cs_create_hashtable
 * @param   name            hash name string
 * @param   minsize         minimum initial size of hashtable
 * @param   hashfunction    function for hashing keys
 * @param   key_eq_fn       function for determining key equality
 * @param   keytype         set to true if key is an allocated structure, false otherwise
 * @return                  newly created hashtable or NULL on failure
 */

CS_HashTablep
cs_create_hashtable(const char *name, uint32_t minsize,
                    uint64_t (*hashfunction) (void*),
                    int32_t (*key_eq_fn) (void*,void*),
                    CS_Hash_KeyType_t keytype);

/*****************************************************************************
 * cs_hashtable_insert
   
 * @name        cs_hashtable_insert
 * @param   h   the hashtable to insert into
 * @param   k   the key - hashtable claims ownership and will free on removal
 * @param   v   the value - does not claim ownership
 * @return      non-zero for successful insertion
 *
 * This function will cause the table to expand if the insertion would take
 * the ratio of entries to table size over the maximum load factor.
 *
 * This function does not check for repeated insertions with a duplicate key.
 * The value returned when using a duplicate key is undefined -- when
 * the hashtable changes size, the order of retrieval of duplicate key
 * entries is reversed.
 * If in doubt, remove before insert.
 */

int32_t cs_hashtable_insert(CS_HashTablep h, void *k, void *v);

#define DEFINE_HASHTABLE_INSERT(fncname, keytype, valuetype) \
int32_t fncname (CS_HashTablep h, keytype *k, valuetype *v) \
{ \
    return cs_hashtable_insert(h,k,v); \
}

/*****************************************************************************
 * cs_hashtable_search
   
 * @name        cs_hashtable_search
 * @param   h   the hashtable to search
 * @param   k   the key to search for  - does not claim ownership
 * @return      the value associated with the key, or NULL if none found
 */

void *cs_hashtable_search(CS_HashTablep h, void *k);

#define DEFINE_HASHTABLE_SEARCH(fncname, keytype, valuetype) \
valuetype * fncname (CS_HashTablep h, keytype *k) \
{ \
    return (valuetype *) (cs_hashtable_search(h,k)); \
}

/*****************************************************************************
 * cs_hashtable_remove
   
 * @name        cs_hashtable_remove
 * @param   h   the hashtable to remove the item from
 * @param   k   the key to search for  - does not claim ownership
 * @return      the value associated with the key, or NULL if none found
 */

void *cs_hashtable_remove(CS_HashTablep h, void *k);

#define DEFINE_HASHTABLE_REMOVE(fncname, keytype, valuetype) \
valuetype * fncname (CS_HashTablep h, keytype *k) \
{ \
    return (valuetype *) (cs_hashtable_remove(h,k)); \
}


/*****************************************************************************
 * cs_hashtable_count
   
 * @name        cs_hashtable_count
 * @param   h   the hashtable
 * @return      the number of items stored in the hashtable
 */
static __inline uint32_t
cs_hashtable_count(CS_HashTablep h)
{
    return h->entrycount;
}


/*****************************************************************************
 * cs_hashtable_destroy
   
 * @name        cs_hashtable_destroy
 * @param   h   the hashtable
 * @param       free_values     whether to call 'free' on the remaining values
 */

void cs_hashtable_destroy(CS_HashTablep h, int32_t free_values);

/*****************************************************************************
 * cs_hashtable_change
 *
 * function to change the value associated with a key, where there already
 * exists a value bound to the key in the hashtable.
 * Source due to Holger Schemel.
 *
 * @name        cs_hashtable_change
 * @param   h   the hashtable
 * @param       key
 * @param       value
 *
 */
int32_t cs_hashtable_change(CS_HashTablep h, void *k, void *v);

/*
 * cs_hashtable_iterator
 */

static __inline void
cs_hashtable_iterator(CS_HashTablep h, CS_HashTableItr_t *itr) {
    itr->h = h;
    itr->e = h->listHead;
}

/*
 * cs_hashtable_iterator_key
 * - return the value of the (key,value) pair at the current position 
 */

static __inline void *
cs_hashtable_iterator_key(CS_HashTableItrp i)
{
    return i->e->k;
}

/*
 * value - return the value of the (key,value) pair at the current position 
 */

static __inline void *
cs_hashtable_iterator_value(CS_HashTableItrp i)
{
    return i->e->v;
}

/*
 * advance - advance the iterator to the next element
 *           returns zero if advanced to end of table 
 */

static __inline int32_t
cs_hashtable_iterator_advance(CS_HashTableItrp itr) {
    if (itr->e != NULL)
        return (itr->e = itr->e->listNext) == NULL ? 0 : -1;
    return 0;
}

/*
 * remove - remove current element and advance the iterator to the next element
 *          NB: if you need the value to free it, read it before
 *          removing. ie: beware memory leaks!
 *          returns zero if advanced to end of table 
 */

int32_t cs_hashtable_iterator_remove(CS_HashTableItrp itr);

/*
 * search - overwrite the supplied iterator, to point to the entry
 *          matching the supplied key.
 *          h points to the hashtable to be searched.
 *          returns zero if not found. 
 */
int32_t cs_hashtable_iterator_search(CS_HashTableItrp itr,
                                     CS_HashTablep h, void *k);

#define DEFINE_HASHTABLE_ITERATOR_SEARCH(fncname, keytype) \
int32_t fncname (CS_HashTableItrp i, CS_HashTablep h, keytype *k) \
{ \
    return (cs_hashtable_iterator_search(i,h,k)); \
}

#endif /* __CS_HASHTABLE_H__ */


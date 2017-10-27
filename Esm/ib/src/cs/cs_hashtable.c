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

#define INLINE
#include "cs_hashtable.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

/*
 * prime table
 */
static const uint32_t primes[] = {
53, 97, 193, 389,
769, 1543, 3079, 6151,
12289, 24593, 49157, 98317,
196613, 393241, 786433, 1572869,
3145739, 6291469, 12582917, 25165843,
50331653, 100663319, 201326611, 402653189,
805306457, 1610612741
};
const uint32_t prime_table_length = sizeof(primes)/sizeof(primes[0]);
#define MAX_LOAD_FACTOR_NUMERATOR 8
#define MAX_LOAD_FACTOR_DENOMINATOR 10
static int32_t cs_hashtable_expand(struct cs_hashtable *h);

/*
 * cs_create_hashtable
 */
struct cs_hashtable *
cs_create_hashtable(const char *name,
                    uint32_t minsize,
                    uint64_t (*hashf) (void*),
                    int32_t (*eqf) (void*,void*),
                    CS_Hash_KeyType_t keytype)
{
    struct cs_hashtable *h;
    uint32_t pindex;

    /* Check requested hashtable isn't too large */
    if (minsize > (1u << 30)) return NULL;
    
    /* Enforce size as prime */
    for (pindex=0; pindex < prime_table_length; pindex++) {
        if (primes[pindex] > minsize) break;
    }

    h = (struct cs_hashtable *)malloc(sizeof(struct cs_hashtable));
    if (NULL == h) return NULL;

    memset(h, 0, sizeof(*h));
    h->name = name;
    h->primeindex   = pindex - 1;
    h->hashfn       = hashf;
    h->eqfn         = eqf;
    h->keyType      = keytype;
    if (cs_hashtable_expand(h) == 0) {
        free(h);
        return NULL;
    }
    return h;
}

/*
 * hash
 */
static uint64_t 
_hash(struct cs_hashtable *h, void *k)
{
    /* 
     * Aim to protect against poor hash functions by adding logic here
     * - logic taken from java 1.4 hashtable source 
     */
    uint64_t i = h->hashfn(k);
    i += ~(i << 9);
    i ^=  ((i >> 14) | (i << 18)); /* >>> */
    i +=  (i << 4);
    i ^=  ((i >> 10) | (i << 22)); /* >>> */
    return i;
}

/*
 * cs_hashtable_expand
 */
static int32_t
cs_hashtable_expand(struct cs_hashtable *h)
{
    /* Double the size of the table to accomodate more entries */
    struct hash_entry **newtable;
    struct hash_entry *e;
    uint32_t newsize;
    uint32_t index;
    /* Check we're not hitting max capacity */
    if (h->primeindex == (prime_table_length - 1)) 
        return 0;
    newsize = primes[++(h->primeindex)];
//  sysPrintf("%s h=%p(%s) primeindex=%lu newsize=%lu\n", __FUNCTION__, h, h->name, h->primeindex, newsize);

    /* malloc table */
    newtable = (struct hash_entry **)malloc(newsize * sizeof(struct hash_entry *));
    if (NULL == newtable) { 
        (h->primeindex)--; 
        return 0;
    }
    memset(newtable, 0, newsize * sizeof(struct hash_entry *));
    for (e = h->listHead; e != NULL; e = e->listNext) {
        index = indexFor(newsize, e->h);
        e->hashNext = newtable[index];
        newtable[index] = e;
    }
    if (h->table)
        free(h->table);
    h->table = newtable;
    h->tablelength = newsize;
    h->loadlimit = ((uint64_t)newsize * MAX_LOAD_FACTOR_NUMERATOR) / MAX_LOAD_FACTOR_DENOMINATOR;
    return -1;
}

/*
 * cs_hashtable_insert
 */
int32_t
cs_hashtable_insert(struct cs_hashtable *h, void *k, void *v)
{
    /* This method allows duplicate keys - but they shouldn't be used */
    uint32_t index;
    struct hash_entry *e;
//  sysPrintf("%s h=%p(%s) k=%p v=%p ra=%p\n", __FUNCTION__, h, h->name, k, v, __builtin_return_address(0));
    if (++h->entrycount > h->loadlimit) {
        /* Ignore the return value. If expand fails, we should
         * still try cramming just this value into the existing table
         * -- we may not have memory for a larger table, but one more
         * element may be ok. Next time we insert, we'll try expanding again.*/
        cs_hashtable_expand(h);
    }
    /* retrieve a free hash_entry or allocate a new one */
    if ((e = h->freeHead) == NULL) {
        e = (struct hash_entry *)malloc(sizeof(struct hash_entry));
        if (NULL == e) { /* out of memory */
            --h->entrycount; 
            return 0;
        }
    } else
        h->freeHead = e->listNext;

    e->h = _hash(h,k);
    //IB_LOG_INFINI_INFOLX("key is", e->h);
    index = indexFor(h->tablelength,e->h);
    e->k = k;
    e->v = v;
    e->hashNext = h->table[index];
    h->table[index] = e;
    e->listNext = NULL;
    if (h->listHead == NULL) {
        h->listHead = h->listTail = e;
        e->listPrev = NULL;
    } else {
        e->listPrev = h->listTail;
        h->listTail = h->listTail->listNext = e;
    }
    return -1;
}

/*
 * cs_hashtable_search
 */
void *cs_hashtable_search(struct cs_hashtable *h, void *k)
{
    struct hash_entry *e;
    uint64_t hashvalue = _hash(h,k);

    for(e = h->table[indexFor(h->tablelength,hashvalue)]; e != NULL; e = e->hashNext) {
        /* Check hash value to short circuit heavier comparison */
        if ((hashvalue == e->h) && (h->eqfn(k, e->k))) 
            return e->v;
    }
    return NULL;
}

void cs_hashentry_delete(struct cs_hashtable *h, struct hash_entry *e) {
    h->entrycount--;
    if (e == h->listHead) 
        h->listHead = e->listNext;
    if (e == h->listTail)
        h->listTail = e->listPrev;
    if (e->listPrev != NULL)
        e->listPrev->listNext = e->listNext;
    if (e->listNext != NULL)
        e->listNext->listPrev = e->listPrev;
    if (h->keyType == CS_HASH_KEY_ALLOCATED) freekey(e->k);
    e->listNext = h->freeHead;
    h->freeHead = e;
}
/*
 * cs_hashtable_remove
 * returns value associated with key 
 */
void *cs_hashtable_remove(struct cs_hashtable *h, void *k) {
    /* TODO: consider compacting the table when the load factor drops enough,
     *       or provide a 'compact' method. */

    struct hash_entry *e;
    struct hash_entry **pE;
    void *v;
    uint64_t hashvalue = _hash(h,k);

    pE = &(h->table[indexFor(h->tablelength,hashvalue)]);
    for(e = *pE; e != NULL; e = e->hashNext) {
        /* Check hash value to short circuit heavier comparison */
        if ((hashvalue == e->h) && (h->eqfn(k, e->k))) {
            *pE = e->hashNext;
            v = e->v;
            cs_hashentry_delete(h, e);
            return v;
        }
        pE = &(e->hashNext);
    }
    return NULL;
}

/*
 * cs_hashtable_destroy
 */
void cs_hashtable_destroy(struct cs_hashtable *h, int32_t free_values)
{
    struct hash_entry *e;
    struct hash_entry *listNext;

    for(e = h->freeHead; e != NULL; e = listNext) {
        listNext = e->listNext;
        free(e);
    }

    for(e = h->listHead; e != NULL; e = listNext) {
        listNext = e->listNext;
        if (h->keyType == CS_HASH_KEY_ALLOCATED) freekey(e->k);
        if (free_values)
            free(e->v);
        free(e);
    }
    free(h->table);
    free(h);
}


/*
 * cs_hashtable_change
 *
 * function to change the value associated with a key, where there already
 * exists a value bound to the key in the hashtable.
 * Source due to Holger Schemel.
 * 
 */
int32_t 
cs_hashtable_change(struct cs_hashtable *h, void *k, void *v)
{
    struct hash_entry *e;
    uint64_t hashvalue = _hash(h,k);

    for(e = h->table[indexFor(h->tablelength,hashvalue)]; e != NULL; e = e->hashNext) {
        /* Check hash value to short circuit heavier comparison */
        if ((hashvalue == e->h) && (h->eqfn(k, e->k))) {
            free(e->v);
            e->v = v;
            return -1;
        }
    }
    return 0;
}

/*
 * remove - remove the entry at the current iterator position
 *          and advance the iterator, if there is a successive
 *          element.
 *          If you want the value, read it before you remove:
 *          beware memory leaks if you don't.
 *          Returns zero if end of iteration. 
 */
int32_t 
cs_hashtable_iterator_remove(struct cs_hashtable_itr *itr) {
    struct cs_hashtable *h = itr->h;
    struct hash_entry *removeEntry = itr->e;
    struct hash_entry *e;
    struct hash_entry **pE;
    int32_t ret;

    ret = cs_hashtable_iterator_advance(itr);

    pE = &(h->table[indexFor(h->tablelength,_hash(h,removeEntry->k))]);
    for(e = *pE; e != NULL; e = e->hashNext) {
        if (e == removeEntry) {
            *pE = e->hashNext;
            cs_hashentry_delete(h, e);
            break;
        }
        pE = &(e->hashNext);
    }
    return ret;
}

/* 
 * cs_hashtable_iterator_search
 * returns zero if not found 
 */
int32_t 
cs_hashtable_iterator_search(struct cs_hashtable_itr *itr,
                                     struct cs_hashtable *h, void *k) {
    struct hash_entry *e;
    uint64_t hashvalue = _hash(h,k);

    for(e = h->table[indexFor(h->tablelength,hashvalue)]; e != NULL; e = e->hashNext) {
        /* Check hash value to short circuit heavier comparison */
        if ((hashvalue == e->h) && (h->eqfn(k, e->k))) {
            itr->e = e;
            itr->h = h;
            return -1;
        }
    }
    return 0;
}


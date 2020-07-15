#include "buzzdict.h"
#include <stdlib.h>
#include <string.h>

/****************************************/
/****************************************/

struct buzzdict_entry_s {
   void* key;
   void* data;
};

#define buzzdict_entry_new(dt, e, k, d)         \
   struct buzzdict_entry_s e;                   \
   e.key = malloc(dt->key_size);                \
   memcpy(e.key, k, dt->key_size);              \
   e.data = malloc(dt->data_size);              \
   memcpy(e.data, d, dt->data_size);

void buzzdict_entry_destroy(const void* key, void* data, void* params) {
   free((void*)key);
   free(data);
}

/****************************************/
/****************************************/

buzzdict_t buzzdict_new(uint32_t buckets,
                        uint32_t key_size,
                        uint32_t data_size,
                        buzzdict_hashfunp hashf,
                        buzzdict_key_cmpp keycmpf,
                        buzzdict_elem_funp dstryf) {
   /* Create new dict. calloc() zeroes everything */
   buzzdict_t dt = (buzzdict_t)calloc(1, sizeof(struct buzzdict_s));
   /* Fill in the info */
   dt->num_buckets = buckets;
   dt->hashf = hashf;
   dt->keycmpf = keycmpf;
   dt->dstryf = dstryf ? dstryf : buzzdict_entry_destroy;
   dt->key_size = key_size;
   dt->data_size = data_size;
   /* Create buckets. Unused buckets are NULL by default. */
   dt->buckets = (buzzdarray_t*)calloc(dt->num_buckets, sizeof(buzzdarray_t*));
   /* All done */
   return dt;
}

/****************************************/
/****************************************/

void buzzdict_destroy(buzzdict_t* dt) {
   /* Destroy buckets */
   uint32_t i, j;
   for(i = 0; i < (*dt)->num_buckets; ++i) {
      /* Is the bucket used? */
      if((*dt)->buckets[i] != NULL) {
         /* Destroy elements in the bucket */
         for(j = 0; j < buzzdarray_size((*dt)->buckets[i]); ++j) {
            const struct buzzdict_entry_s* e = &buzzdarray_get((*dt)->buckets[i], j, struct buzzdict_entry_s);
            (*dt)->dstryf(e->key, e->data, *dt);
         }
         /* Destroy bucket */
         buzzdarray_destroy(&((*dt)->buckets[i]));
      }
   }
   free((*dt)->buckets);
   /* Destroy the rest */
   free(*dt);
   *dt = NULL;
}

/****************************************/
/****************************************/

void* buzzdict_rawget(buzzdict_t dt,
                      const void* key) {
   /* Hash the key */
   uint32_t h = dt->hashf(key) % dt->num_buckets, i;
   /* Is the bucket empty? */
   if(!dt->buckets[h]) return NULL;
   /* Bucket not empty - is the entry present? */
   for(i = 0; i < buzzdarray_size(dt->buckets[h]); ++i) {
      const struct buzzdict_entry_s* e = &buzzdarray_get(dt->buckets[h], i, struct buzzdict_entry_s);
      if(dt->keycmpf(key, e->key) == 0)
         return e->data;
   }
   /* Entry not found */
   return NULL;
}

/****************************************/
/****************************************/

void buzzdict_set(buzzdict_t dt,
                  const void* key,
                  const void* data) {
   /* Hash the key */
   uint32_t h = dt->hashf(key) % dt->num_buckets;
   /* Is the bucket empty? */
   if(!dt->buckets[h]) {
      /* Create new entry list */
      dt->buckets[h] = buzzdarray_new(1, sizeof(struct buzzdict_entry_s), NULL);
      /* Add entry */
      buzzdict_entry_new(dt, e, key, data);
      buzzdarray_push(dt->buckets[h], &e);
      /* Increase size */
      ++(dt->size);
   }
   else {
      /* Bucket not empty - is the entry present? */
      uint32_t i;
      for(i = 0; i < buzzdarray_size(dt->buckets[h]); ++i) {
         const struct buzzdict_entry_s* e = &buzzdarray_get(dt->buckets[h], i, struct buzzdict_entry_s);
         if(dt->keycmpf(key, e->key) == 0) {
            /* Yes, destroy the entry */
            dt->dstryf(e->key, e->data, dt);
            buzzdarray_remove(dt->buckets[h], i);
            --(dt->size);
            break;
         }
      }
      /* Add new entry */
      buzzdict_entry_new(dt, e, key, data);
      buzzdarray_push(dt->buckets[h], &e);
      /* Increase size */
      ++(dt->size);
   }
}

/****************************************/
/****************************************/

int buzzdict_remove(buzzdict_t dt,
                    const void* key) {
   /* Hash the key */
   uint32_t h = dt->hashf(key) % dt->num_buckets, i;
   /* Is the bucket empty? */
   if(!dt->buckets[h]) return 0;
   /* Bucket not empty - is the entry present? */
   for(i = 0; i < buzzdarray_size(dt->buckets[h]); ++i) {
      const struct buzzdict_entry_s* e = &buzzdarray_get(dt->buckets[h], i, struct buzzdict_entry_s);
      if(dt->keycmpf(key, e->key) == 0) {
         /* Entry found - remove it */
         dt->dstryf(e->key, e->data, dt);
         buzzdarray_remove(dt->buckets[h], i);
         /* Is the entry list empty? If so, free the memory */
         if(buzzdarray_isempty(dt->buckets[h]))
            buzzdarray_destroy(&(dt->buckets[h]));
         /* Increase size */
         --(dt->size);
         /* Done */
         return 1;
      }
   }
   /* Entry not found, nothing to do */
   return 0;
}

/****************************************/
/****************************************/

void buzzdict_foreach(buzzdict_t dt,
                      buzzdict_elem_funp fun,
                      void* params) {
   /* Go through buckets */
   uint32_t i, j;
   for(i = 0; i < dt->num_buckets; ++i) {
      /* Is the bucket used? */
      if(dt->buckets[i] != NULL) {
         /* Go through elements in the bucket */
         for(j = 0; j < buzzdarray_size(dt->buckets[i]); ++j) {
            const struct buzzdict_entry_s* e = &buzzdarray_get(dt->buckets[i], j, struct buzzdict_entry_s);
            fun(e->key, e->data, params);
         }
      }
   }
}

/****************************************/
/****************************************/

uint32_t buzzdict_strkeyhash(const void* key) {
   /* Treat the key as a string */
   const char* s = *(const char**)key;
   /* Initialize the hash to something (e.g. a prime number) */
   uint32_t h = 5381;
   /* Go through the string */
   int c;
   while((c = *s++)) {
      /*
       * This is equivalent to
       *
       * h = h * 33 + c
       *   = (h * 32 + h) + c
       *
       * Why 33 is a good choice, nobody knows
       * NOTE: in the Java VM they use 31 instead
       */
      h = ((h << 5) + h) + c;
   }
   return h;
}

/****************************************/
/****************************************/

int buzzdict_strkeycmp(const void* a, const void* b) {
   return strcmp(*(const char**)a, *(const char**)b);
}

/****************************************/
/****************************************/

#define buzzdict_intkeycmp(TYPE)                                        \
   int buzzdict_ ## TYPE ## keycmp(const void* a, const void* b) {      \
      if(*(TYPE ## _t*)a < *(TYPE ## _t*)b) return -1;                  \
      if(*(TYPE ## _t*)a > *(TYPE ## _t*)b) return  1;                  \
      return 0;                                                         \
   }

buzzdict_intkeycmp(int16);
buzzdict_intkeycmp(uint16);
buzzdict_intkeycmp(int32);
buzzdict_intkeycmp(uint32);

/****************************************/
/****************************************/

#define buzzdict_intkeyhash(TYPE)                           \
   uint32_t buzzdict_ ## TYPE ## keyhash(const void* key) { \
      return *(TYPE ## _t*)key;                             \
   }

buzzdict_intkeyhash(int16);
buzzdict_intkeyhash(uint16);
buzzdict_intkeyhash(int32);
buzzdict_intkeyhash(uint32);

/****************************************/
/****************************************/

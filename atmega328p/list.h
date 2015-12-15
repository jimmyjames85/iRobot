#ifndef _list_h_
#define _list_h_

#include <stdint.h>
typedef struct List
{
     uint16_t capacity;
     uint16_t length;
     void ** arr;
} list_t;

void lfree(list_t * list);
void lfreefree(list_t * list);
int ladd(list_t * list, void * data);
void * lget(list_t * list, int i);
void * lremove(list_t * list, int i);
void linsert(list_t * list, int i, void * newData);
list_t * lalloc();
void * lreplace(list_t * list, int i, void * newData);
void lmergesort(list_t * elements, int startIndex, int endIndex, int (*comparator)(const void*,const void*));
int lbinsearch(list_t * elements, void * key, int startIndex, int endIndex, int (*comparator)(const void*, const void*));
#endif // _list_h_

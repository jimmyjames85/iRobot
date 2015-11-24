//# include <stddef.h>
//#include "usart/usart.h" //todo remove
#include <stdlib.h> /* malloc */
#include <stdint.h> /* size_t */
#include "list.h"
#define _LIST_INIT_CAPACITY 2

int lresize(list_t * list)
{
	if (list->length == list->capacity)
	{
		/* grow list */
		list->capacity *= 2;
		void ** newArr = realloc(list->arr, sizeof(void *) * list->capacity);
		if (newArr == NULL)
			return -1;
		list->arr = newArr;
	}
	if (list->capacity > _LIST_INIT_CAPACITY && 2 * list->length < list->capacity)
	{
		/* shrink list */
		list->capacity /= 2;
		void ** newArr = realloc(list->arr, sizeof(void *) * list->capacity);
		if (newArr == NULL)
			return -1;
		list->arr = newArr;
	}
	return 0;
}

list_t * lalloc()
{
	list_t * ret = malloc(sizeof(list_t));
	ret->capacity = _LIST_INIT_CAPACITY;
	ret->length = 0;
	ret->arr = malloc(sizeof(void*) * ret->capacity);
	return ret;

}

int ladd(list_t * list, void * data)
{
	if (lresize(list) != 0)
		return -1;
	list->arr[list->length++] = data;
	return 0;
}

void * lget(list_t * list, int i)
{
	if (i >= 0 && i < list->length)
		return list->arr[i];
	return NULL;
}

void lfree(list_t * list)
{
	if (list != NULL && list->arr != NULL)
	{
		free(list->arr);
		free(list);
	}
}
void lfreefree(list_t * list)
{
	if (list == NULL)
		return;
	int i = 0;
	for (i = 0; i < list->length; i++)
		free(list->arr[i]);
	lfree(list);
}

void * lremove(list_t * list, int i)
{
	void * ret = NULL;
	if (i >= 0 && i < list->length)
	{
		ret = list->arr[i];
		int j;

		for (j = i; j < list->length - 1; j++)
			list->arr[j] = list->arr[j + 1];
		list->length--;
	}
	lresize(list);
	return ret;
}

void linsert(list_t * list, int i, void * newData)
{
	int j = i;
	ladd(list, newData); //append to the end
	//then move down
	for (j = list->length - 1; j > i; j--)
		list->arr[j] = list->arr[j - 1];

	list->arr[i] = newData;
}

void * lreplace(list_t * list, int i, void * newData)
{
	void * ret = NULL;
	if (i >= 0 && i < list->length)
	{
		ret = list->arr[i];
		list->arr[i] = newData;
	}
	return ret;
}

void lmergesort(list_t * elements, int startIndex, int endIndex, int (*comparator)(const void*, const void*))
{
	if (startIndex >= endIndex)
		return;

	//Base Case
	if (1 == endIndex - startIndex)
	{

		void * s = lget(elements, startIndex);
		void * e = lget(elements, endIndex);
		int comp = comparator(s, e);

		if (comp > 0)
		{
			//swap!!
			lreplace(elements, startIndex, e);
			lreplace(elements, endIndex, s);
		}

		return;
	}

	//sort both halves
	int mid = (startIndex + endIndex) / 2;
	lmergesort(elements, startIndex, mid, comparator);
	lmergesort(elements, mid + 1, endIndex, comparator);

	list_t * newList = lalloc();

	int i = startIndex;
	int j = mid + 1;

	while (i <= mid && j <= endIndex)
	{
		void * i_elem = lget(elements, i);
		void * j_elem = lget(elements, j);
		int comp = comparator(i_elem, j_elem);

		if (comp < 0)
		{
			ladd(newList, i_elem);
			i++;
		}
		else
		{
			ladd(newList, j_elem);
			j++;
		}
	}

	while (i <= mid)
		ladd(newList, lget(elements, i++));

	while (j <= endIndex)
		ladd(newList, lget(elements, j++));

	j = 0;	//index for newList starts at 0
	for (i = startIndex; i <= endIndex; i++)
		lreplace(elements, i, lget(newList, j++));

	lfree(newList);
}

/**
 * returns the index of the key
 * - or -
 * returns the index of where the key should go
 */
int lbinsearch(list_t * elements, void * key, int startIndex, int endIndex, int (*comparator)(const void*, const void*))
{

//	printf0("binsearch(%u, %u)\r\n",startIndex,endIndex);
	if (startIndex > endIndex)
		return -1;

	if (startIndex == endIndex)
	{

		void * elem = lget(elements, startIndex);
		int comp = comparator(key, elem);
		if (comp > 0)
			return startIndex + 1;
		else
			return startIndex; //either equal are this is where the element would go
	}


	int mid = (startIndex + endIndex) / 2;
//	printf0("s=%d     m=%d     e=%d     \r\n",startIndex,mid,endIndex);


	void * elem = lget(elements, mid);
	int comp = comparator(key, elem);


	if (comp == 0)
		return mid;
	else if (comp > 0)
		return lbinsearch(elements, key, mid + 1, endIndex, comparator);
	else
		return lbinsearch(elements, key, startIndex, mid, comparator);

}

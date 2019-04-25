#include "main.h"
#include "queue.h"

int n[100];
/* ??: ????????(??rear)??????,?????(??front)??????????
 * ?????????  ?????????   ???????????
 */

/*=====???????????========
 *
 *  ?? ----------------- ??
 *   <--- a1,a2,a3,...,an <---
 *      -----------------
 *
 *================================*/

//???? queueCapacity-????
status initQueue(queue *PQueue,int queueCapacity)
{
    //?????????

    PQueue->pBase = n;//(ElemType *)malloc(sizeof(ElemType)*queueCapacity);

    PQueue->front = 0; //??????,?????0
    PQueue->rear = 0; //??????,?????0
    PQueue->maxSize = queueCapacity;
		return OK;
}

//????
void destroyQueue(queue *PQueue)
{
    free(PQueue);  //?????????????
    PQueue = NULL;    //??????????NULL,???????
}

//????
void clearQueue(queue *PQueue)
{
    PQueue->front = 0; //?????0
    PQueue->rear = 0; //?????0
}

//????????
status isEmpityQueue(queue *PQueue)
{
    if( PQueue->front == PQueue->rear )  //??==??,????
        return TRUE;

    return FALSE;
}

/*
 *??????,�??�?�??�??????????,??front==rear,
 *?????,?????�??�??�??�?
 *??????,?3????????:
 *(1)??????????�??�??�??�?(???/???????�??�/�??�)
 *(2)??????,??????????????
 *(3)????????,?????????????????????�??�???,
 *?�??�???:(PQueue->rear+1)%MAX_SIZE == PQueue->front?
 *  ???????3??????
 */

status isFullQueue(queue *PQueue)
{
    if( (PQueue->rear+1)%PQueue->maxSize == PQueue->front )  //???
        return TRUE;

    return FALSE;
}

//??????
int getQueueLen(queue *PQueue)
{
    return (PQueue->rear - PQueue->front + PQueue->maxSize)%PQueue->maxSize;
}

//????? [??????:????????] element-?????
status enQueue(queue *PQueue,ElemType element)
{


    PQueue->pBase[PQueue->rear] = element;
    PQueue->rear = (PQueue->rear+1) % PQueue->maxSize; //?rear????????
	
		return OK;

}

//?????,????????? [??????:????????]
status deQueue(queue *PQueue,ElemType *pElement)
{


    *pElement = PQueue->pBase[PQueue->front];       //????
    PQueue->front = (PQueue->front+1) % PQueue->maxSize; //??????
	
		return OK;

}

//????
void queueTraverse(queue *PQueue)
{
    int i = PQueue->front;           //??????
    while(i != PQueue->rear)     //??????rear??,???
    {
        //printf("%d  ", PQueue->pBase[i]);
        i = (i+1) % PQueue->maxSize;              //??????
    }
}

void queueSum(queue *PQueue, u32 *sum)
{
		int S=0;
    int i = PQueue->front;           //??????
    while(i != PQueue->rear)     //??????rear??,???
    {
        S += PQueue->pBase[i];
        i = (i+1) % PQueue->maxSize;              //??????
    }
		*sum = S;
}

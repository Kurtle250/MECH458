/* LinkedQueue.h */
#ifndef _LINKEDQUEUE_H
#define _LINKEDQUEUE_H


/* Type definitions */
typedef struct {
	char itemCode; 	/* stores a number describing the element */
	char stage; 	/* 0: part is built, 1: part not built, 2: part is shipped */
} element;

typedef struct link{
	element		e;
	struct link *next;
} link;


/***************************************************
			F U N C T I O N S
****************************************************/
void	initLink	(link **newLink);
void 	setup		(link **h, link **t);
void 	clearQueue	(link **h, link **t);
void 	enqueue		(link **h, link **t, link **nL);
void dequeue(link **h, link **t, link **deQueuedLink);
element firstValue	(link **h);
char 	isEmpty		(link **h);
int 	size		(link **h, link **t);

#endif
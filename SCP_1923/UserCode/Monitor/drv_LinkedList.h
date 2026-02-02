#ifndef __DRV_LINKEDLIST_H__
#define __DRV_LINKEDLIST_H__

#include "main.h"

#include "stdlib.h"

typedef struct node
{
    struct node *previous;
    struct node *next;
    void *data;
} Node_Structure;

extern Node_Structure *LinkedList_Create(void *firstbody);
extern uint8_t LinkedList_Add_Node(Node_Structure *Node, void *Body_To_Add);
extern void LinkedList_Delete_Node(Node_Structure *Node);
extern Node_Structure *Get_Head_Node(const Node_Structure *Node);
extern Node_Structure *Get_Tail_Node(const Node_Structure *Node);
extern uint32_t LinkedList_Node_Quantity(const Node_Structure *Node);
extern Node_Structure *LinkedList_Find_Node(const Node_Structure *Node, void *NodeData);

#endif /* __DRV_LINKEDLIST_H__ */

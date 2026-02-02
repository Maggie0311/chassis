/**
 * @file drv_LinkedList.c
 * @brief 本文件用于做一些链表的相关操作
 * @author fwlh
 * @version 1.0
 * @date 2021-12-13
 * 
 * @copyright Copyright (c) 2021  哈尔滨工业大学(威海)HERO战队
 */
#include "drv_LinkedList.h"

/**
 * @brief 创建一个双向数据链表指针
 * @author fwlh
 * @param  FirstNode        链表头节点里面的内容
 * @return Node_Structure* 
 */
Node_Structure *LinkedList_Create(void *FirstNode)
{
    Node_Structure *LinkedList_Struct = (Node_Structure *)malloc(sizeof(Node_Structure));
    if (LinkedList_Struct == NULL)
    {
        return NULL;
    }

    LinkedList_Struct->previous = NULL;
    LinkedList_Struct->data = FirstNode;
    LinkedList_Struct->next = NULL;
    return LinkedList_Struct;
}

/**
 * @brief 向链表节点的后面添加一个节点
 * @author fwlh
 * @param  Node             待操作的链表节点
 * @param  Body_To_Add      待加入的节点的内容
 * @return uint8_t          返回值为 0 表示添加失败
 */
uint8_t LinkedList_Add_Node(Node_Structure *Node, void *Body_To_Add)
{
    Node_Structure *LinkedList_Struct = (Node_Structure *)malloc(sizeof(Node_Structure));
    if (LinkedList_Struct == NULL)
    {
        return 0;
    }

    LinkedList_Struct->previous = Node;
    LinkedList_Struct->next = Node->next;
    Node->next = LinkedList_Struct;
    LinkedList_Struct->data = Body_To_Add;

    return 1;
}

/**
 * @brief 删除链表中指定的节点
 * @author fwlh
 * @param  Node       待删除的节点
 */
void LinkedList_Delete_Node(Node_Structure *Node)
{
    Node->previous->next = Node->next;
    free(Node);
    Node = NULL;
}

/**
 * @brief 得到链表头节点的指针
 * @author fwlh
 * @param  Node             需要计算的链表中的某个节点
 * @return Node_Structure*  
 */
Node_Structure *Get_Head_Node(const Node_Structure *Node)
{
    if (Node == NULL)
    {
        return NULL;
    }
    Node_Structure *Temp_Node = (Node_Structure *)Node;
    while (Temp_Node->previous != NULL)
    {
        Temp_Node = Temp_Node->previous;
    }
    return Temp_Node;
}

/**
 * @brief 得到链表尾节点的指针
 * @author fwlh
 * @param  Node             需要计算的链表中的某个节点
 * @return Node_Structure*  
 */
Node_Structure *Get_Tail_Node(const Node_Structure *Node)
{
    if (Node == NULL)
    {
        return NULL;
    }
    Node_Structure *Temp_Node = (Node_Structure *)Node;
    while (Temp_Node->next != NULL)
    {
        Temp_Node = Temp_Node->next;
    }
    return Temp_Node;
}

/**
 * @brief 计算链表中节点的个数
 *        这里只判断链表中是不是存在节点, 并不关心节点的数据段中是不是真的有数据
 * @author fwlh
 * @param  Node       需要计算的链表中的某个节点
 * @return uint32_t         
 */
uint32_t LinkedList_Node_Quantity(const Node_Structure *Node)
{
    if (Node == NULL)
    {
        return 0;
    }

    uint32_t Node_Number = 1;                           // 传进来的节点本身肯定就是一个有效节点
    Node_Structure *Temp_Node = (Node_Structure *)Node; // 遍历过程中正在访问的节点

    while (Temp_Node->next != NULL)
    {
        Temp_Node = Temp_Node->next;
        ++Node_Number;
    }

    // 向后遍历完成后需要将指针恢复
    Temp_Node = (Node_Structure *)Node;

    while (Temp_Node->previous != NULL)
    {
        Temp_Node = Temp_Node->previous;
        ++Node_Number;
    }

    return Node_Number;
}

/**
 * @brief 在链表中查询是否有节点含有指定的数据段
 * @author fwlh
 * @param  Node             待查询的链表的某个节点
 * @param  NodeData         待查询的数据段
 * @return Node_Structure*  第一个找到的含有被查询数据段的节点指针, 如果没找到就返回 NULL
 */
Node_Structure *LinkedList_Find_Node(const Node_Structure *Node, void *NodeData)
{
    Node_Structure *Temp_Node = (Node_Structure *)Node;

    // 先向后查找
    do
    {
        if (Temp_Node->data == NodeData)
        {
            return Temp_Node;
        }
        else
        {
            Temp_Node = Temp_Node->next;
        }
    } while (Temp_Node != NULL);

    // 运行到这里肯定是向后没有找到, 所以向前查找
    Temp_Node = (Node_Structure *)Node;
    do
    {
        if (Temp_Node->data == NodeData)
        {
            return Temp_Node;
        }
        else
        {
            Temp_Node = Temp_Node->previous;
        }
    } while (Temp_Node != NULL);

    return NULL;
}

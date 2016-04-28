#ifndef _LINUX_LIST_H
#define _LINUX_LIST_H

#ifdef __cplusplus
extern "C" {
#endif

#define LIST_POISON1  ((void *) 0x00100100)
#define LIST_POISON2  ((void *) 0x00200200)

struct list_head {
    struct list_head *next, *prev;
};

#define LIST_HEAD_INIT(name) { &(name), &(name) }

#define LIST_HEAD(name) \
    struct list_head name = LIST_HEAD_INIT(name)

#define INIT_LIST_HEAD(ptr) do { \
    (ptr)->next = (ptr); (ptr)->prev = (ptr); \
} while (0)

static inline void 
__list_add(struct list_head *newp,
        struct list_head *prev,
        struct list_head *next) 
{
    next->prev = newp;
    newp->next = next;
    newp->prev = prev;
    prev->next = newp;
} 

static inline void 
list_add(struct list_head *newp,
        struct list_head *head) 
{
    __list_add(newp, head, head->next);
}

static inline void 
list_add_tail(struct list_head *newp,
        struct list_head *head) 
{
    /* head->prev --> newp --> head ... */
    __list_add(newp, head->prev, head);
}

static inline void 
__list_del(struct list_head *prev,
        struct list_head *next) 
{
    next->prev = prev;
    prev->next = next;
}

static inline void 
list_del(struct list_head *entry) 
{
    __list_del(entry->prev, entry->next);
    entry->next = (struct list_head *) LIST_POISON1;
    entry->prev = (struct list_head *) LIST_POISON2;
}

static inline void 
list_del_init(struct list_head *entry) 
{
    /* remove entry from list ... */
    __list_del(entry->prev, entry->next);
    /* Reinit the entry ... */
    INIT_LIST_HEAD(entry);
}

static inline void 
list_move(struct list_head *list,
        struct list_head *head) 
{
    __list_del(list->prev, list->next);
    list_add(list, head);
}

static inline void 
list_move_tail(struct list_head *list,
        struct list_head *head) 
{
    __list_del(list->prev, list->next);
    list_add_tail(list, head);
}

static inline int 
list_empty(const struct list_head *head) 
{
    return head->next == head;
}

#ifndef offsetof
#define offsetof(TYPE, MEMBER) ((size_t) &((TYPE *)0)->MEMBER)
#endif

#define container_of(ptr, type, member) ({			\
        const typeof( ((type *)0)->member ) *__mptr = (ptr);	\
        (type *)( (char *)__mptr - offsetof(type,member) );})

#define list_entry(ptr, type, member) \
    container_of(ptr, type, member)

/**
 * list_for_each	-	iterate over a list
 * @pos:	the &struct list_head to use as a loop counter.
 * @head:	the head for your list.
 */
#define list_for_each(pos, head) \
    for (pos = (head)->next; pos != (head); \
            pos = pos->next)

/**
 * __list_for_each	-	iterate over a list
 * @pos:	the &struct list_head to use as a loop counter.
 * @head:	the head for your list.
 *
 * This variant differs from list_for_each() in that it's the
 * simplest possible list iteration code, no prefetching is done.
 * Use this for code that knows the list to be very short (empty
 * or 1 entry) most of the time.
 */
#define __list_for_each(pos, head) \
    for (pos = (head)->next; pos != (head); pos = pos->next)

/**
 * list_for_each_prev	-	iterate over a list backwards
 * @pos:	the &struct list_head to use as a loop counter.
 * @head:	the head for your list.
 */
#define list_for_each_prev(pos, head) \
    for (pos = (head)->prev; pos != (head); \
            pos = pos->prev)

/**
 * list_for_each_entry	-	iterate over list of given type
 * @pos:	the type * to use as a loop counter.
 * @head:	the head for your list.
 * @member:	the name of the list_struct within the struct.
 */
#define list_for_each_entry(pos, head, member)				\
    for (pos = list_entry((head)->next, typeof(*pos), member);	\
            &pos->member != (head); 	\
            pos = list_entry(pos->member.next, typeof(*pos), member))

/**
 * list_for_each_entry_safe - iterate over list of given type safe against removal of list entry
 * @pos:	the type * to use as a loop counter.
 * @n:		another type * to use as temporary storage
 * @head:	the head for your list.
 * @member:	the name of the list_struct within the struct.
 */
#define list_for_each_entry_safe(pos, n, head, member)			\
    for (pos = list_entry((head)->next, typeof(*pos), member),	\
            n = list_entry(pos->member.next, typeof(*pos), member);	\
            &pos->member != (head); 					\
            pos = n, n = list_entry(n->member.next, typeof(*n), member))

#ifdef __cplusplus
}
#endif

#endif

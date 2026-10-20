#include <aarch64/mmu.h>
#include <common/rc.h>
#include <common/spinlock.h>
#include <driver/memlayout.h>
#include <kernel/mem.h>

RefCount kalloc_page_cnt;

/* 链接脚本中的 BSS 结束符号，是内核虚拟地址 */
extern char end[];

/* 空闲物理页：把 next 指针直接存放在页的首部 */
typedef struct PageNode {
    struct PageNode *next;
} PageNode;

static PageNode *free_page_list;
static SpinLock page_lock;

/* 大小类：每类可用的字节数，全部为 8 的倍数 */
#define NCLASS 16
static const u16 class_usable[NCLASS] = {
    8, 16, 24, 32, 48, 64, 96, 128, 192, 256, 384, 512, 768, 1024, 1536, 2048
};

/* 每类一条空闲块链，链表节点就是块的数据区前 8 字节 */
static void *free_block[NCLASS];
static SpinLock block_lock;

void kinit() {
    init_rc(&kalloc_page_cnt);
    init_spinlock(&page_lock);
    init_spinlock(&block_lock);

    /*
     * 可分配物理内存为 [end, PHYSTOP)。
     * end 是链接地址（内核虚拟地址），先 K2P 转成物理地址，再向上按页对齐，
     * 避免把内核镜像尾部所在的那一页也交出去。
     */
    u64 pa = round_up(K2P((u64)end), PAGE_SIZE);
    for (; pa + PAGE_SIZE <= PHYSTOP; pa += PAGE_SIZE) {
        PageNode *p = (PageNode *)P2K(pa);
        p->next = free_page_list;
        free_page_list = p;
    }
}

void* kalloc_page() {
    acquire_spinlock(&page_lock);
    PageNode *p = free_page_list;
    if (p)
        free_page_list = p->next;
    release_spinlock(&page_lock);

    if (p)
        increment_rc(&kalloc_page_cnt);

    return p;
}

void kfree_page(void* ptr) {
    acquire_spinlock(&page_lock);
    ((PageNode *)ptr)->next = free_page_list;
    free_page_list = (PageNode *)ptr;
    release_spinlock(&page_lock);

    decrement_rc(&kalloc_page_cnt);
}

/* 请求大小 -> 类号，找不到返回 -1 */
static int size2class(u64 size) {
    for (int i = 0; i < NCLASS; i++)
        if (size <= class_usable[i])
            return i;
    return -1;
}

static void push_block(int c, void *data) {
    *(void **)data = free_block[c];
    free_block[c] = data;
}

static void* pop_block(int c) {
    void *data = free_block[c];
    if (data)
        free_block[c] = *(void **)data;
    return data;
}

/*
 * 把一整页切成若干块并全部挂到第 c 类空闲链上。
 * 块布局：[8 字节头 = 类号][数据区]，块大小为 8 + class_usable[c]，
 * 是 8 的倍数，因此数据区天然满足 8 字节对齐。
 */
static void carve(int c, void *page) {
    u64 bsz = 8 + class_usable[c];
    int n = PAGE_SIZE / bsz;

    for (int i = 0; i < n; i++) {
        char *blk = (char *)page + i * bsz;
        *(u64 *)blk = (u64)c;
        push_block(c, blk + 8);
    }
}

void* kalloc(unsigned long long size) {
    int c = size2class(size);
    if (c < 0)
        return NULL;

    acquire_spinlock(&block_lock);

    void *p = pop_block(c);
    if (!p) {
        /* 该类的空闲链为空：向页分配器要一整页，切开后重新取 */
        void *page = kalloc_page();
        if (!page) {
            release_spinlock(&block_lock);
            return NULL;
        }
        carve(c, page);
        p = pop_block(c);
    }

    release_spinlock(&block_lock);
    return p;
}

void kfree(void* ptr) {
    if (!ptr)
        return;

    /* 块头在数据区之前 8 字节，记录所属类号 */
    int c = (int)*(u64 *)((char *)ptr - 8);

    acquire_spinlock(&block_lock);
    push_block(c, ptr);
    release_spinlock(&block_lock);
}

/*
 * QEMU IO Bridge
 *
 * Copyright (C) 2016 Intel Corporation
 *
 * Creates an IO bridge with a Qemu VM instance where messages can be passed
 * via messages queues and shared memory.
 *
 * The parent is usually the QEMU instance that runs the operating system (like
 * Linux) on the application processor whilst the child is typically a smaller
 * processor running an embedded firmware. The parent and child do not need to
 * be the same architecture but are expected to communicate over a local bus.
 */

#include <mqueue.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <errno.h>
#include <reef/debug.h>
#include <reef/timer.h>
#include <reef/interrupt.h>
#include <reef/ipc.h>
#include <reef/mailbox.h>
#include <reef/reef.h>
#include <reef/stream.h>
#include <reef/dai.h>
#include <reef/dma.h>
#include <reef/alloc.h>
#include <reef/wait.h>
#include <reef/trace.h>
#include <reef/ssp.h>
#include <platform/interrupt.h>
#include <platform/mailbox.h>
#include <platform/shim.h>
#include <platform/dma.h>
#include <platform/platform.h>
#include <reef/audio/component.h>
#include <reef/audio/pipeline.h>
#include <uapi/intel-ipc.h>
#include <reef/intel-ipc.h>
#include <reef/host-ipc.h>

/* we can either be parent or child */
#define ROLE_NONE    0
#define ROLE_PARENT    1
#define ROLE_CHILD    2

static int role = ROLE_NONE;

#define QEMU_IO_MAX_MSGS    8
#define QEMU_IO_MAX_MSG_SIZE    128
#define QEMU_IO_MAX_SHM_REGIONS    32

#define NAME_SIZE       64

struct io_shm {
    int fd;
    void *addr;
    char name[NAME_SIZE];
    size_t size;
};

struct io_mq {
    char mq_name[NAME_SIZE];
    char thread_name[NAME_SIZE];
    struct mq_attr mqattr;
    mqd_t mqdes;
};

struct io_bridge {
    struct io_mq parent;
    struct io_mq child;
    pthread_t io_thread;
    int (*cb)(void *data, struct host_ipc_msg *msg);
    struct io_shm shm[QEMU_IO_MAX_SHM_REGIONS];
    void *data;
};

static struct io_bridge _iob;
static int _id = 0;

/* parent reader Q */
static void * parent_reader_thread(void *data)
{
    struct io_bridge *io = data;
    char buf[QEMU_IO_MAX_MSG_SIZE];
    int i;

    mq_getattr(io->parent.mqdes, &io->parent.mqattr);

    fprintf(stdout, "bridge-io: %d messages are currently on parent queue.\n",
            (int)io->parent.mqattr.mq_curmsgs);

    /* flush old messages here */
    for (i = 0; i < io->parent.mqattr.mq_curmsgs; i++) {
        mq_receive(io->parent.mqdes, buf, QEMU_IO_MAX_MSG_SIZE, NULL);
        struct host_ipc_msg *hdr = (struct host_ipc_msg*)buf;

        fprintf(stdout, "bridge-io: flushed %d type %d size %d msg %d\n",
                hdr->id, hdr->type, hdr->size, hdr->msg);
    }

    while (mq_receive(io->parent.mqdes, buf, QEMU_IO_MAX_MSG_SIZE, NULL) != -1) {
        struct host_ipc_msg *hdr = (struct host_ipc_msg*)buf;

        fprintf(stdout, "bridge-io: msg recv %d type %d size %d msg %d\n",
                hdr->id, hdr->type, hdr->size, hdr->msg);

        if (io->cb)
            io->cb(io->data, hdr);
    }

    return 0;
}

/* child reader Q */
static void * child_reader_thread(void *data)
{
    struct io_bridge *io = data;
    char buf[QEMU_IO_MAX_MSG_SIZE];
    int i;

    mq_getattr(io->child.mqdes, &io->child.mqattr);

    fprintf(stdout, "bridge-io: %d messages are currently on child queue.\n",
            (int)io->child.mqattr.mq_curmsgs);

    /* flush old messages here */
    for (i = 0; i < io->child.mqattr.mq_curmsgs; i++) {
        mq_receive(io->child.mqdes, buf, QEMU_IO_MAX_MSG_SIZE, NULL);
        struct host_ipc_msg *hdr = (struct host_ipc_msg*)buf;

        fprintf(stdout, "bridge-io: flushed %d type %d size %d msg %d\n",
                hdr->id, hdr->type, hdr->size, hdr->msg);
    }

    while (mq_receive(io->child.mqdes, buf, QEMU_IO_MAX_MSG_SIZE, NULL) != -1) {
        struct host_ipc_msg *hdr = (struct host_ipc_msg*)buf;

        fprintf(stdout, "bridge-io: msg recv %d type %d size %d msg %d\n",
                hdr->id, hdr->type, hdr->size, hdr->msg);

        if (io->cb)
            io->cb(io->data, hdr);
    }

    return 0;
}

static int mq_init(const char *name, struct io_bridge *io)
{
    int ret = 0;

    io->parent.mqattr.mq_maxmsg = QEMU_IO_MAX_MSGS;
    io->parent.mqattr.mq_msgsize = QEMU_IO_MAX_MSG_SIZE;
    io->parent.mqattr.mq_flags = 0;
    io->parent.mqattr.mq_curmsgs = 0;

    io->child.mqattr.mq_maxmsg = QEMU_IO_MAX_MSGS;
    io->child.mqattr.mq_msgsize = QEMU_IO_MAX_MSG_SIZE;
    io->child.mqattr.mq_flags = 0;
    io->child.mqattr.mq_curmsgs = 0;

    if (role == ROLE_PARENT) {

        sprintf(io->parent.thread_name, "io-bridge-%s", name);
        ret = pthread_create(&io->io_thread, NULL, parent_reader_thread, io);
        if (ret < 0) {
        	fprintf(stderr, "failed to create thread %s errno %d\n",
        			io->parent.thread_name, -errno);
        	return -errno;
        }

        /* parent Rx Q */
        sprintf(io->parent.mq_name, "/qemu-io-parent-%s", name);
        io->parent.mqdes = mq_open(io->parent.mq_name, O_RDONLY | O_CREAT,
            0664, &io->parent.mqattr);
        if (io->parent.mqdes < 0) {
            fprintf(stderr, "failed to open parent Rx queue %d\n", -errno);
            ret = -errno;
        }

        /* parent Tx Q */
        sprintf(io->child.mq_name, "/qemu-io-child-%s", name);
        io->child.mqdes = mq_open(io->child.mq_name, O_WRONLY | O_CREAT,
            0664, &io->child.mqattr);
        if (io->child.mqdes < 0) {
            fprintf(stderr, "failed to open parent Tx queue %d\n", -errno);
            ret = -errno;
        }

    } else {

        sprintf(io->child.thread_name, "io-bridge-%s", name);
        ret = pthread_create(&io->io_thread, NULL, child_reader_thread, io);
        if (ret < 0) {
            fprintf(stderr, "failed to create thread %s errno %d\n",
                    io->child.thread_name, -errno);
            return -errno;
        }

        /* child Rx Q */
        sprintf(io->child.mq_name, "/qemu-io-child-%s", name);
        io->child.mqdes = mq_open(io->child.mq_name, O_RDONLY | O_CREAT,
            0664, &io->child.mqattr);
        if (io->child.mqdes < 0) {
            fprintf(stderr, "failed to open child Rx queue %d\n", -errno);
            ret = -errno;
        }

        /* child Tx Q */
        sprintf(io->parent.mq_name, "/qemu-io-parent-%s", name);
        io->parent.mqdes = mq_open(io->parent.mq_name, O_WRONLY | O_CREAT,
            0664, &io->parent.mqattr);
        if (io->parent.mqdes < 0) {
            fprintf(stderr, "failed to open child Tx queue %d\n", -errno);
            ret = -errno;
        }

    }

    if (ret == 0) {
        fprintf(stdout, "bridge-io-mq: added %s\n", io->parent.mq_name);
        fprintf(stdout, "bridge-io-mq: added %s\n", io->child.mq_name);
    }
    return ret;
}

int host_ipc_register_parent(const char *name,
    int (*cb)(void *, struct host_ipc_msg *msg), void *data)
{
    if (role != ROLE_NONE)
        return -EINVAL;

    role = ROLE_PARENT;
    _iob.cb = cb;
    _iob.data = data;

    mq_init(name, &_iob);

    return 0;
}

int host_ipc_register_child(const char *name,
    int (*cb)(void *, struct host_ipc_msg *msg), void *data)
{
    int ret = 0;

    if (role != ROLE_NONE)
        return -EINVAL;

    role = ROLE_CHILD;
    _iob.cb = cb;
    _iob.data = data;

    mq_init(name, &_iob);

    return ret;
}

int host_ipc_register_shm(const char *rname, int region, size_t size, void **addr)
{
    char *name;
    int fd, ret;
    void *a;

    /* check that region is not already in use */
    if (_iob.shm[region].fd)
        return -EBUSY;

    if (rname == NULL) {
        fprintf(stderr, "error: no bridge name for region %d\n", region);
        return -EINVAL;
    }

    name = _iob.shm[region].name;
    sprintf(name, "qemu-bridge-%s", rname);

    fd = shm_open(name, O_RDWR | O_CREAT, 0664);
    if (fd < 0) {
        fprintf(stderr, "bridge-io: cant open SHM %d\n", errno);
        return -errno;
    }

    ret = ftruncate(fd, size);
    if (ret < 0) {
        fprintf(stderr, "bridge-io: cant truncate %d\n", errno);
        shm_unlink(name);
        return -errno;
    }

    a = mmap(*addr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (a == NULL) {
        fprintf(stderr, "bridge-io: cant open mmap %d\n", errno);
        shm_unlink(name);
        return -errno;
    }

    fprintf(stdout, "bridge-io: %s fd %d region %d at %p allocated %zu bytes\n",
            name, fd, region, a, size);
    _iob.shm[region].fd = fd;
    _iob.shm[region].addr = a;
    _iob.shm[region].size = size;
    *addr = a;

    return ret;
}

#define PAGE_SIZE 4096

int host_ipc_sync(int region, unsigned int offset, size_t length)
{
    if (region < 0 || region > QEMU_IO_MAX_SHM_REGIONS)
        return -EINVAL;

    /* check that region is in use */
    if (_iob.shm[region].fd == 0)
        return -EINVAL;

    /* align offset to pagesize */
    offset -= (offset % PAGE_SIZE);

    return msync(_iob.shm[region].addr + offset, length, MS_SYNC | MS_INVALIDATE);
}

int host_ipc_send_msg(struct host_ipc_msg *msg)
{
    int ret;

    msg->id = _id++;

    if (role == ROLE_PARENT)
        ret = mq_send(_iob.child.mqdes, (const char*)msg, msg->size, 0);
    else
        ret = mq_send(_iob.parent.mqdes, (const char*)msg, msg->size, 0);

    fprintf(stdout, "bridge-io: msg send: %d type %d msg %d size %d ret %d\n",
            msg->id, msg->type, msg->msg, msg->size, ret);
    if (ret < 0)
        fprintf(stderr, "bridge-io: msg send failed %d\n", -errno);

    return ret;
}

int host_ipc_send_msg_reply(struct host_ipc_msg *msg)
{
    int ret;

    if (role == ROLE_PARENT)
        ret = mq_send(_iob.child.mqdes, (const char*)msg, msg->size, 0);
    else
        ret = mq_send(_iob.parent.mqdes, (const char*)msg, msg->size, 0);

    fprintf(stdout, "bridge-io: repmsg send: %d type %d msg %d size %d ret %d\n",
            msg->id, msg->type, msg->msg, msg->size, ret);
    if (ret < 0)
        fprintf(stderr, "bridge-io: rmsg send failed %d\n", -errno);

    return ret;
}

void host_ipc_free(void)
{
    int i;

    for (i = 0; i < QEMU_IO_MAX_SHM_REGIONS; i++) {
        if (_iob.shm[i].fd) {
            munmap(_iob.shm[i].addr, _iob.shm[i].size);
            shm_unlink(_iob.shm[i].name);
            close(_iob.shm[i].fd);
        }
    }
    mq_unlink(_iob.parent.mq_name);
    mq_unlink(_iob.child.mq_name);
}

void host_ipc_free_shm(int region)
{
    int err;

    if ((region < QEMU_IO_MAX_SHM_REGIONS) && _iob.shm[region].fd) {
        err = munmap(_iob.shm[region].addr, _iob.shm[region].size);
        if (err < 0)
            fprintf(stderr, "bridge-io: munmap failed %d\n", errno);

        /* client or host can unlink this, so it gets done twice */
        shm_unlink(_iob.shm[region].name);
        close(_iob.shm[region].fd);
        _iob.shm[region].fd = 0;
    }
}

extern struct ipc *_ipc;

#if 0
static void do_notify(void)
{
    uint32_t flags;
    struct ipc_msg *msg;

    tracev_ipc("Not");

    spin_lock_irq(&_ipc->lock, flags);
    msg = _ipc->dsp_msg;
    if (msg == NULL)
        goto out;

    /* copy the data returned from DSP */
    if (msg->rx_size && msg->rx_size < MSG_MAX_SIZE)
        mailbox_inbox_read(msg->rx_data, 0, msg->rx_size);

    /* any callback ? */
    if (msg->cb)
        msg->cb(msg->cb_data, msg->rx_data);

    list_item_append(&msg->list, &_ipc->empty_list);

out:
    spin_unlock_irq(&_ipc->lock, flags);
    /* clear DONE bit - tell Host we have completed */
 //   shim_write(SHIM_IPCDH, shim_read(SHIM_IPCDH) & ~SHIM_IPCDH_DONE);

    /* unmask Done interrupt */
 //   shim_write(SHIM_IMRD, shim_read(SHIM_IMRD) & ~SHIM_IMRD_DONE);
}
#endif

/* test code to check working IRQ */
static void irq_handler(void *arg)
{
#if 0
    uint32_t isr;

    tracev_ipc("IRQ");

    /* Interrupt arrived, check src */
    isr = shim_read(SHIM_ISRD);

    if (isr & SHIM_ISRD_DONE) {

        /* Mask Done interrupt before return */
        shim_write(SHIM_IMRD, shim_read(SHIM_IMRD) | SHIM_IMRD_DONE);
        interrupt_clear(PLATFORM_IPC_INTERUPT);
        do_notify();
    }

    if (isr & SHIM_ISRD_BUSY) {

        /* Mask Busy interrupt before return */
        shim_write(SHIM_IMRD, shim_read(SHIM_IMRD) | SHIM_IMRD_BUSY);
        interrupt_clear(PLATFORM_IPC_INTERUPT);

        /* TODO: place message in Q and process later */
        /* It's not Q ATM, may overwrite */
        if (_ipc->host_pending)
            trace_ipc_error("Pen");
        _ipc->host_msg = shim_read(SHIM_IPCXL);
        _ipc->host_pending = 1;
    }
#endif
}

void ipc_platform_do_cmd(struct ipc *ipc)
{
    struct intel_ipc_data *iipc = ipc_get_drvdata(ipc);
    uint32_t status;

    trace_ipc("Cmd");
    //trace_value(_ipc->host_msg);

    status = ipc_cmd();
    if (status != IPC_INTEL_GLB_REPLY_SUCCESS)
        trace_ipc_error("eIP");

    ipc->host_pending = 0;

    /* clear BUSY bit and set DONE bit - accept new messages */


    // TODO: signal audio work to enter D3 in normal context
    /* are we about to enter D3 ? */
    if (iipc->pm_prepare_D3) {
        while (1)
            wait_for_interrupt(0);
    }

    /* unmask busy interrupt */

}

void ipc_platform_send_msg(struct ipc *ipc)
{
    struct ipc_msg *msg;
    uint32_t flags;

    spin_lock_irq(&ipc->lock, flags);

    /* any messages to send ? */
    if (list_is_empty(&ipc->msg_list)) {
        ipc->dsp_pending = 0;
        goto out;
    }

    /* can't send nofication when one is in progress */
 //   if (shim_read(SHIM_IPCDH) & (SHIM_IPCDH_BUSY | SHIM_IPCDH_DONE))
  //      goto out;

    /* now send the message */
    msg = list_first_item(&ipc->msg_list, struct ipc_msg, list);
    mailbox_outbox_write(0, msg->tx_data, msg->tx_size);
    list_item_del(&msg->list);
    ipc->dsp_msg = msg;
    tracev_ipc("Msg");

    /* now interrupt host to tell it we have message sent */
 //   shim_write(SHIM_IPCDL, msg->header);
 //   shim_write(SHIM_IPCDH, SHIM_IPCDH_BUSY);

out:
    spin_unlock_irq(&ipc->lock, flags);
}


extern void *_mailbox;
extern struct ipc *_ipc;

int platform_ipc_init(struct ipc *ipc)
{
    struct intel_ipc_data *iipc;
    int i, ret;

    _ipc = ipc;

    /* init ipc data */
    iipc = rzalloc(RZONE_DEV, RMOD_SYS, sizeof(struct intel_ipc_data));
    ipc_set_drvdata(_ipc, iipc);
    _ipc->dsp_msg = NULL;
    list_init(&ipc->empty_list);
    list_init(&ipc->msg_list);
    spinlock_init(&ipc->lock);
    for (i = 0; i < MSG_QUEUE_SIZE; i++)
        list_item_prepend(&ipc->message[i].list, &ipc->empty_list);

    /* allocate page table buffer */
    iipc->page_table = rballoc(RZONE_DEV, RMOD_SYS,
        IPC_INTEL_PAGE_TABLE_SIZE);
    if (iipc->page_table)
        bzero(iipc->page_table, IPC_INTEL_PAGE_TABLE_SIZE);

    /* dma */
    iipc->dmac0 = dma_get(DMA_ID_DMAC0);

    /* PM */
    iipc->pm_prepare_D3 = 0;

    /* SHM Mailbox */
    ret = host_ipc_register_shm("mailbox", 0, IPC_MAX_MAILBOX_BYTES, &_mailbox);
    if (ret < 0)
        return ret;

    /* configure interrupt */
    interrupt_register(PLATFORM_IPC_INTERUPT, irq_handler, NULL);
    interrupt_enable(PLATFORM_IPC_INTERUPT);

    return 0;
}

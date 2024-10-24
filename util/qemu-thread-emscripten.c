#include "qemu/osdep.h"
#include "qemu/thread.h"
#include "qemu/notify.h"
#include "qemu-thread-common.h"

static int cur_id = 0;

void qemu_thread_switch(QemuThread *thread)
{
    cur_id = thread->id;
}

void qemu_thread_get_self(QemuThread *thread)
{
    thread->id = cur_id;
}

bool qemu_thread_is_self(QemuThread *thread)
{
   return thread->id == cur_id;
}

void qemu_mutex_init(QemuMutex *mutex)
{
    mutex->is_rec = 0;
    mutex->lock_counter = 0;
    mutex->owner = -1;
    qemu_mutex_post_init(mutex);
}

void qemu_mutex_destroy(QemuMutex *mutex)
{
    mutex->initialized = false;
}

void qemu_mutex_lock_impl(QemuMutex *mutex, const char *file, const int line)
{
    qemu_mutex_pre_lock(mutex, file, line);
    mutex->lock_counter++;
    mutex->owner = cur_id;
    qemu_mutex_post_lock(mutex, file, line);
}

int qemu_mutex_trylock_impl(QemuMutex *mutex, const char *file, const int line)
{
    if (!mutex->lock_counter || (mutex->is_rec && mutex->owner == cur_id)) {
        mutex->lock_counter++;
        mutex->owner = cur_id;
        qemu_mutex_post_lock(mutex, file, line);
        return 0;
    }
    return -EBUSY;
}

void qemu_mutex_unlock_impl(QemuMutex *mutex, const char *file, const int line)
{
    qemu_mutex_pre_unlock(mutex, file, line);
    mutex->lock_counter--;
}

void qemu_rec_mutex_init(QemuRecMutex *mutex)
{
    mutex->is_rec = 1;
    mutex->lock_counter = 0;
    mutex->initialized = true;
}

void qemu_cond_init(QemuCond *cond)
{
    cond->initialized = true;
}

void qemu_cond_destroy(QemuCond *cond)
{
    cond->initialized = false;
}

void qemu_event_init(QemuEvent *ev, bool init)
{
    ev->set = init;
    ev->initialized = true;
}

void qemu_event_destroy(QemuEvent *ev)
{
    ev->initialized = false;
}

void qemu_event_set(QemuEvent *ev)
{
    ev->set = 1;
}

void qemu_event_reset(QemuEvent *ev)
{
    ev->set = 0;
}

static int thread_counter = 1;
void qemu_thread_create(QemuThread *thread, const char *name, void *(*start_routine)(void*), void *arg, int mode)
{
    thread->id = thread_counter++;
}

void qemu_sem_init(QemuSemaphore *sem, int init)
{
    sem->counter = init;
    sem->initialized = true;
}

void qemu_sem_destroy(QemuSemaphore *sem)
{
    sem->initialized = false;
}

void qemu_sem_post(QemuSemaphore *sem)
{
    sem->counter++;
}

void qemu_sem_wait(QemuSemaphore *sem)
{
    sem->counter--;
}

int qemu_sem_timedwait(QemuSemaphore *sem, int ms)
{
    if (sem->counter > 0) {
        sem->counter--;
        return 0;
    }
    return -EBUSY;
}

int qemu_thread_set_affinity(QemuThread *thread, unsigned long *host_cpus, unsigned long nbits)
{
    return -ENOSYS;
}

int qemu_thread_get_affinity(QemuThread *thread, unsigned long **host_cpus, unsigned long *nbits)
{
    return -ENOSYS;
}

bool qemu_cond_timedwait_impl(QemuCond *cond, QemuMutex *mutex, int ms, const char *file, const int line) {
    return true; // hope it's fine
}

void qemu_event_wait(QemuEvent *ev){}
void qemu_thread_atexit_add(Notifier *notifier){}
//void qemu_thread_atexit_remove(Notifier *notifier){}
//void qemu_thread_exit(void *retval){}
void *qemu_thread_join(QemuThread *thread){return NULL;}
void qemu_cond_wait_impl(QemuCond *cond, QemuMutex *mutex, const char *file, const int line){}
void qemu_cond_signal(QemuCond *cond){}
void qemu_cond_broadcast(QemuCond *cond){}
void qemu_thread_naming(bool enable){}
/*
QemuRecMutexLockFunc qemu_rec_mutex_lock_impl = qemu_mutex_lock_impl;
QemuRecMutexLockFunc qemu_rec_mutex_unlock_impl = qemu_mutex_unlock_impl;
QemuRecMutexLockFunc qemu_rec_mutex_trylock_impl = qemu_mutex_trylock_impl;
*/


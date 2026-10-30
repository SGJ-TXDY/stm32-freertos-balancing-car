#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/stat.h>
#include "main.h"
#include "usart.h"

extern char _heap_start, _heap_end;

/* Only the UI task uses newlib formatting; RTOS objects use heap_4 separately. */
void *_sbrk(ptrdiff_t increment)
{
    static uintptr_t current;
    uintptr_t previous;
    if (!current) current = (uintptr_t)&_heap_start;
    if (increment < 0 || (uintptr_t)increment > (uintptr_t)&_heap_end - current) {
        errno = ENOMEM;
        return (void *)-1;
    }
    previous = current;
    current += (uintptr_t)increment;
    return (void *)previous;
}

int _write(int file, char *data, int length)
{
    if ((file != 1 && file != 2) || length < 0) {
        errno = EBADF;
        return -1;
    }
    if (length > 65535 || HAL_UART_Transmit(&huart1, (uint8_t *)data,
                                           (uint16_t)length, 100) != HAL_OK) {
        errno = EIO;
        return -1;
    }
    return length;
}

int _read(int file, char *data, int length)
{
    (void)file; (void)data; (void)length;
    errno = EAGAIN;
    return -1;
}

int _close(int file)
{
    (void)file;
    errno = EBADF;
    return -1;
}

int _fstat(int file, struct stat *status)
{
    if (file < 0 || file > 2) { errno = EBADF; return -1; }
    status->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int file)
{
    return file >= 0 && file <= 2;
}

int _lseek(int file, int offset, int whence)
{
    (void)file; (void)offset; (void)whence;
    errno = ESPIPE;
    return -1;
}

int _getpid(void) { return 1; }

int _kill(int process, int signal)
{
    (void)process; (void)signal;
    errno = EINVAL;
    return -1;
}

void _exit(int status)
{
    (void)status;
    Error_Handler();
    for (;;) {}
}

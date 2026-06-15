#include "usart.h"

#include <errno.h>
#include <stdint.h>
#include <sys/stat.h>
#include <unistd.h>

extern UART_HandleTypeDef huart1;

int _close(int fd)
{
        (void)fd;
        errno = EBADF;
        return -1;
}

int _fstat(int fd, struct stat *st)
{
        (void)fd;
        if(st == 0){
                errno = EINVAL;
                return -1;
        }
        st->st_mode = S_IFCHR;
        return 0;
}

int _isatty(int fd)
{
        return (fd == STDIN_FILENO || fd == STDOUT_FILENO || fd == STDERR_FILENO) ? 1 : 0;
}

int _lseek(int fd, int ptr, int dir)
{
        (void)fd;
        (void)ptr;
        (void)dir;
        errno = ESPIPE;
        return -1;
}

int _read(int fd, char *ptr, int len)
{
        (void)ptr;
        (void)len;
        if(fd == STDIN_FILENO){
                return 0;
        }
        errno = EBADF;
        return -1;
}

int _write(int fd, char *ptr, int len)
{
        if(ptr == 0 || len < 0){
                errno = EINVAL;
                return -1;
        }
        if(fd == STDOUT_FILENO || fd == STDERR_FILENO){
                if(HAL_UART_Transmit(&huart1, (uint8_t *)ptr, (uint16_t)len, 100U) == HAL_OK){
                        return len;
                }
                errno = EIO;
                return -1;
        }
        errno = EBADF;
        return -1;
}

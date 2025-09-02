#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <dirent.h>

#define BAUDRATE B115200

// === Abrir puerto serie ===
int open_serial(const char *portname)
{
    int fd = open(portname, O_RDWR | O_NOCTTY);
    if (fd < 0)
    {
        perror("Error abriendo puerto serie");
        return -1;
    }

    struct termios tty;
    tcgetattr(fd, &tty);
    cfsetospeed(&tty, BAUDRATE);
    cfsetispeed(&tty, BAUDRATE);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag = 0;
    tty.c_oflag = 0;
    tty.c_lflag = 0;
    tty.c_cflag |= (CLOCAL | CREAD);

    tcsetattr(fd, TCSANOW, &tty);
    return fd;
}

// === Buscar puerto STM32 automáticamente ===
int find_stm32_port(char *portname, size_t len)
{
    struct dirent *entry;
    DIR *dp = opendir("/dev/");
    if (!dp)
        return -1;

    while ((entry = readdir(dp)))
    {
        if (strncmp(entry->d_name, "ttyACM", 6) == 0 || strncmp(entry->d_name, "ttyUSB", 6) == 0)
        {
            snprintf(portname, len, "/dev/%s", entry->d_name);
            closedir(dp);
            return 0; // encontró puerto
        }
    }

    closedir(dp);
    return -1; // no encontró puerto
}

int main()
{
    char port[64];
    if (find_stm32_port(port, sizeof(port)) < 0)
    {
        printf("No se encontró puerto STM32.\n");
        return 1;
    }

    printf("Puerto STM32 detectado: %s\n", port);

    int serial_fd = open_serial(port);
    if (serial_fd < 0)
    {
        printf("Error abriendo puerto %s\n", port);
        return 1;
    }

    const char *msg = "Hello STM32\n";
    write(serial_fd, msg, strlen(msg));
    printf("Mensaje enviado: %s", msg);

    close(serial_fd);
    return 0;
}

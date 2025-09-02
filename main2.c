// pc_sync.c
// Uso: ./pc_sync [serial_port]
// Si no se pasa serial_port, intenta /dev/ttyACM0..9 y /dev/ttyUSB0..9

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/select.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>

#define BAUDRATE B115200
#define TOKEN_VALIDITY 30
#define TOKEN_DIGITS 6

// Intenta abrir el puerto y configurar termios. Devuelve fd o -1.
int open_serial_port(const char *portpath)
{
    int fd = open(portpath, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        return -1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0)
    {
        close(fd);
        return -1;
    }

    cfsetospeed(&tty, BAUDRATE);
    cfsetispeed(&tty, BAUDRATE);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8 bits
    tty.c_iflag &= ~IGNBRK;                     // disable break processing
    tty.c_lflag = 0;                            // no signaling chars, no echo
    tty.c_oflag = 0;                            // no remapping, no delays
    tty.c_cc[VMIN] = 0;                         // non-blocking read
    tty.c_cc[VTIME] = 5;                        // 0.5s read timeout

    tty.c_cflag |= (CLOCAL | CREAD);   // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD); // no parity
    tty.c_cflag &= ~CSTOPB;            // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;           // no flow control

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        close(fd);
        return -1;
    }

    // poner en blocking para write/read normales (usamos select)
    // fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) & ~O_NONBLOCK);
    return fd;
}

// Busca primer dispositivo disponible /dev/ttyACM[0..9] o /dev/ttyUSB[0..9]
int autodetect_serial(char *outpath, size_t outcap)
{
    const char *patterns[] = {"/dev/ttyACM%d", "/dev/ttyUSB%d"};
    for (int p = 0; p < (int)(sizeof(patterns) / sizeof(patterns[0])); ++p)
    {
        for (int i = 0; i < 10; ++i)
        {
            char tmp[64];
            snprintf(tmp, sizeof(tmp), patterns[p], i);
            if (access(tmp, F_OK) == 0)
            {
                strncpy(outpath, tmp, outcap - 1);
                outpath[outcap - 1] = '\0';
                return 0;
            }
        }
    }
    return -1;
}

// Generador de token simple (mismo algoritmo en STM32)
unsigned int generate_token(time_t current_time)
{
    unsigned int step = (unsigned int)(current_time / TOKEN_VALIDITY);
    return step % 1000000U; // 6 dígitos
}

// Lee y muestra líneas desde el fd serial durante up_to_ms (no bloqueante)
void drain_serial_input(int fd, int up_to_ms)
{
    fd_set rfds;
    struct timeval tv;
    char buf[256];
    int total_ms = 0;
    const int gran = 100; // ms
    while (total_ms < up_to_ms)
    {
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        tv.tv_sec = gran / 1000;
        tv.tv_usec = (gran % 1000) * 1000;
        int rv = select(fd + 1, &rfds, NULL, NULL, &tv);
        if (rv > 0 && FD_ISSET(fd, &rfds))
        {
            ssize_t r = read(fd, buf, sizeof(buf) - 1);
            if (r > 0)
            {
                buf[r] = '\0';
                printf("[STM32] %s", buf); // STM32 probablemente incluya '\n'
            }
        }
        total_ms += gran;
    }
}

// Espera una línea del stdin con timeout en segundos. Retorna 1 si leido en buf, 0 si timeout.
int stdin_readline_timeout(char *buf, size_t bufcap, int timeout_sec)
{
    fd_set rfds;
    struct timeval tv;
    FD_ZERO(&rfds);
    FD_SET(STDIN_FILENO, &rfds);
    tv.tv_sec = timeout_sec;
    tv.tv_usec = 0;
    int rv = select(STDIN_FILENO + 1, &rfds, NULL, NULL, &tv);
    if (rv > 0 && FD_ISSET(STDIN_FILENO, &rfds))
    {
        if (fgets(buf, (int)bufcap, stdin) != NULL)
            return 1;
        return 0;
    }
    return 0; // timeout
}

int main(int argc, char **argv)
{
    char portpath[64] = {0};
    if (argc >= 2)
    {
        strncpy(portpath, argv[1], sizeof(portpath) - 1);
    }
    else
    {
        if (autodetect_serial(portpath, sizeof(portpath)) != 0)
        {
            fprintf(stderr, "No se encontró /dev/ttyACM* ni /dev/ttyUSB*. Conecta tu adaptador USB-TTL o VCP y vuelve a intentar.\n");
            fprintf(stderr, "Si usas Blackpill sin USB-TTL, debes conectar un adaptador USB-TTL a TX/RX.\n");
            fprintf(stderr, "Prueba: dmesg | tail  después de conectar para ver nombre del dispositivo.\n");
            return 1;
        }
        printf("Puerto detectado automáticamente: %s\n", portpath);
    }

    int fd = open_serial_port(portpath);
    if (fd < 0)
    {
        fprintf(stderr, "Error abriendo puerto '%s': %s\n", portpath, strerror(errno));
        if (errno == EACCES)
        {
            fprintf(stderr, "Permiso denegado. Ejecuta: sudo usermod -a -G dialout $USER  y vuelve a iniciar sesión, o usa sudo temporalmente.\n");
        }
        return 2;
    }

    printf("Presiona ENTER para sincronizar con STM32 por serial (%s)...\n", portpath);
    getchar();

    // Enviar timestamp actual (UNIX seconds) terminando en '\n'
    time_t now = time(NULL);
    char outbuf[128];
    snprintf(outbuf, sizeof(outbuf), "%ld\n", (long)now);
    ssize_t w = write(fd, outbuf, strlen(outbuf));
    if (w < 0)
    {
        fprintf(stderr, "Error escribiendo en puerto serial: %s\n", strerror(errno));
        close(fd);
        return 3;
    }
    printf("Hora enviada al STM32: %s", ctime(&now));

    // Esperar/leer posibles respuestas del STM32 durante 1500 ms
    drain_serial_input(fd, 1500);

    // Generar token (debe coincidir con STM32 si este usó la misma hora y algoritmo)
    unsigned int token = generate_token(now);
    printf("TOKEN (PC/STM32): %06u\n", token);

    // Calcular tiempo restante para el slot actual
    int remaining = TOKEN_VALIDITY - (int)(now % TOKEN_VALIDITY);
    if (remaining == TOKEN_VALIDITY)
        remaining = 0;
    printf("Tienes %d segundos para ingresar el token (timeout):\n", remaining);

    // Leer linea de stdin con timeout 'remaining' segundos
    char inbuf[128];
    int got = stdin_readline_timeout(inbuf, sizeof(inbuf), remaining);
    if (!got)
    {
        printf("Tiempo expirado. Token inválido.\n");
        close(fd);
        return 0;
    }
    // quitar newline
    size_t L = strlen(inbuf);
    if (L && (inbuf[L - 1] == '\n' || inbuf[L - 1] == '\r'))
        inbuf[L - 1] = '\0';

    // parsear
    unsigned int entered = (unsigned int)strtoul(inbuf, NULL, 10);
    if (entered == token)
    {
        printf("Token válido. Acceso permitido.\n");
    }
    else
    {
        printf("Token inválido. Esperado %06u, recibido %s\n", token, inbuf);
    }

    // Mostrar cualquier dato adicional que envíe STM32 (500 ms) y cerrar
    drain_serial_input(fd, 500);

    close(fd);
    return 0;
}

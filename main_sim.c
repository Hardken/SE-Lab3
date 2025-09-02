#include <SDL2/SDL.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/select.h>

#define TOKEN_VALIDITY 30
#define TOKEN_DIGITS 6
#define BAUDRATE B115200

// === Apertura del puerto serie ===
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

// === Generador simple de token ===
unsigned int generate_token(time_t current_time)
{
    unsigned int step = (unsigned int)(current_time / TOKEN_VALIDITY);
    return step % 1000000U; // 6 dígitos
}

// === Leer token ingresado con timeout ===
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
    }
    return 0; // timeout
}

int main()
{
    const int width = 200, height = 200;
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window *win = SDL_CreateWindow("Sync Signal", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, width, height, 0);
    SDL_Renderer *ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED);

    // Abrir puerto serial
    int serial_fd = open_serial("/dev/ttyUSB0"); // <-- ajusta según corresponda (/dev/ttyACM0, etc.)

    int quit = 0;
    SDL_Event e;

    while (!quit)
    {
        printf("Presiona ENTER para iniciar sincronización...\n");
        getchar();

        int interval = 1000; // 1 segundo
        int cycles = 6;      // número de parpadeos
        int state = 0;

        for (int i = 0; i < cycles; i++)
        {
            while (SDL_PollEvent(&e))
            {
                if (e.type == SDL_QUIT)
                    quit = 1;
            }

            // Alternar color del cuadro
            if (state)
                SDL_SetRenderDrawColor(ren, 255, 255, 255, 255);
            else
                SDL_SetRenderDrawColor(ren, 0, 0, 0, 255);
            SDL_RenderClear(ren);
            SDL_RenderPresent(ren);

            state = !state;

            // Obtener hora actual
            time_t now = time(NULL);
            char buffer[64];
            snprintf(buffer, sizeof(buffer), "%ld\n", (long)now);

            // Mostrar en consola
            printf("Hora actual: %s", ctime(&now));

            // Enviar al STM32
            if (serial_fd > 0)
            {
                write(serial_fd, buffer, strlen(buffer));
            }

            SDL_Delay(interval);
        }

        // --- Después de sincronización ---
        time_t now = time(NULL);
        unsigned int token = generate_token(now);
        printf("\nTOKEN (PC/STM32): %06u\n", token);

        // Tiempo restante del slot
        int remaining = TOKEN_VALIDITY - (int)(now % TOKEN_VALIDITY);
        if (remaining == TOKEN_VALIDITY)
            remaining = 0;
        printf("Tienes %d segundos para ingresar el token que muestra el STM32:\n", remaining);

        // Leer token del usuario
        char inbuf[64];
        int got = stdin_readline_timeout(inbuf, sizeof(inbuf), remaining);
        if (!got)
        {
            printf("Tiempo expirado. Token inválido.\n\n");
            continue;
        }
        // limpiar newline
        size_t L = strlen(inbuf);
        if (L && (inbuf[L - 1] == '\n' || inbuf[L - 1] == '\r'))
            inbuf[L - 1] = '\0';

        unsigned int entered = (unsigned int)strtoul(inbuf, NULL, 10);
        if (entered == token)
        {
            printf("Token válido. Acceso permitido.\n\n");
        }
        else
        {
            printf("Token inválido. Esperado %06u, recibido %s\n\n", token, inbuf);
        }
    }

    if (serial_fd > 0)
        close(serial_fd);
    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}

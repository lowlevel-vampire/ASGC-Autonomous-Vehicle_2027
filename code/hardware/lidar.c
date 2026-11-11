#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>

typedef enum {
    IDLE,
    CAPTURE,
} Lidar_State;

Lidar_State curr_state;
Lidar_State next_state;

int byte_count;
int data[12][2];
uint8_t data_byte;
uint8_t packets[47];

void report() {
    uint16_t speed =        packets[2]  | (packets[3]  << 8);
    uint16_t start_angle =  packets[4]  | (packets[5]  << 8);
    uint16_t end_angle =    packets[42] | (packets[43] << 8);
    uint16_t timestamp =    packets[44] | (packets[45] << 8);
    uint8_t  crc =          packets[46];

    for (int i = 0; i < 12; ++i) {
        int offset = 6 + (i * 3);
        data[i][0] = packets[offset] | (packets[offset + 1] << 8);
        data[i][1] = packets[offset + 2];
    }

    int degrees = (end_angle - start_angle + 36000) % 36000;
    float degree_offset = degrees / 11.0f;


    for (int i = 0; i < 12; ++i) {
        float angle = (start_angle / 100.0f) + ((degree_offset * i) / 100.0f);
        angle = (float)((int)(angle * 100.0f + 36000.0f) % 36000) / 100.0f;

        printf("%f,%d\n", angle, data[i][0]);
    }
}

void readable_output() {

}

// Based on elecrow.com/download/product/SLD06360F/LD19_Development%20Manual_V2.3.pdf
void Lidar_FSM(int uart)
{
    next_state = curr_state;

    if ((read(uart, &data_byte, 1)) == 1) {
        //printf("Got Byte: %02X\n", (unsigned char)data_byte);

        switch (curr_state) {
            case IDLE:
                if (data_byte == 0x54) {
                    byte_count = 1;
                    packets[byte_count - 1] = data_byte;
                    next_state = CAPTURE;
                }
                break;

            case CAPTURE:
                ++byte_count;
                packets[byte_count - 1] = data_byte;

                if (byte_count == 47) {
                    next_state = IDLE;
                    report();
                }

                break;
        }

        if (curr_state != next_state) {
            curr_state = next_state;
        }
    }
}

int main()
{
    // See man7.org/linux/man-pages/man3/termios.3.html
    printf("Opening Serial Port...\n");
    int uart = open("/dev/ttyAMA0", O_RDWR);

    if (uart < 0) {
        perror("Failed to open UART\n");
        return 1;
    }

    printf("Setting up default values...\n");
    struct termios options;
    tcgetattr(uart, &options);

    cfmakeraw(&options);
    cfsetispeed(&options, B230400);     // only need to set input speed, no writes
    options.c_cflag &= ~PARENB;         // No Parity
    options.c_cflag &= ~CSTOPB;         // 1 Stop Bit
    options.c_cflag &= ~CSIZE;          // Clear current size mask
    options.c_cflag |= CS8;             // 8 bits data
    options.c_cflag |= (CREAD | CLOCAL);
    options.c_cflag &= ~CRTSCTS;

    tcsetattr(uart, TCSANOW, &options);

    // Reset Values
    byte_count = 0;
    curr_state = IDLE;
    next_state = IDLE;
    memset(data, 0, sizeof(data));

    printf("Starting FSM...\n");

    while (1) { Lidar_FSM(uart); }

    close(uart);
    return 0;
}

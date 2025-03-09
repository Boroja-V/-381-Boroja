/*#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

int main() {
    const char *device = "/dev/serial0";  // Serial device on RPi 4
    int serial_port = open(device, O_RDWR);

    if (serial_port < 0) {
        perror("Error opening serial port");
        return 1;
    }

    // Set up serial port settings
    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(serial_port, &tty) != 0) {
        perror("Error getting serial port attributes");
        return 1;
    }

    // Set baud rate
    cfsetospeed(&tty, B9600);  // Output baud rate
    cfsetispeed(&tty, B9600);  // Input baud rate

    // Configure the serial port (8N1: 8 data bits, no parity, 1 stop bit)
    tty.c_cflag &= ~PARENB;    // No parity
    tty.c_cflag &= ~CSTOPB;    // 1 stop bit
    tty.c_cflag &= ~CSIZE;     // Mask the character size bits
    tty.c_cflag |= CS8;        // 8 data bits
    tty.c_cflag &= ~CRTSCTS;   // Disable hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Enable receiver, local mode

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control
    tty.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input mode
    tty.c_oflag &= ~OPOST;    // Raw output mode
    tty.c_cc[VMIN] = 1;       // Minimum number of characters to read
    tty.c_cc[VTIME] = 10;     // Timeout in deciseconds

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        perror("Error setting serial port attributes");
        return 1;
    }
    printf("Hello world\n");
    printf("Do ovde radi\n");


    // Sending data over UART
    const char *message = "Hello, UART!\n";
    write(serial_port, message, strlen(message));
    printf("Radi i do ovde\n");
    // Reading data from UART (optional, to read what is received)
    char buffer[256];
    int num_bytes = read(serial_port, buffer, sizeof(buffer));
    if (num_bytes < 0) {
        perror("Error reading from serial port");
    } else {
        buffer[num_bytes] = '\0';  // Null-terminate string
        printf("Received: %s\n", buffer);
    }
    printf("Evo i ja radim\n");
    // Close the serial port
    close(serial_port);

    return 0;
    printf("\n Kraj koda");
}
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

int main() {
    const char *device = "/dev/serial0"; // Serial device on RPi 4
    int serial_port = open(device, O_RDWR);

    if (serial_port < 0) {
        perror("Error opening serial port");
        return 1;
    }

    // Set up serial port settings
    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(serial_port, &tty) != 0) {
        perror("Error getting serial port attributes");
        return 1;
    }

    // Set baud rate
    cfsetospeed(&tty, B9600); // Output baud rate
    cfsetispeed(&tty, B9600); // Input baud rate

    // Configure the serial port (8N1: 8 data bits, no parity, 1 stop bit)
    tty.c_cflag &= ~PARENB;        // No parity
    tty.c_cflag &= ~CSTOPB;        // 1 stop bit
    tty.c_cflag &= ~CSIZE;         // Mask the character size bits
    tty.c_cflag |= CS8;            // 8 data bits
    tty.c_cflag &= ~CRTSCTS;       // Disable hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Enable receiver, local mode

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);         // Disable software flow control
    tty.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input mode
    tty.c_oflag &= ~OPOST;                          // Raw output mode
    tty.c_cc[VMIN] = 1;                             // Minimum number of characters to read
    tty.c_cc[VTIME] = 10;                           // Timeout in deciseconds

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        perror("Error setting serial port attributes");
        return 1;
    }

    // Definišemo x i y koje šaljemo
    float x = 381.3123;
    float y = 69.431;
    x = x * 10;
    y = y * 10;
    int x_INT;
    int y_INT ;
    char x_high1;
    char x_low1 ;
    int x_received_int;
    float x_received ;

    char y_high1;
    char y_low1 ;
    int y_received_int ;
    float y_received ;

    // Podelimo x i y na high i low bajtove
    char x_high = (x_INT >> 8) & 0xFF;
    char x_low = x_INT & 0xFF;
    char y_high = (y_INT >> 8) & 0xFF;
    char y_low = y_INT & 0xFF;

    char message[7];  // Poruka: s + 4 bajta + e + \n
    char buffer[256]; // Buffer za prijem podataka

    // Beskonačna petlja za slanje i primanje podataka
    while (1) {
        // Formatiramo x i y kao poruku
        message[0] = 's';
        message[1] = x_high;
        message[2] = x_low;
        message[3] = y_high;
        message[4] = y_low;
        message[5] = 'e';
        message[6] = '\n';

        // Šaljemo poruku preko UART-a
        write(serial_port, message, 7);

        // Čekamo odgovor sa serijskog porta
        int num_bytes = read(serial_port, buffer, sizeof(buffer) - 1);
        if (num_bytes < 0) {
            perror("Error reading from serial port");
        } else if (num_bytes > 0) {
            buffer[num_bytes] = '\0'; // Dodajemo null terminator

            // Proverimo da li je poruka u ispravnom formatu
            if (buffer[0] == 's' && buffer[5] == 'e') {
                // Rekonstruišemo x i y
                 x_high1 = buffer[1];
                 x_low1 = buffer[2];
                 x_received_int = (x_high1 << 8) | x_low1;
                 x_received = x_received_int * 0.1f;

                 y_high1 = buffer[3];
                 y_low1 = buffer[4];
                 y_received_int = (y_high1 << 8) | y_low1;
                 y_received = y_received_int * 0.1f;

                // Ispisujemo primljene vrednosti
                printf("Примљено: x=%.2f, y=%.2f\n", x_received, y_received);
            } 
        }

        
    }

    close(serial_port); // Ovaj deo se nikada neće izvršiti zbog petlje
    return 0;
}
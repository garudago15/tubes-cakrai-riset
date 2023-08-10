#include "mbed.h"

// INITIALIZE SERIAL USING PRINTF 
static BufferedSerial serial_port(USBTX, USBRX, 115200);
FileHandle *mbed::mbed_override_console(int fd){
    return &serial_port;
}

int main()
{
    /* Debugging Purpose */
    const int MAX_MSG = 64;
    static char message[MAX_MSG];
    static float kpKiKd[3];
    int constPID_pos = 0;
    static unsigned int message_pos = 0;
    char inByte;
    char kode[5];

    while (1)
    {
        while (serial_port.readable())
        {
            serial_port.read(&inByte, 1);

            if (inByte == '\n') {
                continue; // Skip newline characters
            }

            if ((inByte != '\r') && message_pos < MAX_MSG - 1)
            {
                message[message_pos] = inByte;
                message_pos++;
            }
            else
            {
                message[message_pos] = '\0';
                message_pos = 0;
                char *token = strtok(message, " ");

                while (token != NULL)
                {
                    printf("%s %d\n", token, constPID_pos);

                    if (constPID_pos > 0){
                        kpKiKd[constPID_pos] = atof(token);
                    } else {
                        strcpy(kode, token);
                    }
                    
                    // printf("%s\n", token);
                    token = strtok(NULL, " ");
                    constPID_pos++;
                    if (constPID_pos == 4)
                    {
                        constPID_pos = 0;
                    }
                }

                // Change left pid left motor tuning

                printf("Kode = |%s| kp = %f || ki = %f || kd = %f\n", kode, kpKiKd[1], kpKiKd[2], kpKiKd[3]);

                if (strcmp(kode, "HJ") == 0) {
                    printf("Kode MERAH\n");
                } else if (strcmp(kode, "AD") == 0) {
                    printf("Kode HIJAU\n");
                }

                // printf("kp = %f || ki = %f || kd = %f\n", pidLeftMotor.getPParam(), pidLeftMotor.getIParam(), pidLeftMotor.getDParam());
                // printf("kp = %f || ki = %f || kd = %f\n", kpKiKd[0], kpKiKd[1], kpKiKd[2]);
            }
        }
    }

    return 0;
}

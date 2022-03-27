#include <stdbool.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <signal.h>
#include <sys/select.h>
#include <arpa/inet.h>
#include <string.h>
#include <sys/stat.h>
#include <fcntl.h>

#define LISTEN_GZSERVER_PORT 9006
#define GZSERVER_PORT 9007
#define LISTEN_COPTER_PORT 9002
#define COPTER_PORT 9003

#define MAX_MOTORS 255

const char *cmd_prefix = "> ";

// A servo packet. for gazebo
struct ServoPacket {
    /// \brief Motor speed data.
    float motorSpeed[MAX_MOTORS];
};

// Flight Dynamics Model packet that is sent back to the ArduPilot
struct fdmPacket {
    // \brief packet timestamp
    double timestamp;

    /// \brief IMU angular velocity
    double imuAngularVelocityRPY[3];

    /// \brief IMU linear acceleration
    double imuLinearAccelerationXYZ[3];

    /// \brief IMU quaternion orientation
    double imuOrientationQuat[4];

    /// \brief Model velocity in NED frame
    double velocityXYZ[3];

    /// \brief Model position in NED frame
    double positionXYZ[3];
};

enum {
    spoof_none,
    spoof_set,
    spoof_random,
    spoof_offset,
};

struct SpoofMeta {
    char *name;
    int element, spoof_type;
    double value;
};

struct SpoofMeta spoof_meta_array[] = {
    // timestamp
    {
        .name = "timestamp",
        .element = 0,
        .spoof_type = spoof_none,
    },

    // IMU angular velocity
    {
        .name = "imuAngVelX",
        .element = 1,
        .spoof_type = spoof_none,
    },
    {
        .name = "imuAngVelY",
        .element = 2,
        .spoof_type = spoof_none,
    },
    {
        .name = "imuAngVelZ",
        .element = 3,
        .spoof_type = spoof_none,
    },

    // IMU linear acceleration
    {
        .name = "imuAccelX",
        .element = 4,
        .spoof_type = spoof_none,
    },
    {
        .name = "imuAccelY",
        .element = 5,
        .spoof_type = spoof_none,
    },
    {
        .name = "imuAccelZ",
        .element = 6,
        .spoof_type = spoof_none,
    },

    // IMU quaternion orientation
    {
        .name = "imuQuat1",
        .element = 7,
        .spoof_type = spoof_none,
    },
    {
        .name = "imuQuat2",
        .element = 8,
        .spoof_type = spoof_none,
    },
    {
        .name = "imuQuat3",
        .element = 9,
        .spoof_type = spoof_none,
    },
    {
        .name = "imuQuat4",
        .element = 10,
        .spoof_type = spoof_none,
    },

    // Model velocity in NED frame
    {
        .name = "velX",
        .element = 11,
        .spoof_type = spoof_none,
    },
    {
        .name = "velY",
        .element = 12,
        .spoof_type = spoof_none,
    },
    {
        .name = "velZ",
        .element = 13,
        .spoof_type = spoof_none,
    },

    // Model position in NED frame
    {
        .name = "posX",
        .element = 14,
        .spoof_type = spoof_none,
    },
    {
        .name = "posY",
        .element = 15,
        .spoof_type = spoof_none,
    },
    {
        .name = "posZ",
        .element = 16,
        .spoof_type = spoof_none,
    },
};

static void show_fdm (const struct fdmPacket *fdm, int fd) {
    dprintf(fd, "timestamp: %f\n", fdm->timestamp);
    dprintf(fd, "imuAngularVelocityRPY: %f, %f, %f\n",
            fdm->imuAngularVelocityRPY[0],
            fdm->imuAngularVelocityRPY[1],
            fdm->imuAngularVelocityRPY[2]);
    dprintf(fd, "imuLinearAccelerationXYZ: %f, %f, %f\n",
            fdm->imuLinearAccelerationXYZ[0],
            fdm->imuLinearAccelerationXYZ[1],
            fdm->imuLinearAccelerationXYZ[2]);
    dprintf(fd, "imuOrientationQuat: %f, %f, %f, %f\n",
            fdm->imuOrientationQuat[0],
            fdm->imuOrientationQuat[1],
            fdm->imuOrientationQuat[2],
            fdm->imuOrientationQuat[3]);
    dprintf(fd, "velocityXYZ: %f, %f, %f\n",
            fdm->velocityXYZ[0],
            fdm->velocityXYZ[1],
            fdm->velocityXYZ[2]);
    dprintf(fd, "positionXYZ: %f, %f, %f\n",
            fdm->positionXYZ[0],
            fdm->positionXYZ[1],
            fdm->positionXYZ[2]);
    dprintf(fd, "\n");
}

int listen_gzserver_fd = 0, gzserver_fd = 0;
int listen_copter_fd = 0, copter_fd = 0;
int redirect_fd = 0;
void sigint_handler (int signum) {
    printf("bye\n");
    if (listen_gzserver_fd > 0)
        close(listen_gzserver_fd);
    if (gzserver_fd > 0)
        close(gzserver_fd);
    if (listen_copter_fd > 0)
        close(listen_copter_fd);
    if (copter_fd > 0)
        close(copter_fd);
    if (redirect_fd > 0)
        close(redirect_fd);
    exit(0);
}

void normal_termination () {
    if (listen_gzserver_fd > 0)
        close(listen_gzserver_fd);
    if (gzserver_fd > 0)
        close(gzserver_fd);
    if (listen_copter_fd > 0)
        close(listen_copter_fd);
    if (copter_fd > 0)
        close(copter_fd);
    if (redirect_fd > 0)
        close(redirect_fd);
}

static char **init_args (char *msg) {
    bool new_word = true;
    int count = 0;
    char *tmp = strdup(msg);
    for (char *ptr = tmp; *ptr; ptr++) {
        if (new_word && *ptr != ' ') {
            new_word = false;
            count++;
        }
        else if (*ptr == ' ')
            new_word = true;
    }

    char **ret = malloc(sizeof(char *) * (count + 2));
    ret[0] = tmp;
    ret[count + 1] = NULL;
    count = 1;
    new_word = true;
    for (char *ptr = tmp; *ptr; ptr++) {
        if (*ptr == ' ' || *ptr == '\n') {
            *ptr = '\0';
            new_word = true;
        }
        else if (new_word && *ptr != ' ') {
            new_word = false;
            ret[count++] = ptr;
        }
    }
    return ret;
}

static inline void release_args (char **args) {
    free(args[0]);
    free(args);
}

static int find_var (char *name) {
    int num = sizeof(spoof_meta_array) / sizeof(struct SpoofMeta);

    for (int i = 0; i < num; i++)
        if (!strcmp(name, spoof_meta_array[i].name))
            return i;

    return -1;
}

static int set_cmd (char **args) {
    if (!args[2] || !args[3])
        return -1;
    int offset = find_var(args[2]);
    if (offset < 0) {
        fprintf(stderr, "[ERROR] no variable %s\n", args[2]);
        return -1;
    }

    char *endptr;
    double value = strtod(args[3], &endptr);
    if (*endptr)
        return -1;
    spoof_meta_array[offset].spoof_type = spoof_set;
    spoof_meta_array[offset].value = value;
    return 0;
}

void parse_cmd (char *cmd) {
    char **args = init_args(cmd);
    // empty command
    if (!args[1]) {
        // do nothing
    }
    else if (!strcmp(args[1], "set")) {
        if (set_cmd(args)) {
            fprintf(stderr, "[ERROR] the format of command is wrong\n");
            fprintf(stderr, "    set <variable> <value>\n");
        }
    }
    else {
        fprintf(stderr, "[ERROR] unknown command\n");
    }
    release_args(args);
    return;
}

int main (int argc, char *argv[]) {
    for (int i = 0; i < argc; i++) {
        if (!strcmp(argv[i], "-tty") && i < argc - 1) {
            if ((redirect_fd = open(argv[i + 1], O_WRONLY)) < 0) {
                perror("failed to open file\n");
                return -1;
            }
        }
    }

    srand(time(NULL));
    // register SIGINT handler
    if (signal(SIGINT, sigint_handler) == SIG_ERR) {
        perror("failed to register signal handler");
        return -1;
    }

    // register atexit
    if (atexit(normal_termination)) {
        perror("failed to register atexit");
        return -1;
    }

    // create udp socket for listening gzserver messages
    // AF_INET: IPv4 Internet protocols
    // SOCK_DGRAM: Supports datagrams (UDP)
    if ((listen_gzserver_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("failed to create socket");
        return -1;
    }
    struct sockaddr_in listen_gzserver_addr = {0};
    listen_gzserver_addr.sin_family = AF_INET;
    listen_gzserver_addr.sin_port = htons(LISTEN_GZSERVER_PORT); // big-endian
    listen_gzserver_addr.sin_addr.s_addr = INADDR_ANY;
    if (bind(listen_gzserver_fd, (struct sockaddr *)&listen_gzserver_addr,
             sizeof(listen_gzserver_addr)) < 0) {
        perror("failed to bind address");
        return -1;
    }
    printf("create udp port at %d\n", LISTEN_GZSERVER_PORT);

    // create udp socket for listening copter messages
    if ((listen_copter_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("failed to create socket");
        return -1;
    }
    struct sockaddr_in listen_copter_addr = {0};
    listen_copter_addr.sin_family = AF_INET;
    listen_copter_addr.sin_port = htons(LISTEN_COPTER_PORT); // big-endian
    listen_copter_addr.sin_addr.s_addr = INADDR_ANY;
    if (bind(listen_copter_fd, (struct sockaddr *)&listen_copter_addr,
             sizeof(listen_copter_addr)) < 0) {
        perror("failed to bind address");
        return -1;
    }
    printf("create udp port at %d\n", LISTEN_COPTER_PORT);

    // create fd to send message to gzserver
    if ((gzserver_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("failed to create socket");
        return -1;
    }
    struct sockaddr_in gzserver_addr = {0};
    gzserver_addr.sin_family = AF_INET;
    gzserver_addr.sin_port = htons(GZSERVER_PORT); // big-endian
    gzserver_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    printf("create a fd for sending messge to prot %d\n", GZSERVER_PORT);

    // create fd to send message to copter
    if ((copter_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("failed to create socket");
        return -1;
    }
    struct sockaddr_in copter_addr = {0};
    copter_addr.sin_family = AF_INET;
    copter_addr.sin_port = htons(COPTER_PORT); // big-endian
    copter_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    printf("create a fd for sending messge to prot %d\n", COPTER_PORT);

    struct fdmPacket fdm;
    struct ServoPacket sp = {0};

    // initialize select data
    int nfds = 0, ret;
    fd_set rfds, active_rfds;
    FD_ZERO(&active_rfds);
    FD_SET(0, &active_rfds); nfds = 1;
    FD_SET(listen_gzserver_fd, &active_rfds);
    nfds = nfds > listen_gzserver_fd + 1 ? nfds : listen_gzserver_fd + 1;
    FD_SET(listen_copter_fd, &active_rfds);
    nfds = nfds > listen_copter_fd + 1 ? nfds : listen_copter_fd + 1;
    struct timeval timeout = {.tv_sec = 1, .tv_usec = 0};
    rfds = active_rfds;
    bool is_spoof = false;
    printf("%s", cmd_prefix); fflush(stdout);
    while ((ret = select(nfds, &rfds, NULL, NULL, &timeout)) >= 0) {
        // stdin
        if (FD_ISSET(0, &rfds)) {
            char buffer[256] = {0};
            int size = read(0, buffer, 255);
            if (!size)
                break;

            parse_cmd(buffer);
            printf("%s", cmd_prefix); fflush(stdout);
        }

        // receive message from gzserver
        if (FD_ISSET(listen_gzserver_fd, &rfds)) {
            // FIX: return value
            int rstatus = recv(listen_gzserver_fd, &fdm, sizeof(fdm), 0);
            if (rstatus < 0) {
                perror("failed to receive message from gzserver");
                return -1;
            }
            // TODO: spoof data
            int num = sizeof(spoof_meta_array) / sizeof(struct SpoofMeta);
            double *ptr = (double *)&fdm;
            for (int i = 0; i < num; i++) {
                int type = spoof_meta_array[i].spoof_type;
                if (type == spoof_set)
                    ptr[i] = spoof_meta_array[i].value;
            }

            if (redirect_fd > 0)
                show_fdm(&fdm, redirect_fd);

            // Send message to copter.
            sendto(copter_fd, &fdm, sizeof(fdm), 0,
                   (struct sockaddr *)&copter_addr, sizeof(copter_addr));
        }

        // receive message from copter
        if (FD_ISSET(listen_copter_fd, &rfds)) {
            int size = recv(listen_copter_fd, &sp, sizeof(fdm), 0);
            if (size < 0) {
                perror("failed to receive message from copter");
                return -1;
            }
            // Send message to gzserver directly. We don't need to modify packet here.
            sendto(gzserver_fd, &sp, size, 0, (struct sockaddr *)&gzserver_addr,
                   sizeof(gzserver_addr));
        }

        // reset rfds
        rfds = active_rfds;
    }

    if (ret < 0) {
        perror("select fails");
        return -1;
    }
    return 0;
}

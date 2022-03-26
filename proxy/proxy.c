#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <signal.h>
#include <sys/select.h>
#include <arpa/inet.h>

#define LISTEN_GZSERRVER_PORT 9003
#define GZSERRVER_PORT 9002

#define MAX_MOTORS 255
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

static void show_fdm (const struct fdmPacket *fdm) {
    printf("timestamp: %f\n", fdm->timestamp);
    printf("imuAngularVelocityRPY: %f, %f, %f\n",
           fdm->imuAngularVelocityRPY[0],
           fdm->imuAngularVelocityRPY[1],
           fdm->imuAngularVelocityRPY[2]);
    printf("imuLinearAccelerationXYZ: %f, %f, %f\n",
           fdm->imuLinearAccelerationXYZ[0],
           fdm->imuLinearAccelerationXYZ[1],
           fdm->imuLinearAccelerationXYZ[2]);
    printf("imuOrientationQuat: %f, %f, %f, %f\n",
           fdm->imuOrientationQuat[0],
           fdm->imuOrientationQuat[1],
           fdm->imuOrientationQuat[2],
           fdm->imuOrientationQuat[3]);
    printf("velocityXYZ: %f, %f, %f\n",
           fdm->velocityXYZ[0],
           fdm->velocityXYZ[1],
           fdm->velocityXYZ[2]);
    printf("positionXYZ: %f, %f, %f\n",
           fdm->positionXYZ[0],
           fdm->positionXYZ[1],
           fdm->positionXYZ[2]);
    printf("\n");
}

int listen_gzserver_fd = 0, gzserver_fd = 0;
void sigint_handler (int signum) {
    printf("bye\n");
    if (listen_gzserver_fd > 0)
        close(listen_gzserver_fd);
    if (gzserver_fd > 0)
        close(gzserver_fd);
    exit(0);
}

void normal_termination () {
    if (listen_gzserver_fd > 0)
        close(listen_gzserver_fd);
    if (gzserver_fd > 0)
        close(gzserver_fd);
}

int main () {
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
    listen_gzserver_addr.sin_port = htons(LISTEN_GZSERRVER_PORT); // big-endian
    listen_gzserver_addr.sin_addr.s_addr = INADDR_ANY;
    if (bind(listen_gzserver_fd, (struct sockaddr *)&listen_gzserver_addr,
             sizeof(listen_gzserver_addr)) < 0) {
        perror("failed to bind address");
        return -1;
    }
    printf("create udp port at %d\n", LISTEN_GZSERRVER_PORT);

    // create fd to send message to gzserver
    if ((gzserver_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("failed to create socket");
        return -1;
    }
    struct sockaddr_in gzserver_addr = {0};
    gzserver_addr.sin_family = AF_INET;
    gzserver_addr.sin_port = htons(GZSERRVER_PORT); // big-endian
    gzserver_addr.sin_addr.s_addr = inet_addr("127.0.0.1");


    // TODO: create socketr for listening arducopter messages

    struct fdmPacket fdm;
    struct ServoPacket sp = {0};

    // initialize select data
    int nfds = 0, ret;
    fd_set rfds, active_rfds;
    FD_ZERO(&active_rfds);
    FD_SET(0, &active_rfds); nfds = 1;
    FD_SET(listen_gzserver_fd, &active_rfds);
    nfds = nfds > listen_gzserver_fd + 1 ? nfds : listen_gzserver_fd + 1;
    struct timeval timeout = {.tv_sec = 1, .tv_usec = 0};
    rfds = active_rfds;
    while ((ret = select(nfds, &rfds, NULL, NULL, &timeout)) >= 0) {
        // stdin
        if (FD_ISSET(0, &rfds)) {
            char buffer[256] = {0};
            read(0, buffer, 255);
            printf("read: %s\n", buffer);
            printf("hello\n");
        }

        // receive message from gzserver
        if (FD_ISSET(listen_gzserver_fd, &rfds)) {
            int rstatus = recv(listen_gzserver_fd, &fdm, sizeof(fdm), 0);
            if (rstatus < 0) {
                perror("failed to receive message from gzserver");
                return -1;
            }
            show_fdm(&fdm);
            sendto(gzserver_fd, &sp, 64, 0, (struct sockaddr *)&gzserver_addr,
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

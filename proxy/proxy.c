#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <signal.h>
#include <sys/select.h>

#define LISTEN_GZSERRVER_PORT 9003

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

int listen_gzserver_fd = 0;
void sigint_handler (int signum) {
    printf("bye\n");
    if (listen_gzserver_fd > 0)
        close(listen_gzserver_fd);
    exit(0);
}

void normal_termination () {
    if (listen_gzserver_fd > 0)
        close(listen_gzserver_fd);
}

int main () {
    // register SIGINT handler
    if (signal(SIGINT, sigint_handler) == SIG_ERR) {
        perror("failed to register signal handler\n");
        return -1;
    }

    // register atexit
    if (!atexit(normal_termination)) {
        perror("failed to register atexit\n");
        return -1;
    }

    // AF_INET: IPv4 Internet protocols
    // SOCK_DGRAM: Supports datagrams (UDP)
    if ((listen_gzserver_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("failed to create socket\n");
        return -1;
    }
    struct sockaddr_in listen_gzserver_addr = {0};
    listen_gzserver_addr.sin_family = AF_INET;
    listen_gzserver_addr.sin_port = htons(LISTEN_GZSERRVER_PORT); // big-endian
    listen_gzserver_addr.sin_addr.s_addr = INADDR_ANY;
    if (bind(listen_gzserver_fd, (struct sockaddr *)&listen_gzserver_addr,
             sizeof(listen_gzserver_addr)) < 0) {
        perror("failed to bind address\n");
        close(listen_gzserver_fd);
        return -1;
    }

    printf("create udp port at %d\n", LISTEN_GZSERRVER_PORT);

    struct fdmPacket fdm;
    while (1) {
        int ret = recv(listen_gzserver_fd, &fdm, sizeof(fdm), 0);
        if (ret < 0) {
            perror("failed to receive fdm packet\n");
            break;
        }
        show_fdm(&fdm);
    }

    close(listen_gzserver_fd);
    return 0;
}

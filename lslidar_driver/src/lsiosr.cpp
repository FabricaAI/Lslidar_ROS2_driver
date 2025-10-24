/*******************************************************
@company: Copyright (C) 2022, Leishen Intelligent System
@product: LSM10 and N10
@filename: lsiosr.cpp
@brief:
@version:       date:       author:     comments:
@v1.0           21-2-4      yao          new
*******************************************************/
#include "lslidar_driver/lsiosr.h"
#include <errno.h>
#include <sys/ioctl.h>

namespace lslidar_driver {

LSIOSR* LSIOSR::instance(std::string name, int speed, int fd) {
    static LSIOSR obj(name, speed, fd);
    return &obj;
}

LSIOSR::LSIOSR(std::string port, int baud_rate, int fd) : port_(port), baud_rate_(baud_rate), fd_(fd) {
    printf("port = %s, baud_rate = %d\n", port.c_str(), baud_rate);
}

LSIOSR::~LSIOSR() { close(); }
/* 串口配置的函数 */
int LSIOSR::setOpt(int nBits, uint8_t nEvent, int nStop) {
    struct termios newtio, oldtio;
    /* Save and test the existing serial port settings, if there are errors it will be shown here. */
    if (tcgetattr(fd_, &oldtio) != 0) {
        perror("SetupSerial 1");
        return -1;
    }
    bzero(&newtio, sizeof(newtio));
    /* Set control lines */
    newtio.c_cflag |= CLOCAL; // Ignore modem control lines
    newtio.c_cflag |= CREAD; // Enable receiver
    /* Set number of data bits */
    switch (nBits) {
    case 7:
        newtio.c_cflag |= CS7;
        break;
    case 8:
        newtio.c_cflag |= CS8;
        break;
    }
    /* Set parity bits
     * PARENB enables parity generation on output and parity checking for input
     * */
    switch (nEvent) {
    case 'O': // odd parity
        newtio.c_iflag |= (INPCK | ISTRIP); // input parity enable, strip eighth bit
        newtio.c_cflag |= PARENB;
        newtio.c_cflag |= PARODD; // Odd parity
        break;
    case 'E': // even parity
        newtio.c_iflag |= (INPCK | ISTRIP);
        newtio.c_cflag |= PARENB;
        newtio.c_cflag &= ~PARODD;
        break;
    case 'N': // no parity
        newtio.c_cflag &= ~PARENB;
        break;
    }
    /* Baud rate */
    switch (baud_rate_) {
    case 230400:
        cfsetispeed(&newtio, B230400);
        cfsetospeed(&newtio, B230400);
        break;
    case 460800:
        cfsetispeed(&newtio, B460800);
        cfsetospeed(&newtio, B460800);
        break;
    case 500000:
        cfsetispeed(&newtio, B500000);
        cfsetospeed(&newtio, B500000);
        break;
    case 921600:
        cfsetispeed(&newtio, B921600);
        cfsetospeed(&newtio, B921600);
        break;
    default:
        cfsetispeed(&newtio, B460800);
        cfsetospeed(&newtio, B460800);
        break;
    }

    /* Set the number of stop bits */
    if (nStop == 1) newtio.c_cflag &= ~CSTOPB;
    else if (nStop == 2) newtio.c_cflag |= CSTOPB;
    /* Set wait time and minimum rx character count */
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;
    /* Flush the serial buffer */
    tcflush(fd_, TCIFLUSH);
    /* Set the configuration for the serial device */
    if ((tcsetattr(fd_, TCSANOW, &newtio)) != 0) {
        perror("serial set error");
        return -1;
    }

    return 0;
}

void LSIOSR::flushinput() { tcflush(fd_, TCIFLUSH); }

/* Read bytes from serial device */
int LSIOSR::read(unsigned char* buffer, int length, int timeout) {
    int totalBytesRead = 0;
    int rc;
    int unlink = 0;
    unsigned char* pb = buffer;

    if (timeout > 0) {
        rc = waitReadable(timeout);
        if (rc <= 0) {
            return (rc == 0) ? 0 : -1;
        }

        int retry = 3;
        while (length > 0) {
            rc = ::read(fd_, pb, (size_t)length);

            if (rc > 0) {
                length -= rc;
                pb += rc;
                totalBytesRead += rc;

                if (length == 0) {
                    break;
                }
            } else if (rc < 0) {
                printf("error \n");
                retry--;
                if (retry <= 0) {
                    break;
                }
            }
            unlink++;
            rc = waitReadable(20);
            if (unlink > 10) return -1;

            if (rc <= 0) {
                break;
            }
        }
    } else {
        rc = ::read(fd_, pb, (size_t)length);

        if (rc > 0) {
            totalBytesRead += rc;
        } else if ((rc < 0) && (errno != EINTR) && (errno != EAGAIN)) {
            // printf("read error\n");
            return -1;
        }
    }

    return totalBytesRead;
}

// attempt to async read from serial with timeout of millis
// returns 1 if no error 0 if timeout, -1 if error
int LSIOSR::waitReadable(int millis) {
    if (fd_ < 0) {
        return -1;
    }
    int serial = fd_;

    fd_set fdset;
    struct timeval tv;
    int rc = 0;

    while (millis > 0) {
        if (millis < 5000) {
            tv.tv_usec = millis % 1000 * 1000;
            tv.tv_sec = millis / 1000;

            millis = 0;
        } else {
            tv.tv_usec = 0;
            tv.tv_sec = 5;

            millis -= 5000;
        }

        FD_ZERO(&fdset);
        FD_SET(serial, &fdset);

        rc = select(serial + 1, &fdset, NULL, NULL, &tv);
        if (rc > 0) {
            rc = (FD_ISSET(serial, &fdset)) ? 1 : -1;
            break;
        } else if (rc < 0) {
            rc = -1;
            break;
        }
    }

    return rc;
}

int LSIOSR::waitWritable(int millis) {
    if (fd_ < 0) {
        return -1;
    }
    int serial = fd_;

    fd_set fdset;
    struct timeval tv;
    int rc = 0;

    while (millis > 0) {
        if (millis < 5000) {
            tv.tv_usec = millis % 1000 * 1000;
            tv.tv_sec = millis / 1000;

            millis = 0;
        } else {
            tv.tv_usec = 0;
            tv.tv_sec = 5;

            millis -= 5000;
        }

        FD_ZERO(&fdset);
        FD_SET(serial, &fdset);

        rc = select(serial + 1, NULL, &fdset, NULL, &tv);
        if (rc > 0) {
            rc = (FD_ISSET(serial, &fdset)) ? 1 : -1;
            break;
        } else if (rc < 0) {
            rc = -1;
            break;
        }
    }

    return rc;
}

/* 向串口中发送数据 */
int LSIOSR::send(const char* buffer, int length, int timeout) {
    if (fd_ < 0) {
        return -1;
    }

    if ((buffer == 0) || (length <= 0)) {
        return -1;
    }

    int totalBytesWrite = 0;
    int rc;
    char* pb = (char*)buffer;

    if (timeout > 0) {
        rc = waitWritable(timeout);
        if (rc <= 0) {
            return (rc == 0) ? 0 : -1;
        }

        int retry = 3;
        while (length > 0) {
            rc = write(fd_, pb, (size_t)length);
            if (rc > 0) {
                length -= rc;
                pb += rc;
                totalBytesWrite += rc;

                if (length == 0) {
                    break;
                }
            } else {
                retry--;
                if (retry <= 0) {
                    break;
                }
            }

            rc = waitWritable(50);
            if (rc <= 0) {
                break;
            }
        }
    } else {
        rc = write(fd_, pb, (size_t)length);
        if (rc > 0) {
            totalBytesWrite += rc;
        } else if ((rc < 0) && (errno != EINTR) && (errno != EAGAIN)) {
            return -1;
        }
    }

    return totalBytesWrite;
}

int LSIOSR::init() {
    int error_code = 0;

    fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_ > 0) {
        error_code = 0;
        setOpt(DATA_BIT_8, PARITY_NONE, STOP_BIT_1); // Configure settings
        // printf("open_port %s  OK !\n", port_.c_str());
    } else {
        error_code = errno;
    }

    return error_code;
}

int LSIOSR::close() { return ::close(fd_); }

std::string LSIOSR::getPort() { return port_; }

int LSIOSR::setPortName(std::string name) {
    port_ = name;
    return 0;
}

int LSIOSR::GetRxQueueCurrentSize() {
    int size { -1 };
    ioctl(fd_, FIONREAD, &size);
    return size;
}

}

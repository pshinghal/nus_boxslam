#include "ros/ros.h"
#include "std_msgs/UInt8.h"

#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cstdio>
#include <iostream>

int set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                fprintf (stderr, "error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // ignore break signal
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                fprintf (stderr, "error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

void set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                fprintf (stderr, "error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                fprintf (stderr, "error %d setting term attributes", errno);
}


int main (int argc, char **argv) {
        ros::init(argc, argv, "sonarReader");

        ros::NodeHandle n;

        ros::Publisher front_pub = n.advertise<std_msgs::UInt8>("frontSonar", 1);

        ros::Rate loop_rate(100);

        char *portname = "/dev/ttyUSB0";
        int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
        if (fd < 0)
        {
                fprintf (stderr, "error %d opening %s: %s", errno, portname, strerror (errno));
                return 1;
        }

        set_interface_attribs (fd, B9600, 0);  // set speed to 115,200 bps, 8n1 (no parity)
        set_blocking (fd, 0);                // set no blocking

        write (fd, "hello!\n", 7);           // send 7 character greeting
        char buf [10];
        int i, len, inches;
        while (ros::ok()) {
                i = 0;
                while (i < 10) {
                        len = read (fd, &buf[i], 1);
                        if (buf[i] == '\n')
                                break;
                        i++;
                }
                buf[i] = '\0';
                inches = atoi(buf);
                // std::cout << inches;
                std_msgs::UInt8 msg;
                msg.data = inches;
                front_pub.publish(msg);
                ROS_INFO("%d", msg.data);

                ros::spinOnce();
                loop_rate.sleep();
        }
        return 0;
}
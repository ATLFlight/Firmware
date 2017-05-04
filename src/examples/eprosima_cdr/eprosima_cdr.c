#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <termios.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>

#include "serialize_sensor.h"
#include "publish_sensor.h"

int uart_init(const char * uart_name, uint32_t speed);

__EXPORT int eprosima_cdr_main(int argc, char *argv[]);

int eprosima_cdr_main(int argc, char *argv[])
{
  if (argc < 2) {
    errx(1, "need a serial port name or IP address as argument");
  }
  /* if a device path is specified, then assume references a UART */
  int fd;
  void *rtps_handle = NULL;

  if (argv[1] == '/') {
    fd = uart_init(argv[1], 115200);
  } else {
    rtps_handle = rtps_init();
  }

	int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
	orb_set_interval(sensor_sub_fd, 200);
	px4_pollfd_struct_t fds[] = {
		{ .fd = sensor_sub_fd,   .events = POLLIN },
	};

	int error_counter = 0;

	for (int i = 0; i < 50; i++) {
		int poll_ret = px4_poll(fds, 1, 1000);

		if (poll_ret == 0) {
			PX4_ERR("Got no data within a second");
		} else if (poll_ret < 0) {
			if (error_counter < 10 || error_counter % 50 == 0) {
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}
			error_counter++;
		} else {
			if (fds[0].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				struct sensor_combined_s raw;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);
				PX4_INFO("Accelerometer:\t%8.4f\t%8.4f\t%8.4f",
					 (double)raw.accelerometer_m_s2[0],
					 (double)raw.accelerometer_m_s2[1],
					 (double)raw.accelerometer_m_s2[2]);

				if (rtps_handle != NULL) {
				  rtps_publish(rtps_handle, &raw);
				} else {
          char buffer[BUFFER_LENGTH];
          uint32_t s_length = 0;
          serialize_sensor_combined(raw, buffer, &s_length);

          // Write serialized data trough UART
          for(int j=0; j < s_length; j++){
              dprintf(fd, "%c", buffer[j]);
          }
				}
			}
		}
	}

    PX4_INFO("exiting");
    close(fd);
    fflush(stdout);
    return OK;
}

int uart_init(const char * uart_name, uint32_t speed){

    // Open a serial port
    int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);

    if (serial_fd < 0) {
        err(1, "failed to open port: %s", uart_name);
    }

    // Try to set baud rate
    struct termios uart_config;
    int termios_state;
    // Back up the original uart configuration to restore it after exit
    if ((termios_state = tcgetattr(serial_fd, &uart_config)) < 0) {
        warnx("ERR GET CONF %s: %d\n", uart_name, termios_state);
        close(serial_fd);
        return -1;
    }

    // Clear ONLCR flag (which appends a CR for every LF)
    uart_config.c_oflag &= ~ONLCR;

    // USB serial is indicated by /dev/ttyACM0
    if (strcmp(uart_name, "/dev/ttyACM0") != OK && strcmp(uart_name, "/dev/ttyACM1") != OK) {
        // Set baud rate
        if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
            warnx("ERR SET BAUD %s: %d\n", uart_name, termios_state);
            close(serial_fd);
            return -1;
        }
    }

    if ((termios_state = tcsetattr(serial_fd, TCSANOW, &uart_config)) < 0) {
        warnx("ERR SET CONF %s\n", uart_name);
        close(serial_fd);
        return -1;
    }
    return serial_fd;
}

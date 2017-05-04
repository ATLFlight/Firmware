/****************************************************************************
 *   Copyright (c) 2017 James Y. Wilson. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ATLFlight nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

//#include <unistd.h>         //Used for UART
//#include <fcntl.h>          //Used for UART
//#include <termios.h>        //Used for UART

using namespace std;

#include <fastrtps/participant/Participant.h>
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/publisher/Publisher.h>
#include <fastrtps/attributes/PublisherAttributes.h>

#include <fastrtps/Domain.h>

#include <fastrtps/utils/eClock.h>

#include <fastcdr/Cdr.h>
#include <fastcdr/FastCdr.h>
#include <fastcdr/exceptions/Exception.h>

#include <uORB/topics/sensor_combined.h>

#include "SensorCombinedPublisher_direct.h"
#include "publish_sensor.h"


SensorCombinedPublisher_direct::SensorCombinedPublisher_direct(): mp_participant(nullptr),
                                                    mp_publisher(nullptr),
{}

SensorCombinedPublisher_direct::~SensorCombinedPublisher_direct() {	Domain::removeParticipant(mp_participant);}

//uint8_t SensorCombinedPublisher_direct::init_uart()
//{
//
//    //OPEN THE UART
//    //The flags (defined in fcntl.h):
//    //  Access modes (use 1 of these):
//    //      O_RDONLY - Open for reading only.
//    //      O_RDWR - Open for reading and writing.
//    //      O_WRONLY - Open for writing only.
//    //
//    //  O_NDELAY / O_NONBLOCK (same function) - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
//    //                                          if there is no input immediately available (instead of blocking). Likewise, write requests can also return
//    //                                          immediately with a failure status if the output can't be written immediately.
//    //
//    //  O_NOCTTY - When set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.
//    m_uart_filestream = open(m_uart.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);      //Open in non blocking read/write mode
//    if (m_uart_filestream == -1)
//    {
//        //ERROR - CAN'T OPEN SERIAL PORT
//        printf("Error - Unable to open UART '%s'.  Ensure it is not in use by another application\n", m_uart.c_str());
//    }
//
//    //CONFIGURE THE UART
//    //The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
//    //  Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
//    //  CSIZE:- CS5, CS6, CS7, CS8
//    //  CLOCAL - Ignore modem status lines
//    //  CREAD - Enable receiver
//    //  IGNPAR = Ignore characters with parity errors
//    //  ICRNL - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
//    //  PARENB - Parity enable
//    //  PARODD - Odd parity (else even)
//    struct termios options;
//    tcgetattr(m_uart_filestream, &options);
//    options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;     //<Set baud rate
//    options.c_iflag = IGNPAR;
//    options.c_oflag = 0;
//    options.c_lflag = 0;
//    tcflush(m_uart_filestream, TCIFLUSH);
//    tcsetattr(m_uart_filestream, TCSANOW, &options);
//
//    return 0;
//}

bool SensorCombinedPublisher_direct::init()
{
	// Create RTPSParticipant

	ParticipantAttributes PParam;
	PParam.rtps.builtin.domainId = 0;
	PParam.rtps.builtin.leaseDuration = c_TimeInfinite;
	PParam.rtps.setName("Participant_publisher");  //You can put here the name you want
	mp_participant = Domain::createParticipant(PParam);
	if(mp_participant == nullptr)
		return false;

	//Register the type

	Domain::registerType(mp_participant,(TopicDataType*) &myType);

	// Create Publisher

	PublisherAttributes Wparam;
	Wparam.topic.topicKind = NO_KEY;
	Wparam.topic.topicDataType = myType.getName();  //This type MUST be registered
	Wparam.topic.topicName = "SensorCombinedPubSubTopic";
	mp_publisher = Domain::createPublisher(mp_participant,Wparam,(PublisherListener*)&m_listener);
	if(mp_publisher == nullptr)
		return false;
	cout << "Publisher created, waiting for Subscribers." << endl;

	return true;
}

void SensorCombinedPublisher_direct::PubListener::onPublicationMatched(Publisher* pub,MatchingInfo& info)
{
	if (info.status == MATCHED_MATCHING)
	{
		n_matched++;
		cout << "Publisher matched" << endl;
	}
	else
	{
		n_matched--;
		cout << "Publisher unmatched" << endl;
	}
}

uint8_t SensorCombinedPublisher_direct::publish(SensorCombined &st)
{
////----- CHECK FOR ANY RX BYTES -----
//    if (m_uart_filestream != -1)
//    {
//        // Read up to 255 characters from the port if they are there
//        char rx_buffer[1014];
//        int rx_length = read(m_uart_filestream, (void*)rx_buffer, sizeof(rx_buffer)); //Filestream, buffer to store in, number of bytes to read (max)
//        if (rx_length != 72)
//        {
//            printf(".");
//            return 1;
//        }
//        else
//        {
//            //Bytes received
//            eprosima::fastcdr::FastBuffer cdrbuffer(rx_buffer, sizeof(rx_buffer));
//            eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
//            cdr_des >> st.timestamp();
//            cdr_des >> st.gyro_rad();
//            cdr_des >> st.gyro_integral_dt();
//            cdr_des >> st.accelerometer_timestamp_relative();
//            cdr_des >> st.accelerometer_m_s2()[0];
//            cdr_des >> st.accelerometer_m_s2()[1];
//            cdr_des >> st.accelerometer_m_s2()[2];
//            cdr_des >> st.accelerometer_integral_dt();
//            cdr_des >> st.magnetometer_timestamp_relative();
//            cdr_des >> st.magnetometer_ga()[0];
//            cdr_des >> st.magnetometer_ga()[1];
//            cdr_des >> st.magnetometer_ga()[2];
//            cdr_des >> st.baro_timestamp_relative();
//            cdr_des >> st.baro_alt_meter();
//            cdr_des >> st.baro_temp_celcius();
//            printf("read %d bytes ", rx_length);
//            printf("accelerometer: %04f %04f %04f\n",
//                    st.accelerometer_m_s2()[0],
//                    st.accelerometer_m_s2()[1],
//                    st.accelerometer_m_s2()[2]);
//            return 0;
//        }
//    }
//
//    return -1;
  mp_publisher->write(&st);  ++msgsent;
  cout << "Sending sample, count=" << msgsent << endl;
}

// C wrapper for the SensorCombinedPublisher_direct object defined above:
// ======================================================================

void *rtsp_init()
{
  SensorCombinedPublisher_direct publisher;

  if (!publisher.init()) {
    cout << "error: SensorCombinedPublisher_direct initialization failed." << endl;
    return nullptr;
  } else {
    return (void *)&publisher;
  }
}

uint8_t rtsp_publish(void *handle, void *sensor_combined_raw_t)
{
  SensorCombinedPublisher_direct publisher =
      (SensorCombinedPublisher_direct *)handle;
  struct sensor_combined_s *raw = (sensor_combined_s *)sensor_combined_raw_t;
  SensorCombined st;

  if (publisher == nullptr | raw == nullptr) {
    cout << "error: handle or SensorCombined topic was null.";
    return -1;
  }

  // Map the uORB data to the SensorCombined object that will be published
  // via RTPS.
  st.timestamp(raw->timestamp);
  st.gyro_rad(raw->gyro_rad);
  st.gyro_integral_dt(raw->gyro_integral_dt);
  st.accelerometer_timestamp_relative(raw->accelerometer_timestamp_relative);
  st.accelerometer_m_s2(raw->accelerometer_m_s2);
  st.accelerometer_integral_dt(raw->accelerometer_integral_dt);
  st.magnetometer_timestamp_relative(raw->magnetometer_timestamp_relative);
  st.magnetometer_ga(raw->magnetometer_ga);
  st.baro_timestamp_relative(raw->baro_timestamp_relative);
  st.baro_alt_meter(raw->baro_alt_meter);
  st.baro_temp_celcius(raw->baro_temp_celcius);

  //            cdr_des >> st.timestamp();
  //            cdr_des >> st.gyro_rad();
  //            cdr_des >> st.gyro_integral_dt();
  //            cdr_des >> st.accelerometer_timestamp_relative();
  //            cdr_des >> st.accelerometer_m_s2()[0];
  //            cdr_des >> st.accelerometer_m_s2()[1];
  //            cdr_des >> st.accelerometer_m_s2()[2];
  //            cdr_des >> st.accelerometer_integral_dt();
  //            cdr_des >> st.magnetometer_timestamp_relative();
  //            cdr_des >> st.magnetometer_ga()[0];
  //            cdr_des >> st.magnetometer_ga()[1];
  //            cdr_des >> st.magnetometer_ga()[2];
  //            cdr_des >> st.baro_timestamp_relative();
  //            cdr_des >> st.baro_alt_meter();
  //            cdr_des >> st.baro_temp_celcius();
  //            printf("read %d bytes ", rx_length);
  //            printf("accelerometer: %04f %04f %04f\n",
  //                    st.accelerometer_m_s2()[0],
  //                    st.accelerometer_m_s2()[1],
  //                    st.accelerometer_m_s2()[2]);


  publisher->publish(st);

  return 0;
}

// TODO-JYW: LEFT-OFF: Make this object build in the PX4 source tree.
// We will need to target compile PX4, because of the dependency on
// the RTSP CMake package.

//void SensorCombinedPublisher_direct::run()
//{
//	while(m_listener.n_matched == 0)
//	{
//		eClock::my_sleep(250); // Sleep 250 ms
//	}
//
//	// Publication code
//	SensorCombined st;
//
//	/* Initialize your structure here */
//
//	int msgsent = 0;
//	do
//	{
//		if(0 == readFromUART(st))
//		{
//			readFromUART(st);
//			mp_publisher->write(&st);  ++msgsent;
//			cout << "Sending sample, count=" << msgsent << endl;
//		}
//		usleep(100000);
//
//	}while(true);
//}

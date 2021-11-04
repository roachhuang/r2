// r2serial.cpp

// communicate via RS232 serial with a remote uController.
// communicate with ROS using String type messages.
// subscribe to command messages from ROS
// publish command responses to ROS

// program parameters - ucontroller# (0,1), serial port, baud rate

//Thread main
//  Subscribe to ROS String messages and send as commands to uController
//Thread receive
//  Wait for responses from uController and publish as a ROS messages
/*
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <rclcpp/rclcpp.hpp>
//#include <stdlib.h>
//#include <sys/stat.h>
//#include <sys/time.h>
//#include <sys/types.h>
#include <termios.h>
#include <unistd.h>
#include <sstream>
*/
#include <boost/asio.hpp>

#define DEFAULT_BAUDRATE 115200
#define DEFAULT_SERIALPORT "/dev/ttyUSB0"

//Global data
//FILE * fpSerial = NULL;  //serial port file pointer
//int ucIndex;             //ucontroller index number
namespace uart
{
class SimpleSerial
{
public:
  /**
     * Constructor.
     * \param port device name, example "/dev/ttyUSB0" or "COM4"
     * \param baud_rate communication speed, example 9600 or 115200
     * \throws boost::system::system_error if cannot open the
     * serial device
     */
  SimpleSerial(std::string port, unsigned int baud_rate) : io(), serial(io, port)
  {
    serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
  }

  /**
     * Write a string to the serial device.
     * \param s string to write
     * \throws boost::system::system_error on failure
     */
  void writeString(std::string s)
  {
    boost::asio::write(serial, boost::asio::buffer(s.c_str(), s.size()));
  }

  /**
     * Blocks until a line is received from the serial device.
     * Eventual '\n' or '\r\n' characters at the end of the string are removed.
     * \return a string containing the received line
     * \throws boost::system::system_error on failure
     */
  std::string readLine()
  {
    //Reading data char by char, code is optimized for simplicity, not speed
    using namespace boost;
    char c;
    std::string result;
    for (;;)
    {
      asio::read(serial, asio::buffer(&c, 1));
      switch (c)
      {
        case '\r':
          break;
        case '\n':
          return result;
        default:
          result += c;
      }
    }
  }
  /*
  //Initialize serial port, return file descriptor
  FILE * serialInit(char * port, int baud)
  {
    int BAUD = 0;

    struct termios newtio;
    FILE * fp = NULL;

    //Open the serial port as a file descriptor for low level configuration
    // read/write, not controlling terminal for process,
    s_port = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (s_port < 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("sp"), "Error creating serial port: %s", port);
      return fp;
    }

    // set up new settings
    memset(&newtio, 0, sizeof(newtio));
    newtio.c_cflag = CS8 | CLOCAL | CREAD;  //no parity, 1 stop bit

    newtio.c_iflag = IGNCR;    //ignore CR, other options off
    newtio.c_iflag |= IGNBRK;  //ignore break condition

    newtio.c_oflag = 0;  //all options off

    newtio.c_lflag = ICANON;  //process input as lines

    // activate new settings
    tcflush(s_port, TCIFLUSH);
    BAUD = B115200;

    if (cfsetispeed(&newtio, BAUD) < 0 || cfsetospeed(&newtio, BAUD) < 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("sp"), "Error set baud rate: %d", baud);
      close(s_port);
      return NULL;
    }
    tcsetattr(s_port, TCSANOW, &newtio);
    tcflush(s_port, TCIOFLUSH);

    //Open file as a standard I/O stream
    fp = fdopen(s_port, "r+");
    if (!fp)
    {
      RCLCPP_ERROR(rclcpp::get_logger("sp"), "serialInit: Failed to open serial stream %s", port);
      fp = NULL;
    }
    return fp;
  }  //serialInit
  
  void close_port()
  {
    fclose(fpSerial);
    RCLCPP_INFO(rclcpp::get_logger("sp"), "r2Serial stopping");
  }

  

  void sread(){    
    int n = read(s_port, &read_buf, sizeof(read_buf));    
  }

  //Receive command responses from robot uController
  //and publish as a ROS message
  void * rcvThread(void * arg)
  {
    int rcvBufSize = 200;
    char ucResponse[rcvBufSize];  //response string from uController
    char * bufPos;

    std::stringstream ss;

    RCLCPP_INFO(rclcpp::get_logger("sp"), "rcvThread: receive thread running");

    bufPos = fgets(ucResponse, rcvBufSize, fpSerial);
    if (bufPos != NULL)
    {
      // ROS_DEBUG("uc%dResponse: %s", ucIndex, ucResponse);
      // msg.data = ucResponse;
      // ucResponseMsg.publish(msg);
    }

    return NULL;
  }  //rcvThread
  */
private:
  int s_port = -1;
  char read_buf[256];
  boost::asio::io_service io;
  boost::asio::serial_port serial;
};
}  // namespace uart

/*
int main(int argc, char ** argv)
{
  char port[20];  //port name
  int baud;       //baud rate

  char topicSubscribe[20];
  char topicPublish[20];

  pthread_t rcvThrID;  //receive thread ID
  int err;

  RCLCPP_INFO(rclcpp::get_logger("sp"), "connection initializing (%s) at %d baud", port, baud);
  fpSerial = serialInit(port, baud);
  if (!fpSerial)
  {
    RCLCPP_ERROR(rclcpp::get_logger("sp"), "unable to create a new serial port");
    return 1;
  }
  RCLCPP_INFO(rclcpp::get_logger("sp"), "serial connection successful");

  Create receive thread
  err = pthread_create(&rcvThrID, NULL, rcvThread, NULL);
  if (err != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("sp"), "unable to create receive thread");
    return 1;
  }

  return 0;
}
*/
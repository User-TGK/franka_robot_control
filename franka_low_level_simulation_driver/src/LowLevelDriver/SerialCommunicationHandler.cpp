#include <exception>

#include <ros/ros.h>

#include <SerialBlockingReader.hpp>
#include <SerialCommunicationHandler.hpp>

namespace RobotControl
{
namespace FrankaLowLevelDriver
{
SerialCommunicationHandler::SerialCommunicationHandler(const std::string& serialPortPath)
    : port(ioservice, serialPortPath)
{
  port.set_option(boost::asio::serial_port_base::baud_rate(9600));
  port.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
  port.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
  port.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
  port.set_option(boost::asio::serial_port::character_size(boost::asio::serial_port::character_size(8)));

  if(!port.is_open())
  {
    throw std::runtime_error("UNABLE TO ESTABLISH SERIAL CONNECTION WITH SERIAL DEVICE SSC32U.");
  }
}

SerialCommunicationHandler::~SerialCommunicationHandler()
{
}

bool SerialCommunicationHandler::write(const std::string& message)
{
  if(!port.is_open())
  {
    throw std::runtime_error("SERIAL DEVICE SSC32U HAS CLOSED THE CONNECTION.");
  }

  boost::asio::streambuf b;
  std::ostream os(&b);
  os << message;
  boost::asio::write(port, b.data());
  os.flush();

  ROS_DEBUG("WROTE SSC32U SERIAL COMMAND: %s", message.c_str());

  return true;
}

std::string SerialCommunicationHandler::timedRead(long timeout)
{
  if(!port.is_open())
  {
    throw std::runtime_error("SERIAL DEVICE SSC32U HAS CLOSED THE CONNECTION.");
  }

  SerialBlockingReader reader(port, timeout);

  char c;
  std::string rsp;

  while(reader.readChar(c) && c != '\n' && rsp != "+" && rsp != ".")
  {
    rsp += c;
  }

  if(rsp == "." || rsp == "+")
  {
    ROS_DEBUG("READ SSC32U SERIAL FEEDBACK: %s", rsp.c_str());
    return rsp;
  }

  if(c != '\n')
  {
    // Timeout
    throw std::runtime_error("UNABLE TO READ DATA FROM SSC32U WITHIN DEADLINE TIMER DURATION.");
  }

  ROS_DEBUG("READ SSC32U SERIAL FEEDBACK: %s", rsp.c_str());
  return rsp;
}
}
}
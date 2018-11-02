#include <SerialBlockingReader.hpp>

namespace RobotControl
{
namespace FrankaLowLevelDriver
{
SerialBlockingReader::SerialBlockingReader(boost::asio::serial_port& port, size_t timeout)
    : port(port), timeout(timeout), c('\0'), timer(port.get_io_service()), readError(true)
{
}

bool SerialBlockingReader::readChar(char& val)
{
  val = c = '\0';
  port.get_io_service().reset();

  // Asynchronously read 1 character
  boost::asio::async_read(port,
                          boost::asio::buffer(&c, 1),
                          boost::bind(&SerialBlockingReader::readComplete,
                                      this,
                                      boost::asio::placeholders::error,
                                      boost::asio::placeholders::bytes_transferred));

  // Setup deadline timer
  timer.expires_from_now(boost::posix_time::seconds(0.9));
  timer.async_wait(boost::bind(&SerialBlockingReader::timeOut, this, boost::asio::placeholders::error));

  // This will block until a character is read or until it is cancelled
  port.get_io_service().run();

  if(!readError)
  {
    val = c;
  }
  return !readError;
}

void SerialBlockingReader::readComplete(const boost::system::error_code& error, size_t bytesTransferred)
{
  readError = (error || bytesTransferred == 0);

  // Read has finished, so cancel the timer
  timer.cancel();
}

void SerialBlockingReader::timeOut(const boost::system::error_code& error)
{
  if(error)
  {
    // Timeout was cancelled
    return;
  }
  // Timeout was reached
  port.cancel();
}
}
}
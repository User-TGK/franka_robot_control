#ifndef SERIAL_BLOCKING_READER_HPP_
#define SERIAL_BLOCKING_READER_HPP_

#include <boost/asio/serial_port.hpp> 
#include <boost/bind.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/placeholders.hpp>

namespace RobotControl
{
namespace Al5dLowLevelDriver
{

/**
 * @brief this implementation is based on the following code:
 * http://www.ridgesolutions.ie/index.php/2012/12/13/boost-c-read-from-serial-port-with-timeout-example/
 * Credits to Kevin Godden (2013)
 * 
 * In the comment section below the article Kevin Godden states he has no problems whatsoever with other people
 * using his code.
 */
class SerialBlockingReader
{
public:
    /**
     * @brief Construct a new Serial Blocking Reader object
     * 
     * @param port an open serial port
     * @param timeout in milliseconds
     */
    SerialBlockingReader(boost::asio::serial_port& port, size_t timeout);

    /**
     * @brief function that reads a character of times out
     * 
     * @param val the variable that will store the read character
     * @return true if a character is read before the timeout
     * @return false if the read times out
     */
    bool readChar(char& val);

private:
    /**
     * @brief function that is called when an asynchronous read completes of has been cancelled
     * 
     * @param error the error if the asynchronous read was cancelled
     * @param bytesTransferred the number of bytes that were read
     */
    void readComplete(const boost::system::error_code& error, size_t bytesTransferred);
    
    /**
     * @brief function that is called when the asynchronous deadline timer expires
     * 
     * @param error the error message to be displayed when the timeout is reached
     */
    void timeOut(const boost::system::error_code& error);
    
    boost::asio::serial_port& port; ///< Reference to the serial port used to read data from
    size_t timeout; ///< Timeout in milliseconds
    char c; ///< Contains the last read character
    boost::asio::deadline_timer timer; ///< Boost deadline timer instance
    bool readError; ///< True if an error was read
};

} // namespace RobotControl
} // namespace Al5dLowLevelDriver

#endif //SERIAL_BLOCKING_READER_HPP_
 #ifndef SERIAL_COMMUNICATION_HANDLER_HPP_
#define SERIAL_COMMUNICATION_HANDLER_HPP_

#include <string>

#include <boost/asio.hpp>

namespace RobotControl
{
namespace Al5dLowLevelDriver
{

class SerialCommunicationHandler
{
public:
    /**
     * @brief Construct a new Serial Communication Handler object
     * @author Ties Klappe
     * 
     * @param serialPortPath the path to the serial device (SSC32U)
     */
    explicit SerialCommunicationHandler(const std::string& serialPortPath);

    /**
     * @brief Destroy the Serial Communication Handler object
     * @author Ties Klappe
     */
    ~SerialCommunicationHandler();

    /**
     * @brief write a string message to the attached SSC32U serial device (or the simulation version)
     * @author Ties Klappe
     * 
     * @param message the message to be written
     * @return true if the message was successfully written
     * @return false if the message was not successfully written
     */
    bool write(const std::string& message);

    /**
     * @brief function that reads a character from a serial device and throws an exception if the
     * function call timedout before data was read
     * 
     * @param timeout time in milliseconds to wait before timeout (and throwing the exception)
     * @return std::string returns the data that was read
     * @exception if no character was read before the timeout
     */
    std::string timedRead(long timeout);

private:
    /** IOservice for the boost::asio serial communication */
    boost::asio::io_service ioservice;
    /** Port: the port the SSC32U is connected to */
    boost::asio::serial_port port;
};

} // namespace RobotControl
} // namespace Al5dLowLevelDriver

#endif //SERIAL_COMMUNICATION_HANDLER_HPP_
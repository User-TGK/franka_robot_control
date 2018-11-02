#ifndef DEGREE_OF_FREEDOM_HPP_
#define DEGREE_OF_FREEDOM_HPP_

namespace RobotControl
{
namespace Al5dLowLevelDriver
{

class DegreeOfFreedom
{
public:
    /**
     * @brief Construct a new Degree Of Freedom object
     * @author Ties Klappe
     * 
     * @param aChannel the channel the DoF is attached to
     * @param aMinPulseWidth the minimum pulse width that can be send to the DoF
     * @param aMaxPulseWidth the maximum pulse width that can be send to the DoF
     * @param aMinAngle the minimum angle the DoF can be set to in degrees
     * @param aMaxAngle the maximum angle the DoF can be set to in degrees
     * @param aMaxSpeedPerSecond the maximum speed a servo can rotate within one second (in degrees)
     * @param aCurrentPos the current position of the DoF (in pulse width)
     * @param aTargetPos the position the servo is trying to reach (in pulse width)
     */
    DegreeOfFreedom(unsigned short aChannel, unsigned long aMinPulseWidth, unsigned long aMaxPulseWidth,
                    double aMinAngle, double aMaxAngle, double aMaxSpeedPerSecond, unsigned long aCurrentPos, unsigned long aTargetPos);

    /**
     * @brief function that returns a pulseWidth from an angle (in degrees) for a specific DoF
     * @author Ties Klappe
     * 
     * @param angle the angle in degrees to be converted
     * @return unsigned long the pulseWidth
     */
    unsigned long pulseWidthFromAngle(double angle) const;

    /**
     * @brief function that returns an angle (in degrees) from a pulseWidth for a specific DoF
     * @author Ties Klappe
     * 
     * @param pulseWidth the pulseWidth to be converted
     * @return double the angle in degrees
     */
    double angleFromPulseWidth(unsigned long pulseWidth) const;

    /**
     * @brief Get the Channel of a DoF
     * @author Ties Klappe
     * 
     * @return unsigned short the channel
     */
    unsigned short getChannel() const;

    /**
     * @brief Get the Current Pos object
     * @author Ties Klappe
     *  
     * @return unsigned long the current position in pulse width
     */
    unsigned long getCurrentPos() const;

    /**
     * @brief Get the Converted Speed From Degrees Per Second object
     * @author Ties Klappe
     * 
     * @param speed the speed to be converted
     * @return unsigned long the pulse width converted from speed (degrees)
     */
    unsigned long getConvertedSpeedFromDegreesPerSecond(double speed) const;

    /**
     * @brief Get the Target Position of a doF
     * @author Ties Klappe
     * 
     * @return unsigned long the target position of a doF
     */
    unsigned long getTargetPos() const;

    /**
     * @brief Set the Current Position in pulse width
     * @author Ties Klappe
     * 
     * @param aCurrentPos the current position in pulse width
     */
    void setCurrentPos(unsigned long aCurrentPos);

    /**
     * @brief Set the Target Posisiton in pulse width
     * @author Ties Klappe
     * 
     * @param aTargetPos the target position to be reached
     */
    void setTargetPos(unsigned long aTargetPos);

private:
    unsigned short channel; ///< The channel a specific DoF is attached to
    unsigned long minPulseWidth; ///< The minimum pulse width that can be send to the DoF
    unsigned long maxPulseWidth; ///< The maximum pulse width that can be send to the DoF
    double minAngle; ///< The minimum angle the DoF can be set to in degrees
    double maxAngle; ///< The maximum angle the DoF can be set to in degrees
    double maxSpeedPerSecond; ///< The maximum speed per second without weight (no-load speed in degrees)
    unsigned long currentPos; ///< The current position the DoF is set to in pulse width
    unsigned long targetPos; ///< The position the DoF is trying to reach in pulse width
    static const unsigned long minSpeed = 500; ///< The minimum speed in pw
    static const unsigned long maxSpeed = 2500; ///< The maximum speed in pw
};

} // namespace RobotControl
} // namespace Al5dLowLevelDriver

#endif //DEGREE_OF_FREEDOM_HPP_
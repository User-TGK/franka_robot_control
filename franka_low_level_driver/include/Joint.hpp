#ifndef JOINT_HPP_
#define JOINT_HPP_

namespace RobotControl
{
namespace FrankaLowLevelDriver
{
/**
 * @brief For more information about the Franka joints please 
 * see <a href="https://frankaemika.github.io/docs/control_parameters.html">The FCI documentation.</a>
 * @author Ties Klappe
 */
class Joint
{
public:
    /**
     * @brief Construct a new Joint object that is default in the softmode
     * 
     * @param aQMax the hard upper limit of a joint in radians
     * @param aQMin the hard lower limit of a joint in radians
     * @param aQMaxSoft the soft (default) upper limit of a joint in radians
     * @param aQMinSoft the soft (default) lower limit of a joint in radians
     * @author Ties Klappe
     */
    Joint(const double aQMax, const double aQMin, const double aQMaxSoft, const double aQMinSoft);

    /**
     * @brief Destroy the Joint object
     * @author Ties Klappe
     */
    ~Joint();

    /**
     * @brief function that will get the corresponding amount of radians for an amount of degrees
     * if the converted radians are above the maximum limit the maximum limit will be returned, 
     * if the converted radians are under the minimum limit the minimum limit will be returned.
     * 
     * @author Ties Klappe
     */
    double getRadians(const double degrees) const;

    /**
     * @brief static function that will convert an amount of radians to the corresponding amount of degrees
     * 
     * @param radians the amount of radians to be converted
     * @return double the amount of degrees corresponding with the amount of radians
     * @author Ties Klappe
     */
    static double getDegrees(const double radians);

private:
    /**
     * @brief function that will check if an angle is between the upper and lower limits of a joint
     * 
     * @param angle the angle in radians
     * @return double the same value as the angle parameter if the value was between the upper and lower limit,
     * @return the upper limit if the angle was above the upper limit
     * @return the lower limit if the angle was under the lower limit
     * @author Ties Klappe
     */
    double validateAngle(const double angle) const;

    double QMax; ///< The maximum amount of radians a joint can be set to (hard)
    double QMin; ///< The minimum amount of radians a joint can be set to (hard)
    double QMaxSoft; ///< The maximum amount of radians a joint can be set to (soft)
    double QMinSoft; ///< The minimum amount of radians a joint can be set to (soft)
    bool softMode; ///< True if the soft limits are set, false if the hard limits are set
};

} //namespace RobotControl
} //nameSpace FrankaLowLevelDriver

#endif //JOINT_HPP_
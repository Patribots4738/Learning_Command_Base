package math;

public class ArmCalcuations {
    
    /* 
     *   |
     *   |         C
     *   |         *
     *   |     a /   \ b
     * x |      /     \
     *   |    A/_______\B
     *   |         c
     *   |_____________________
     *             y
     */ 

    /**
     * Get the offset of the second arm relative to arm 1
     * @param x the absolute x position from the first pivot point to position the claw
     * @param y the absolute y position from the first pivot point to position the claw
     * Please keep in mind the x and y value must be under Constants.kMaxReachX,Y respectivly
     * Due to an axis controlling the range, they will not go over
     */
    public double getLowerAngle(double x, double y)
    {
        double q2 = Math.acos (
            (x * x + y * y) - (Constants.ArmConstants.kLowerArmLength * Constants.ArmConstants.kLowerArmLength + Constants.ArmConstants.kUpperArmLength * Constants.ArmConstants.kUpperArmLength)
            / (2 * Constants.ArmConstants.kLowerArmLength * Constants.ArmConstants.kUpperArmLength));

        return q2 * ((x < 0) ? 1 : -1);
    }


    /**
     * Get the offset of the first arm relative to the robot
     * @param x the absolute x position from the first pivot point to position the claw
     * @param y the absolute y position from the first pivot point to position the claw
     * Please keep in mind the x and y value must be under Constants.kMaxReachX,Y respectivly
     * Due to an axis controlling the range, they will not go over
     */
    public double getUpperAngle(double x, double y, double q2)
    {
        double q1Left = Math.atan(y / x) + 
                        Math.atan((Constants.ArmConstants.kUpperArmLength * Math.sin(q2))
                                / (Constants.ArmConstants.kLowerArmLength + Constants.ArmConstants.kUpperArmLength * Math.cos(q2)));

        // Currently q1Right is not used, but it is here for future testing
//        double q1Right = Math.atan(y / x) -
//                Math.atan((Constants.ArmConstants.kUpperArmLength * Math.sin(q2))
//                        /(Constants.ArmConstants.kLowerArmLength + Constants.ArmConstants.kUpperArmLength*Math.cos(q2)));

        return q1Left;
    }
}
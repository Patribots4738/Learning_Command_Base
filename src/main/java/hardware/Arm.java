package hardware;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import math.ArmCalculations;
import math.Constants.ArmConstants;

import java.util.ArrayList;

public class Arm implements Loggable {

    /**
     * What the arm positions look like and the index in the array
     * 4
     * O        __     8
     * 1      |      7
     * 3 | 5
     * 2  |||||  6
     */

    // All armPos values are in inches
    final Translation2d[][] armPos = {
            {
                    new Translation2d(-20, 30),
                    new Translation2d(-36, 23),
            },
            {
                    new Translation2d(0, ArmConstants.MAX_REACH)
            },
            {
                    new Translation2d(-12, 19),
                    new Translation2d(-28, 13),
                    new Translation2d(-32, 10)
            }
    };

    Translation2d[][] placementPositions = {
            {
                    new Translation2d(-20, 1),
                    new Translation2d(-36, 23),
                    new Translation2d(-52.6, 33),
            },
            {
                    new Translation2d(0, ArmConstants.MAX_REACH)
            },
            {
                    new Translation2d(-20, 2),
                    new Translation2d(-36, 25),
                    new Translation2d(-52.6, 33)
            }
    };
    // ceil -- force round up
    int armPosDimension1 = (int) Math.ceil(armPos.length / 2.0);
    int armPosDimension2 = 0;

    private boolean operatorOverride = false;

    private double armXPos = 0;
    private double armYPos = 0;

    // The current rotation of the upper arm
    @Log
    private double upperRotation = 0;
    private final ArrayList<Double> upperRotationList = new ArrayList<>();

    // The current rotation of the lower arm
    @Log
    private double lowerRotation = 0;
    private final ArrayList<Double> lowerRotationList = new ArrayList<>();

    // The DESIRED rotation of the upper and lower arm(s)
    private double upperReference = 0;
    private double lowerReference = 0;

    private final CANSparkMax _lowerArmLeft;
    private final CANSparkMax _lowerArmRight;
    private final CANSparkMax _upperArm;
    
    private final RelativeEncoder _lowerArmEncoder;
    private final RelativeEncoder _upperArmEncoder;
    
    private final SparkMaxPIDController _lowerArmPIDController;
    private final SparkMaxPIDController _upperArmPIDController;
    
    ArmCalculations armCalculations = new ArmCalculations();

    /**
     * Constructs a new Arm and configures the encoders and PID controllers.
     */
    public Arm() {

        _lowerArmLeft = new CANSparkMax(ArmConstants.LOWER_ARM_LEFT_MOTOR_CAN_ID, MotorType.kBrushless);
        _lowerArmRight = new CANSparkMax(ArmConstants.LOWER_ARM_RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);
        _lowerArmRight.follow(_lowerArmLeft, true);
        _upperArm = new CANSparkMax(ArmConstants.UPPER_ARM_MOTOR_CAN_ID, MotorType.kBrushless);

        _lowerArmLeft.setIdleMode(IdleMode.kBrake);
        _lowerArmRight.setIdleMode(IdleMode.kBrake);
        _upperArm.setIdleMode(IdleMode.kBrake);

        // Factory reset, so we get the SPARK MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.
        _lowerArmLeft.restoreFactoryDefaults();
        _lowerArmRight.restoreFactoryDefaults();
        _upperArm.restoreFactoryDefaults();

        // Setup encoders and PID controllers for the lower and upper SPARK MAX(s)
        _lowerArmEncoder = _lowerArmLeft.getEncoder();
        _upperArmEncoder = _upperArm.getEncoder();
        _lowerArmEncoder.setPositionConversionFactor(ArmConstants.LOWER_ENCODER_POSITION_FACTOR);
        _upperArmEncoder.setPositionConversionFactor(ArmConstants.UPPER_ENCODER_POSITION_FACTOR);

        // _lowerArmEncoder = _lowerArm.getAbsoluteEncoder(Type.kDutyCycle);
        // _upperArmEncoder = _upperArm.getAbsoluteEncoder(Type.kDutyCycle);
        _lowerArmPIDController = _lowerArmLeft.getPIDController();
        _upperArmPIDController = _upperArm.getPIDController();
        _lowerArmPIDController.setFeedbackDevice(_lowerArmEncoder);
        _upperArmPIDController.setFeedbackDevice(_upperArmEncoder);

        // Set PID constants for the lower and upper SPARK MAX(s)
        _lowerArmPIDController.setP(ArmConstants.LOWER_P);
        _lowerArmPIDController.setI(ArmConstants.LOWER_I);
        _lowerArmPIDController.setD(ArmConstants.LOWER_D);
        _lowerArmPIDController.setFF(ArmConstants.LOWER_FF);
        _lowerArmPIDController.setOutputRange(
                ArmConstants.LOWER_MIN_OUTPUT,
                ArmConstants.LOWER_MAX_OUTPUT);

        _upperArmPIDController.setP(ArmConstants.UPPER_P);
        _upperArmPIDController.setI(ArmConstants.UPPER_I);
        _upperArmPIDController.setD(ArmConstants.UPPER_D);
        _upperArmPIDController.setFF(ArmConstants.UPPER_FF);
        _upperArmPIDController.setOutputRange(
                ArmConstants.UPPER_MIN_OUTPUT,
                ArmConstants.UPPER_MAX_OUTPUT);

        _lowerArmLeft.setSmartCurrentLimit(ArmConstants.LOWER_CURRENT_LIMIT);
        _lowerArmRight.setSmartCurrentLimit(ArmConstants.LOWER_CURRENT_LIMIT);
        _upperArm.setSmartCurrentLimit(ArmConstants.UPPER_CURRENT_LIMIT);

        // Save the SPARK MAX configuration. If a SPARK MAX
        // browns out, it will retain the last configuration
        _lowerArmLeft.burnFlash();
        _lowerArmRight.burnFlash();
        _upperArm.burnFlash();

        resetEncoders();
    }

    public void toggleOperatorOverride() {
        this.operatorOverride = !operatorOverride;
    }

    /**
     * Reset the encoders to zero the arm when initiating the arm
     * Will not be needed in the future because we will have absolute encoders
     */
    public void resetEncoders() {

        _lowerArmEncoder.setPosition(0);
        _upperArmEncoder.setPosition(0);//-0.5687823825412326);

    }

    public void periodic() {
        indexPeriodic();
    }

    public void indexPeriodic() {

        // armPos[armPosDimension1][armPosDimension2]
        if (armPosDimension1 == armPos.length ||
                armPosDimension2 == armPos[armPosDimension1].length) {
            return;
        }

        if (Math.abs(getLowerArmPosition() - lowerReference) > ArmConstants.LOWER_ARM_DEADBAND ||
                Math.abs(getUpperArmPosition() - upperReference) > ArmConstants.UPPER_ARM_DEADBAND) {
            armPosDimension2++;
            drive(armPos[armPosDimension1][armPosDimension2]);
        }
    }

    public boolean getOperatorOverride() {
        return this.operatorOverride;
    }

    public void setLowerArmReference(double reference) {
        this.lowerReference = reference;
    }

    public void setUpperArmReference(double reference) {
        this.upperReference = reference;
    }

    /**
     * Sets arm index from the armPos array
     *
     * @param index the direction to change the arm index by
     */
    public void setArmIndex(int index) {

        index = MathUtil.clamp(index, 0, armPos.length - 1);

        armPosDimension1 = index;
        armPosDimension2 = 0;

    }

    public int getArmIndex() {
        return armPosDimension1;
    }

    /**
     * Calculate the position of the arm based on the joystick input
     * as an absolute position in inches, multiplied by
     * Constants.ArmConstants.kMaxReachX,Y respectively
     *
     * @param position either the joystick input or the desired absolute position
     *                 this case is handled under OperatorOverride
     */
    public void drive(Translation2d position) {

        // If operatorOverride is true, add the joystick input to the current position
        // recall that this value is in inches
        if (operatorOverride) {
            this.armXPos += position.getX();
            this.armYPos += position.getY();
        } else {
            this.armXPos = position.getX();
            this.armYPos = position.getY();
        }

        // Make sure armX and armY are within the range of 0 to infinity
        // Because we cannot reach below the ground.
        // Even though our arm starts 11 inches above the ground,
        // the claw will be 11 inches from the arm end
        armYPos = (position.getY() < 0) ? 0 : armYPos;

        Translation2d armPos = new Translation2d(armXPos, armYPos);

        // Proof: https://www.desmos.com/calculator/ppsa3db9fa
        // If the distance from zero is greater than the max reach, cap it at the max reach
        // Give it a one-inch cushion
        if (armPos.getDistance(new Translation2d(0, 0)) > ArmConstants.MAX_REACH) {
            armPos = armPos.times((ArmConstants.MAX_REACH) / armPos.getDistance(new Translation2d(0, 0)));
        }

        if (armPos.getY() > ArmConstants.MAX_REACH_Y) {
            armPos = new Translation2d(armPos.getX(), ArmConstants.MAX_REACH_Y);
        }
        
        // Get lowerArmAngle and upperArmAngle, the angles of the lower and upper arm
        // Q2 must be gotten first, because lowerArmAngle is reliant on upperArmAngle
        double upperArmAngle = armCalculations.getUpperAngle(armPos.getX(), armPos.getY());
        double lowerArmAngle = armCalculations.getLowerAngle(armPos.getX(), armPos.getY(), upperArmAngle);

        // If upperArmAngle is NaN, then tell the arm not to change position
        // We only check upperArmAngle because lowerArmAngle is reliant on upperArmAngle
        if (Double.isNaN(upperArmAngle)) {
            return;
        }

        setLowerArmReference(Units.radiansToRotations(lowerArmAngle));
        setUpperArmReference(Units.radiansToRotations(upperArmAngle));

        System.out.println("Upper: " + Units.radiansToDegrees(upperArmAngle) +
                " Lower: " + Units.radiansToDegrees(lowerArmAngle));
    }

    /**
     * Set the position of an arm
     *
     * @param position the position to set the upper arm to
     *                 This unit is in revolutions
     */
    public void setUpperArmPosition(double position) {

        position = MathUtil.clamp(
                position,
                ArmConstants.UPPER_ARM_FREEDOM_DEGREES,
                -ArmConstants.UPPER_ARM_FREEDOM_DEGREES
        );

        // Description of FF in Constants :D
        ArmFeedforward feedForward = new ArmFeedforward(
                ArmConstants.S_UPPER,
                ArmConstants.G_UPPER,
                ArmConstants.V_UPPER,
                ArmConstants.A_UPPER);

        // Get the feedforward value for the position,
        // Using a predictive formula with sysID given data of the motor
        double FF = feedForward.calculate(position, 0);
        _upperArmPIDController.setFF(FF);

        // Calculate the rotations needed to get to the position
        double neoPosition = position;

        // Set the position of the neo controlling the upper arm to
        // the converted position, neoPosition
        _upperArmPIDController.setReference(neoPosition, ControlType.kPosition);

        upperRotation = _upperArmEncoder.getPosition();

        upperRotationList.add(upperRotation);
    }

    /**
     * Set the position of the lower arm
     *
     * @param position the position to set the lower arm to
     *                 This unit is in full rotations
     */
    public void setLowerArmPosition(double position) {

        position = MathUtil.clamp(
                position,
                ArmConstants.LOWER_ARM_FREEDOM_DEGREES,
                -ArmConstants.LOWER_ARM_FREEDOM_DEGREES
        );

        ArmFeedforward feedForward = new ArmFeedforward(
                ArmConstants.S_LOWER,
                ArmConstants.G_LOWER,
                ArmConstants.V_LOWER,
                ArmConstants.A_LOWER);

        // Get the feedforward value for the position,
        // Using a predictive formula with sysID given data of the motor
        double FF = feedForward.calculate(position, 0);
        _lowerArmPIDController.setFF(FF);

        // Calculate the rotations needed to get to the position
        // By multiplying the position by the gear ratio
        double neoPosition = position;

        // Set the position of the neo controlling the upper arm to
        // the converted position, neoPosition
        _lowerArmPIDController.setReference(neoPosition, ControlType.kPosition);

        lowerRotation = _lowerArmEncoder.getPosition();

        lowerRotationList.add(lowerRotation);
    }

    /**
     * Get the current position of the upper arm
     *
     * @return the current position of the upper arm
     * This unit is in revolutions
     */
    public double getUpperArmPosition() {
        return _upperArmEncoder.getPosition();
    }

    /**
     * Get the current position of the lower arm
     *
     * @return the current position of the lower arm
     * This unit is in revolutions
     */
    public double getLowerArmPosition() {
        return _lowerArmEncoder.getPosition();
    }

    /**
     * Set the motor to coast mode
     */
    public void setCoastMode() {
      _lowerArmLeft.setIdleMode(CANSparkMax.IdleMode.kCoast);
      _lowerArmRight.setIdleMode(CANSparkMax.IdleMode.kCoast);
      _upperArm.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    /**
     * Set the motor to brake mode
     */
    public void setBrakeMode() {
      _lowerArmLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);
      _lowerArmRight.setIdleMode(CANSparkMax.IdleMode.kBrake);
      _upperArm.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void printList() {
        System.out.println(upperRotationList);
    }
}

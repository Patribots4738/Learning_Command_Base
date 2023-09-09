package hardware;

import calc.ArmCalculations;
import calc.Constants.ArmConstants;
import calc.Constants.PlacementConstants;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

// We use the black solution as seen in: https://www.desmos.com/calculator/fqyyldertp
public class Arm {

    int armPosDimension1 = PlacementConstants.STOWED_INDEX;
    int armPosDimension2 = 1;

    private boolean startedTransition = false;

    private boolean operatorOverride = false;

    private double armXReference = 0;
    private double armYReference = 0;

    // The current rotation of the upper arm
    private double upperRotation = 0;

    // The current rotation of the lower arm
    private double lowerRotation = 0;

    // The DESIRED rotation of the arms
    private double upperReferenceAngle = 0;
    private double lowerReferenceAngle = 0;

    private double lowerDiff = 0;
    private double upperDiff = 0;

    private boolean armsAtDesiredPosition = false;
    
    private double armXPos = 0;
    private double armYPos = 0;

    private final CANSparkMax lowerArmRight;
    private final CANSparkMax lowerArmLeft;
    private final CANSparkMax upperArm;

    private final AbsoluteEncoder lowerArmEncoder;
    private final AbsoluteEncoder upperArmEncoder;
    private final SparkMaxPIDController lowerArmPIDController;
    private final SparkMaxPIDController upperArmPIDController;

    private final ArmCalculations armCalculations;

    /**
     * Constructs a new Arm and configures the encoders and PID controllers.
     */
    public Arm() {

      lowerArmRight = new CANSparkMax(ArmConstants.LOWER_ARM_RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);
      lowerArmLeft = new CANSparkMax(ArmConstants.LOWER_ARM_LEFT_MOTOR_CAN_ID, MotorType.kBrushless);
      upperArm = new CANSparkMax(ArmConstants.UPPER_ARM_MOTOR_CAN_ID, MotorType.kBrushless);

      lowerArmRight.setIdleMode(IdleMode.kBrake);
      lowerArmLeft.setIdleMode(IdleMode.kBrake);
      upperArm.setIdleMode(IdleMode.kCoast);

      // Factory reset, so we get the SPARK MAX to a known state before configuring
      // them. This is useful in case a SPARK MAX is swapped out.
      lowerArmRight.restoreFactoryDefaults();
      lowerArmLeft.restoreFactoryDefaults();
      upperArm.restoreFactoryDefaults();

      lowerArmEncoder = lowerArmRight.getAbsoluteEncoder(Type.kDutyCycle);
      upperArmEncoder = upperArm.getAbsoluteEncoder(Type.kDutyCycle);

      lowerArmPIDController = lowerArmRight.getPIDController();
      upperArmPIDController = upperArm.getPIDController();

      lowerArmPIDController.setFeedbackDevice(lowerArmEncoder);
      upperArmPIDController.setFeedbackDevice(upperArmEncoder);

      lowerArmEncoder.setPositionConversionFactor(ArmConstants.LOWER_ENCODER_POSITION_FACTOR);
      lowerArmEncoder.setVelocityConversionFactor(ArmConstants.LOWER_ENCODER_VELOCITY_FACTOR);

      upperArmEncoder.setPositionConversionFactor(ArmConstants.UPPER_ENCODER_POSITION_FACTOR);
      upperArmEncoder.setVelocityConversionFactor(ArmConstants.UPPER_ENCODER_VELOCITY_FACTOR);

      lowerArmLeft.setSmartCurrentLimit(ArmConstants.LOWER_FREE_LIMIT);
      lowerArmRight.setSmartCurrentLimit(ArmConstants.LOWER_FREE_LIMIT);
      upperArm.setSmartCurrentLimit(ArmConstants.UPPER_FREE_LIMIT);

      // Set PID constants for the lower and upper SPARK MAX(s)
      lowerArmPIDController.setP(ArmConstants.LOWER_P);
      lowerArmPIDController.setI(ArmConstants.LOWER_I);
      lowerArmPIDController.setD(ArmConstants.LOWER_D);
      lowerArmPIDController.setFF(ArmConstants.LOWER_FF);
      lowerArmPIDController.setOutputRange(
      ArmConstants.LOWER_MIN_OUTPUT,
      ArmConstants.LOWER_MAX_OUTPUT);

      upperArmPIDController.setP(ArmConstants.UPPER_P);
      upperArmPIDController.setI(ArmConstants.UPPER_I);
      upperArmPIDController.setD(ArmConstants.UPPER_D);
      upperArmPIDController.setFF(ArmConstants.UPPER_FF);
      upperArmPIDController.setOutputRange(
      ArmConstants.UPPER_MIN_OUTPUT,
      ArmConstants.UPPER_MAX_OUTPUT);

      // Save the SPARK MAX configuration. If a SPARK MAX
      // browns out, it will retain the last configuration
      lowerArmLeft.follow(lowerArmRight, true);
      upperArmEncoder.setInverted(true);

      // zeroUpperArmEncoder();
      // zeroLowerArmEncoder();

      lowerArmRight.burnFlash();
      lowerArmLeft.burnFlash();
      upperArm.burnFlash();

      armCalculations = new ArmCalculations();
      setBrakeMode();
    }

    public void periodic() {
        if (!operatorOverride) { indexPeriodic();}
        setLowerArmPosition(lowerReferenceAngle);
        setUpperArmAngle(upperReferenceAngle);
        upperDiff = (Units.radiansToDegrees(upperReferenceAngle) - Units.radiansToDegrees(getUpperArmAngle()));
        lowerDiff = (Units.radiansToDegrees(lowerReferenceAngle) - Units.radiansToDegrees(getLowerArmAngle()));
        // Use forward kinematics to get the x and y position of the end effector
        armXPos = ((ArmConstants.LOWER_ARM_LENGTH * Math.cos((getLowerArmAngle() - (Math.PI/2)))) + (ArmConstants.UPPER_ARM_LENGTH * Math.cos((getUpperArmAngle() - Math.PI) + (getLowerArmAngle() - (Math.PI/2)))));
        armYPos = ((ArmConstants.LOWER_ARM_LENGTH * Math.sin((getLowerArmAngle() - (Math.PI/2)))) + (ArmConstants.UPPER_ARM_LENGTH * Math.sin((getUpperArmAngle() - Math.PI) + (getLowerArmAngle() - (Math.PI/2)))));
    }

    public void indexPeriodic() {

      boolean atDesiredCoarse = (
          Math.abs(lowerReferenceAngle - getLowerArmAngle()) < ArmConstants.LOWER_ARM_DEADBAND_COARSE &&
          Math.abs(upperReferenceAngle - getUpperArmAngle()) < ArmConstants.UPPER_ARM_DEADBAND_COARSE);

      boolean atDesiredFine = (
          Math.abs(lowerReferenceAngle - getLowerArmAngle()) < ArmConstants.LOWER_ARM_DEADBAND_FINE &&
          Math.abs(upperReferenceAngle - getUpperArmAngle()) < ArmConstants.UPPER_ARM_DEADBAND_FINE);

      boolean finalDeadband =
          (armPosDimension2 < PlacementConstants.ARM_POSITIONS[armPosDimension1].length-1)
          ? atDesiredCoarse : atDesiredFine;

      // Syntax example: PlacementConstants.ARM_POSITIONS[armPosDimension1][armPosDimension2]
      if (!startedTransition)
      {
        startedTransition = true;
        drive(PlacementConstants.ARM_POSITIONS[armPosDimension1][armPosDimension2]);
        return;
      }
      
      // Notice that this code is getting the difference in angle between the arms.
      // It might be better to instead use the difference in position, but I'm not sure. - Hamilton
      if (finalDeadband && !armsAtDesiredPosition)
      {
        armPosDimension2++;

        // This line isn't strictly necessary, but could be included...
        // armPosDimension2 = MathUtil.clamp(armPosDimension2, 0, PlacementConstants.ARM_POSITIONS[armPosDimension1].length);
        
        if (armPosDimension2 >= PlacementConstants.ARM_POSITIONS[armPosDimension1].length)
        {
          armsAtDesiredPosition = true;
          if (armPosDimension1 == PlacementConstants.ARM_FLIP_INDEX) {
            armPosDimension1 = PlacementConstants.STOWED_INDEX;
            armPosDimension2 = PlacementConstants.ARM_POSITIONS[PlacementConstants.STOWED_INDEX].length-1;
          }
          return;
        }
        drive(PlacementConstants.ARM_POSITIONS[armPosDimension1][armPosDimension2]);
      }
    }

    /**
     * Sets arm index from the PlacementConstants.ARM_POSITIONS 2d array
     *
     * @param index the direction to change the arm index by
     */
    public void setArmIndex(int index) {

        index = MathUtil.clamp(index, 0, PlacementConstants.ARM_POSITIONS.length-1);

        // Check if we are already at the desired index, and if we are not operator overriding,
        // This is because, if we are operator overriding, we want to be able to go to any index
        // If we are trying to go to a floor index, allow it to restart itself
        if ((index == armPosDimension1 || 
              (index == PlacementConstants.STOWED_INDEX && 
              (armPosDimension1 == PlacementConstants.HIGH_TO_STOWED_INDEX ||
              (armPosDimension1 == PlacementConstants.ARM_FLIP_INDEX && armsAtDesiredPosition)))) &&
            index != PlacementConstants.CONE_FLIP_INDEX &&
            index != PlacementConstants.CONE_INTAKE_INDEX &&
            index != PlacementConstants.CUBE_INTAKE_INDEX &&
            !operatorOverride) 
        {
          startedTransition = true;
          return;
        }
        else {
          startedTransition = false;
          armsAtDesiredPosition = false;
        }

        armPosDimension2 = 0;

        if (index == PlacementConstants.CONE_HIGH_PREP_INDEX &&
             armPosDimension1 == PlacementConstants.CONE_HIGH_PREP_TO_PLACE_INDEX ||
              index == PlacementConstants.CONE_MID_PREP_INDEX && 
               armPosDimension1 == PlacementConstants.CONE_MID_PREP_TO_PLACE_INDEX) 
        {
            armPosDimension2 = PlacementConstants.ARM_POSITIONS[index].length-1;
        }

        armPosDimension1 = index;
        // Turn off operator override to prevent arm.drive from setting values wrong
        this.operatorOverride = false;

    }

    public int getArmIndex() {
        return this.armPosDimension1;
    }

    /**
     * Calculate the position of the arm based on the joystick input
     * as an absolute position in inches, multiplied by
     * Constants.ArmConstants.kMaxReachX/Y respectively
     *
     * @param position either the joystick input or the desired absolute position
     *                 this case is handled under OperatorOverride
     */
    public void drive(Translation2d position) {

        // If operatorOverride is true, add the joystick input to the current position
        // recall that this value is in inches
        if (operatorOverride) {
          // If the robot is facing left, have left joystick be positive
          // If the robot is facing right, have left joystick be negative
          this.armXReference += (position.getX());
          this.armYReference += (position.getY());
        } else {
          // If the arm is mirrored, invert all incoming X values
          this.armXReference = position.getX();
          this.armYReference = position.getY();
        }

        // Make sure armX and armY are within the range of 0 to infinity
        // Because we cannot reach below the ground.
        // Even though our arm starts 11 inches above the ground,
        // the claw will be 11 inches from the arm end
        armYReference = (armYReference < 0) ? 0 : armYReference;

        Translation2d armPosition = new Translation2d(armXReference, armYReference);

        // Proof: https://www.desmos.com/calculator/ppsa3db9fa
        // If the distance from zero is greater than the max reach, cap it at the max reach
        if (armPosition.getNorm() > ArmConstants.MAX_REACH) {
          armPosition = armPosition.times((ArmConstants.MAX_REACH - 0.1) / armPosition.getNorm());
        }

        // If the distance from zero is less than the min reach, cap it at the min reach
        // This min reach is the lower arm length - the upper arm length
        else if (armPosition.getNorm() < ArmConstants.MIN_REACH) {
          armPosition = armPosition.times((ArmConstants.MIN_REACH + 0.1) / armPosition.getNorm());
        }

        // If the arm is trying to reach higher than 6'6", cap it at 6'6"
        // The field gives us this limit.
        if (armPosition.getY() > ArmConstants.MAX_REACH_Y) {
          armPosition = new Translation2d(armPosition.getX(), ArmConstants.MAX_REACH_Y);
        }
        // If the arm is trying to reach further than 48" out from the chassis, cap it at 48"
        // The field gives us this limit.
        if (armPosition.getX() > ArmConstants.MAX_REACH_X) {
          armPosition = new Translation2d(ArmConstants.MAX_REACH_X, armPosition.getY());
        }
        else if (armPosition.getX() < -ArmConstants.MAX_REACH_X) {
          armPosition = new Translation2d(-ArmConstants.MAX_REACH_X, armPosition.getY());
        }

        // Get lowerArmAngle and upperArmAngle, the angles of the lower and upper arm
        // Q2 must be gotten first, because lowerArmAngle is reliant on upperArmAngle
        double upperArmAngle = armCalculations.getUpperAngle(armPosition.getX(), armPosition.getY());
        double lowerArmAngle = armCalculations.getLowerAngle(armPosition.getX(), armPosition.getY(), upperArmAngle);

        // If upperArmAngle is NaN, then tell the arm not to change position
        // We only check upperArmAngle because lowerArmAngle is reliant on upperArmAngle
        if (Double.isNaN(upperArmAngle)) {
          System.out.println("Upper angle NAN " + armPosition + " " + armPosition.getNorm());
          return;
        }

        // Add PI/2 to lowerArmAngle...
        // because the calculated angle is relative to the ground,
        // And the zero for the encoder is set to the direction of gravity
        // Add PI to upperArmAngle...
        // because armCalculations gives us the angle relative to the upper arm
        lowerArmAngle += (Math.PI/2);
        upperArmAngle += Math.PI;

        // Clamp the output angles as to not murder our precious hard stops
        upperArmAngle = MathUtil.clamp(
          upperArmAngle,
          ArmConstants.UPPER_ARM_LOWER_LIMIT,
          ArmConstants.UPPER_ARM_UPPER_LIMIT
        );

        // Clamp the output angles as to not murder our precious hard stops
        lowerArmAngle = MathUtil.clamp(
          lowerArmAngle,
          ArmConstants.LOWER_ARM_LOWER_LIMIT,
          ArmConstants.LOWER_ARM_UPPER_LIMIT
        );

        // Set the reference values to the modified X and Y values
        // This is especially important for the operatorOverride going OOB
        this.armXReference = armPosition.getX();
        this.armYReference = armPosition.getY();

        // Finally, set the reference values for the lower and upper arm:
        setLowerArmReference(lowerArmAngle);
        setUpperArmReference(upperArmAngle);
    }

    public void setLowerArmReference(double reference) {
      this.lowerReferenceAngle = reference;
    }

    public void setUpperArmReference(double reference) {
      this.upperReferenceAngle = reference;
    }

    /**
     * Set the position of an arm
     *
     * @param angle the position to set the upper arm to
     *                This unit is in rads
     */
    public void setUpperArmAngle(double angle) {

        angle = MathUtil.clamp(
            angle,
            ArmConstants.UPPER_ARM_LOWER_LIMIT,
            ArmConstants.UPPER_ARM_UPPER_LIMIT
        );

        // Description of FF in Constants :D
        ArmFeedforward feedForward = new ArmFeedforward(
            ArmConstants.S_UPPER,
            ArmConstants.G_UPPER,
            ArmConstants.V_UPPER,
            ArmConstants.A_UPPER);

        // Get the feedforward value for the position,
        // Using a predictive formula with sysID given data of the motor
        double FF = feedForward.calculate((angle), 0);
        upperArmPIDController.setFF(FF);

        // Set the position of the neo controlling the upper arm to
        upperArmPIDController.setReference((angle), ControlType.kPosition);

        upperRotation = upperArmEncoder.getPosition();
    }

    /**
     * Set the position of the lower arm
     *
     * @param position the position to set the lower arm to
     *                 This unit is in rads
     */
    public void setLowerArmPosition(double position) {

        position = MathUtil.clamp(
            position,
            ArmConstants.LOWER_ARM_LOWER_LIMIT,
            ArmConstants.LOWER_ARM_UPPER_LIMIT
        );

        ArmFeedforward feedForward = new ArmFeedforward(
            ArmConstants.S_LOWER,
            ArmConstants.G_LOWER,
            ArmConstants.V_LOWER,
            ArmConstants.A_LOWER);

        // Get the feedforward value for the position,
        // Using a predictive formula with sysID given data of the motor
        double FF = feedForward.calculate((position), 0);
        lowerArmPIDController.setFF(FF);
        // Set the position of the neo controlling the upper arm to
        // the converted position, neoPosition
        lowerArmPIDController.setReference((position), ControlType.kPosition);

        lowerRotation = lowerArmEncoder.getPosition();
    }

    /**
     * Get the current position of the upper arm
     *
     * @return the current position of the upper arm
     * This unit is in rads
     */
    public double getUpperArmAngle() {
        return upperArmEncoder.getPosition();
    }

    /**
     * Get the current position of the lower arm
     *
     * @return the current position of the lower arm
     * This unit is in rads
     */
    public double getLowerArmAngle() {
        return lowerArmEncoder.getPosition();
    }

    public boolean getAtDesiredPositions() {
        return this.armsAtDesiredPosition;
    }

    public double getXPosition() {
        return this.armXPos;
    }
    
    public double getYPosition() {
      return this.armYPos;
    }

    // Check if the arm is at its desired position and that position is a placement index,
    // Note that the stowed position is not a placement index, but can be used as hybrid placement
    public boolean getAtPlacementPosition() {
      return (armPosDimension1 == PlacementConstants.CUBE_HIGH_INDEX ||
              armPosDimension1 == PlacementConstants.AUTO_CUBE_HIGH_INDEX ||
              armPosDimension1 == PlacementConstants.CONE_HIGH_PLACEMENT_INDEX ||
              armPosDimension1 == PlacementConstants.CONE_HIGH_PREP_TO_PLACE_INDEX ||
              armPosDimension1 == PlacementConstants.CUBE_MID_INDEX ||
              armPosDimension1 == PlacementConstants.AUTO_CUBE_MID_INDEX ||
              armPosDimension1 == PlacementConstants.CONE_MID_PLACEMENT_INDEX ||
              armPosDimension1 == PlacementConstants.CONE_MID_PREP_TO_PLACE_INDEX ||
              armPosDimension1 == PlacementConstants.HYBRID_PLACEMENT_INDEX) &&
              armsAtDesiredPosition;
    }

    public boolean getAtPrepIndex() {
      return (armPosDimension1 == PlacementConstants.FLOOR_INTAKE_PREP_INDEX ||
              armPosDimension1 == PlacementConstants.CONE_HIGH_PREP_INDEX ||
              armPosDimension1 == PlacementConstants.CONE_MID_PREP_INDEX);
    }

    // If we are at a prep index,
    // set the index to the prep_to_place equivelent
    public void finishPlacement() {
      if (getAtPrepIndex()) {
        switch (armPosDimension1) {
          case PlacementConstants.CONE_HIGH_PREP_INDEX:
            setArmIndex(PlacementConstants.CONE_HIGH_PREP_TO_PLACE_INDEX);
            break;
          case PlacementConstants.CONE_MID_PREP_INDEX:
            setArmIndex(PlacementConstants.CONE_MID_PREP_TO_PLACE_INDEX);
            break;
        }
      }
    }

    public void toggleOperatorOverride() {
      this.operatorOverride = !operatorOverride;
    }

    public boolean getOperatorOverride() {
      return this.operatorOverride;
    }

    // Set the motors to coast mode
    public void setCoastMode() {
        lowerArmRight.setIdleMode(CANSparkMax.IdleMode.kCoast);
        lowerArmLeft.setIdleMode(CANSparkMax.IdleMode.kCoast);
        upperArm.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    // This exists for the purpose of being able to manually move the upper arm easier.
    public void setUpperArmCoastMode() {
      upperArm.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    // Set the motors to brake mode
    public void setBrakeMode() {
      lowerArmLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);
      lowerArmRight.setIdleMode(CANSparkMax.IdleMode.kBrake);
      upperArm.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    // Very rough code below: this method is meant to take an encoder
    // and zero it with the knowledge that it is at the hard stop
    // when the method is called
    public void zeroLowerArmEncoder() {
      // The constant found here can be found in 
      // REV Hardware client when the arm is pointed straight up
      // Then, depending on if the value is more or less than PI,
      // Add or subtract PI to the value
      
      // Note, that this value must be within the range of 0-2PI
      // absolutePosition = lowerArmEncoder.getPosition() + lowerArmEncoder.getZeroOffset();
      // test that 2024 friends :) ^^ itll be kinda nice to just do
      // lowerArmEncoder.setZeroOffset(absolutePosition + (absolutePosition > Math.PI) ? -Math.PI : Math.PI);
      
      lowerArmEncoder.setZeroOffset(5.4559006-Math.PI);
    }

    // Very rough code below: this method is meant to take an encoder
    // and zero it with the knowledge that it is at the hard stop
    // when the method is called
    public void zeroUpperArmEncoder() {
      // The constant found here can be found in 
      // REV Hardware client when the arm is pointed straight up
      // Then, depending on if the value is more or less than PI,
      // Add or subtract PI to the value
      upperArmEncoder.setZeroOffset(2.7098798+Math.PI);
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.ADIS16470_IMU;
import frc.robot.util.Pose3dLogger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.Constants.DriveConstants;
import frc.robot.util.Constants.FieldConstants;

public class Swerve extends SubsystemBase {

    private double speedMultiplier = 1;

    private final MAXSwerveModule frontLeft = new MAXSwerveModule(
            DriveConstants.FRONT_LEFT_DRIVING_CAN_ID,
            DriveConstants.FRONT_LEFT_TURNING_CAN_ID,
            DriveConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET);

    private final MAXSwerveModule frontRight = new MAXSwerveModule(
            DriveConstants.FRONT_RIGHT_DRIVING_CAN_ID,
            DriveConstants.FRONT_RIGHT_TURNING_CAN_ID,
            DriveConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET);

    private final MAXSwerveModule rearLeft = new MAXSwerveModule(
            DriveConstants.REAR_LEFT_DRIVING_CAN_ID,
            DriveConstants.REAR_LEFT_TURNING_CAN_ID,
            DriveConstants.BACK_LEFT_CHASSIS_ANGULAR_OFFSET);

    private final MAXSwerveModule rearRight = new MAXSwerveModule(
            DriveConstants.REAR_RIGHT_DRIVING_CAN_ID,
            DriveConstants.REAR_RIGHT_TURNING_CAN_ID,
            DriveConstants.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET);

    private double[] desiredModuleStates = new double[6];
    private double[] realModuleStates = new double[6];

    // The gyro sensor
    private final ADIS16470_IMU gyro = new ADIS16470_IMU();

    private final MAXSwerveModule[] swerveModules = new MAXSwerveModule[] {
            frontLeft,
            frontRight,
            rearLeft,
            rearRight
    };

    private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.DRIVE_KINEMATICS,
            getGyroAngle(),
            getModulePositions(),
            new Pose2d(),
            // Trust the information of the vision more
            // Nat.N1()).fill(0.1, 0.1, 0.1) --> trust more
            // Nat.N1()).fill(1.25, 1.25, 1.25) --> trust less
            // Notice that the theta on the vision is very large,
            // and the state measurement is very small.
            // This is because we assume that the IMU is very accurate.
            // You can visualize these graphs working together here:
            // https://www.desmos.com/calculator/a0kszyrwfe
            new MatBuilder<>(
                    Nat.N3(),
                    Nat.N1()).fill(0.1, 0.1, 0.05),
            // State measurement
            // standard deviations
            // X, Y, theta
            new MatBuilder<>(
                    Nat.N3(),
                    Nat.N1()).fill(0.6, 0.6, 2)
    // Vision measurement
    // standard deviations
    // X, Y, theta
    );

    /**
     * Creates a new DriveSu1stem.
     */
    public Swerve() {
        resetEncoders();
        zeroHeading();
        setBrakeMode();

    }

    @Override
    public void periodic() {

      SmartDashboard.putNumberArray("RobotPose3d",
                Pose3dLogger.composePose3ds(
                        new Pose3d(
                                new Translation3d(
                                        getPose().getX(),
                                        getPose().getY(),
                                        Math.hypot(
                                                getRoll().getSin()
                                                        * DriveConstants.ROBOT_LENGTH_METERS / 2.0,
                                                getPitch().getSin() *
                                                        DriveConstants.ROBOT_LENGTH_METERS / 2.0)),
                                new Rotation3d(getRoll().getRadians(), getPitch().getRadians(),
                                        getYaw().getRadians()))));
                                        
        //Update the poseEstimator using the current timestamp (from DriverUI.java), the gyro angle, and the current module states
        
        poseEstimator.updateWithTime(Timer.getFPGATimestamp(), getGyroAngle(), getModulePositions());
        if (FieldConstants.IS_SIMULATION) {
            
            for (MAXSwerveModule mod : swerveModules) {
                mod.tick();
            }

            SwerveModuleState[] measuredStates = new SwerveModuleState[] {
                    frontLeft.getState(), frontRight.getState(), rearLeft.getState(), rearRight.getState()
            };

            ChassisSpeeds speeds = DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(measuredStates);

            resetOdometry(
                    getPose().exp(
                            new Twist2d(
                                    0, 0,
                                    speeds.omegaRadiansPerSecond * .02)));

            realModuleStates = new double[] {
                    measuredStates[0].angle.getRadians(), measuredStates[0].speedMetersPerSecond,
                    measuredStates[1].angle.getRadians(), measuredStates[1].speedMetersPerSecond,
                    measuredStates[2].angle.getRadians(), measuredStates[2].speedMetersPerSecond,
                    measuredStates[3].angle.getRadians(), measuredStates[3].speedMetersPerSecond
            };
        }

    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public SwerveDrivePoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative) {

        xSpeed *= (DriveConstants.MAX_SPEED_METERS_PER_SECOND * speedMultiplier);
        ySpeed *= (DriveConstants.MAX_SPEED_METERS_PER_SECOND * speedMultiplier);
        rotSpeed *= (DriveConstants.DYNAMIC_MAX_ANGULAR_SPEED * speedMultiplier);

        SwerveModuleState[] swerveModuleStates = 
            DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rotSpeed));
                
        setModuleStates(swerveModuleStates);
    }

    public void driveInAuto(ChassisSpeeds speeds) {
        drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false);
    }

    /**
     * This is the most theoretical thing that is in the code.
     * It takes our current position and then adds an offset to it, knowing that the
     * robot's estimated position
     * is not following the exact position of the robot.
     * 
     * @param speeds the speeds about to be inputted into the robot.
     * @return the same thing as we input.
     *         Think of this method as an interceptor,
     *         not changing the parameter but using it for calculations.
     */
    /**
     * Credit: WPIlib 2024
     * Discretizes a continuous-time chassis speed.
     *
     * @param vx    Forward velocity.
     * @param vy    Sideways velocity.
     * @param omega Angular velocity.
     */
    public ChassisSpeeds discretize(ChassisSpeeds speeds) {
        if (FieldConstants.GAME_MODE == FieldConstants.GameMode.TEST) {
            return speeds;
        }

        double dt = 0.02;

        var desiredDeltaPose = new Pose2d(
                speeds.vxMetersPerSecond * dt,
                speeds.vyMetersPerSecond * dt,
                new Rotation2d(speeds.omegaRadiansPerSecond * dt));

        var twist = new Pose2d().log(desiredDeltaPose);

        return new ChassisSpeeds((twist.dx / dt), (twist.dy / dt), (speeds.omegaRadiansPerSecond));
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setWheelsX() {
        frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    }

    public Command getSetWheelsXCommand() {
        return runOnce(this::setWheelsX);
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        rearLeft.setDesiredState(desiredStates[2]);
        rearRight.setDesiredState(desiredStates[3]);

        desiredModuleStates = new double[] {
                desiredStates[0].angle.getRadians(), desiredStates[0].speedMetersPerSecond,
                desiredStates[1].angle.getRadians(), desiredStates[1].speedMetersPerSecond,
                desiredStates[2].angle.getRadians(), desiredStates[2].speedMetersPerSecond,
                desiredStates[3].angle.getRadians(), desiredStates[3].speedMetersPerSecond
        };
    }

    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(
                getGyroAngle(),
                getModulePositions(),
                pose);
    }

    public SwerveModuleState[] getModuleStates() {

        SwerveModuleState[] states = new SwerveModuleState[4];

        for (int modNum = 0; modNum < swerveModules.length; modNum++) {
            states[modNum] = swerveModules[modNum].getState();
        }
        return states;

    }

    public SwerveModulePosition[] getModulePositions() {

        SwerveModulePosition[] positions = new SwerveModulePosition[4];

        for (int modNum = 0; modNum < swerveModules.length; modNum++) {
            positions[modNum] = swerveModules[modNum].getPosition();
        }
        return positions;

    }

    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        gyro.reset();
    }

    public Rotation2d getGyroAngle() {

        // gyro.setYawAxis(IMUAxis.kZ);

        Rotation2d yawRotation2d = Rotation2d.fromDegrees(gyro.getAngle());

        if (DriveConstants.GYRO_REVERSED) {
            yawRotation2d = yawRotation2d.unaryMinus();
        }

        return yawRotation2d;
    }

    public Rotation2d getYaw() {
        return this.getPose().getRotation();
    }

    public Rotation2d getPitch() {

        Rotation2d pitchRotation2d = Rotation2d
                .fromDegrees(gyro.getXComplementaryAngle() - ((gyro.getXComplementaryAngle() > 0) ? 180 : -180));

        return pitchRotation2d;

    }

    public Rotation2d getRoll() {

        Rotation2d rollRotation2d = Rotation2d
                .fromDegrees(gyro.getYComplementaryAngle() - ((gyro.getYComplementaryAngle() > 0) ? 180 : -180));

        return rollRotation2d;

    }

    public void resetEncoders() {
        for (MAXSwerveModule mSwerveMod : swerveModules) {
            mSwerveMod.resetEncoders();
        }
    }

    public Command toggleSpeed() {
        return runOnce(() -> this.speedMultiplier = (this.speedMultiplier == 1) ? 0.35 : 1);
    }

    public void setSpeedMultiplier(double speedMultiplier) {
        this.speedMultiplier = speedMultiplier;
    }

    public double getSpeedMultiplier() {
        return this.speedMultiplier;
    }

    /**
     * Sets the brake mode for the drive motors.
     * This is useful for when the robot is enabled
     * So we can stop the robot quickly
     * (This is the default mode)
     */
    public void setBrakeMode() {
        for (MAXSwerveModule mSwerveMod : swerveModules) {
            mSwerveMod.setBrakeMode();
        }
    }

    /**
     * Sets the coast mode for the drive motors.
     * This is useful for when the robot is disabled
     * So we can freely move the robot around
     */
    public void setCoastMode() {
        for (MAXSwerveModule mSwerveMod : swerveModules) {
            mSwerveMod.setCoastMode();
        }
    }
}

// Refrenced from https://github.com/Stampede3630/2022-Code/blob/MK3Practice/src/main/java/frc/robot/SwerveTrajectory.java
package auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import hardware.Swerve;
import io.github.oblarg.oblog.Loggable;
import math.Constants;
import debug.*;

public class SwerveTrajectory implements Loggable {

  // Create config for trajectory
  public static double timetrajectoryStarted;
  public static String trajectoryStatus = "";

  public static double elapsedTime;

  /**
   * Returns a new HolonomicDriveController with constant PID gains.
   * <p>
   * The 2 PID controllers are controllers that should correct
   * for error in the field-relative x and y directions respectively.
   * i.e. X_CORRECTION_P,I,D is PID for X correction
   * and Y_CORRECTION_P,I,D is PID for Y correction
   * <p>
   * The ProfiledPIDController for the rotation of the robot,
   * utilizing a Trapezoidprofile for smooth locomotion in terms of max velocity and acceleration
   */
  public static HolonomicDriveController HDC = new HolonomicDriveController(
      new PIDController(Constants.AutoConstants.X_CORRECTION_P, Constants.AutoConstants.X_CORRECTION_I, Constants.AutoConstants.X_CORRECTION_D),
      new PIDController(Constants.AutoConstants.Y_CORRECTION_P, Constants.AutoConstants.Y_CORRECTION_I, Constants.AutoConstants.Y_CORRECTION_D),
      new ProfiledPIDController(Constants.AutoConstants.ROTATION_CORRECTION_P, Constants.AutoConstants.ROTATION_CORRECTION_I, Constants.AutoConstants.ROTATION_CORRECTION_D,
          new TrapezoidProfile.Constraints(Constants.AutoConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, Constants.AutoConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED)));

  /**
   * This is PathPlanner.
   * It's awesome :)
   * Open up pathplanner.exe on the driverstation laptop.
   * Point the application to the location of your coding project (must contain build.gradle).
   * Draw the path.
   * It will autosave.
   * If everything is characterized correctly and your odometry reflects reality,
   * i.e. when the robot goes 1 meter, it says it went one meter--
   * it will work like a charm.
   *
   * @param _pathTraj   run Pathplanner.loadpath("name of file without an extension") pass it here
   * @param _odometry   SwerveDrive.java's odometry
   * @param _rotation2d Pass in the current angle of the robot
   */
  public static void PathPlannerRunner(PathPlannerTrajectory _pathTraj, Swerve swerve, Pose2d _odometry, Rotation2d _rotation2d) {

    elapsedTime = Timer.getFPGATimestamp() - timetrajectoryStarted;

    switch (trajectoryStatus) {

      case "setup":
        timetrajectoryStarted = Timer.getFPGATimestamp();
        trajectoryStatus = "execute";
        break;

      case "execute":

        Debug.debugPeriodic(
            _pathTraj.sample(elapsedTime).poseMeters.getX() - _odometry.getX(),
            _pathTraj.sample(elapsedTime).poseMeters.getY() - _odometry.getY(),
            _pathTraj.sample(elapsedTime).poseMeters.getRotation().getDegrees() - _odometry.getRotation().getDegrees());

        // If the path has not completed time wise
        if (elapsedTime < _pathTraj.getEndState().timeSeconds + 1) {

          // Use elapsedTime as a refrence for where we NEED to be
          // Then, sample the position and rotation for that time,
          // And calculate the ChassisSpeeds required to get there
          ChassisSpeeds _speeds = HDC.calculate(
              _odometry,
              _pathTraj.sample(elapsedTime),
              ((PathPlannerState) _pathTraj.sample(elapsedTime)).holonomicRotation);

          // Set the states for the motor using calculated values above
          // It is important to note that fieldRelative is false,
          // but calculations make it, so it is true i.e. rotation is independent
          // (This is seen 6-5 lines above)
          swerve.drive(
              _speeds.vxMetersPerSecond,
              _speeds.vyMetersPerSecond,
              _speeds.omegaRadiansPerSecond, false);

        } else {

          swerve.drive(0, 0, 0, false);
          trajectoryStatus = "done";

        }

        break;

      default:

        swerve.drive(0, 0, 0, false);
        break;

    }
  }

  public static void resetTrajectoryStatus() {

    trajectoryStatus = "setup";

  }

}
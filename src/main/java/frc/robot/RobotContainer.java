package frc.robot;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;
import frc.robot.util.PatriBoxController;
import frc.robot.util.Constants.AutoConstants;
import frc.robot.util.Constants.DriveConstants;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.NeoMotorConstants;
import frc.robot.util.Constants.OIConstants;
import frc.robot.util.Constants.FieldConstants.GameMode;
import io.github.oblarg.oblog.Logger;

public class RobotContainer {

    private final PatriBoxController driver;
    private final PatriBoxController operator;

    private final Swerve swerve;
    private final Elevator elevator;
    private final Claw claw;

    public RobotContainer() {
        Logger.configureLoggingAndConfig(this, false);

        DriverStation.silenceJoystickConnectionWarning(true);

        driver = new PatriBoxController(OIConstants.DRIVER_CONTROLLER_PORT, OIConstants.DRIVER_DEADBAND);
        operator = new PatriBoxController(OIConstants.OPERATOR_CONTROLLER_PORT, OIConstants.OPERATOR_DEADBAND);

        swerve = new Swerve();
        elevator = new Elevator();
        claw = new Claw();

        swerve.setDefaultCommand(new Drive(
                swerve,
                driver::getLeftY,
                driver::getLeftX,
                () -> -driver.getRightX(),
                () -> !driver.y().getAsBoolean(),
                () -> !driver.y().getAsBoolean(),
                () -> (driver.y().getAsBoolean() && FieldConstants.ALLIANCE == Alliance.Blue)));

        incinerateMotors();
        configureButtonBindings();

        // Wait wait wait wait for the DS to connect
        // then assign our alliance color
        while (DriverStation.getAlliance() == Alliance.Invalid) {
            DriverStation.refreshData();
        }

        FieldConstants.ALLIANCE = DriverStation.getAlliance();

    }

    private void configureButtonBindings() {

        new Trigger(() -> FieldConstants.GAME_MODE == FieldConstants.GameMode.TELEOP
                || FieldConstants.GAME_MODE == FieldConstants.GameMode.TEST)
                .onTrue(
                        setDriveSpeed(DriveConstants.MAX_TELEOP_SPEED_METERS_PER_SECOND));

        
        new Trigger(() -> FieldConstants.GAME_MODE == FieldConstants.GameMode.AUTONOMOUS)
                .onTrue(
                        setDriveSpeed(DriveConstants.MAX_SPEED_METERS_PER_SECOND));

        driver.y().or(driver.x()).onTrue(
                Commands.runOnce(() -> swerve.resetOdometry(
                        new Pose2d(
                                swerve.getPose().getTranslation(),
                                Rotation2d.fromDegrees(
                                        FieldConstants.ALLIANCE == Alliance.Red
                                                ? 0
                                                : 180))),
                        swerve));

        driver.a().onTrue(
            setDriveSpeed(FieldConstants.ALIGNMENT_SPEED)
        ).onFalse(
            setDriveSpeed(DriveConstants.MAX_TELEOP_SPEED_METERS_PER_SECOND)
        );
        driver.leftBumper().whileTrue(
            swerve.getSetWheelsXCommand()
        );

        driver.leftTrigger(.5).whileTrue(
            claw.setSpeedCommand(driver.getLeftTriggerAxis())
        );

    }

    private CommandBase setDriveSpeed(double desiredSpeedMetersPerSecond) {
        return Commands.runOnce(() -> {
            DriveConstants.MAX_SPEED_METERS_PER_SECOND = desiredSpeedMetersPerSecond;
        });
    }

    public Command getAutonomousCommand() {
        // TODO: make auto cmd
        return null;
    }

    public Command getDisabledCommand() {
        return Commands.run(() -> FieldConstants.ALLIANCE = DriverStation.getAlliance()).ignoringDisable(true);
    }

    public void onEnabled(GameMode gameMode) {
        Commands.runOnce(() -> Robot.modeStartTimestamp = Robot.currentTimestamp)
                .andThen(Commands.runOnce(() -> FieldConstants.GAME_MODE = gameMode))
                .schedule();
    }

    private void incinerateMotors() {
        Timer.delay(0.25);
        for (CANSparkMax neo : NeoMotorConstants.motorList) {
            neo.burnFlash();
            Timer.delay(0.005);
        }
        Timer.delay(0.25);
    }
}

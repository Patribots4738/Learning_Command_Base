package frc.robot.util;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.OIConstants;

public class PatriBoxController extends CommandXboxController {

    private final double deadband;
    private final boolean isDriverController;

    public PatriBoxController(int port, double deadband) {
        super(port);

        isDriverController = (port == OIConstants.DRIVER_CONTROLLER_PORT);

        this.deadband = deadband;
    }

    @Override
    public double getLeftX() {
        return getLeftAxis().getX();
    }

    @Override
    public double getLeftY() {
        return getLeftAxis().getY();
    }

    public Translation2d getLeftAxis() {
        Translation2d leftAxis = toCircle(MathUtil.applyDeadband(super.getLeftX(), deadband),
                MathUtil.applyDeadband(super.getLeftY(), deadband));

        if (isDriverController && FieldConstants.ALLIANCE == Alliance.Blue) {
            leftAxis = leftAxis.unaryMinus();
        }

        return leftAxis;
    }

    @Override
    public double getRightX() {
        return getRightAxis().getX();
    }

    @Override
    public double getRightY() {
        return getRightAxis().getY();
    }

    public Translation2d getRightAxis() {
        return toCircle(MathUtil.applyDeadband(super.getRightX(), deadband),
                MathUtil.applyDeadband(super.getRightY(), deadband));
    }

    // All calculations can be referenced here
    // https://www.desmos.com/calculator/b4oo5nehle
    private Translation2d toCircle(double x, double y) {

        Translation2d intercept;
        Translation2d output;

        double slope = y / x;

        if (slope == 0 || Double.isNaN(slope) || Double.isInfinite(slope)) {

            output = new Translation2d(x, y);
            return output;

        } else if (0 > slope && slope >= -OIConstants.CONTROLLER_CORNER_SLOPE_2) {

            intercept = new Translation2d(-1, -slope);

        } else if (-OIConstants.CONTROLLER_CORNER_SLOPE_1 < slope && slope < -OIConstants.CONTROLLER_CORNER_SLOPE_2) {

            double intersectionX = (1.7) / (slope - 1);
            double intersectionY = (intersectionX + 1.7);

            intercept = new Translation2d(intersectionX, intersectionY);

        } else if (slope < -OIConstants.CONTROLLER_CORNER_SLOPE_1 || slope > OIConstants.CONTROLLER_CORNER_SLOPE_1) {

            intercept = new Translation2d(1 / slope, 1);

        } else if (OIConstants.CONTROLLER_CORNER_SLOPE_1 > slope && slope > OIConstants.CONTROLLER_CORNER_SLOPE_2) {

            double intersectionX = (1.7) / (slope + 1);
            double intersectionY = -intersectionX + 1.7;

            intercept = new Translation2d(intersectionX, intersectionY);

        } else if (0 < slope && slope <= OIConstants.CONTROLLER_CORNER_SLOPE_2) {

            intercept = new Translation2d(1, slope);

        } else {

            intercept = new Translation2d(0, 0);

            System.out.println("Error... Slope = " + slope);

        }

        double distance = getDistance(x, y, intercept.getX(), intercept.getY());

        distance = (distance > 1) ? 1 : distance;

        output = new Translation2d(Math.signum(x) * distance, new Rotation2d(Math.atan(y / x)));
        return output;
    }

    private double getDistance(double x1, double y1, double x2, double y2) {
        return Math.sqrt(
                (Math.pow(x1, 2) + Math.pow(y1, 2)) /
                        (Math.pow(x2, 2) + Math.pow(y2, 2)));
    }

    public Command setRumble(DoubleSupplier rumble) {
        return Commands.runOnce(() -> this.getHID().setRumble(RumbleType.kBothRumble, rumble.getAsDouble()));
    }

}

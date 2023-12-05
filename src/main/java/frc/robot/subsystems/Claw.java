package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Neo;

public class Claw extends SubsystemBase {

    private final CANSparkMax claw;
    private double desiredSpeed = 0;

    public Claw() {

        // TODO: set claw can id
        claw = new Neo(100);
        claw.restoreFactoryDefaults();

        claw.setSmartCurrentLimit(30);
        // See https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces
        claw.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        claw.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
        claw.setInverted(false);

        setBrakeMode();

    }

    @Override
    public void periodic() {
        set the current speed of the motors to the desired speed

    }

    private void setSpeed(double speed) {
        claw.set(-speed);
    }

    public void setBrakeMode() {
        claw.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void setCoastMode() {
        claw.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    public void setDesiredSpeed(double speed) {
        this.desiredSpeed = speed;
    }

    public void stopClaw() {
        this.desiredSpeed = 0;
    }

    public double getDesiredSpeed() {
        return this.desiredSpeed;
    }

    public Command setDesiredSpeedCommand(DoubleSupplier speed) {
        return null;
    }

}

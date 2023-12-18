package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Neo;

public class Claw extends SubsystemBase {

    private final CANSparkMax claw;

    public Claw() {

        claw = new Neo(1);
        claw.restoreFactoryDefaults();

        claw.setSmartCurrentLimit(30);
        // See https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces
        claw.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        claw.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
        claw.setInverted(false);

        setBrakeMode();

    }

    public void setSpeed(double speed) {
        claw.set(-speed);
    }

    public void setBrakeMode() {
        claw.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void setCoastMode() {
        claw.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }
}

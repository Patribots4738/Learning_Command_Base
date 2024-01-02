package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Neo;

public class Elevator extends SubsystemBase {
    private final Neo elevator;

    public Elevator() {
        elevator = new Neo(9);
        elevator.setPID(0.1, 0, 0);
    
        
    }

    public void setSpeed(double speed) {
        elevator.setPosition(speed);

    }


    public double getPosition() {
        return elevator.getPosition();
    }
}
package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.Neo;



public class Elevator extends SubsystemBase {
    private final Neo elevator;

    public Elevator() {
        elevator = new Neo(9);
        elevator.setPID(0.1, 0, 0); // 😳
    }

    @Override
    public void periodic() {
      // CommandScheduler.getInstance().run(); //This method will be called once per scheduler run
    }

   
    public Command motionUp(){
            return runOnce(() -> elevator.set(0.5));
    }

    public Command motionDown(){
            return runOnce(() -> elevator.set(-0.5));
    }
    public void limits(){
        double maxlimit = elevator.setrotation(0.8);
        double minlimit = elevator.setrotation(1.0);
    }

}
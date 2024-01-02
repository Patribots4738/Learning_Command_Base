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
        elevator.setSoftLimit(null, 0);
    }

    @Override
    public void periodic() {
      // CommandScheduler.getInstance().run(); //This method will be called once per scheduler run
    }

    private Command moveUpOrDown(boolean goingUp) {
        elevator.getPosition();
        if (goingUp) {
            return runOnce(() -> elevator.set(0.8));
        }
        else {
            return runOnce(() -> elevator.set(-0.8));
        }
    }
   
    public Command motionUp() {
        if(elevator.getPosition() >= 1) {
            return runOnce(() -> elevator.set(0.5));
        }
        else{
            return runOnce(() -> elevator.set(0));
        }
           // return moveUpOrDown(true);
    }
    
    public Command motionDown() {
        if (elevator.getPosition() >= 1 && elevator.getPosition() <= 9) {
            return runOnce(() -> elevator.set(-0.5));
        }
        else{
            return runOnce(() -> elevator.set(0));
        }
        //return moveUpOrDown(false);
    }
}
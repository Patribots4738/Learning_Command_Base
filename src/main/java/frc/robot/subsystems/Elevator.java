package frc.robot.subsystems;

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
    // This method will be called once per scheduler run
  }
}
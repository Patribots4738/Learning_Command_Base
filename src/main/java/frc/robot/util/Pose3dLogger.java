package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Log;

public class Pose3dLogger extends Logger{
    @Log
    private Pose3d pose = new Pose3d();

    public Pose3dLogger(Pose3d pose) {
        this.pose = pose;
    }

    public Pose3d getPose() {
        return this.pose;
    }

    public void setPose(Pose3d pose) {
        this.pose = pose;
    }
}   
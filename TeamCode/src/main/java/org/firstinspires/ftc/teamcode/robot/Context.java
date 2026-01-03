package org.firstinspires.ftc.teamcode.robot;

public interface Context {
    RampageRobot getRobot();
    void registerSequence(Sequence sequence);
    int getSequenceCount();
}

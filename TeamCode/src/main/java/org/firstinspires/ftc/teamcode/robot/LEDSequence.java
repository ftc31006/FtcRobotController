package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Objects;

public class LEDSequence implements Sequence {
    public LEDState state;
    public Integer frequency;
    private boolean hasNewValue = false;
    private boolean isOn = false;
    private ElapsedTime timer = new ElapsedTime();

    public void setState(LEDState state, Integer frequency) {
        if (this.state == state && Objects.equals(this.frequency, frequency)) {
            return;
        }

        this.hasNewValue = true;
        this.state = state;
        this.frequency = frequency;
    }

    @Override
    public boolean hasCompleted() {
        return false;
    }

    @Override
    public void executeFrame(Context context) {
        RampageRobot robot = context.getRobot();

        if (hasNewValue) {
            hasNewValue = false;
            robot.setAprilTagLEDState(state);
            isOn = true;
            timer.reset();
            return;
        }

        if (frequency == null) {
            return;
        }

        if (timer.milliseconds() > frequency) {
            isOn = !isOn;
            robot.setAprilTagLEDState(isOn ? state : LEDState.OFF);
            timer.reset();
        }
    }
}

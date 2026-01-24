package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Backup")
public class BackupAuto extends AutoBase {
    @Override
    protected double getFrontLeftPower() {
        return -1;
    }

    @Override
    protected double getFrontRightPower() {
        return 1;
    }

    @Override
    protected double getBackLeftPower() {
        return 1;
    }

    @Override
    protected double getBackRightPower() {
        return -1;
    }

    @Override
    protected boolean shouldShoot(){return false;}
}

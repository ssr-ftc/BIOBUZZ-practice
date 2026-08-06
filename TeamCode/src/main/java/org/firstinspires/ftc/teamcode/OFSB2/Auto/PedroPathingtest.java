package org.firstinspires.ftc.teamcode.OFSB2.Auto;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
@Autonomous

public class PedroPathingtest extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        //START POSITION_END POSITION
        //DRIVE > MOVEMENT STATE
        //SHOOT > ATTEMPT TO SCORE THE ARTIFACT

        DRIVE_STARTPOS_SHOOTPOS,
        SHOOT_PRELOAD

    }
    PathState pathState;

    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}


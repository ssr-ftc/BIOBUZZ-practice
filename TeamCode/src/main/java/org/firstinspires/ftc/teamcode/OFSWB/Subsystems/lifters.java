package org.firstinspires.ftc.teamcode.OFSWB.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class lifters {
    public Servo liftLeft;
    public Servo liftBack;
    public Servo liftRight;

    private ElapsedTime timer = new ElapsedTime();
    private int sequenceStep = 0;
    private boolean sequenceRunning = false;

    private static final double LEFT_UP = 0.43;
    private static final double LEFT_DOWN = 0.055;
    private static final double BACK_UP = 0.41;
    private static final double BACK_DOWN = 0.01;
    private static final double RIGHT_UP = 0.40;
    private static final double RIGHT_DOWN = 0.03;
    private static final double UP_HOLD_SECONDS = 0.25;   // 100 ms up before coming down
    private static final double DOWN_WAIT_SECONDS = 0.4;  // 300 ms down before next lift starts

    public lifters(HardwareMap hardwareMap) {
        liftLeft = hardwareMap.get(Servo.class, "lift_left");
        liftBack = hardwareMap.get(Servo.class, "lift_back");
        liftRight = hardwareMap.get(Servo.class, "lift_right");
        liftRight.setDirection(Servo.Direction.REVERSE);
    }

    // Call this once (e.g. on a button press) to kick off the full sequence:
    // left up -> wait 100ms -> left down -> wait 300ms ->
    // back up -> wait 100ms -> back down -> wait 300ms ->
    // right up -> wait 100ms -> right down -> wait 300ms -> done
    public void startSequence() {
        sequenceRunning = true;
        sequenceStep = 1;
        timer.reset();
        leftUp();
    }

    // Call this every loop() cycle. It checks elapsed time and advances the
    // sequence step by step, without blocking anything else.
    public void update() {
        if (!sequenceRunning) return;

        if (sequenceStep == 1 && timer.seconds() >= UP_HOLD_SECONDS) {
            leftDown();
            sequenceStep = 2;
            timer.reset();
        } else if (sequenceStep == 2 && timer.seconds() >= DOWN_WAIT_SECONDS) {
            backUp();
            sequenceStep = 3;
            timer.reset();
        } else if (sequenceStep == 3 && timer.seconds() >= UP_HOLD_SECONDS) {
            backDown();
            sequenceStep = 4;
            timer.reset();
        } else if (sequenceStep == 4 && timer.seconds() >= DOWN_WAIT_SECONDS) {
            rightUp();
            sequenceStep = 5;
            timer.reset();
        } else if (sequenceStep == 5 && timer.seconds() >= UP_HOLD_SECONDS) {
            rightDown();
            sequenceStep = 6;
            timer.reset();
        } else if (sequenceStep == 6 && timer.seconds() >= DOWN_WAIT_SECONDS) {
            sequenceRunning = false;
            sequenceStep = 0;
        }
    }

    public boolean isSequenceRunning() {
        return sequenceRunning;
    }

    public void allDown() {
        leftDown();
        backDown();
        rightDown();
        sequenceRunning = false;
        sequenceStep = 0;
    }

    public void leftUp() { liftLeft.setPosition(LEFT_UP); }
    public void leftDown() { liftLeft.setPosition(LEFT_DOWN); }
    public void backUp() { liftBack.setPosition(BACK_UP); }
    public void backDown() { liftBack.setPosition(BACK_DOWN); }
    public void rightUp() { liftRight.setPosition(RIGHT_UP); }
    public void rightDown() { liftRight.setPosition(RIGHT_DOWN); }
}
//hi
package org.firstinspires.ftc.teamcode.OFSWB.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OFSWB.Subsystems.intakelm2;

@TeleOp(name = "intake lm2 test", group = "tests")
public class intakelm2test extends OpMode {
    private intakelm2 intake;

    @Override
    public void init() {
        intake = new intakelm2(hardwareMap);
    }

    @Override
    public void start() {
        intake.start();

    }
    @Override
    public void loop() {
        if (gamepad1.dpadUpWasPressed()){
            intake.up_1();
        }
        if (gamepad1.dpadDownWasPressed()){
            intake.down_1();
        }
        if (gamepad1.circle){
            intake.intake_ball_height();
        }
        else if (gamepad1.triangle){
            intake.up();
        } else if (gamepad1.cross) {
            intake.down();
        }
        if (gamepad1.square) {
            intake.intake_on();
        }

        telemetry.addData("servo position",intake.getArmPosition());
    }


}
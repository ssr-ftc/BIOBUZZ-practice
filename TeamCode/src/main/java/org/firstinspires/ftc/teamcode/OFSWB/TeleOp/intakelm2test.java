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
        if (gamepad2.dpadUpWasPressed()){
            intake.up_1();
        }
        if (gamepad2.dpadDownWasPressed()){
            intake.down_1();
        }
        if (gamepad2.circle){
            intake.intake_ball_height();
        }
        else if (gamepad2.triangle){
            intake.up();
        } else if (gamepad2.cross) {
            intake.down();
        }

        if (intake.getArmPosition() > 0.21 && intake.getArmPosition() < 0.23) {
            telemetry.addLine("its at right height");

            if (gamepad2.rightBumperWasPressed()) {
                if (intake.isIntakeOn()) {
                    intake.turn_off_intake();
                } else {
                    intake.turn_on_intake();
                }
            }
        } else {
            if (intake.isIntakeOn()) {
                intake.turn_off_intake();
            }
        }

        if (gamepad2.left_bumper){
            intake.turn_off_intake();
        }
        if (gamepad2.square) {
            intake.outtake();
        }
        telemetry.addData("servo position",intake.getArmPosition());
    }


}
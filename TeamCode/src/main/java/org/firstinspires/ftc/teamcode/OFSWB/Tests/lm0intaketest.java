package org.firstinspires.ftc.teamcode.OFSWB.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OFSWB.Subsystems.LM0Subsystem;

@TeleOp(name = "LM0 Intake", group = "LM0")
public class lm0intaketest extends OpMode {

    private LM0Subsystem robot;

    @Override
    public void init() {
        robot = new LM0Subsystem(hardwareMap);

        telemetry.addData("Status", "LM0 Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad2.left_bumper) {
            robot.intake.outtake();
        } else if (gamepad2.right_bumper) {
            robot.intake.turn_on_intake();
        } else {
            robot.intake.turn_off_intake();
        }

        telemetry.addData("Intake", robot.intake.getMotorPower() < 0 ? "IN" : robot.intake.getMotorPower() > 0 ? "OUT" : "OFF");
        telemetry.addData("Left bumper", gamepad2.left_bumper);
        telemetry.addData("Right bumper", gamepad2.right_bumper);
        telemetry.addData("Motor power", robot.intake.getMotorPower());
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stopAll();
    }
}

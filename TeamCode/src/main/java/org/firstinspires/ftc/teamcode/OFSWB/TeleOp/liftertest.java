package org.firstinspires.ftc.teamcode.OFSWB.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OFSWB.Subsystems.lifters;

@TeleOp(name = "lifters test", group = "tests")
public class liftertest extends OpMode {
    private lifters lift;

    @Override
    public void init() {
        lift = new lifters(hardwareMap);
    }

    @Override
    public void start() {
        lift.allDown();
    }

    @Override
    public void loop() {
        lift.update();

        if (gamepad2.triangleWasPressed()) {
            lift.startSequence();
        }

        telemetry.addData("left position", lift.liftLeft.getPosition());
        telemetry.addData("back position", lift.liftBack.getPosition());
        telemetry.addData("right position", lift.liftRight.getPosition());
        telemetry.update();
    }
}
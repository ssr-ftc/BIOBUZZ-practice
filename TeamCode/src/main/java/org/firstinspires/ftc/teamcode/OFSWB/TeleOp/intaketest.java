package org.firstinspires.ftc.teamcode.OFSWB.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.OFSWB.Subsystems.intake;

@TeleOp(name = "intake test")
public class intaketest extends LinearOpMode {

    private intake intakeSubsystem;

    @Override
    public void runOpMode() {
        intakeSubsystem = new intake(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad2.leftBumperWasPressed()) {
                if (intakeSubsystem.isIntakeOn()) {
                    intakeSubsystem.turn_off_intake();
                } else {
                    intakeSubsystem.turn_on_intake();
                }
            }
            if (gamepad2.rightBumperWasPressed()) {
                intakeSubsystem.outtake();
            }
        }
    }
}
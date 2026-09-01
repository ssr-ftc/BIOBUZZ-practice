package org.firstinspires.ftc.teamcode.OFSWB.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.OFSWB.Subsystems.depo;

@TeleOp(name = "depo test")
public class depotest extends LinearOpMode {
    @Override
    public void runOpMode() {
        depo depositSubsystem = new depo(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad2.crossWasPressed()) {
                if (depositSubsystem.isDepositOn()) {
                    depositSubsystem.turn_off_deposit();
                } else {
                    depositSubsystem.turn_on_deposit();
                }
            }

            telemetry.addData("Depo!", depositSubsystem.isDepositOn());
            telemetry.update();
        }
    }
}
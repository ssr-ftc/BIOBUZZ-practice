package org.firstinspires.ftc.teamcode.OFSWB.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.OFSWB.Subsystems.limitswitchlm2;

@TeleOp(name = "Limit Switch Test")
public class limitswitchlm2test extends LinearOpMode {

    @Override
    public void runOpMode() {
        limitswitchlm2 limitSwitch = new limitswitchlm2(hardwareMap, "limitswitch");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Pressed", limitSwitch.isPressed());
            telemetry.update();
        }
    }
}
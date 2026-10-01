package org.firstinspires.ftc.teamcode.DepoCH;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.OFSB2.Auto.Constants;

@TeleOp(name = "Depo Test", group = "TeleOp")
public class DepoTest extends OpMode {
    private Follower follower;

    private DcMotorEx depo1;

    private DcMotorEx depo2;
    double power = 0.3;


    @Override
    public void init() {
        depo1 = hardwareMap.get(DcMotorEx.class, "depo1");
        depo2 = hardwareMap.get(DcMotorEx.class, "depo2");
    }


    @Override
    public void loop() {


        if (gamepad1.dpad_up) {
            power = power + 0.1;
        }
        if (gamepad1.dpad_down) {
            power = power + 0.1;
        }

        if (gamepad1.right_bumper) {
            depo1.setPower(power);
        } else {
            depo1.setPower(0);
        }
        if (gamepad1.left_bumper) {
            depo2.setPower(-power);
        } else {
            depo2.setPower(0);
        }

        telemetry();
    }


    public void telemetry() {
        telemetry.addData("Depo Speed:", power);
    }


    @Override
    public void stop() {

    }
}







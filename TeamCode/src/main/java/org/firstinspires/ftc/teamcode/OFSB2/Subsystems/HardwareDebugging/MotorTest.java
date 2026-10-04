package org.firstinspires.ftc.teamcode.OFSB2.Subsystems.HardwareDebugging;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "MotorTest", group = "TeleOp")
public class MotorTest extends OpMode {
    private DcMotorEx rfmotor, lfmotor, lbmotor, rbmotor;
    final double MAX_DRIVE_VELOCITY = 2787.625;

    @Override
    public void init() {
        rfmotor = hardwareMap.get(DcMotorEx.class, "rfmotor");
        lfmotor = hardwareMap.get(DcMotorEx.class, "lfmotor");
        lbmotor = hardwareMap.get(DcMotorEx.class, "lbmotor");
        rbmotor = hardwareMap.get(DcMotorEx.class, "rbmotor");

        lbmotor.setDirection(DcMotor.Direction.REVERSE);
        lfmotor.setDirection(DcMotor.Direction.REVERSE);

        lfmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rfmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rbmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lfmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        rfmotor.setVelocity(0.1 * MAX_DRIVE_VELOCITY);
        lfmotor.setVelocity(0.1 * MAX_DRIVE_VELOCITY);
        lbmotor.setVelocity(0.1 * MAX_DRIVE_VELOCITY);
        rbmotor.setVelocity(0.1 * MAX_DRIVE_VELOCITY);
    }

    }

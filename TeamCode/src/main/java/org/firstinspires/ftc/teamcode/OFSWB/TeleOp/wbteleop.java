package org.firstinspires.ftc.teamcode.OFSWB.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OFSWB.Subsystems.intake;
import org.firstinspires.ftc.teamcode.OFSWB.Subsystems.depo;
import org.firstinspires.ftc.teamcode.OFSWB.Subsystems.lifters;

@TeleOp(name = "wb teleop", group = "tests")
public class wbteleop extends OpMode {

    private DcMotor lfmotor;
    private DcMotor lbmotor;
    private DcMotor rfmotor;
    private DcMotor rbmotor;

    private intake intake;
    private depo depo;
    private lifters lifters;
    private double speedScale = 0.8;

    private ElapsedTime shotTimer = new ElapsedTime();
    private int shotStep = 0;
    private boolean shooting = false;

    private static final double warmup_seconds = 0.75;
    private static final double up_hold_seconds = 0.4;
    private static final double down_wait_seconds = 0.6;

    @Override
    public void init() {
        lfmotor = hardwareMap.get(DcMotor.class, "lfmotor");
        lbmotor = hardwareMap.get(DcMotor.class, "lbmotor");
        rfmotor = hardwareMap.get(DcMotor.class, "rfmotor");
        rbmotor = hardwareMap.get(DcMotor.class, "rbmotor");

        lfmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        lbmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rfmotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rbmotor.setDirection(DcMotorSimple.Direction.FORWARD);

        intake = new intake(hardwareMap);
        depo = new depo(hardwareMap);
        lifters = new lifters(hardwareMap);
    }

    @Override
    public void start() {
        lifters.allDown();
    }

    private void driveMecanum() {
        double forward = -gamepad1.left_stick_y * speedScale;
        double strafe = gamepad1.left_stick_x * speedScale;
        double turn = gamepad1.right_stick_x * speedScale;

        lfmotor.setPower(forward + strafe + turn);
        lbmotor.setPower(forward - strafe + turn);
        rfmotor.setPower(forward - strafe - turn);
        rbmotor.setPower(forward + strafe - turn);
    }

    @Override
    public void loop() {
        driveMecanum();

        if (gamepad2.rightBumperWasPressed() && !shooting) {
            if (intake.isIntakeOn()) {
                intake.turn_off_intake();
            } else {
                intake.turn_on_intake();
            }
        }

        if (gamepad2.triangleWasPressed() && !shooting) {
            intake.turn_off_intake();
            depo.turn_on_deposit();
            shooting = true;
            shotStep = 0;
            shotTimer.reset();
        }

        if (shooting) {
            if (shotStep == 0 && shotTimer.seconds() >= warmup_seconds) {
                lifters.leftUp();
                shotStep = 1;
                shotTimer.reset();
            } else if (shotStep == 1 && shotTimer.seconds() >= up_hold_seconds) {
                lifters.leftDown();
                shotStep = 2;
                shotTimer.reset();
            } else if (shotStep == 2 && shotTimer.seconds() >= down_wait_seconds) {
                lifters.backUp();
                shotStep = 3;
                shotTimer.reset();
            } else if (shotStep == 3 && shotTimer.seconds() >= up_hold_seconds) {
                lifters.backDown();
                shotStep = 4;
                shotTimer.reset();
            } else if (shotStep == 4 && shotTimer.seconds() >= down_wait_seconds) {
                lifters.rightUp();
                shotStep = 5;
                shotTimer.reset();
            } else if (shotStep == 5 && shotTimer.seconds() >= up_hold_seconds) {
                lifters.rightDown();
                shotStep = 6;
                shotTimer.reset();
            } else if (shotStep == 6 && shotTimer.seconds() >= down_wait_seconds) {
                shooting = false;
                shotStep = 0;
                depo.turn_off_deposit();
            }
        }

        telemetry.addData("intake on", intake.isIntakeOn());
        telemetry.addData("depo on", depo.isDepositOn());
        telemetry.addData("shooting", shooting);
    }
}
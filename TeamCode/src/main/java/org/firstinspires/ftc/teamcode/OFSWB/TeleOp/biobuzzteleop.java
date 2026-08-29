package org.firstinspires.ftc.teamcode.OFSWB.TeleOp;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.OFSWB.Subsystems.intakelm2;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@TeleOp(name = "biobuzz teleop", group = "tests")
public class biobuzzteleop extends OpMode {
    private Follower follower;
    private intakelm2 intake;
    private double speedScale = 0.8;

    @Override
    public void init() {
        intake = new intakelm2(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

    }
    private void driveWithPedro() {

        double forward = -gamepad1.left_stick_y * speedScale;
        double strafe = -gamepad1.left_stick_x * speedScale;
        double turn = -gamepad1.right_stick_x * speedScale;

        follower.setTeleOpDrive(forward, strafe, turn, true);
    }


    @Override
    public void start() {
        intake.start();
        follower.startTeleopDrive();

    }
    @Override
    public void loop() {
        follower.update();
        driveWithPedro();
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
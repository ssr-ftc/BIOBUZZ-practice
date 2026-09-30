package org.firstinspires.ftc.teamcode.OFSB1.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OFSB1.Constants;
import org.firstinspires.ftc.teamcode.OFSB1.Subsystems.OFSB1Subsystem;
import org.firstinspires.ftc.teamcode.OFSB1.Vision.APRILTAG_Biobuzz.Alliance;
import org.firstinspires.ftc.teamcode.OFSB1.Vision.APRILTAG_Biobuzz.AllianceConfig;
import org.firstinspires.ftc.teamcode.OFSB1.Vision.APRILTAG_Biobuzz.BioBuzzVision;
import org.firstinspires.ftc.teamcode.OFSB1.Vision.APRILTAG_Biobuzz.ShootingTarget;
import org.firstinspires.ftc.teamcode.OFSB1.Vision.APRILTAG_Biobuzz.VisionAssist;
import org.firstinspires.ftc.teamcode.OFSB1.Vision.APRILTAG_Biobuzz.VisionState;

/**
 * BIOBUZZ TeleOp. Driving matches OFSB1 TeleOp. HIVE vision only supplies
 * a target; it does not path the robot to the other side of the field.
 *
 * INIT (before START):
 *   Circle / B  -> RED
 *   Square / X  -> BLUE
 * The choice locks when the match starts.
 *
 * MATCH:
 *   Left stick Y     forward / back
 *   L2 / R2          strafe left / right
 *   Right stick X    turn (also overrides vision aim while held)
 *   Right bumper     toggle vision-aim assist
 *   D-pad down       toggle raw-tag debug telemetry
 *
 * Assist only adds a turn toward a valid upward cell. It never drives
 * forward or sideways, and it never fires a shooter.
 *
 * Shootable means a same-alliance cell is facing up. That is true from
 * any spot on the audience side or the scoring side where the camera
 * can see that cell's tags. The robot does not need a specific pose.
 */
@TeleOp(name = "BIOBUZZ Hive Vision", group = "OFSB1")
public class BioBuzzHiveTeleOp extends OpMode {

    private Follower follower;
    private OFSB1Subsystem robot;
    private BioBuzzVision vision;
    private VisionAssist assist;

    private boolean debug;
    private boolean rightBumperPrev;
    private boolean dpadDownPrev;

    @Override
    public void init() {
        AllianceConfig.reset();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        robot = new OFSB1Subsystem(hardwareMap);

        vision = new BioBuzzVision();
        vision.init(hardwareMap, telemetry);
        assist = new VisionAssist();

        telemetry.addData("Status", "Select alliance, then START");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if (gamepad1.b) {
            AllianceConfig.select(Alliance.RED);
        } else if (gamepad1.x) {
            AllianceConfig.select(Alliance.BLUE);
        }

        vision.update(getRuntime());

        telemetry.addLine("Alliance: Circle/B = RED, Square/X = BLUE");
        telemetry.addData("Selected", AllianceConfig.isSelected()
                ? AllianceConfig.get().name()
                : "NONE - pick before START");
        vision.addTelemetry(telemetry, true);
        telemetry.update();
    }

    @Override
    public void start() {
        AllianceConfig.lock();
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        vision.update(getRuntime());

        boolean rightBumper = gamepad1.right_bumper && !rightBumperPrev;
        boolean dpadDown = gamepad1.dpad_down && !dpadDownPrev;
        rightBumperPrev = gamepad1.right_bumper;
        dpadDownPrev = gamepad1.dpad_down;

        if (rightBumper) {
            assist.setEnabled(!assist.isEnabled());
        }
        if (dpadDown) {
            debug = !debug;
        }

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_trigger - gamepad1.right_trigger;
        double driverTurn = -gamepad1.right_stick_x;

        VisionState state = vision.getState();
        double turn = assist.turnPower(state, gamepad1.right_stick_x, driverTurn);
        follower.setTeleOpDrive(forward, strafe, turn, true);

        if (state.hasValidTarget()) {
            ShootingTarget target = state.getTarget();
            double distance = target.getDistance();
            double bearing = target.getBearing();
            // Distance and bearing are available for a future shooter.
            // Nothing here launches a game piece.
            telemetry.addData("Aim distance", "%.1f", distance);
            telemetry.addData("Aim bearing", "%.1f", bearing);
        }

        telemetry.addData("Assist", assist.getMode().name());
        telemetry.addData("Odometry X", follower.getPose().getX());
        telemetry.addData("Odometry Y", follower.getPose().getY());
        telemetry.addData("Odometry heading", Math.toDegrees(follower.getPose().getHeading()));
        vision.addTelemetry(telemetry, debug);
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stopAll();
        if (vision != null) {
            vision.stop();
        }
    }
}

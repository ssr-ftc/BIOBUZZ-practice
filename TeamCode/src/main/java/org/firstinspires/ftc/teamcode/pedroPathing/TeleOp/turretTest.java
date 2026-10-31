package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.DepoLM1;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.LM0.depoLM0;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.LM0.intakeLM0;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.turret;

@TeleOp(name = "turret test w/ pidf", group = "TeleOp")
public class turretTest extends OpMode{

    private turret turret;


    @Override
    public void init() {
        turret = new turret(hardwareMap);
        turret.ResetTurretEncoders();
    }

    @Override
    public void loop() {
        turret.run_using_pid();

        if (gamepad1.aWasPressed()) turret.setPositionDeg(90);
       if (gamepad1.bWasPressed()) turret.setPositionDeg(-90);
       if (gamepad1.yWasPressed()) turret.setPositionDeg(0);

       if (gamepad1.dpadUpWasPressed()) turret.setPositionDeg(turret.getPositionDeg() + 25);
       if (gamepad1.dpadDownWasPressed()) turret.setPositionDeg(turret.getPositionDeg() - 25);
       if (gamepad1.dpadRightWasPressed()) turret.ResetTurretEncoders();

       telemetry.addData("position", turret.getPositionDeg());
       telemetry.update();

    }
}

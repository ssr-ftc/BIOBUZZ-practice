package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = org.firstinspires.ftc.teamcode.OFSB1.Constants.followerConstants;

    public static PathConstraints pathConstraints = org.firstinspires.ftc.teamcode.OFSB1.Constants.pathConstraints;


    public static Follower createFollower(HardwareMap hardwareMap) {
        return org.firstinspires.ftc.teamcode.OFSB1.Constants.createFollower(hardwareMap);
    }
}

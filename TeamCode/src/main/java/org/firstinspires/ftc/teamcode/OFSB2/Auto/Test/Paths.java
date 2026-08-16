package org.firstinspires.ftc.teamcode.OFSB2.Auto.Test;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class Paths {
    public final Pose start = new Pose(70.85831960461286, 134.72981878088962, Math.toRadians(90));
    public final Pose shootPos = new Pose(47.40449664771886, 70.25789384267753, Math.toRadians(90));
    public final Pose sample1 = new Pose(93.91480375429161, 40.852017956911546, Math.toRadians(360));
    public final Pose sample2 = new Pose(23.31136738056013, 47.088962108731465, Math.toRadians(55));
    public final Pose returnMid = new Pose(71.20195460911026, 99.05104624197372, Math.toRadians(55));
    public final Pose end = new Pose(70.85831960461286, 129.72981878088962, Math.toRadians(90));

    public final PathChain toShoot;
    public final PathChain shootToSample1;
    public final PathChain sample1ToSample2;
    public final PathChain sample2ToReturnMid;
    public final PathChain returnMidToEnd;

    public Paths(Follower follower) {
        toShoot = follower.pathBuilder()
                .addPath(new BezierLine(start, shootPos))
                .setLinearHeadingInterpolation(start.getHeading(), shootPos.getHeading())
                .build();
        shootToSample1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPos, sample1))
                .setLinearHeadingInterpolation(shootPos.getHeading(), sample1.getHeading())
                .build();
        sample1ToSample2 = follower.pathBuilder()
                .addPath(new BezierLine(sample1, sample2))
                .setLinearHeadingInterpolation(sample1.getHeading(), sample2.getHeading())
                .build();
        sample2ToReturnMid = follower.pathBuilder()
                .addPath(new BezierLine(sample2, returnMid))
                .setLinearHeadingInterpolation(sample2.getHeading(), returnMid.getHeading())
                .build();
        returnMidToEnd = follower.pathBuilder()
                .addPath(new BezierLine(returnMid, end))
                .setLinearHeadingInterpolation(returnMid.getHeading(), end.getHeading())
                .build();
    }
}

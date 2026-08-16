package org.firstinspires.ftc.teamcode.OFSB2.Auto.TestCurveTangent;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class PiecewiseUTurnPaths {

    public final PathChain mainChain;

    /*
     * I temporarily commented these 5 out. In Java, if a variable is marked "final",
     * it MUST be assigned a value in the constructor. Since you aren't building
     * these 5 chains yet, commenting them out stops Java from throwing an error.
     */
    // public final PathChain toShoot;
    // public final PathChain shootToSample1;
    // public final PathChain sample1ToSample2;
    // public final PathChain sample2ToReturnMid;
    // public final PathChain returnMidToEnd;

    public PiecewiseUTurnPaths(Follower follower) {
        mainChain = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(18.702, 125.956),
                                new Pose(73.717, 96.363),
                                new Pose(4.745, 96.529),
                                new Pose(35.118, 79.094),
                                new Pose(38.983, 55.348)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-45))
                .addPath(
                        new BezierCurve(
                                new Pose(38.983, 55.348),
                                new Pose(55.568, 30.826),
                                new Pose(81.500, 88.754),
                                new Pose(58.964, 126.822)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-45))
                .addPath(
                        new BezierCurve(
                                new Pose(58.964, 126.822),
                                new Pose(47.664, 140.226),
                                new Pose(39.627, 104.602)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-45))
                .addPath(
                        new BezierCurve(
                                new Pose(39.627, 104.602),
                                new Pose(10.917, 76.689),
                                new Pose(34.344, 117.379),
                                new Pose(18.879, 125.764)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-45))
                .build();
    }
}
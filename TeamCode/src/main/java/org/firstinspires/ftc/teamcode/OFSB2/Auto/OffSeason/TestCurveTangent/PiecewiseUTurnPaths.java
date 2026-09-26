package org.firstinspires.ftc.teamcode.OFSB2.Auto.OffSeason.TestCurveTangent;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.api.Paths;

public class PiecewiseUTurnPaths {

    public final Path mainChain;

    public PiecewiseUTurnPaths(Follower follower) {
        mainChain = com.pedropathing.api.Paths.path(
                com.pedropathing.api.Paths.curve(
                                new Pose(18.702, 125.956, 0),
                                new Pose(73.717, 96.363, 0),
                                new Pose(4.745, 96.529, 0),
                                new Pose(35.118, 79.094, 0),
                                new Pose(38.983, 55.348, 0)
                        )
                        .linear(Math.toRadians(-45), Math.toRadians(-45)),
                com.pedropathing.api.Paths.curve(
                                new Pose(38.983, 55.348, 0),
                                new Pose(55.568, 30.826, 0),
                                new Pose(81.500, 88.754, 0),
                                new Pose(58.964, 126.822, 0)
                        )
                        .linear(Math.toRadians(-45), Math.toRadians(-45)),
                com.pedropathing.api.Paths.curve(
                                new Pose(58.964, 126.822, 0),
                                new Pose(47.664, 140.226, 0),
                                new Pose(39.627, 104.602, 0)
                        )
                        .linear(Math.toRadians(-45), Math.toRadians(-45)),
                com.pedropathing.api.Paths.curve(
                                new Pose(39.627, 104.602, 0),
                                new Pose(10.917, 76.689, 0),
                                new Pose(34.344, 117.379, 0),
                                new Pose(18.879, 125.764, 0)
                        )
                        .linear(Math.toRadians(-45), Math.toRadians(-45))
        );
    }
}

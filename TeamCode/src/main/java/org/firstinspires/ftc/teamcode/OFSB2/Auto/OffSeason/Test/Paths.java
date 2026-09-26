package org.firstinspires.ftc.teamcode.OFSB2.Auto.OffSeason.Test;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class Paths {
    public final Pose start = new Pose(70.85831960461286, 134.72981878088962, Math.toRadians(90));
    public final Pose shootPos = new Pose(47.40449664771886, 70.25789384267753, Math.toRadians(90));
    public final Pose sample1 = new Pose(93.91480375429161, 40.852017956911546, Math.toRadians(360));
    public final Pose sample2 = new Pose(23.31136738056013, 47.088962108731465, Math.toRadians(55));
    public final Pose returnMid = new Pose(71.20195460911026, 99.05104624197372, Math.toRadians(55));
    public final Pose end = new Pose(70.85831960461286, 129.72981878088962, Math.toRadians(90));

    public final Path toShoot;
    public final Path shootToSample1;
    public final Path sample1ToSample2;
    public final Path sample2ToReturnMid;
    public final Path returnMidToEnd;

    public Paths(Follower follower) {
        toShoot = com.pedropathing.api.Paths.line(start, shootPos)
                .linear(start.heading(), shootPos.heading());
        shootToSample1 = com.pedropathing.api.Paths.line(shootPos, sample1)
                .linear(shootPos.heading(), sample1.heading());
        sample1ToSample2 = com.pedropathing.api.Paths.line(sample1, sample2)
                .linear(sample1.heading(), sample2.heading());
        sample2ToReturnMid = com.pedropathing.api.Paths.line(sample2, returnMid)
                .linear(sample2.heading(), returnMid.heading());
        returnMidToEnd = com.pedropathing.api.Paths.line(returnMid, end)
                .linear(returnMid.heading(), end.heading());
    }
}

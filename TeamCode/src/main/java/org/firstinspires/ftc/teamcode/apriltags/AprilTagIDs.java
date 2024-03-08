package org.firstinspires.ftc.teamcode.apriltags;

import com.acmerobotics.roadrunner.Vector2d;
import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.enums.Location;
import java.util.HashMap;

public class AprilTagIDs {
    public static Backdrop blueBackdrop = new Backdrop(Alliance.BLUE, 1, 2, 3);
    public static Backdrop redBackdrop = new Backdrop(Alliance.RED, 4, 5, 6);
    private final HashMap<Integer, Vector2d> tagPoses;
    public AprilTagIDs() {
        tagPoses = new HashMap<>();
        double backdropX = 58.5+.5;
        tagPoses.put(blueBackdrop.LEFT, new Vector2d(backdropX, 42));
        tagPoses.put(blueBackdrop.CENTER, new Vector2d(backdropX, 29));
        tagPoses.put(blueBackdrop.RIGHT, new Vector2d(backdropX, 19));
        tagPoses.put(redBackdrop.LEFT, tagPoses.get(blueBackdrop.RIGHT));
        tagPoses.put(redBackdrop.CENTER, tagPoses.get(blueBackdrop.CENTER));
        tagPoses.put(redBackdrop.RIGHT, tagPoses.get(blueBackdrop.LEFT));
    }
    public static Backdrop getBackdrop(Alliance alliance) {
        if(alliance == Alliance.BLUE)
            return blueBackdrop;
        else
            return redBackdrop;
    }
    public static Object[] getLocation(int id) {
        Backdrop backdrop;
        if(blueBackdrop.contains(id))
            backdrop = blueBackdrop;
        else if(redBackdrop.contains(id))
            backdrop = redBackdrop;
        else return null;

        Location location = backdrop.getLocation(id);

        return new Object[]{backdrop, location};
    }
    public Vector2d getPose(int id) {
        return tagPoses.get(id);
    }
    public boolean contains(int id) {
        return blueBackdrop.contains(id) || redBackdrop.contains(id);
    }
}
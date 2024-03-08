package org.firstinspires.ftc.teamcode.apriltags;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.enums.Location;

public class Backdrop {
    public Alliance alliance;
    public int LEFT, CENTER, RIGHT;
    public Backdrop(Alliance alliance, int l, int c, int r) {
        this.alliance = alliance;
        LEFT = l;
        CENTER = c;
        RIGHT = r;
    }
    public int getId(Location location) {
        switch (location) {
            case LEFT:
                return LEFT;
            case CENTER:
                return CENTER;
            case RIGHT:
                return RIGHT;
        }
        return 0;
    }
    public Location getLocation(int id) {
        if (id == LEFT)
            return Location.LEFT;
        else if (id == CENTER)
            return Location.CENTER;
        else if (id == RIGHT)
            return Location.RIGHT;
        else
            return null;
    }
    public boolean contains(int id) {
        return getLocation(id) != null;
    }
    @NonNull
    public String toString() {
        return alliance.toString();
    }
}

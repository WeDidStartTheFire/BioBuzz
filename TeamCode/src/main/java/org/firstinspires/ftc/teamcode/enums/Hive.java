package org.firstinspires.ftc.teamcode.enums;

public enum Hive {
    BLUE_AUDIENCE(new int[]{38, 39, 40, 41}),
    BLUE_STAGE(new int[]{42, 43, 44, 45}),
    RED_AUDIENCE(new int[]{4, 5, 6, 7}),
    RED_STAGE(new int[]{0, 1, 2, 3});

    private final int[] aprilTags;

    Hive(int[] aprilTags) {
        this.aprilTags = aprilTags;
    }

    public static boolean ifTagMatchesHive(int tagID, Hive hive) {
        for (int tag : hive.aprilTags) {
            if (tag == tagID) return true;
        }
        return false;
    }
}
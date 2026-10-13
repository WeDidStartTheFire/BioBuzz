package org.firstinspires.ftc.teamcode.enums;

public enum Hive {
    BLUE_AUDIENCE(new int[]{38, 39, 40, 41}, Color.BLUE),
    BLUE_STAGE(new int[]{42, 43, 44, 45}, Color.BLUE),
    RED_AUDIENCE(new int[]{34, 35, 36, 37}, Color.RED),
    RED_STAGE(new int[]{30, 31, 32, 33}, Color.RED);

    private final int[] aprilTags;
    private final Color color;

    Hive(int[] aprilTags, Color color) {
        this.aprilTags = aprilTags;
        this.color = color;
    }

    public static boolean ifTagMatchesHive(int tagID, Hive hive) {
        for (int tag : hive.aprilTags) if (tag == tagID) return true;
        return false;
    }

    public static Color matchTagToColor(int tagID) {
        for (Hive hive : values()) if (ifTagMatchesHive(tagID, hive)) return hive.color;
        return null;
    }
}
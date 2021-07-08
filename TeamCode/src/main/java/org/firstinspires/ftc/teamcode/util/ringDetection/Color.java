package org.firstinspires.ftc.teamcode.util.ringDetection;

public enum Color {
    RED(255, 0, 0),
    GREEN(0, 255, 0),
    BLUE(0, 0, 255),
    ORANGE(255, 127, 39),
    YELLOW(255, 255, 0),
    CYAN(0, 255, 255),
    MAGENTA(255, 0, 255),
    PURPLE(163, 73, 164),
    BLACK(0, 0, 0),
    WHITE(255, 255, 255),
    GRAY(127, 127, 127);

    private double R, G, B;

    private Color(double R, double G, double B) {
        this.R = R;
        this.G = G;
        this.B = B;
    }

    public double toChannel1() {
        return R * 0.299 + G * 0.587 + B * 0.114;
    }
    public double[] toChannel3() {
        return new double[] {R, G, B};
    }
    public double[] toChannel4() {
        return new double[] {R, G, B, 0};
    }

    public boolean isThisColor(double[] RGB, double similarity) {
        double zone = 255 - 255 * similarity;
        if (Math.abs(RGB[0] - this.R) <= zone && Math.abs(RGB[1] - this.G) <= zone && Math.abs(RGB[2] - this.B) <= zone)
            return true;
        return false;
    }

    public boolean isThisColor(double[] RGB) {
        return isThisColor(RGB, 0.85);
    }

    public static boolean isRingColor(double[] RGB) {
        if (RGB[0]/RGB[1] > 1 && RGB[0]/RGB[1] < 4 && RGB[1]/RGB[2] > 1.2 && RGB[1]/RGB[2] < 6) return true;
        return false;
    }
}
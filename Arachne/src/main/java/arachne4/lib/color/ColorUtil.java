package arachne4.lib.color;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.util.Color;

public class ColorUtil {
    public static float[] getNormalisedHsv(Color color) {
        return java.awt.Color.RGBtoHSB((int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255), null);
    }

    public static Color fromHsv(float[] normalisedHsv) {
        // We go via the awt Color class because WPI's implementation gives completely wrong hues
        var awtColor = java.awt.Color.getHSBColor(normalisedHsv[0], normalisedHsv[1], normalisedHsv[2]);

        return new Color(awtColor.getRed(), awtColor.getGreen(), awtColor.getBlue());
    }

    public static Color shift(Color color, float hueOffset, double saturationMultiplier, double valueMultiplier) {
        float[] normalisedHsv = getNormalisedHsv(color);

        normalisedHsv[0] = (float) MathUtil.inputModulus(normalisedHsv[0] + hueOffset, 0, 1);
        normalisedHsv[1] = (float) MathUtil.clamp(normalisedHsv[1] * saturationMultiplier, 0, 1);
        normalisedHsv[2] = (float) MathUtil.clamp(normalisedHsv[2] * valueMultiplier, 0, 1);

        return fromHsv(normalisedHsv);
    }
}

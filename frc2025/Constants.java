package team10034.frc2025;

public final class Constants {
    public static final class PID {
        public static final double[] kP = {0.0036,0.00025,0.0036,0.0036};
    }

    public static final class ConvertConstants {
        public static final double kEncoderResolution = 4096; // 移到這裡
        public static final double kEncoderValuePerDegree = kEncoderResolution / 360;

        public static double kCANcoderValueToDegrees(double currentValue, double canCoderMagOffset) {
            // TODO
            return 0;
        }

        public static double kAtan2To360(double atan2Value) {
            double atan2Degrees = Math.toDegrees(atan2Value);
            atan2Degrees = -atan2Degrees + 90; // 先轉成順時針的0~180 / -180~0
            if (atan2Degrees < 0) {
                atan2Degrees += 360; // 轉換為0~360範圍
            }
            return atan2Degrees;
        }        
    }
}
package redbacks.lib.hardware.vision.limelight;

import java.nio.ByteBuffer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

public record MegatagData(
        Pose2d fieldRelativePose,
        double timestampSeconds,
        double latency,
        double averageDistanceFromTag,
        int[] visibleTags) implements StructSerializable {

    private static final int
        MIN_TAG_ID = 1,
        MAX_TAG_ID = 16;

    public static final MegatagDataStruct struct = new MegatagDataStruct();

    public static class MegatagDataStruct implements Struct<MegatagData> {
        @Override
        public Class<MegatagData> getTypeClass() {
            return MegatagData.class;
        }

        @Override
        public String getTypeString() {
            return "struct:MegatagData";
        }

        @Override
        public int getSize() {
            return Pose2d.struct.getSize() + kSizeDouble * 3 + 2;
        }

        @Override
        public String getSchema() {
            return "Pose2d fieldRelativePose;double timestampSeconds;double latency;double averageDistanceFromTag;uint16 visibleTagBits";
        }

        @Override
        public Struct<?>[] getNested() {
            return new Struct<?>[] { Pose2d.struct };
        }

        @Override
        public MegatagData unpack(ByteBuffer bb) {
            return new MegatagData(Pose2d.struct.unpack(bb), bb.getDouble(), bb.getDouble(), bb.getDouble(), fromTagBitRepresenation(bb.getShort()));
        }

        @Override
        public void pack(ByteBuffer bb, MegatagData value) {
            Pose2d.struct.pack(bb, value.fieldRelativePose);
            bb.putDouble(value.timestampSeconds);
            bb.putDouble(value.latency);
            bb.putDouble(value.averageDistanceFromTag);
            bb.putShort(toTagBitRepresenation(value.visibleTags));
        }

        private static short toTagBitRepresenation(int[] visibleTags) {
            short bitRep = 0;

            for (int tagId : visibleTags) bitRep |= 1 << (tagId - MIN_TAG_ID);

            return bitRep;
        }

        private static int[] fromTagBitRepresenation(short tagBitRepresenation) {
            // Count number of 1 bits
            int nTagsSeen = 0;
            for (int tagId = MIN_TAG_ID, bits = tagBitRepresenation; tagId <= MAX_TAG_ID; tagId++) {
                if ((bits & 1) != 0) nTagsSeen++;
                bits >>>= 1;
            }

            // Fill array with tags seen
            int[] tagsSeen = new int[nTagsSeen];
            for (int tagId = MIN_TAG_ID, i = 0; tagId <= MAX_TAG_ID; tagId++) {
                if ((tagBitRepresenation & 1) != 0) tagsSeen[i++] = tagId;
                tagBitRepresenation >>>= 1;
            }

            return tagsSeen;
        }
    }
}

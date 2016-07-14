package steurer.infineon.com.flightcontroller_gui;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by SteurerE on 03.02.2015.
 * <p/>
 * Class for implementing a Communication Protocol
 * based on byte-streams.
 * <p/>
 * It's for Communication with a XMC4500 Microcontroller
 * over a Bluetooth Serial Connection
 */

public class BluetoothProtocol {

    public static final int PACKET_SIZE = 19; //19 bytes for the Protocol
    public static final int DATA_SIZE = 14; //14 bytes data for data-package
    public static final int HEADER_SIZE = 1; //1 byte header
    public static final int CHECKSUM_SIZE = 4; //4 bytes checksum
    public static final byte CONTROL_HEADER = 0x00;

    public static byte[] prepareControlPackage(ControlPacket packet) {
        byte[] b = new byte[PACKET_SIZE];
        byte speed = packet.getSpeed();
        byte heightcontrol = packet.getHeightcontrol();
        int azimuth = Float.floatToIntBits(packet.getAzimuth());
        int pitch = Float.floatToIntBits(packet.getPitch());
        int roll = Float.floatToIntBits(packet.getRoll());

        int checksum = CONTROL_HEADER;
        checksum ^= (heightcontrol << 8 | speed) & 0xFFFF;
        checksum ^= azimuth;
        checksum ^= pitch;
        checksum ^= roll;

        b[0] = CONTROL_HEADER;

        b[1] = (byte) (heightcontrol & 0xFF);
        b[2] = (byte) (speed & 0xFF);

        b[3] = (byte) ((azimuth >> 24) & 0xFF);
        b[4] = (byte) ((azimuth >> 16) & 0xFF);
        b[5] = (byte) ((azimuth >> 8) & 0xFF);
        b[6] = (byte) (azimuth & 0xFF);

        b[7] = (byte) ((pitch >> 24) & 0xFF);
        b[8] = (byte) ((pitch >> 16) & 0xFF);
        b[9] = (byte) ((pitch >> 8) & 0xFF);
        b[10] = (byte) (pitch & 0xFF);

        b[11] = (byte) ((roll >> 24) & 0xFF);
        b[12] = (byte) ((roll >> 16) & 0xFF);
        b[13] = (byte) ((roll >> 8) & 0xFF);
        b[14] = (byte) (roll & 0xFF);

        b[15] = (byte) ((checksum >> 24) & 0xFF);
        b[16] = (byte) ((checksum >> 16) & 0xFF);
        b[17] = (byte) ((checksum >> 8) & 0xFF);
        b[18] = (byte) (checksum & 0xFF);

        return b;
    }

    public static List<byte[]> prepareDataPackages(DataPacket packet) {
        String all_data = packet.getData();
        List<String> packetlist = new ArrayList<String>();
        List<byte[]> bytelist = new ArrayList<byte[]>();
        int mod = 0;
        if (all_data.length() > DATA_SIZE) {
            int length = all_data.length() / DATA_SIZE;
            mod = all_data.length() % DATA_SIZE;
            if (mod == 0) {
                for (int i = 0; i < length; i++) {
                    packetlist.add(all_data.substring(i * DATA_SIZE, (i + 1) * DATA_SIZE));
                }
            } else {
                for (int i = 0; i < (length + 1); i++) {
                    if (i == length) {
                        String s = all_data.substring(i * DATA_SIZE, i * DATA_SIZE + mod);
                        char[] stringbuffer = s.toCharArray();
                        char[] last_packet = new char[DATA_SIZE];
                        System.arraycopy(stringbuffer, 0, last_packet, 0, mod);
                        String last_packet_s = new String(last_packet);
                        packetlist.add(last_packet_s);
                    } else {
                        packetlist.add(all_data.substring(i * DATA_SIZE, (i + 1) * DATA_SIZE));
                    }
                }
            }
        } else {
            mod = all_data.length();
            char[] stringbuffer = all_data.toCharArray();
            char[] last_packet = new char[DATA_SIZE];
            System.arraycopy(stringbuffer, 0, last_packet, 0, stringbuffer.length);
            String last_packet_s = new String(last_packet);
            packetlist.add(last_packet_s);
        }
        int start_byte_count = 0;
        if (packetlist.size() == 1) {
            start_byte_count = mod;
        } else if (mod == 0) {
            start_byte_count = DATA_SIZE * packetlist.size();
        } else if (mod != 0) {
            start_byte_count = DATA_SIZE * (packetlist.size() - 1) + mod;
        }
        for (int i = 0; i < packetlist.size(); i++) {
            byte[] dest = new byte[PACKET_SIZE];
            byte[] src = packetlist.get(i).getBytes();
            byte[] header = new byte[1];
            header[0] = (byte) start_byte_count;
            System.arraycopy(header, 0, dest, 0, 1);
            System.arraycopy(src, 0, dest, 1, src.length);
            bytelist.add(dest);
            start_byte_count = start_byte_count - DATA_SIZE;
        }
        for (int i = 0; i < bytelist.size(); i++) {
            byte[] checksum_byte = new byte[CHECKSUM_SIZE];
            byte[] checksum_helper = bytelist.get(i);
            int checksum = checksum_helper[0];
            for (int j = HEADER_SIZE; j < PACKET_SIZE - CHECKSUM_SIZE; j += 4) {
                checksum ^= (checksum_helper[j] << 24) | (checksum_helper[j + 1] << 16) | (checksum_helper[j + 2] << 8) | (checksum_helper[j + 3]);
            }
            checksum_byte[0] = (byte) ((checksum >> 24) & 0xFF);
            checksum_byte[1] = (byte) ((checksum >> 16) & 0xFF);
            checksum_byte[2] = (byte) ((checksum >> 8) & 0xFF);
            checksum_byte[3] = (byte) (checksum & 0xFF);
            System.arraycopy(checksum_byte, 0, checksum_helper, PACKET_SIZE - CHECKSUM_SIZE, checksum_byte.length);
        }
        return bytelist;
    }
}

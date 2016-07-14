package steurer.infineon.com.flightcontroller_gui;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.List;
import java.util.Set;
import java.util.UUID;


/**
 * Created by steurere on 25.03.2016.
 */
public class DroneCommunicator extends Thread {

    private enum ReceiveState {RECEIVE_BYTES, NO_DATA};

    //This is the well-known default UUID for bluetooth SPP-Devices(used for encryption of endpoint)
    private static final String BLUETOOTH_UUID_STRING = "00001101-0000-1000-8000-00805F9B34FB";
    private static final String BLUETOOTH_MODUL_NAME = "XMC-Bluetooth";
    private static final int BLUETOOTH_HEARTBEAT_INTERVAL  = 1000;

    //some codes for communication over handler
    public static final int SHOW_TOAST = 100;
    public static final int LOST_BLUETOOTH = 99;
    public static final int TOO_MANY_HOSTS = 98;
    public static final int PAIRED_BUT_NOT_CONNECTABLE = 91;
    public static final int NO_HOST = 93;
    public static final int DISCONNECT = 97;
    public static final int RECEIVE_BYTES = 94;
    public static final int SEND_CONTROL_DATA = 96;
    public static final int SEND_DATA_PACKET = 95;
    public static final int CONNECTED = 92;

    private Handler uiHandler;
    private BluetoothAdapter btAdapter;
    private BluetoothSocket mXMCSocket;
    private InputStream mXMCInputStream;
    private OutputStream mXMCOutputStream;
    private ControlPacket myControlPacket;
    private DataPacket myDataPacket;

    private ReceiveState recstate = ReceiveState.NO_DATA;

    private boolean connected = false;

    public DroneCommunicator(Handler uiHandler, BluetoothAdapter btAdapter) {
        this.uiHandler = uiHandler;
        this.btAdapter = btAdapter;
    }


    private void delay(int ms){
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {}
    }

    /**
     * while loop for bluetooth communication
     * read messages from Controller
     */
    public void run() {
        try {
            createDroneConnection();
        } catch (IOException e) {}//doesn't get run anyway
        while (connected) {
            delay(BLUETOOTH_HEARTBEAT_INTERVAL);//wait specified heartbeat-interval
            if (receiveBytes() == false) {
                sendState(LOST_BLUETOOTH);
            }
        }
    }


    public boolean isConnected(){
    return connected;
    }

    public Handler getBTHandler() {
        return btHandler;
    }

    private void sendState(int message) {
        Bundle myBundle = new Bundle();
        myBundle.putInt("message", message);
        sendBundle(myBundle);
    }

    private void sendBundle(Bundle myBundle) {
        Message myMessage = btHandler.obtainMessage();
        myMessage.setData(myBundle);
        uiHandler.sendMessage(myMessage);
    }

    private void sendToast(String toastText) {
        Bundle myBundle = new Bundle();
        myBundle.putInt("message", SHOW_TOAST);
        myBundle.putString("toastText", toastText);
        sendBundle(myBundle);
    }

    private void createDroneConnection() throws IOException {
        BluetoothDevice dev = null;
        Set<BluetoothDevice> list = btAdapter.getBondedDevices();
        int xmcHosts = 0;
        for (BluetoothDevice device : list) {
            if (device.getName().equals(BLUETOOTH_MODUL_NAME)) {
                dev = device;
                xmcHosts++;
            }
        }
        if (xmcHosts > 1) {
            sendState(TOO_MANY_HOSTS);
        } else if (dev == null) {
            sendState(NO_HOST);
        } else {
            try {
                BluetoothSocket tempBTSocket = null;
                try {
                    tempBTSocket = dev.createRfcommSocketToServiceRecord(UUID.fromString(BLUETOOTH_UUID_STRING));
                    tempBTSocket.connect();
                } catch (IOException e) {
                    if (tempBTSocket != null) {
                       sendState(PAIRED_BUT_NOT_CONNECTABLE);
                    }
                    return;
                }
                //get neccessary references
                mXMCSocket = tempBTSocket;
                mXMCInputStream = tempBTSocket.getInputStream();
                mXMCOutputStream = tempBTSocket.getOutputStream();
                connected = true;
                sendState(CONNECTED);
            } catch (IOException e) {
                throw e;
            }
        }
    }

    public boolean closeDroneConnection(){
        try {
            if (mXMCSocket != null) {
                connected = false;
                mXMCSocket.close();
                mXMCSocket = null;
                mXMCInputStream = null;
                mXMCOutputStream = null;
                return true;
            }

        } catch (IOException e) {}
        return false;
    }

    /**
     * Receive Keep-Alive Messages and return false if nothing has arrived
     * Closes connection if nothing arrives
     * Todo: Add Handling of messages from Drone
     */
    private boolean receiveBytes(){
           int receivedBytes = 0;
           int bytes = 0;
           byte[] buffer = new byte[128];
           if (mXMCInputStream != null) {
               try {
                   while (mXMCInputStream.available() > 0) {
                       bytes = mXMCInputStream.read(buffer);
                       receivedBytes += bytes;
                   }
                } catch (IOException e) {
                   sendState(LOST_BLUETOOTH);
                }
               if (receivedBytes == 0) {
                    return false;
               }
           }
        return true;
    }

    private void sendControlPacket(byte[] controlbytes){
        try {
            mXMCOutputStream.write(controlbytes);
        } catch (IOException e) {
            sendState(LOST_BLUETOOTH);
        }
    }

    private void sendDataPacket(List<byte[]> databytes){
        try {
            for (int i = 0; i < databytes.size(); i++) {
                mXMCOutputStream.write(databytes.get(i));
            }
        } catch (IOException e) {
            sendState(LOST_BLUETOOTH);
        }
    }


    final Handler btHandler = new Handler() {
        @Override
        public void handleMessage(Message myMessage) {
            int message;
            switch (message = myMessage.getData().getInt("message")) {
                case DISCONNECT:
                    //send message only when connection was established before
                    if(closeDroneConnection()) {
                        sendToast("Connection closed");
                    }
                    break;
                case SEND_CONTROL_DATA:
                    myControlPacket = (ControlPacket)myMessage.obj;
                    if(connected) {
                        sendControlPacket(BluetoothProtocol.prepareControlPackage(myControlPacket));
                    }
                    break;
                case SEND_DATA_PACKET:
                    myDataPacket = (DataPacket)myMessage.obj;
                    sendDataPacket(BluetoothProtocol.prepareDataPackages(myDataPacket));
                    break;
                case RECEIVE_BYTES:
                    recstate = ReceiveState.RECEIVE_BYTES;
                    break;
            }
        }
    };

}

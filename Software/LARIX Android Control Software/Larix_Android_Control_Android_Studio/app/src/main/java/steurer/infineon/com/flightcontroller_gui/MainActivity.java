package steurer.infineon.com.flightcontroller_gui;

import android.app.Activity;
import android.app.ProgressDialog;
import android.bluetooth.BluetoothAdapter;
import android.content.Context;
import android.content.Intent;
import android.graphics.Color;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.util.Log;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.RelativeLayout;
import android.widget.Toast;
import java.util.List;

public class MainActivity extends Activity implements SurfaceHolder.Callback, SensorEventListener, View.OnClickListener {

    private static final String DEBUG_TAG = "DEBUG";
    private static final int ENABLE_BLUETOOTH = 1;
    private boolean mHeightControlActivated = false;
    private View mViewholder;
    private SurfaceHolder mSurfaceholder;
    private SurfaceView mSurfaceView;
    private ProgressDialog mProgressBar;
    private CustomSliderViewVertical mSeekBarTrottle;
    private CustomSliderView mSeekBarYaw;
    private Button mHeightControlButton;
    private DroneCommunicator mDroneCommunicator;
    private SensorManager mSensorManager;
    private SensorFusion mSensorFusion;

    //reference of Handler Object of DroneCommunicator Class
    private Handler droneHandler;
    private ControlPacket mControlPacket;
    private Handler mFusionUpdateHandler = new Handler();//handler to inform activity of new fused data

    private float[] mSensorValues = new float[]{(float) 0.0, (float) 0.0, (float) 0.0};
    private float[] mViewDimensions = new float[]{(float) 0.0, (float) 0.0};

    //For Visualisation to get the SensorValues
    public float[] getSensorValues() {
        return mSensorValues;
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        registerSensors();
        //Instantiate Sensor-Fusion Class
        mSensorFusion = new SensorFusion(mFusionUpdateHandler, doSensorValuesUpdate);
        mSensorFusion.setMode(SensorFusion.Mode.FUSION);
        initBluetooth();
    }

    //Methods to get informed about underlying changes at Surface-View
    @Override
    public void surfaceCreated(SurfaceHolder holder) {
        mViewDimensions[1] = holder.getSurfaceFrame().height() / 1.45f;
        mViewDimensions[0] = holder.getSurfaceFrame().width() / 10.885f;
    }

    @Override
    public void surfaceChanged(SurfaceHolder holder, int format, int w, int h) {
    }


    @Override
    public void surfaceDestroyed(SurfaceHolder holder) {
    }


    private void registerSensors() {
        SensorManager sm = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        this.mSensorManager = sm;
        boolean failflag = false;

        if (mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER) == null) {
            showToast("Accelerometer missing");
            failflag = true;
        }

        if (mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD) == null) {
            showToast("Magnetometer missing");
            failflag = true;
        }

        if (mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE) != null) {
            mSensorManager.registerListener(this, mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER), SensorManager.SENSOR_DELAY_FASTEST);
            mSensorManager.registerListener(this, mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE), SensorManager.SENSOR_DELAY_FASTEST);
            mSensorManager.registerListener(this, mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD), SensorManager.SENSOR_DELAY_FASTEST);
        } else if (mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED) != null) {
            mSensorManager.registerListener(this, mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER), SensorManager.SENSOR_DELAY_FASTEST);
            mSensorManager.registerListener(this, mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED), SensorManager.SENSOR_DELAY_FASTEST);
            mSensorManager.registerListener(this, mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD), SensorManager.SENSOR_DELAY_FASTEST);
        } else {
            showToast("Gyroscope missing");
            failflag = true;
        }

        if (failflag) {
            showToast("SensorFusion will not work properly!");
        }
    }

    /**
     * Debug Method - Get all Sensors available on target Device
     */
    private void queryAllSensors() {
        SensorManager sm = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        List<Sensor> allSensors = sm.getSensorList(Sensor.TYPE_ALL);
        Log.d(DEBUG_TAG, "LOG-Message");
        for (Sensor sensor : allSensors) {
            Log.d(DEBUG_TAG, "Sensor-Name: " + sensor.getName() + "  Vendor: " + sensor.getVendor() + "  Resolution:" + sensor.getResolution());
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
    }


    @Override
    public void onSensorChanged(SensorEvent event) {
        if (mSensorFusion != null) {
            switch (event.sensor.getType()) {
                case Sensor.TYPE_ACCELEROMETER:
                    mSensorFusion.setAccel(event.values);
                    mSensorFusion.calculateAccMagOrientation();
                    break;
                case Sensor.TYPE_GYROSCOPE:
                    mSensorFusion.gyroFunction(event);
                    break;
                case Sensor.TYPE_MAGNETIC_FIELD:
                    mSensorFusion.setMagnet(event.values);
                    break;
            }
        }
    }

    /**
     * Creates a new Thread to communicate to Drone via Bluetooth Connection
     */
    private void createDroneCommunicator() {
        mControlPacket = new ControlPacket();
        mDroneCommunicator = new DroneCommunicator(myHandler, BluetoothAdapter.getDefaultAdapter());
        droneHandler = mDroneCommunicator.getBTHandler();
    }

    /**
     * Starts the Communicator to manage Bluetooth Connection to Drone
     */
    public void startDroneCommunicator() {
        mProgressBar = new ProgressDialog(MainActivity.this);
        //prevent ending fullscreen-mode
        mProgressBar.getWindow().setFlags(WindowManager.LayoutParams.FLAG_NOT_FOCUSABLE, WindowManager.LayoutParams.FLAG_NOT_FOCUSABLE);
        mProgressBar.setMessage("Connecting to Larix-Drone...");
        mProgressBar.show();
        if (mDroneCommunicator != null) {
            mDroneCommunicator.closeDroneConnection();
        }
        createDroneCommunicator();
        mDroneCommunicator.start();
    }

    public void stopDroneCommunicator() {
        if (mDroneCommunicator != null) {
            clearHandlerMessageQueue();
            sendDroneMessage(DroneCommunicator.DISCONNECT, 0);
        }
    }

    public boolean isConnected() {
        if (mDroneCommunicator != null) {
            return mDroneCommunicator.isConnected();
        }
        return false;
    }

    private void clearHandlerMessageQueue() {
        droneHandler.removeCallbacksAndMessages(null);
    }

    private void sendBTDataPacket(DataPacket datapacket, int message) {
        Bundle bundle = new Bundle();
        bundle.putInt("message", DroneCommunicator.SEND_DATA_PACKET);
        Message msg = myHandler.obtainMessage();
        msg.setData(bundle);
        msg.obj = datapacket;
        droneHandler.sendMessage(msg);
    }

    private void sendBTControlPacket(ControlPacket controlpacket, int message) {
        Bundle bundle = new Bundle();
        bundle.putInt("message", DroneCommunicator.SEND_CONTROL_DATA);
        Message msg = myHandler.obtainMessage();
        msg.setData(bundle);
        msg.obj = controlpacket;
        droneHandler.sendMessage(msg);
    }

    private void sendDroneMessage(int message, int delay) {
        Bundle bundle = new Bundle();
        bundle.putInt("message", message);
        Message droneMessage = droneHandler.obtainMessage();
        droneMessage.setData(bundle);
        if (delay == 0) {
            droneHandler.sendMessage(droneMessage);
        } else {
            droneHandler.sendMessageDelayed(droneMessage, delay);
        }
    }

    /**
     * will get called by sensorfusion-class
     * and informs about new fused data with a period of approx. 20ms
     * triggers also sending of data to drone
     */
    private Runnable doSensorValuesUpdate = new Runnable() {
        @Override
        public void run() {
            if (mSensorFusion != null) {
                mSensorValues[0] = mSensorFusion.getPitch();
                mSensorValues[1] = -mSensorFusion.getRoll();
                mSensorValues[2] = mSensorFusion.getAzimuth();
                if (mDroneCommunicator != null && mDroneCommunicator.isConnected()) {
                    mControlPacket.setRoll(mSensorValues[1]);
                    mControlPacket.setPitch(mSensorValues[0]);
                    mControlPacket.setAzimuth((float) mSeekBarYaw.getScaledValue());
                    mControlPacket.setSpeed((byte) mSeekBarTrottle.getScaledValue());
                    sendBTControlPacket(mControlPacket, DroneCommunicator.SEND_CONTROL_DATA);
                }
            }
        }
    };

    private void setSystemUIVisibility(View view) {
        int mUIFlag = View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN
                | View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION
                | View.SYSTEM_UI_FLAG_LAYOUT_STABLE
                | View.SYSTEM_UI_FLAG_LOW_PROFILE
                | View.SYSTEM_UI_FLAG_FULLSCREEN
                | View.SYSTEM_UI_FLAG_HIDE_NAVIGATION
                | View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY;
        view.setSystemUiVisibility(mUIFlag);
    }

    private void initBluetooth() {
        BluetoothAdapter ba = BluetoothAdapter.getDefaultAdapter();
        if (!ba.isEnabled()) {
            Intent intent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
            startActivityForResult(intent, ENABLE_BLUETOOTH);
        } else {
            initializeUI();
        }
    }

    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        if (requestCode == ENABLE_BLUETOOTH) {
            if (resultCode == RESULT_OK) {
                initializeUI();
            } else {
                Intent intent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
                startActivityForResult(intent, ENABLE_BLUETOOTH);
            }
        }
    }

    /**
     * initializes the user-interface with default values
     * and adds the needed references to gui-objects
     */
    private void initializeUI() {
        setContentView(R.layout.activity_main);
        RelativeLayout layout = (RelativeLayout) findViewById(R.id.relative);
        layout.setBackgroundColor(Color.rgb(255, 255, 255));
        mViewholder = getWindow().getDecorView();
        mHeightControlButton = (Button) findViewById(R.id.button_height_control);
        mHeightControlButton.setOnClickListener(this);
        mSeekBarTrottle = (CustomSliderViewVertical) findViewById(R.id.slider1);
        mSeekBarTrottle.setResourceIds(R.drawable.slider_front, R.drawable.speed);
        mSeekBarTrottle.setRange(0, 100);
        mSeekBarTrottle.setScaledValue(0);
        mSeekBarTrottle.setStartPos(0);
        mSeekBarYaw = (CustomSliderView) findViewById(R.id.slider2);
        mSeekBarYaw.setResourceIds(R.drawable.slider_front1, R.drawable.rotation);
        mSeekBarYaw.setRange(90,-90);//-90,90
        mSeekBarYaw.setScaledValue(0);
        mSeekBarYaw.setStartPos(0);
        //sets flags to get fullscreen-mode
        setSystemUIVisibility(mViewholder);
        //Get Reference to SurfaceView on which Visualisation takes place
        mSurfaceView = (SurfaceView) findViewById(R.id.visualisationView);
        mSurfaceholder = (SurfaceHolder) mSurfaceView.getHolder();
        mSurfaceholder.addCallback(this);
        mViewholder.setOnSystemUiVisibilityChangeListener(new View.OnSystemUiVisibilityChangeListener() {
            @Override
            public void onSystemUiVisibilityChange(int visibility) {
                setSystemUIVisibility(mViewholder);
            }
        });
    }

    //for Height Control Click
    public void onClick(View v) {
        Button b = (Button) v;
        if (mHeightControlActivated == false) {
            b.setBackgroundColor(Color.parseColor("green"));
            mHeightControlActivated = true;
        } else {
            b.setBackgroundColor(Color.parseColor("white"));
            mHeightControlActivated = false;
        }
    }


    @Override
    public void onWindowFocusChanged(boolean hasFocus) {
        super.onWindowFocusChanged(hasFocus);
    }

    @Override
    protected void onResume() {
        super.onResume();
        if (mSensorFusion == null) {
            registerSensors(); //Re-Register the Sensors
            mSensorFusion = new SensorFusion(mFusionUpdateHandler, doSensorValuesUpdate);
            mSensorFusion.setMode(SensorFusion.Mode.FUSION);
        }
        setSystemUIVisibility(getWindow().getDecorView());
    }

    @Override
    protected void onStop() {
        super.onStop();
        if (mSensorFusion != null) {
            mFusionUpdateHandler.removeCallbacks(doSensorValuesUpdate);
            mSensorFusion.getFuseTimer().cancel();
            mSensorFusion = null;
        }
        mSensorManager.unregisterListener(this);
        if (mDroneCommunicator != null) {
            clearHandlerMessageQueue();
            sendDroneMessage(DroneCommunicator.DISCONNECT, 0);
        }
    }

    @Override
    protected void onPause() {
        super.onPause();
        if (mSensorFusion != null) {
            mFusionUpdateHandler.removeCallbacks(doSensorValuesUpdate);
            mSensorFusion.getFuseTimer().cancel();
            mSensorFusion = null;
        }
        mSensorManager.unregisterListener(this);
        if (mDroneCommunicator != null) {
            clearHandlerMessageQueue();
            sendDroneMessage(DroneCommunicator.DISCONNECT, 0);
        }

    }


    public void showToast(final String value) {
        runOnUiThread(new Runnable() {
            public void run() {
                Toast t = Toast.makeText(MainActivity.this, value, Toast.LENGTH_SHORT);
                t.show();
            }
        });
    }

    /**
     * Receive messages from DroneCommunicator
     */
    final Handler myHandler = new Handler() {
        @Override
        public void handleMessage(Message myMessage) {
            switch (myMessage.getData().getInt("message")) {
                case DroneCommunicator.SHOW_TOAST:
                    showToast(myMessage.getData().getString("toastText"));
                    break;
                case DroneCommunicator.LOST_BLUETOOTH:
                    stopDroneCommunicator();
                    showToast("Lost Bluetooth Connection");
                    break;
                case DroneCommunicator.NO_HOST:
                    Intent intentOpenBluetoothSettings = new Intent();
                    intentOpenBluetoothSettings.setAction(android.provider.Settings.ACTION_BLUETOOTH_SETTINGS);
                    startActivity(intentOpenBluetoothSettings);
                    showToast("Pair with XMC-Bluetooth!");
                    break;
                case DroneCommunicator.TOO_MANY_HOSTS:
                    Intent OpenBluetoothSettings = new Intent();
                    OpenBluetoothSettings.setAction(android.provider.Settings.ACTION_BLUETOOTH_SETTINGS);
                    startActivity(OpenBluetoothSettings);
                    showToast("Too many Hosts! Only 1 allowed");
                    break;
                case DroneCommunicator.PAIRED_BUT_NOT_CONNECTABLE:
                    showToast("Establishing Connection failed!");
                    break;
                case DroneCommunicator.CONNECTED:
                    showToast("Connected");
                    break;
            }
            mProgressBar.dismiss();
            setSystemUIVisibility(getWindow().getDecorView());
        }
    };
}

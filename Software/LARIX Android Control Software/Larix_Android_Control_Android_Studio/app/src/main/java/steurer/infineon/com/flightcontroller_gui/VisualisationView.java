package steurer.infineon.com.flightcontroller_gui;

import android.content.Context;
import android.content.res.Resources;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.drawable.Drawable;
import android.os.Handler;
import android.os.Message;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.SurfaceHolder;
import android.view.SurfaceView;


/**
 * Created by SteurerE on 23.03.2015.
 */
public class VisualisationView extends SurfaceView implements SurfaceHolder.Callback {

    private MainActivity mainActivity;
    private int distLeft = 0;
    private int distTop = 0;
    private int distRight = 0;
    private int distBottom = 0;

    class AnimationThread extends Thread{

        private static final int REFRESH_RATE = 130;
        private static final int COPTER_SIZE = 200;

        //reference to itself
        private AnimationThread mAnimationThread;
        private Drawable mObject;
        private Bitmap mBackgroundImage;
        private SurfaceHolder mSurfaceHolder;
        private int mElapsedSinceDraw = 0;
        private int[] measuredSize;
        private float[] mSensorValues;
        private boolean mRun;
        private long mLastTimeMeasure;

        public AnimationThread(SurfaceHolder holder, Context context, Handler handler){
            mAnimationThread = this;
            mSurfaceHolder = holder;
            mainActivity = (MainActivity) context;
            Resources res = context.getResources();
            mObject = context.getResources().getDrawable(R.drawable.quadro);
            mBackgroundImage = BitmapFactory.decodeResource(res,
                    R.drawable.background_2);
            mSensorValues = new float[] {(float)0.0,(float)0.0,(float)0.0};
            measuredSize = new int[]{0,0};
        }

        public int[] getMeasuredSize() {
            return measuredSize;
        }

        public void setMeasuredSize(int measuredsizeX,int measuredsizeY) {
            this.measuredSize[0] = measuredsizeX;
            this.measuredSize[1] = measuredsizeY;
            mBackgroundImage = Bitmap.createScaledBitmap(mBackgroundImage,measuredsizeY,measuredsizeX,false);
        }

        /**
         *get Sensor-Values when needed for Animation
         *no need for registering another Sensor-Listener
        */
        private void updateSensorValues() {
            if(mainActivity != null) {
                    mSensorValues = mainActivity.getSensorValues();
            }
        }

        @Override
        public void run(){
            while (mRun) {
                if(updateTime()) {
                    updateSensorValues();
                    lockCanvasAndDraw(mSensorValues);
                }
            }
        }

        public void setRunning(boolean b) {
            mRun = b;
        }

        public void lockCanvasAndDraw(float[] sensorvalues) {
            Canvas c = null;
            try {
                synchronized (mSurfaceHolder) {
                    c = mSurfaceHolder.lockCanvas(null);
                    doDraw(c,sensorvalues);
                }
            } finally {
                if (c != null) {
                    mSurfaceHolder.unlockCanvasAndPost(c);
                    mElapsedSinceDraw = 0;
                }
            }
        }

        private void doDraw(Canvas mCanvas,float[] sensorvalues) {
            if(updateTime()){
                if (mObject != null && mCanvas != null && mBackgroundImage != null){
                    mCanvas.drawBitmap(mBackgroundImage, 0, 0, null);

                    distLeft = (int)(((-sensorvalues[0]+90.f) / 180.f) * (float)measuredSize[1]) - COPTER_SIZE / 2;
                    distTop = (int)(((sensorvalues[1]+90.f) / 180.f) * (float)measuredSize[0]) - COPTER_SIZE / 2;
                    distRight = (int)(((-sensorvalues[0]+90.f) / 180.f) * (float)measuredSize[1]) + COPTER_SIZE / 2;
                    distBottom = (int)(((sensorvalues[1]+90.f) / 180.f) * (float)measuredSize[0])+ COPTER_SIZE / 2;

                    if(distLeft <= 0){
                        distLeft = 0;
                        distRight = COPTER_SIZE;
                    }

                    if(distRight >= mCanvas.getWidth()){
                        distRight = mCanvas.getWidth();
                        distLeft = mCanvas.getWidth() - COPTER_SIZE;
                    }

                    if(distTop <= 0){
                        distTop = 0;
                        distBottom = COPTER_SIZE;
                    }

                    if(distBottom >= mCanvas.getHeight()){
                        distBottom = mCanvas.getHeight();
                        distTop = mCanvas.getHeight() - COPTER_SIZE;
                    }

                    mObject.setBounds(distLeft, distTop,distRight, distBottom);
                    mObject.draw(mCanvas);
                }
            }
        }
        private boolean updateTime(){
            long current_time = System.currentTimeMillis();
            if((current_time - mLastTimeMeasure) > REFRESH_RATE)
                return true;
            mLastTimeMeasure = current_time;
            return false;
        }
    }

    private AnimationThread mAnimationThread;
    private Context mContext;
    private float[] mStartPos;

    public VisualisationView(Context context){
        super(context);
    }


    public VisualisationView(Context context, AttributeSet set) {
        super(context,set);
        getHolder().addCallback(this);
        this.mContext = context;
        mStartPos =  new float[]{(float)0.0,(float)0.0,(float)0.0};
        mAnimationThread = new AnimationThread(this.getHolder(),context,this.getHandler());
    }

    public AnimationThread getThread() {
        return mAnimationThread;
    }

    /**
     *  Check if user touched drone and connect to it if so...
     */
    @Override
    public boolean onTouchEvent(MotionEvent event) {
        switch(event.getAction()) {
            //press and release
            case MotionEvent.ACTION_UP:
                //mObject.setBounds(distLeft, distTop,distRight, distBottom);
                int x =  (int)event.getX();
                int y = (int)event.getY();
                if(x < distRight && x > distLeft){
                    if(y > distTop && y < distBottom){
                        if(mainActivity.isConnected()){
                            mainActivity.stopDroneCommunicator();
                        }else{
                            mainActivity.startDroneCommunicator();//connect to drone
                        }

                    }
                }
                break;
        }
        return true;
    }

    @Override
    protected void onMeasure(int wMeasureSpec,int hMeasureSpec) {
        int measuredWidth = measure(wMeasureSpec);
        int measuredHeight = measure(hMeasureSpec);
        if(mAnimationThread != null)
            mAnimationThread.setMeasuredSize(measuredHeight,measuredWidth);
        //int dimension = Math.min(measuredWidth,measuredHeight);
        setMeasuredDimension(measuredWidth, measuredHeight);
    }

    @Override
    public void surfaceDestroyed(SurfaceHolder holder) {
        boolean retry = true;
        getThread().setRunning(false);
        while (retry) {
            try {
                getThread().join();
                retry = false;
            } catch (InterruptedException e) {
            }
        }
    }

    @Override
    public void surfaceChanged(SurfaceHolder holder, int format, int width,
                               int height) {}

    @Override
    public void surfaceCreated(SurfaceHolder holder){
        if (mAnimationThread.getState() != Thread.State.TERMINATED){
            mAnimationThread.setRunning(true);
            mAnimationThread.start();
        } else {
            //SurfaceHolder holder, Context context, Handler handler
            mAnimationThread = new AnimationThread(holder, mContext,
                    new Handler() {
                        @Override
                        public void handleMessage(Message m) {
                        }
                    });
            mAnimationThread.setMeasuredSize(getMeasuredHeight(),getMeasuredWidth());
            mAnimationThread.setRunning(true);
            mAnimationThread.start();
        }
    }

    private int measure(int measureSpec){
        int result = 0;
        int specMode = MeasureSpec.getMode(measureSpec);
        int specSize = MeasureSpec.getSize(measureSpec);
        if (specMode == MeasureSpec.UNSPECIFIED) {
            result = 200;
        }else{
            result = specSize;
        }
        return result;
    }
}

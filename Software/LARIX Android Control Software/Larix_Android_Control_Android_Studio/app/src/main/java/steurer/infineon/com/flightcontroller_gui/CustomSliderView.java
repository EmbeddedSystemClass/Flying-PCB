/**
  CustomSlider.  See  http://www.permadi.com/blog/2011/11/android-sdk-custom-slider-bar-seekbar.
  
  Copyright F. Permadi

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
 **/

package steurer.infineon.com.flightcontroller_gui;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.View;
import android.view.View.OnTouchListener;
import android.widget.FrameLayout;
import android.widget.ImageView;

public class CustomSliderView extends FrameLayout implements OnTouchListener
{
	// The touch position relative to the slider view (ie: left=0, right=width of the slider view)
	private float mTouchXPosition;
	private float mStartPos = 0;

	// Images used for thumb and the bar
    protected ImageView mThumbImageView, mSliderBarImageView;
    protected Bitmap mThumbBitmap;
    protected Bitmap mSliderBarBitmap;

    // These two variables are useful if you want to programatically reskin the slider
    protected int mThumbResourceId;
    protected int mSliderBarResourceId;
    
    // Default ranges
    protected int mMinValue=0;
    protected int mMaxValue=100;

    // Used internally during touches event
    protected float mTargetValue=0;
    protected boolean touched;
    protected int mSliderLeftPosition, mSliderRightPosition;


    // Holds the object that is listening to this slider.
    protected OnTouchListener mDelegateOnTouchListener;
    
	/**
	 * Default constructors.  
	 * Just tell Android that we're doing custom drawing and that we want to listen to touch events.
	 */
	public CustomSliderView(Context context) 
	{
		super(context);
		setWillNotDraw(false);
		setOnTouchListener(this);
	}

	public CustomSliderView(Context context, AttributeSet attrs) 
	{
		super(context, attrs);
		setWillNotDraw(false);
		setOnTouchListener(this);
	}

	public CustomSliderView(Context context, AttributeSet attrs, int defStyle) 
	{
		super(context, attrs, defStyle);
		setWillNotDraw(false);
		this.setOnTouchListener(this);
	}

	/*
	 * This should be called by the object that wants to listen to the touch events
	 */
	public void setDelegateOnTouchListener(OnTouchListener onTouchListener)
	{
		mDelegateOnTouchListener=onTouchListener;
	}
	
	
	public void setResourceIds(int thumbResourceId, int sliderBarResourceId)
	{
		mThumbResourceId=thumbResourceId;
		mSliderBarResourceId=sliderBarResourceId;
		mThumbImageView=null;
		mSliderBarImageView=null;
	}
	

    public boolean onTouch(View view, MotionEvent event)
    {
        switch (event.getAction()) 
        {
	        case MotionEvent.ACTION_DOWN:
	        	invalidate();
	        	mTouchXPosition=event.getX();

	        	if (mDelegateOnTouchListener!=null)
	        		mDelegateOnTouchListener.onTouch(view, event);
	        	return true;

	        case MotionEvent.ACTION_MOVE:
	        	invalidate();
	        	mTouchXPosition=event.getX();
	        	if (mDelegateOnTouchListener!=null)
	        		mDelegateOnTouchListener.onTouch(view, event);
	        	
	            return true;
	        case MotionEvent.ACTION_UP:
	        	invalidate();
	        	setScaledValue((int)mStartPos);
	        	if (mDelegateOnTouchListener!=null)
	        		mDelegateOnTouchListener.onTouch(view, event);
	            return true;
        }
    	return false;
    }
    
    /* 
     * This sets the range of the slider values.  Eq: 0 to 100 or 20 to 70.
     */
    public void setRange(int min, int max)
    {
    	mMinValue=min;
    	mMaxValue=max;
    }


	public void setStartPos(int start)
	{
		this.mStartPos = (float)start;
	}

    /* 
     * This sets the value, between mMinValue and mMaxValue
     */
    public void setScaledValue(int value)
    {        
    	mTargetValue=(float)value;//((value-mMinValue)/range)*fillWidth;
        touched = true;
    	invalidate();
    }
    
    /**
     * @return The actual value of the thumb position, scaled to the min and max value
     */
    public int getScaledValue()
    {
		return (int) mMinValue+(int)((mMaxValue-mMinValue)*getPercentValue());
    }
    
    /**
     * @return The percent value of the current thumb position.
     */
    public float getPercentValue()
    {
    	float fillWidth=mSliderBarImageView.getWidth();
    	float relativeTouchX=mTouchXPosition-mSliderBarImageView.getLeft();//+mThumbImageView.getWidth()/2;
    	float percentage=relativeTouchX/fillWidth;
		//if(percentage < 0.f)percentage = 0.f;
		if(percentage > 0.96)percentage=1.f;
		return percentage;
    }
    
    /** 
     * 
     * @param percentValue	between 0 to 1.0f
     */
    public void setPercentValue(float percentValue)
    {
    	float position=mSliderLeftPosition+percentValue*(mSliderRightPosition-mSliderLeftPosition-mThumbBitmap.getWidth());
    	mTouchXPosition=position;
    	invalidate();
    }
    
	@Override 
	protected void onDraw (Canvas canvas)
	{
		// Load the resources if not already loaded
		if (mThumbImageView==null)
		{
			mThumbImageView=(ImageView)this.getChildAt(1);
			this.removeView(mThumbImageView);

			if (mThumbResourceId>0)
			{
				mThumbBitmap=BitmapFactory.decodeResource(getContext().getResources(), mThumbResourceId);
				mThumbImageView.setImageBitmap(mThumbBitmap);
			}
			
			// USe the drawing cache so that we don't have to scale manually.
			mThumbImageView.setDrawingCacheEnabled(true);
			mThumbBitmap = mThumbImageView.getDrawingCache(true);
			//mThumbImageView.setDrawingCacheEnabled(false);
		}
		if (mSliderBarImageView==null)
		{
			mSliderBarImageView=(ImageView)this.getChildAt(0);
			this.removeView(mSliderBarImageView);

			// If user has specified a different skin, load it
			if (mSliderBarResourceId>0)
			{
				mSliderBarBitmap=BitmapFactory.decodeResource(getContext().getResources(), mSliderBarResourceId);
				mSliderBarImageView.setImageBitmap(mSliderBarBitmap);
			}

			// USe the drawing cache so that we don't have to scale manually.
			mSliderBarImageView.setDrawingCacheEnabled(true);
	        mSliderBarBitmap = mSliderBarImageView.getDrawingCache(true);
	        //mSliderBarImageView.setDrawingCacheEnabled(false);
	        
	        mSliderLeftPosition=mSliderBarImageView.getLeft();
	        mSliderRightPosition=mSliderBarImageView.getLeft()+mSliderBarBitmap.getWidth();	        
		}

		// Adjust thumb position (this handles the case where setScaledValue() was called)
		if (touched)
        {
	        float fillWidth=mSliderBarImageView.getMeasuredWidth();
	    	float range=(mMaxValue-mMinValue);
	    	mTouchXPosition=((mTargetValue-mMinValue)/range)*fillWidth;
	        touched = false;
        }
        
        // Don't allow going out of bounds
        if (mTouchXPosition<mSliderLeftPosition)
        	mTouchXPosition=mSliderLeftPosition;
        else if (mTouchXPosition>mSliderRightPosition)
        	mTouchXPosition=mSliderRightPosition;
       
        if (mSliderBarBitmap!=null)
        	canvas.drawBitmap(mSliderBarBitmap, mSliderLeftPosition, mSliderBarImageView.getTop(), null);
        if (mThumbBitmap!=null)
        	canvas.drawBitmap(mThumbBitmap, mTouchXPosition-mThumbBitmap.getWidth()/2, mThumbImageView.getTop(), null);
	}
}
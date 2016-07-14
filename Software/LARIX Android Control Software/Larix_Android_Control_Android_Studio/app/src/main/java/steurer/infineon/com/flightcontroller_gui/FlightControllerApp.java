package steurer.infineon.com.flightcontroller_gui;

import android.app.Application;
import android.content.res.Configuration;

/**
 * Created by SteurerE on 04.02.2015.
 */
public class FlightControllerApp extends Application
{

    private static FlightControllerApp singleton;

    public static FlightControllerApp getInstance()
    {
        return singleton;
    }


    @Override
    public final void onCreate()
    {
        super.onCreate();
        singleton = this;
    }

    @Override
    public final void onLowMemory()
    {
        super.onLowMemory();
    }


    @Override
    public final void onTrimMemory(int level)
    {
        super.onTrimMemory(level);
    }


    @Override
    public final void onConfigurationChanged(Configuration newConfig)
    {
        super.onConfigurationChanged(newConfig);
    }


}

package steurer.infineon.com.flightcontroller_gui;

/**
 * Created by SteurerE on 04.02.2015.
 */

public class DataPacket
{

private String data;

    public DataPacket(){}


    public DataPacket(String data) {
        this.data = data;
    }

    public String getData() {
        return data;
    }

    public void setData(String data) {
        this.data = data;
    }


}

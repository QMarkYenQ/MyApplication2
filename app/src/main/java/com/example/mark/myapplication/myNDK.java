package com.example.mark.myapplication;

/**
 * Created by Mark on 2016/5/7.
 */
public class myNDK
{
    static{
        System.loadLibrary("myJNI");
    }
    public native String getMystring();
    public native int[] gray( int[] buf, int w, int h );
    public native float geCount();
    public native int setone();
}

package com.example.jjdl.gpsms;

import android.content.Context;
import android.location.Location;
import android.location.LocationListener;
import android.os.Bundle;
import android.app.Activity;

import android.telephony.SmsManager;

import android.util.Log;

import android.location.LocationManager;

import java.util.Timer;
import java.util.TimerTask;

public class MainActivity extends Activity implements LocationListener {

    private static final int MY_PERMISSIONS_REQUEST_SEND_SMS =0 ;
    String lat;
    String lng;

    protected LocationManager locationManager;

    private Timer timer;
    private TimerTask timerTask = new TimerTask() {

        @Override
        public void run() {
            //sendSMSMessage("07824485809");
            //sendSMSMessage("07591849894");
            sendSMSMessage("07889602604");
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        locationManager = (LocationManager) getSystemService(Context.LOCATION_SERVICE);

        try {
            locationManager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 0, 0, this);
        } catch (SecurityException e) {

        }

        timer = new Timer();
        timer.scheduleAtFixedRate(timerTask, 0, 20000);
    }

    protected void sendSMSMessage(String phoneNo) {

        String message;

        if (lat == null) message = "No GPS fix yet :(";
        else message = lat + "\n" + lng;

        SmsManager smsManager = SmsManager.getDefault();
        smsManager.sendTextMessage(phoneNo, null, message, null, null);

    }

    @Override
    public void onLocationChanged(Location location) {

        lat = String.valueOf(location.getLatitude());
        lng = String.valueOf(location.getLongitude());

    }

    @Override
    public void onProviderDisabled(String provider) {
        Log.d("Latitude","disable");
    }

    @Override
    public void onProviderEnabled(String provider) {
        Log.d("Latitude","enable");
    }

    @Override
    public void onStatusChanged(String provider, int status, Bundle extras) {
        Log.d("Latitude","status");
    }


}

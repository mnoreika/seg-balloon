package jjdl.example.com.gpsms_reader;

import android.database.Cursor;
import android.net.Uri;
import android.os.Bundle;
import android.support.v4.app.FragmentActivity;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;

import com.google.android.gms.maps.CameraUpdateFactory;
import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.OnMapReadyCallback;
import com.google.android.gms.maps.SupportMapFragment;
import com.google.android.gms.maps.model.CameraPosition;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.MarkerOptions;


public class MainActivity extends FragmentActivity implements OnMapReadyCallback {

    Button update;
    TextView lat;
    TextView lng;
    GoogleMap googleMap;

    String latlng[] = {"22","22"};

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        lat = (TextView)findViewById(R.id.txtLat);
        lng = (TextView)findViewById(R.id.txtLng);

        update = (Button) findViewById(R.id.btnUpdate);
        update.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View arg0) {

                googleMap.clear();

                Uri uri = Uri.parse("content://sms/inbox");
                String[] projection = new String[] { "_id", "address", "person", "body", "date", "type" };
                Cursor cursor = getContentResolver().query(uri, projection, null, null, "date desc");

                if (cursor.moveToFirst()) { // must check the result to prevent exception

                    int index_Address = cursor.getColumnIndex("address");
                    int index_Body = cursor.getColumnIndex("body");

                    String body = cursor.getString(index_Body);
                    String sender = cursor.getString(index_Address);

                    latlng = body.split("\n");


                    if (latlng.length > 1 && sender.equals("+447889602604")) {

                        double lt = Double.parseDouble(latlng[0]);
                        double ln = Double.parseDouble(latlng[1]);

                        lat.setText("lat: " + lt);
                        lng.setText("lng: " + ln);

                        LatLng pos = new LatLng(lt, ln);
                        googleMap.addMarker(new MarkerOptions().position(pos)
                                .title("Current location"));

                        runThread();
                    } else {
                        lat.setText("Invalid/No fix");
                        lng.setText(":(");
                    }


                } else {
                    // empty box, no SMS
                }



            }
        });

        SupportMapFragment mapFragment = (SupportMapFragment) getSupportFragmentManager()
                .findFragmentById(R.id.map);
        mapFragment.getMapAsync(this);


    }

    @Override
    public void onMapReady(GoogleMap temp) {

        googleMap = temp;


    }

    public void runThread() {

        runOnUiThread(new Runnable() {

            @Override
            public void run() {

                double lt = Double.parseDouble(latlng[0]);
                double ln = Double.parseDouble(latlng[1]);

                LatLng pos = new LatLng(lt, ln);

                CameraPosition cameraPosition =
                        new CameraPosition.Builder()
                                .target(pos)
                                .tilt(15)
                                .zoom(16)
                                .build();
                googleMap.animateCamera(CameraUpdateFactory.newCameraPosition(cameraPosition));
            }
        });

    }

}

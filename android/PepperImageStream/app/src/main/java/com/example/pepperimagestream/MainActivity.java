package com.example.pepperimagestream;

import androidx.appcompat.app.AppCompatActivity;

import android.content.Intent;
import android.os.Bundle;
import android.os.StrictMode;
import android.util.Log;
import android.view.View;
import android.widget.TextView;


public class MainActivity extends AppCompatActivity {

    // Connection status manager
    Thread connection_status;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        // These two lines required to allow getting the network address and creating a socket on
        // the main thread
        StrictMode.ThreadPolicy policy = new StrictMode.ThreadPolicy.Builder().permitAll().build();
        StrictMode.setThreadPolicy(policy);

        // Get internet address and set to TextView
        TextView textViewIP = findViewById(R.id.textViewIP);
        textViewIP.setText(getString(R.string.ip_address, Global.tcp_server.getServerAddress()));
        TextView textViewPort = findViewById(R.id.textViewPort);
        textViewPort.setText(getString(R.string.port, Global.tcp_server.getServerPort()));
        final TextView textViewConnectionStatus = findViewById(R.id.textViewConnectionStatus);

        // This thread continuously checks for a client connection regardless of whether in the
        // MainActivity or in the DisplayFeedActivity and updates the UI's labels accordingly.
        connection_status = new Thread(new Runnable() {
            public void run() {
                while (!Thread.interrupted()) {
                    if (Global.tcp_server.isConnectedToClient()) {
                        // Display the client's IP address
                        textViewConnectionStatus.post(new Runnable() {
                            public void run() {
                                textViewConnectionStatus.setText(String.format("Connected to client at: %s", Global.tcp_server.getClientAddress()));
                            }
                        });
                    } else {
                        // Display waiting status
                        textViewConnectionStatus.post(new Runnable() {
                            public void run() {
                                textViewConnectionStatus.setText(R.string.waiting);
                            }
                        });
                        // Wait until a client connects to the server and update label
                        final String message = Global.tcp_server.waitForConnection();
                        textViewConnectionStatus.post(new Runnable() {
                            public void run() {
                                textViewConnectionStatus.setText(message);
                            }
                        });
                    }
                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                        break;
                    }
                }
            }
        });
        connection_status.start();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        connection_status.interrupt();
    }

    public void viewFeed(View view) {
        /*
         * Called when user presses the "View Feed" button. Starts the viewing activity.
         */
        Intent intent = new Intent(this, DisplayFeedActivity.class);
        startActivity(intent);
    }

    public void closeApp(View view) {
        /*
         * Closes the app through a button.
         */
        Log.i("info", "Closing server");
        Global.tcp_server.closeServer();
        MainActivity.this.finish();
        onDestroy();
        System.exit(0);
    }
}
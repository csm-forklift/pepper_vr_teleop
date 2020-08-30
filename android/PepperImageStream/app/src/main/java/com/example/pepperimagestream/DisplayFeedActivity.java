package com.example.pepperimagestream;

import androidx.appcompat.app.AppCompatActivity;

import android.content.Intent;
import android.content.res.Resources;
import android.graphics.Bitmap;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.ImageView;
import android.widget.TextView;

public class DisplayFeedActivity extends AppCompatActivity {

    // Image reading thread
    Thread image_feed;

    // Used to set the size of the display image
    int screen_height = Resources.getSystem().getDisplayMetrics().heightPixels;
    int screen_width = Resources.getSystem().getDisplayMetrics().widthPixels;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_display_feed);

        // Get the Intent that started this activity
        Intent intent = getIntent();

        // Get label and image view UI objects used in the thread
        final TextView textViewNoFeed = findViewById(R.id.textViewNoFeed);
        final ImageView imageView = findViewById(R.id.imageView);

        // Create background thread
        image_feed = new Thread(new Runnable() {
            public void run() {
                watchFeed(textViewNoFeed, imageView);
            }
        });
    }

    @Override
    public void onResume() {
        image_feed.start();
        super.onResume();
    }

    @Override
    public void onStop() {
        image_feed.interrupt();
        super.onStop();
    }

    public void viewStatus(View view) {
        image_feed.interrupt();
        super.onBackPressed();
    }

    private void watchFeed(final TextView textViewNoFeed, final ImageView imageView) {
        /*
         * Iterates through the client/server communication loop. Reads the image from the Request
         * message and displays it on the screen. Then sends a Response message indicating the
         * server is ready for a new image.
         */
        while (true) {
            if (Global.tcp_server.isConnectedToClient()) {
                if (Global.tcp_server.getReadyForRequest()) {
                    // Read Request
                    final Bitmap display_image = Global.tcp_server.readMessage();

                    // Display Image
                    if (display_image == null) {
                        // No image to display
                        if (imageView.getVisibility() == View.VISIBLE) {
                            imageView.post(new Runnable() {
                                @Override
                                public void run() {
                                    imageView.setVisibility(View.INVISIBLE);
                                }
                            });
                        }
                        if (textViewNoFeed.getVisibility() != View.VISIBLE) {
                            textViewNoFeed.post(new Runnable() {
                                @Override
                                public void run() {
                                    textViewNoFeed.setVisibility(View.VISIBLE);
                                }
                            });
                        }
                        textViewNoFeed.post(new Runnable() {
                            @Override
                            public void run() {
                                textViewNoFeed.setText("Image Error");
                            }
                        });
                    } else {
                        // Resize the image to fill the ImageView
                        int view_height = screen_height;
                        int view_width = screen_width;
                        int image_height = display_image.getHeight();
                        int image_width = display_image.getWidth();
                        // Find max error and scale to fit that dimension
                        int height_error = image_height - view_height;
                        int width_error = image_width - view_width;
                        double scale_factor;
                        if (height_error > width_error) {
                            scale_factor = (double) view_height/image_height;
                        } else {
                            scale_factor = (double) view_width/image_width;
                        }
                        final Bitmap scaled_image = Bitmap.createScaledBitmap(display_image, (int) (scale_factor*image_width), (int) (scale_factor*image_height), true);

                        if (textViewNoFeed.getVisibility() == View.VISIBLE) {
                            textViewNoFeed.post(new Runnable() {
                                public void run() {
                                    textViewNoFeed.setVisibility(View.INVISIBLE);
                                }
                            });
                        }
                        if (imageView.getVisibility() != View.VISIBLE) {
                            imageView.post(new Runnable() {
                                @Override
                                public void run() {
                                    imageView.setVisibility(View.VISIBLE);
                                }
                            });
                        }
                        imageView.post(new Runnable() {
                            @Override
                            public void run() {
                                imageView.setImageBitmap(scaled_image);
                            }
                        });
                    }

                    if (Thread.interrupted()) {
                        break;
                    }
                }
                if (!Global.tcp_server.getReadyForRequest()){
                    // Image processing was successful, send Response
                    Global.tcp_server.writeMessage();

                    if (Thread.interrupted()) {
                        break;
                    }
                }
            } else {
                // No connection established yet
                if (imageView.getVisibility() == View.VISIBLE) {
                    imageView.post(new Runnable() {
                        @Override
                        public void run() {
                            imageView.setVisibility(View.INVISIBLE);
                        }
                    });
                }
                if (textViewNoFeed.getVisibility() != View.VISIBLE) {
                    textViewNoFeed.post(new Runnable() {
                        public void run() {
                            textViewNoFeed.setVisibility(View.VISIBLE);
                        }
                    });
                }
                textViewNoFeed.post(new Runnable() {
                    @Override
                    public void run() {
                        textViewNoFeed.setText(getString(R.string.no_feed));
                    }
                });
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                    break;
                }
            }
        }
    }
}
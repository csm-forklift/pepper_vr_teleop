package com.example.pepperimagestream;

import android.app.Application;


public class Global extends Application {
    /*
     * This class is used to contain global variables that can be accessed across all of the
     * Application. The TCPServer object needs to be accessed in MainActivity as well as
     * DisplayFeedActivity.
     */
    public static TCPServer tcp_server;

    public Global() {
        tcp_server = new TCPServer();
    }
}

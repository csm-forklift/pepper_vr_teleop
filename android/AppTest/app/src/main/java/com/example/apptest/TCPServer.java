package com.example.apptest;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.util.Log;

import org.json.JSONException;
import org.json.JSONObject;

import java.io.IOException;
import java.io.PushbackInputStream;
import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.NetworkInterface;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketException;
import java.nio.charset.StandardCharsets;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.locks.ReentrantLock;


public class TCPServer {
    //============================================================================================//
    // Fields
    //============================================================================================//
    // Network Variables
    private InetAddress local_host;
    private InetAddress server_inet_address;
    private int server_port;
    private ServerSocket server_socket;
    private boolean connected_to_client = false;
    private Socket client_socket;
    private PushbackInputStream client_input_stream;

    // Connection Checking Thread
    private ReentrantLock stream_lock = new ReentrantLock();
    private Thread client_connection_status;

    // Message Variables
    private int segment_size = 4096;
    private byte[] receive_buffer = new byte[segment_size];
    private int num_segments = 1;
    private int data_size = 0;
    private byte[] send_buffer;
    private int protocol_header_size = 2;
    private int json_header_size;
    private int json_header_start;
    private int json_header_end;
    private int content_size;
    private int content_start;
    private int content_end;
    private Bitmap content;

    // State Variables
    private boolean processed_protocol_header = false;
    private boolean processed_json_header = false;
    private boolean processed_content = false;
    private boolean ready_for_request = true;

    //============================================================================================//
    // Methods
    //============================================================================================//
    public TCPServer() {
        setupServer();
    }

    //============================================================================================//
    // Getters and Setters
    //============================================================================================//
    private void setServerAddress() {
        /*
         * Get address by going through all available networks
         * (looking for IPv4 address that is not the local host)
         * Modified from: https://stackoverflow.com/questions/6064510/how-to-get-ip-address-of-the-device-from-code
         */
        try {
            List<NetworkInterface> interfaces = Collections.list(NetworkInterface.getNetworkInterfaces());
            for (NetworkInterface intf : interfaces) {
                List<InetAddress> addresses = Collections.list(intf.getInetAddresses());
                for (InetAddress addr : addresses) {
                    if (addr instanceof Inet4Address) {
                        // Save if address does not equal the local host
                        if (addr.getHostAddress().equals("127.0.0.1")) {
                            local_host = addr;
                        } else {
                            server_inet_address = addr;
                        }
                    }
                }
            }
        } catch (Exception ignored) {
            // Ignore exceptions for now
        }

        if (server_inet_address == null) {
            // No non-localhost IPv4 address was found
            server_inet_address = local_host;
        }
    }

    public String getServerAddress() {
        return server_inet_address.getHostAddress();
    }

    public String getServerPort() {
        return Integer.valueOf(server_port).toString();
    }

    public String getClientAddress() {
        return client_socket.getRemoteSocketAddress().toString();
    }

    public boolean isConnectedToClient() {
        return connected_to_client;
    }

    public boolean getReadyForRequest() {
        return ready_for_request;
    }

    //============================================================================================//
    // Network Socket Connections
    //============================================================================================//
    public void setupServer() {
        //===== Get the server Address =====//
        setServerAddress();

        // NOTE: define port here
        server_port = 65432;

        // Set socket address
        InetSocketAddress server_socket_address = new InetSocketAddress(server_inet_address, server_port);

        //===== Create server socket =====//
        try {
            server_socket = new ServerSocket();
            server_socket.bind(server_socket_address);
        } catch (IOException e) {
            Log.e("TCP", "Could not create server socket");
            e.printStackTrace();
        }
    }

    public String waitForConnection() {
        try {
            client_socket = server_socket.accept();
            client_input_stream = new PushbackInputStream(client_socket.getInputStream());
            connected_to_client = true;

            //===== Set up thread for checking client connection status =====//
            client_connection_status = new Thread(new Runnable() {
                @Override
                public void run() {
                    checkForClosedClient();
                }
            });
            client_connection_status.start();

            return String.format("Connected to client at: %s", getClientAddress());
        } catch (IOException e) {
            e.printStackTrace();
            return "Failed to connect to any clients";
        }
    }

    public void closeServer() {
        try {
            closeClient();
            server_socket.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void closeClient() {
        try {
            client_connection_status.interrupt();
            connected_to_client = false;
            client_socket.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private void checkForClosedClient() {
        /*
         * This function is intended to run on another thread and continuously checks if the next
         * byte on the client's input stream is an "end of file" signal indicating the client has
         * been closed.
         */
        while (!Thread.interrupted()) {
            try {
                if (client_input_stream.available() == 0) {
                    stream_lock.lock();
                    try {
                        int next_byte = client_input_stream.read();
                        if (next_byte == -1) {
                            // Connection has been closed by peer
                            Log.i("TCP", "Peer closed");
                            closeClient();
                            return;
                        } else {
                            client_input_stream.unread(next_byte);
                        }
                    } finally {
                        stream_lock.unlock();
                    }
                }
            } catch (IOException e) {
                e.printStackTrace();
                if (e instanceof SocketException) {
                    closeClient();
                    Log.i("TCP", "Connection lost");
                }
            }
        }
    }

    //============================================================================================//
    // Communication Functions (read/write/process data)
    //============================================================================================//
    private void read() {
        /*
         * Adds data to the receive buffer byte array by reading the input stream from the client
         * socket. Dynamically increases the buffer if more room is needed.
         */
        // Check if buffer is full, if so expand
        if (data_size == segment_size*num_segments) {
            receive_buffer = concatenate(receive_buffer, new byte[segment_size]);
            num_segments += 1;
        }

        // Read in new data bytes
        int remaining_length = segment_size*num_segments - data_size;
        try {
            stream_lock.lock();
            int bytes_read;
            try {
                bytes_read = client_input_stream.read(receive_buffer, data_size, remaining_length);
            } finally {
                stream_lock.unlock();
            }
            if (bytes_read > -1) {
                data_size += bytes_read;
            }
            else {
                closeClient();
                Log.i("TCP", "Peer closed");
            }
        } catch (IOException e) {
            e.printStackTrace();
            if (e instanceof SocketException) {
                closeClient();
                Log.i("TCP", "Connection lost");
            }
        }
    }

    private boolean write() {
        /*
         * Sends the Response message bytes to the client's output stream.
         */
        if (send_buffer.length > 0) {
            try {
                client_socket.getOutputStream().write(send_buffer);
            } catch (IOException e) {
                e.printStackTrace();
                return false;
            }
        }
        return true;
    }

    private void processProtocolHeader() {
        /*
         * Reads the JSON header size contained in the first 2 bytes of the Request message.
         */
        // Calculate bounds for protocol header
        if (data_size >= protocol_header_size) {
            int protocol_header_start = 0;
            int protocol_header_end = protocol_header_start + protocol_header_size;

            // Determine the JSON header size
            json_header_size = bytesToUint16(Arrays.copyOfRange(receive_buffer, protocol_header_start, protocol_header_end));

            // Calculate bounds for JSON header
            json_header_start = protocol_header_end;
            json_header_end = json_header_start + json_header_size;

            processed_protocol_header = true;
        }
    }

    private void processJSONHeader() {
        /*
         * JSON Header should be in the following format:
         * {
         *     "is_big_endian": <boolean>,
         *     "content-type": {"image/bmp", "image/png", "image/jpg"},
         *     "content-encoding": "binary",
         *     "colorspace": <str>,
         *     "image-height": <int>,
         *     "image-width": <int>,
         *     "content-length": <int>
         * }
         *
         * The bytes should be from a string encoded as "UTF-8" following the JSON format.
         *
         * Currently the only variable being used is the "content-length". The other variables are
         * provided in case they are useful in future development. The
         * BitmapFactory.decodeByteArray() function handles interpreting the image type, colorspace,
         * and the dimensions.
         */
        if (data_size >= (protocol_header_size + json_header_size)) {
            // Decode the JSON header
            JSONObject json_header = decodeJSON(Arrays.copyOfRange(receive_buffer, json_header_start, json_header_end));

            // Find the content length
            try {
                if (json_header == null) {
                    return;
                }
                content_size = json_header.getInt("content-length");
                int image_height = json_header.getInt("image-height");
                int image_width = json_header.getInt("image-width");
            } catch (JSONException e) {
                e.printStackTrace();
                content_size = 0;
            }

            // Calculate the bounds for the content
            content_start = json_header_end;
            content_end = content_start + content_size;

            processed_json_header = true;
        }
    }

    private void processContent() {
        /*
         * Processes the data bytes and converts them into the message content.
         */
        if (data_size >= (protocol_header_size + json_header_size + content_size)) {
            // Decode the data
            content = BitmapFactory.decodeByteArray(receive_buffer, content_start, (content_end - content_start));
            if (content == null) {
                Log.i("connecting", "Returned bitmap is NULL");
            }

            processed_content = true;
        }
    }

    private void processMessage() {
        /*
         * Iterates through processing the protocol header, the JSON header, and the content bytes.
         */
        if (!processed_protocol_header) processProtocolHeader();
        if (processed_protocol_header && !processed_json_header) processJSONHeader();
        if (processed_json_header && !processed_content) processContent();
    }

    public Bitmap readMessage() {
        /*
         * Reads and processes the Request message, which contains an image to be displayed.
         * Returns the image as a Bitmap object.
         */
        while (!processed_content) {
            read(); // Checks if client is closed
            if (connected_to_client) {
                processMessage();
            } else {
                resetReceiveBuffer();;
                return null;
            }
        }

        resetReceiveBuffer();
        ready_for_request = false;
        return content;
    }

    private void createMessage() {
        /*
         * Creates the Response message to send to the client. The content is a JSON string with a
         * single object containing a boolean indicating whether the image was successfully decoded.
         * Currently, the message always returns 'true'. If there are any errors, no Response is
         * sent and the client will stop transmitting images.
         *
         * The JSON Header should be in the following format:
         * {
         *     "is_big_endian": <boolean>,
         *     "content-type": "text/json",
         *     "content-encoding": "utf-8",
         *     "content-length": <int>
         * }
         */
        // Create Content
        JSONObject json_response = new JSONObject();
        try {
            json_response.put("success", true);
        } catch (JSONException e) {
            e.printStackTrace();
        }
        byte[] json_response_bytes = json_response.toString().getBytes(StandardCharsets.UTF_8);

        // Create JSON Header
        JSONObject json_response_header = new JSONObject();
        try {
            json_response_header.put("is_big_endian", false);
            json_response_header.put("content-type", "text/json");
            json_response_header.put("content-encoding", "utf-8");
            json_response_header.put("content-length", json_response_bytes.length);
        } catch (JSONException e) {
            e.printStackTrace();
        }
        byte[] json_response_header_bytes = json_response_header.toString().getBytes(StandardCharsets.UTF_8);

        // Create protocol header
        byte[] protocol_header_bytes = uint16ToBytes(json_response_header_bytes.length);

        // Concatenate byte arrays together
        send_buffer = concatenate(protocol_header_bytes, json_response_header_bytes);
        send_buffer = concatenate(send_buffer, json_response_bytes);
    }

    public void writeMessage() {
        /*
         * Creates the Response message and then transmits it to the client.
         */
        // Create message
        createMessage();

        // Send
        boolean successfully_sent = false;
        while (!successfully_sent) {
            successfully_sent = write();
        }

        // Get ready for next Request
        ready_for_request = true;
    }

    private void resetReceiveBuffer() {
        receive_buffer = new byte[segment_size];
        num_segments = 1;
        data_size = 0;
        processed_protocol_header = false;
        processed_json_header = false;
        processed_content = false;
    }

    //============================================================================================//
    // Utility Functions
    //============================================================================================//
    private int bytesToUint16(byte[] header) throws IllegalArgumentException {
        /*
         * Converts the protocol header from a big-endian 2-byte array into a Uint16 length
         * representing the number of bytes in the JSON header.
         */
        if (header.length != 2) {
            throw new IllegalArgumentException(String.format("Protocol header byte array must " +
                    "contain 2 elements. Length: %d", header.length));
        }
        return ((header[0] & 0xFF) << 8) |
               ((header[1] & 0xFF) << 0);
    }

    private byte[] uint16ToBytes(int length) {
        /*
         * Converts the JSON header length to a Uint16 byte array in big-endian format.
         */
        return new byte[] {(byte) (length >> 8), (byte) length};
    }

    private JSONObject decodeJSON(byte[] json_bytes) {
        /*
         * Takes a byte array as a UTF-8 encoded string and decodes it using the JSON format into
         * variables.
         */
        // Convert bytes to string
        String json_str;
        json_str = new String(json_bytes, StandardCharsets.UTF_8);

        // Convert string to JSON object
        JSONObject json_obj;
        try {
            json_obj = new JSONObject(json_str);
            return json_obj;
        } catch (JSONException e) {
            e.printStackTrace();
            return null;
        }
    }

    private byte[] concatenate(byte[] x, byte[] y) {
        /*
         * Concatenates the byte arrays x and y, then returns the merged array.
         */
        byte[] merge = new byte[x.length + y.length];
        System.arraycopy(x, 0, merge, 0, x.length);
        System.arraycopy(y, 0, merge, x.length, y.length);

        return merge;
    }
}

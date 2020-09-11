""" Module containing classes used to create TCP connections using Python.

The TCPClient class is used by the 'image_stream' node to communicate with
Pepper's tablet to send images from a ROS topic publishing a video stream.
Pepper has an app developed in Android Studio, called "Pepper Image Stream",
which waits for a TCP connection, then proceeds to display the images it
receives in fullscreen on the tablet. The message flow of the client is
specifically designed to communicate with the server in a particular sequence. A
server class is provided in this module as a reference and for testing, but it
is not used in the node. The Python ROS node only acts as a client. The Android
app acts as the server.

Here is the general message passing procedure:
Client:
1) Connect to server
2) Generate request
3) Send to server
4) Wait for response
5) Process response
6) Go to 2

Server:
1) Create connection and listen
2) Accept connection
3) Wait for request
4) Process request
5) Perform action with data
6) Generate response
7) Send response
8) Go to 3

Classes
-------
TCP : object
    A set of functions for communicating using TCP.
TCPServer : TCP
    Creates a TCP server socket using Python's socket module.
TCPClient : TCP
    Creates a TCP client connection using Python's socket module.

Exceptions
----------
TCPError : Exception
    Base class for TCP exceptions.
ConnectionClosedError : TCPError
    Exception raised when a connection is lost.
"""

import socket
import json
import struct
import atexit
import random
import time
import cv2
from threading import Lock

class TCP(object):
    """ A set of functions for communicating using TCP.

    Contains the basic functions needed to send and receive messages over a TCP
    socket in Python. The functions provided assume a message with the following
    structure:
    Protocol Header
        2 bytes, unsigned int, big-endian
        The size of the JSON header
    JSON Header
        Contains information about the content, including its size in bytes.
    Content
        The raw bytes of the content which must be processed into whatever
        pre-determined format.

    The server and client subclasses must implement the 'create_message' and
    'process_content' messages based on the format of the Request and Response
    messages.

    Attributes
    ----------
    socket : socket.socket
        The local socket object.
    local_address
        The local IP address and port combined
    local_host
        The local IP address as a string.
    local_port
        The local port number.
    remote_address
        The remote IP address and port combined.
    remote_host
        The remote IP address as a string.
    remote_port
        The remote port number.
    receive_size
        The max number of bytes to read at one time.
    _recv_buffer
        The receive buffer array.
    _send_buffer
        The send buffer array.
    _json_header_len
        The size of the JSON header.
    json_header
        The JSON header contents.
    content
        The content from the message being received.

    Methods
    -------
    read()
        Reads raw data bytes from the socket.
    write() -> bool
        Sends data in buffer. Returns whether message was fully sent.
    _json_encode(obj, encoding) -> byte array
        Encode dictionary into JSON byte array.
    _json_decode(json_bytes, encoding) -> dict
        Decode a dictionary from a JSON byte array.
    _int_to_bytes(size) -> byte array
        Convert an Int into two raw bytes, big-endian.
    _bytes_to_int(bytes) -> int
        Convert two bytes into an unsigned int.
    process_protocol_header()
        Creates the protocol header bytes.
    process_json_header()
        Creates the JSON header bytes.
    process_message()
        Converts the receive buffer bytes into data.
    read_message()
        Reads from the socket until a full message is received.
    write_message()
        Creates message and sends over socket.
    close()
        Closes the socket.

    Abstract Methods
    ----------------
    process_content()
    create_message()

    Parameters
    ----------
    sock : socket.socket
        The local socket object.
    receive_size : int
        The max number of bytes to read at one time.
    """
    def __init__(self, sock=None, receive_size=4096):
        self.socket = sock
        self.local_address = self.socket.getsockname()
        self.local_host = self.local_address[0]
        self.local_port = self.local_address[1]
        self.remote_address = self.socket.getpeername()
        self.remote_host = self.remote_address[0]
        self.remote_port = self.remote_address[1]
        self.receive_size = receive_size
        self._recv_buffer = b''
        self._send_buffer = b''
        self._json_header_len = None
        self.json_header = None
        self.content = None

        # This makes sure to close the socket regardless of how the prcess ends
        atexit.register(self.close)

    def read(self):
        """ Reads raw data bytes from the socket. """
        try:
            data = self.socket.recv(self.receive_size)
        except socket.error:
            pass
        else:
            if data:
                self._recv_buffer += data
            else:
                raise ConnectionClosedError('Peer closed at {0}:{1}'.format(self.remote_host, self.remote_port))

    def write(self):
        """ Sends data in buffer.

        Returns
        -------
        True : if message is fully sent
        False : if data is still in the buffer
        """
        if self._send_buffer:
            try:
                sent = self.socket.send(self._send_buffer)
            except socket.error:
                return False
            else:
                self._send_buffer = self._send_buffer[sent:]
                if sent and not self._send_buffer:
                    # Message fully sent
                    return True
                else:
                    # Message not fully sent yet
                    return False

    def _json_encode(self, obj, encoding):
        """ Encode dictionary into JSON byte array.

        Parameters
        ----------
        obj : dict
            The object to be converted into a JSON byte array.
        encoding : str
            The encoding type for the JSON text. Most common is 'utf-8'.

        Returns
        -------
        Array of bytes containing the JSON string encoded as specified.
        """
        return json.dumps(obj, ensure_ascii=False).encode(encoding)

    def _json_decode(self, json_bytes, encoding):
        """ Decode a dictionary from a JSON byte array.

        Parameters
        ----------
        json_bytes : byte array
            The data bytes to be converted into a Python variables.
        encoding : str
            The encoding type for the JSON text. Most common is 'utf-8'.

        Returns
        -------
        Dictionary representing JSON object.
        """
        return json.loads(json_bytes.decode(encoding))

    def _int_to_bytes(self, size):
        """ Convert Int into two raw bytes.

        Parameters
        ----------
        size : int
            The size of the JSON header array.

        Returns
        -------
        Big-endian byte array
        """
        return struct.pack('>H', size)

    def _bytes_to_int(self, bytes):
        """ Convert two bytes into an unsigned Int.

        Parameters
        ----------
        bytes : byte array
            The pair of bytes to be converted into an Int (JSON header size).

        Returns
        -------
        int
        """
        return struct.unpack('>H', bytes)[0]

    def process_protocol_header(self):
        """ Creates the protocol header bytes. """
        header_length = 2
        if len(self._recv_buffer) >= header_length:
            self._json_header_len = self._bytes_to_int(self._recv_buffer[:header_length])
            self._recv_buffer = self._recv_buffer[header_length:]

    def process_json_header(self):
        """ Creates the JSON header bytes. """
        header_length = self._json_header_len
        if (len(self._recv_buffer)) >= header_length:
            self.json_header = self._json_decode(self._recv_buffer[:header_length], 'utf-8')
            self._recv_buffer = self._recv_buffer[header_length:]
            for required_header in ['is_big_endian', 'content-type', 'content-encoding', 'content-length']:
                if required_header not in self.json_header:
                    raise ValueError('Missing required header "{0}".'.format(required_header))

    def process_message(self):
        """ Converts the receive buffer bytes into data. """
        if self._json_header_len is None:
            self.process_protocol_header()

        if self._json_header_len is not None:
            if self.json_header is None:
                self.process_json_header()

        if self.json_header:
            self.process_content()

    def read_message(self):
        """ Reads from the socket until a full message is received. """
        while self.content is None:
            self.read()
            self.process_message()

    def write_message(self):
        """ Creates message and sends over socket. """
        self.create_message()
        successfully_sent = False
        while not successfully_sent:
            successfully_sent = self.write()

    def close(self):
        """ Closes the socket. """
        print('Closing connection:\n\t {0}:{1} --> {2}:{3}'.format(self.local_host, self.local_port, self.remote_host, self.remote_port))
        self.socket.close()

    #==========================================================================#
    # Abstract Methods
    #==========================================================================#
    def process_content(self):
        raise NotImplementedError()

    def create_message(self):
        raise NotImplementedError()

class TCPServer(TCP):
    """ Creates a TCP server socket using Python's socket module.

    This class contains the variables and functions required to make a TCP
    server socket available for accepting clients. A specific message passing
    sequence has been coded into the functions. See the module description to
    see the flow of communication.

    Attributes
    ----------
    encoding : str
        The JSON text encoding type. Should be 'utf-8' unless the code has been
        changed to handle other formats.
    response : bool
        The Response message content.
    server_host : str
        The server IP address
    server_port : int
        The server port.
    server_address : tuple
        The server IP address and port combined.
    server_socket : socket.socket
        The server socket object.
    connection : socket.socket
        The client socket object.
    connection_address : tuple
        The client IP address and port combined.

    Methods
    -------
    create_message()
        Generate the Response message bytes.
    process_content()
        Process the Request content.
    process_loop()
        Loops through the message passing process for each request/response
        cycle.
    reset()
        Resets the variables and waits for a re-connection.
    close_server()
        Close the server socket.

    Parameters
    ----------
    host : str
        The IP address of the server socket.
    port : int
        The port of the server socket.
    """
    def __init__(self, host='127.0.0.1', port=65432):
        #======================================================================#
        # Create Socket
        #======================================================================#
        # Options
        self.encoding = 'utf-8'

        # Response command (determined after the processing request)
        self.response = True

        # Server address
        self.server_host = host
        self.server_port = port
        self.server_address = (self.server_host, self.server_port)

        # Create listening socket
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        atexit.register(self.close_server)

        # Bind and listen
        print('Establishing TCP server on {0}:{1}'.format(self.server_host, self.server_port))
        self.server_socket.bind(self.server_address)
        self.server_socket.listen()

        print('Waiting for connection...')
        self.wait_for_connection()

    def wait_for_connection(self):
        self.connection, self.connection_address = self.server_socket.accept()
        print('Server connected to {0}:{1}'.format(self.connection_address[0], self.connection_address[1]))

        # Initialize TCP connection socket object
        super(TCPServer, self).__init__(sock=self.connection)

    #==========================================================================#
    # Define Response Message
    #==========================================================================#
    def create_message(self):
        """ Generate the Response message bytes. """
        # Create response content
        content = {'success': self.response}
        content_bytes = self._json_encode(content, self.encoding)

        # Create JSON header
        json_header = {
            'is_big_endian': False,
            'content-type': 'text/json',
            'content-encoding': self.encoding,
            'content-length': len(content_bytes)
        }
        json_header_bytes = self._json_encode(json_header, self.encoding)

        # Create protocol header
        protocol_header_bytes = self._int_to_bytes(len(json_header_bytes))

        self._send_buffer = protocol_header_bytes + json_header_bytes + content_bytes

    #==========================================================================#
    # Define Request Interpretation
    #==========================================================================#
    def process_content(self):
        """ Process the Request content.

        Processes the content bytes and performs the desired action with them.
        The expected message should have a JSON header with the following
        structure:

        json_header = {
            'is_big_endian': <boolean>,
            'content-type': {'image/bmp', 'image/png', 'image/jpeg'},
            'content-encoding': 'binary',
            'colorspace': 'bgr8',
            'image-height': <int>,
            'image-width': <int>,
            'content-length': <int>
        }

        The actall message content is a byte array containing the image data in
        whatever format is specified by 'content-type'.
        """
        content_length = self.json_header['content-length']
        if len(self._recv_buffer) >= content_length:
            self.content = self._json_decode(self._recv_buffer[:content_length], self.json_header['content-encoding'])
            self._recv_buffer = self._recv_buffer[content_length:]

    #==========================================================================#
    # Define State Flow
    #==========================================================================#
    def process_loop(self):
        """ Loops through the message passing process for each request/response
        cycle.
        """
        while True:
            try:
                # Read request data until you have a full message
                self.read_message()

                # Perform action associated with the data

                # Send back response
                self.write_message()

                # Reset variables and start process over
                # (DO NOT reset the 'recv_buffer' in case it still has data)
                self._send_buffer = b''
                self._json_header_len = None
                self.json_header = None
                self.content = None
            except KeyboardInterrupt:
                break
            except ConnectionClosedError as err:
                print('[Connection closed]: {0}'.format(err))
                self.reset()
                print('Waiting for new connection...')
                self.wait_for_connection()

    def reset(self):
        """ Resets the variables and waits for re-connection. """
        self._recv_buffer = b''
        self._send_buffer = b''
        self._json_header_len = None
        self.json_header = None
        self.content = None

    def close_server(self):
        """ Close the server socket. """
        print('Closing server at {0}:{1}'.format(self.server_host, self.server_port))
        self.server_socket.close()

class TCPClient(TCP):
    """ Creates a TCP client connection using Python's socket module.

    This class contains the variables and functions required to make a TCP
    client connection to a server. A specific message passing sequence has been
    coded into the functions. See the module description to see the flow of
    communication.

    Attributes
    ----------
    encoding : str
        The JSON text encoding type. Should be 'utf-8' unless the code has been
        changed to handle other formats.
    available_image_types : list of str
        The possible image extensions that can be used when compressing and
        streaming to the Android app.
    image_type : str
        The image extension to use when streaming the image to the app.
    compression_value : int
        The value relating to how much the image should be compressed. PNG has a
        range from 0 (least compressed) to 9 (most compressed). JPEG has a range
        from 100 percent quality (least compressed) to 0 percent quality (most
        compressed).
    socket : socket.socket
        The client socket that connects to the server.
    new_image : bool
        Indicates whether a new images has been received since the last
        transfer.
    image_lock : thread.Lock
        A lock used to inhibit another process thread from editing the image
        variable while it is either being stored or sent.

    Methods
    -------
    create_message()
        Generate the Request message bytes.
    process_content()
        Process the Response content.
    spin_once() -> bool
        Writes a message then waits for the response. Returns whether
        successful.
    set_new_image(image)
        Updates the 'image' variable to be sent to the app.
    get_available_image_types() -> list of str
        Returns the list of available image extensions.

    Parameters
    ----------
    remote_host : str
        The IP address of the server socket.
    remote_port : int
        The port of the server socket.
    image_type : {'bmp', 'png', 'jpeg'}
        The image type to use when sending the image to the server. 'bmp'
        has no compression. 'png' has lossless compression. 'jpeg' has lossy
        compression but can be reduced to a smaller file size if desired.
        The default type is 'jpeg' since it compresses the best for image
        transfer.
    compression : int
        Compression percentage from 0 to 100. The 'png' format has
        compression values as integers from 0 to 9. In this case the
        compression will be mapped from 0 to 100 to the range 0 to 9 (9
        being the most compression). The 'jpeg' format compresses on a 0 to
        100 scale based on quality where 0 is the worse quality and 100 is
        the best. In this case the compression value will be inverted by
        being mapped from 0 to 100 to the range 100 to 0.
    """
    def __init__(self, remote_host='127.0.0.1', remote_port=65432, image_type='jpeg', compression=50):
        #======================================================================#
        # Create Socket
        #======================================================================#
        # Parameters
        self.encoding = 'utf-8'
        self.available_image_types = self.get_available_image_types()
        if (image_type not in self.available_image_types):
            raise ValueError('image_type \'{0}\' does not match the available types: {1}'.format(image_type, self.available_image_types))
        self.image_type = image_type
        self.compression_value = 0
        if (self.image_type == self.available_image_types[1]):
            # PNG range 0 to 9
            self.compression_value = int(round((9.0 - 0.0)/(100.0 - 0.0)*(compression)))
        elif (self.image_type == self.available_image_types[2]):
            # JPEG range 100 to 0
            self.compression_value = int(100 - compression)

        # Create client socket and connect to server (TCP, IPv4)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print('Connecting to server at {0}:{1}'.format(remote_host, remote_port))
        self.socket.connect((remote_host, remote_port))

        # Initialize TCP connection socket object
        super(TCPClient, self).__init__(self.socket)
        print('Local socket at {0}:{1}'.format(self.local_host, self.local_port))

        # Set to "True" everytime the callback updates the image variable
        self.new_image = False

        # Makes sure the image is not being updated while being sent
        self.image_lock = Lock()

    #==========================================================================#
    # Define Request Message
    #==========================================================================#
    def create_message(self):
        """ Generate the Request message bytes.

        Converts the message data into a byte array, generates the message
        headers and converts them into byte arrays, thens stores the bytes into
        a buffer.
        """
        # Convert image to appropriate encoding
        if (self.image_type == 'bmp'):
            retval, content_bytes = cv2.imencode('.bmp', self.image)
        elif (self.image_type == 'png'):
            retval, content_bytes = cv2.imencode('.png', self.image, [cv2.IMWRITE_PNG_COMPRESSION, self.compression_value])
        else:
            retval, content_bytes = cv2.imencode('.jpg', self.image, [cv2.IMWRITE_JPEG_QUALITY, self.compression_value])

        content_bytes = content_bytes.tobytes()

        # Create JSON Header
        json_header = {
            'is_big_endian': False,
            'content-type': 'image/' + self.image_type,
            'content-encoding': 'binary',
            'colorspace': 'bgr8',
            'image-height': self.image.shape[0],
            'image-width': self.image.shape[1],
            'content-length': len(content_bytes)
        }
        json_header_bytes = self._json_encode(json_header, self.encoding)

        # Create protocol header
        protocol_header_bytes = self._int_to_bytes(len(json_header_bytes))

        self._send_buffer = protocol_header_bytes + json_header_bytes + content_bytes

    #==========================================================================#
    # Define Response Interpretation
    #==========================================================================#
    def process_content(self):
        """ Process the Response content.

        Processes the content bytes from the Response. It indicates whether the
        image was successfully read. A message must be successfully received
        before moving on to sending a new request.

        json_message = {
            'success': <boolean>
        }
        """
        content_length = self.json_header['content-length']
        if len(self._recv_buffer) >= content_length:
            self.content = self._json_decode(self._recv_buffer[:content_length], self.json_header['content-encoding'])
            self._recv_buffer = self._recv_buffer[content_length:]

    #==========================================================================#
    # Define State Flow
    #==========================================================================#
    def spin_once(self):
        """ Process loop: writes a message, waits for response. """
        try:
            # Send out request
            if (self.new_image):
                self.image_lock.acquire()
                self.write_message()
                self.new_image = False
                self.image_lock.release()

                # Read back response
                self.read_message()

                # Reset variables and start process over
                # (don't erase the recv_buffer array in case data is still there)
                self._send_buffer = b''
                self._json_header_len = None
                self.json_header = None
                self.content = None
        except ConnectionClosedError as e:
            print(e)
            print("ConnectionClosedError")
            self.close()
            return False
        except KeyboardInterrupt:
            print("KeyboardInterrupt")
            self.close()
            return False

        return True

    def set_new_image(self, image):
        """ Receives the image and stores it to the local member variable.

        Parameters
        ----------
        image : numpy array
            This is the image to be streamed to the Android device. It should be
            in an Opencv format which is a numpy array representing the image as
            a bitmap with the BGR8 colorspace.
        """
        self.image_lock.acquire()
        self.image = image
        self.new_image = True
        self.image_lock.release()

    # Used to access the available image types before creating a client object.
    @staticmethod
    def get_available_image_types():
        return ['bmp', 'png', 'jpeg']

#==============================================================================#
# Exception Classes
#==============================================================================#
class TCPError(Exception):
    """ Base class for TCP exceptions. """
    pass

class ConnectionClosedError(TCPError):
    """ Exception raised when a connection is lost.

    Attributes
    ----------
    message : str
        The error message to raise.
    """
    def __init__(self, message):
        self.message = message

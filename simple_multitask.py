import threading
import multiprocessing
import time
import serial
import cv2
import queue

# Function to read data from Arduino (I/O-bound task)
def read_arduino_data(data_queue):
    # Simulate Arduino data fetching
    ser = serial.Serial('/dev/ttyACM0', 9600)  # Adjust the port and baud rate as needed
    while True:
        if ser.in_waiting > 0:
            data = ser.readline().decode('utf-8').strip()
            data_queue.put(data)  # Put the data into the shared queue
        time.sleep(0.5)  # To avoid busy-waiting

# Function to process data from Arduino (CPU-bound task)
def process_arduino_data(data_queue):
    while True:
        try:
            data = data_queue.get(timeout=1)  # Timeout to prevent blocking indefinitely
            print(f"Processing data: {data}")
        except queue.Empty:
            pass

# Function to capture camera frames (I/O-bound task)
def capture_camera_feed():
    cap = cv2.VideoCapture(0)  # 0 is the default camera index
    while True:
        ret, frame = cap.read()
        if ret:
            cv2.imshow("Camera Feed", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to exit
                break
    cap.release()
    cv2.destroyAllWindows()

def main():
    # Queue for sharing data between threads and processes
    data_queue = multiprocessing.Queue()

    # Create and start the threads
    arduino_thread = threading.Thread(target=read_arduino_data, args=(data_queue,))
    camera_thread = threading.Thread(target=capture_camera_feed)

    arduino_thread.start()
    camera_thread.start()

    # Create and start the multiprocessing pool for processing Arduino data
    data_processor = multiprocessing.Process(target=process_arduino_data, args=(data_queue,))
    data_processor.start()

    # Wait for all to complete (if needed)
    arduino_thread.join()
    camera_thread.join()
    data_processor.join()

if __name__ == '__main__':
    main()

import serial
import time
from struct import pack, unpack

# Se configura el puerto y el BAUD_Rate
PORT = 'COM3'  # Esto depende del sistema operativo
BAUD_RATE = 115200  # Debe coincidir con la configuracion de la ESP32

# Se abre la conexion serial
ser = serial.Serial(PORT, BAUD_RATE, timeout = 1)

# Funciones
def send_message(message):
    """ Funcion para enviar un mensaje a la ESP32 """
    ser.write(message)

def receive_response():
    """ Funcion para recibir un mensaje de la ESP32 """
    response = ser.readline()
    print(response)
    return response

def receive_data():
    """ Funcion que recibe tres floats (fff) de la ESP32 
    y los imprime en consola """
    data = receive_response()
    data = unpack("fff", data)
    print(f'Received: {data}')
    return data

def send_end_message():
    """ Funcion para enviar un mensaje de finalizacion a la ESP32 """
    end_message = pack('4s', 'END\0'.encode())
    ser.write(end_message)

# Se envia el mensaje de inicio de comunicacion

def send_begin_message():
    """ Funcion para enviar un mensaje de inicio a la ESP32 """
    message = pack('6s','BEGIN\0'.encode())
    send_message(message)


def receive_ready_message():

    data = receive_response()
    data = data.decode('utf-8', errors='ignore').strip()

    while data != 'Ready':
        data = receive_response()
        # Enviar el mensaje 'Okay' a la ESP32

def clean_serial_buffer():
    message = receive_response()
    while 'Clean' not in message.decode('utf-8', errors='ignore').strip():
        message = receive_response()
    
    message_cleaned = pack('8s','Cleaned\0'.encode())
    ser.write(message_cleaned)
    
clean_serial_buffer()


def receive_window_data():
# Se lee data por la conexion serial
    counter = 0
    message = receive_response()

    while ('Ready' not in message.decode('utf-8', errors='ignore').strip()):
        message = receive_response()
        send_begin_message()

    while True:
        if ser.in_waiting > 0:
            try:
                message = receive_data()
            except:
                #print('Error en leer mensaje')
                continue
            else: 
                counter += 1
                print(counter)
            finally:
                if counter == 15:
                    print('Lecturas listas!')
                    # Se envia el mensaje de termino de comunicacion
                    send_end_message()
                    message = receive_response()
                    while ('Ready' not in message.decode('utf-8', errors='ignore').strip()):
                        message = receive_response()
                        send_end_message()
                    break


receive_window_data()
ser.close()
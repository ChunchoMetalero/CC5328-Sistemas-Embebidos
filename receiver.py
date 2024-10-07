import serial
import time
from struct import pack, unpack

# Se configura el puerto y el BAUD_Rate
PORT = 'COM3'  # Esto depende del sistema operativo
BAUD_RATE = 115200  # Debe coincidir con la configuracion de la ESP32

# Se abre la conexion serial
ser = serial.Serial(PORT, BAUD_RATE, timeout = 1)

window_size = 0

# Funciones
def send_message(message):
    """ Funcion para enviar un mensaje a la ESP32 """
    ser.write(message)

def receive_response():
    """ Funcion para recibir un mensaje de la ESP32 """
    response = ser.readline()
    return response

def receive_data():
    """ Funcion que recibe tres floats (fff) de la ESP32 
    y los imprime en consola """

    data = receive_response()
    data = unpack("ffffffffffffffffffffffffffffffffffff", data)
    print("Temperature: ", data[0])
    print("Pressure: ", data[1])
    print("Humidity: ", data[2])
    print("CO concentration: ", data[3])
    print()

    return data

def receive_data_size():
    """ Funcion que recibe tres floats (fff) de la ESP32 
    y los imprime en consola """
    data = receive_response()
    data = unpack("ffff", data)
    return data

def send_end_message():
    """ Funcion para enviar un mensaje de finalizacion a la ESP32 """
    end_message = pack('4s', 'END\0'.encode())
    ser.write(end_message)

def receive_handshake():
    """ Función que recibe un mensaje de la ESP32 y lo imprime en consola """
    # El mensaje que vamos a enviar
    message = pack('6s', 'Okay\0'.encode())

    # Recibir el primer mensaje de la ESP32
    data = receive_response()

    # Decodificar el mensaje recibido para poder hacer la comparación
    data = data.decode('utf-8', errors='ignore').strip()

    # Mantener el ciclo mientras el mensaje no sea 'Okay'
    while data != 'Okay':
        # Enviar el mensaje 'Okay' a la ESP32
        send_message(message)
        
        # Recibir el siguiente mensaje
        data = receive_response()
        
        # Decodificar y limpiar el mensaje
        data = data.decode('utf-8', errors='ignore').strip()
    return


def receive_window_data():
# Se lee data por la conexion serial
    counter = 0

    fft_re_t = []
    fft_im_t = []

    fft_re_p = []
    fft_im_p = []

    fft_re_h = []
    fft_im_h = []

    fft_re_co = []
    fft_im_co = []
    
    while True:
        if ser.in_waiting > 0:
            try:
                message = receive_data()
                fft_re_t.append(message[8])
                fft_re_p.append(message[9])
                fft_re_h.append(message[10])
                fft_re_co.append(message[11])
                
                fft_im_t.append(message[12])
                fft_im_p.append(message[13])
                fft_im_h.append(message[14])
                fft_im_co.append(message[15])
            except:
                #print('Error en leer mensaje')
                continue
            else: 
                counter += 1
                print(counter)
            finally:
                if counter == window_size:
                    print()

                    # RMS values
                    print("RMS values:")
                    print("Temperature Rms: ", message[4])
                    print("Pressure Rms: ", message[5])
                    print("Humidity Rms: ", message[6])
                    print("CO Rms: ", message[7])
                    print()

                    # Five peaks
                    print("Five peaks:")
                    print("Five peaks of each sensor:")
                    print("Temperature: ")
                    i = 0
                    while(i < 5 and i < window_size):
                        print("Temperature: ", message[16 + i])
                        i += 1

                    print()
                    print("Pressure: ")
                    i = 0
                    while(i < 5 and i < window_size):
                        print("Pressure: ", message[21 + i])
                        i += 1

                    print()
                    print("Humidity: ")
                    i = 0
                    while(i < 5 and i < window_size):
                        print("Humidity: ", message[26 + i])
                        i += 1
                    
                    print()
                    print("CO: ")
                    i = 0
                    while(i < 5 and i < window_size):
                        print("CO: ", message[31 + i])
                        i += 1
                    
                    print()
                    print("FFT values:   Real / Imaginary")
                    print("Temperature: ")
                    i = 0
                    while(i < window_size):
                        print("Temperature FFT: ", fft_re_t[i], fft_im_t[i])
                        i += 1

                    print()
                    print("Pressure: ")
                    i = 0
                    while(i < window_size):
                        print("Pressure FFT: ", fft_re_p[i], fft_im_p[i])
                        i += 1

                    print()
                    print("Humidity: ")
                    i = 0
                    while(i < window_size):
                        print("Humidity FFT: ", fft_re_h[i], fft_im_h[i])
                        i += 1

                    print()
                    print("CO: ")
                    i = 0
                    while(i < window_size):
                        print("CO FFT: ", fft_re_co[i], fft_im_co[i])
                        i += 1

                    print('Lecturas listas!')
                    print()
                    # Se envia el mensaje de termino de comunicacion
                    send_end_message()
                    break

def receive_window_size():
    global window_size

    print("Recibiendo tamaño de ventana de datos...")
    message = pack('6s','ready\0'.encode())
    send_message(message)

    time.sleep(1)
    counter = 0
    while True:
        if ser.in_waiting > 0:
            try:
                message = receive_data_size()
            except:
                #print('Error en leer mensaje')
                continue
            else: 
                counter += 1
            finally:
                if counter == 1:
                    print()
                    print("Tamaño de ventana de datos almacenado: ", int(message[0]))
                    window_size = int(message[0])
                    # Se envia el mensaje de termino de comunicacion
                    send_end_message()
                    break
    return

def receive_menu_2():
    print("Recibiendo confirmación de cambio de tamaño de ventana de datos...")
    message = pack('6s','ready\0'.encode())
    send_message(message)

    time.sleep(1)
    counter = 0
    while True:
        if ser.in_waiting > 0:
            try:
                message = receive_data_size()
            except:
                #print('Error en leer mensaje')
                continue
            else: 
                counter += 1
            finally:
                if counter == 1:
                    print()
                    print("Tamaño de ventana de datos cambiado exitosamente!")
                    # Se envia el mensaje de termino de comunicacion
                    send_end_message()
                    break
    return

def menu():
    global window_size 
    print()
    print('Menu: A continuación se presentan las opciones disponibles')
    while True:
        print("Tamaño actual de la ventana de datos: ", window_size)
        print('1. Solicitar una ventana de datos')
        print('2. Cambiar el tamaño de la ventana de datos')
        print('3. Cerrar la conexión')
        option = int(input('Ingrese el número de la opción deseada: '))
        print()

        if option not in [1, 2, 3]:
            print('Opción no válida')
            continue
        elif option == 1:

            message1 = pack('2s','1\0'.encode())
            send_message(message1)
            print('Solicitando ventana de datos...')
            print()
            time.sleep(10)
            message2 = pack('6s','BEGIN\0'.encode())
            send_message(message2)
            time.sleep(7)
            receive_window_data()
            send_end_message()
            

        elif option == 2:
            message = pack('2s','2\0'.encode())
            send_message(message)
            time.sleep(4)
            new_window_size = int(input('Ingrese el nuevo tamaño de la ventana de datos: '))
            print('Cambiando tamaño de ventana de datos...')
            message2 = pack('4s', str(new_window_size).encode())
            send_message(message2)
            time.sleep(4)
            window_size = new_window_size
            print('Tamaño de ventana de datos cambiado exitosamente!')
            print()
        
        elif option == 3:
            print('Cerrando la conexión...')
            print()
            time.sleep(2)
            break

time.sleep(2)
# Se envia el mensaje de inicio de comunicacion
receive_handshake()
print('Conexión establecida!')
time.sleep(1)

# Se recibe el tamaño de la ventana de datos
receive_window_size()

# Se muestra el menú
menu()


ser.close()
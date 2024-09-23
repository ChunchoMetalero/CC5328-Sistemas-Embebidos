import time
import serial
from struct import pack, unpack

# Se configura el puerto y el BAUD_Rate
PORT = 'COM5'  # Esto depende del sistema operativo
BAUD_RATE = 115200  # Debe coincidir con la configuracion de la ESP32

# Se abre la conexion serial
ser = serial.Serial(PORT, BAUD_RATE, timeout = 1)

window_size = 1 # Tamaño de la ventana de datos

def receive_handshake():
    """ Función que recibe un mensaje de la ESP32 y lo imprime en consola """
    # El mensaje que vamos a enviar
    message = pack('6s', 'Okay\0'.encode())

    # Recibir el primer mensaje de la ESP32
    data = receive_response()

    # Decodificar el mensaje recibido para poder hacer la comparación
    data = data.decode('utf-8').strip()

    # Mantener el ciclo mientras el mensaje no sea 'Okay'
    while data != 'Okay':
        # Enviar el mensaje 'Okay' a la ESP32
        send_message(message)
        
        # Recibir el siguiente mensaje
        data = receive_response()
        
        # Decodificar y limpiar el mensaje
        data = data.decode('utf-8', errors='ignore').strip()
    
    print('Handshake completado con éxito')
    return


# Menu de opciones
def menu():
    global window_size
    print();
    print('Menu: A continuación se presentan las opciones disponibles')
    print(f'1. Solicitar una ventana de datos: el tamaño actual de la ventana es de {window_size}')
    print('2. Cambiar el tamaño de la ventana de datos')
    print('3. Cerrar la conexión')
    option = int(input('Ingrese una opción: '))
    if option not in [1, 2, 3]:
        print('Opción inválida')
        print('Por favor, ingrese una opción válida')

    elif option == 1:
        print("Solicitando ventana de datos")
        message = pack('2s', '1\0'.encode())
        send_message(message)
        while True:
            if ser.in_waiting > 0:
                try:
                    message = receive_window_data()
                except:
                    print('Error en leer mensaje')
                    continue
                else:
                    print(message)
                finally:
                    print('Mensaje recibido')
                    break

    elif option == 2:
        window_size = int(input('Ingrese el nuevo tamaño de la ventana de datos: '))
        print(f'El nuevo tamaño de la ventana es de {window_size}')
    
    elif option == 3:
        # Se envia el mensaje de termino de comunicacion
        ser.close()

# Funciones
def send_message(message):
    """ Funcion para enviar un mensaje a la ESP32 """
    ser.write(message)

def receive_response():
    """ Funcion para recibir un mensaje de la ESP32 """
    response = ser.readline()
    return response

# Configura el puerto serial (ajusta '/dev/ttyUSB0' o 'COMx' al que corresponda)

def receive_window_data():
    # El tamaño de los datos es 3 floats (3 * 4 bytes = 12 bytes)
    data_size = 3 * 4

    while True:
        # Lee los bytes esperados
        if ser.in_waiting >= data_size:
            raw_data = ser.read(data_size)

            # Desempaqueta los datos (3 floats)
            data = unpack('3f', raw_data)

            # Muestra los valores recibidos
            print(f"Temperatura: {data[0]}, Presión: {data[1]}, RMS Temp: {data[2]}")

        # Espera 1 segundo antes de la siguiente lectura
        time.sleep(1)
    

def send_end_message():
    """ Funcion para enviar un mensaje de finalizacion a la ESP32 """
    end_message = pack('4s', 'END\0'.encode())
    ser.write(end_message)


# Se lee data por la conexion serial
counter = 0
receive_handshake()
while True:
    menu()
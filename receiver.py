import serial
from struct import pack, unpack

# Se configura el puerto y el BAUD_Rate
PORT = 'COM5'  # Esto depende del sistema operativo
BAUD_RATE = 115200  # Debe coincidir con la configuracion de la ESP32

# Se abre la conexion serial
ser = serial.Serial(PORT, BAUD_RATE, timeout = 1)

window_size = 10

# Menu de opciones
def menu():
    print('Menu: A continuación se presentan las opciones disponibles')
    print('1. Solicitar una ventana de datos')
    print('2. Cambiar el tamaño de la ventana de datos')
    print('3. Cerrar la conexión')
    option = int(input('Ingrese una opción: '))
    if option not in [1, 2, 3]:
        print('Opción inválida')
        print('Por favor, ingrese una opción válida')
        return

    elif option == 1:
        message = pack('2s', '1\0'.encode())
        send_message(message)
        while True:
            if ser.in_waiting > 0:
                try:
                    message = receive_temps()
                except:
                    print('Error en leer mensaje')
                    continue
                else:
                    print(message)
                finally:
                    print('Mensaje recibido')
                    break
    elif option == 2:
        # por mientras no hace nada
        return 
    
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

def receive_data():
    """ Funcion que recibe tres floats (fff) de la ESP32 
    y los imprime en consola """
    data = receive_response()
    data = unpack("fff", data)
    print(f'Received: {data}')
    return data

def receive_temps():
    """ Funcion que recibe char*[window_size] de la ESP32 y los imprime en consola.
    Cada temperatura se almacena en un float """
    data = receive_response()
    data = unpack(f'{window_size}s', data)
    print(f'Received: {data}')
    return data

def send_end_message():
    """ Funcion para enviar un mensaje de finalizacion a la ESP32 """
    end_message = pack('4s', 'END\0'.encode())
    ser.write(end_message)


# Se lee data por la conexion serial
counter = 0
while True:
    menu()